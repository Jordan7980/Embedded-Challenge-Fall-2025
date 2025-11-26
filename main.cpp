#include <cstdio>
#include "mbed.h"

// Initialize I2C on pins PB_11 (SDA) and PB_10 (SCL)
I2C i2c(PB_11, PB_10);

// LSM6DSL I2C address (0x6A shifted left by 1 for Mbed's 8-bit addressing)
#define LSM6DSL_ADDR        (0x6A << 1)
// Register addresses
#define WHO_AM_I            0x0F  // Device identification register
#define CTRL1_XL            0x10  // Accelerometer control register
#define CTRL2_G             0x11  // Gyroscope control register
#define CTRL3_C             0x12  // Common control register
#define DRDY_PULSE_CFG      0x0B  // Data-ready pulse configuration
#define INT1_CTRL           0x0D  // INT1 pin routing control
#define STATUS_REG          0x1E  // Status register (data ready flags)
#define OUTX_L_G            0x22  // Gyroscope X-axis low byte start address
#define OUTX_L_XL           0x28  // Accelerometer X-axis low byte start address

// INT1 interrupt pin connected to PD_11
#define LSM6DSL_INT1_PIN    PD_11

// 全局或静态
const int N = 312;          // 3 秒窗口
float acc_buf[N];
int buf_idx = 0;

// Structure to hold one IMU sample
typedef struct {
    float acc[3];
    float gyro[3];
} ImuSample;

// Event queue for print task
EventQueue print_queue;

// Configure INT1 as interrupt input with pull-down resistor
InterruptIn int1(LSM6DSL_INT1_PIN, PullDown);
// Flag set by interrupt when new data is ready
volatile bool data_ready = false;

// Interrupt service routine - sets flag when data is ready
void data_ready_isr() { 
    data_ready = true; 
}

// Write a single byte to a register
bool write_reg(uint8_t reg, uint8_t val) {
    // Create buffer with register address and value
    char buf[2] = {(char)reg, (char)val};
    // Write to I2C and return success status
    return (i2c.write(LSM6DSL_ADDR, buf, 2) == 0);
}

// Read a single byte from a register
bool read_reg(uint8_t reg, uint8_t &val) {
    // Store register address to read from
    char r = (char)reg;
    // Write register address with repeated start condition
    if (i2c.write(LSM6DSL_ADDR, &r, 1, true) != 0) return false;
    // Read the register value
    if (i2c.read(LSM6DSL_ADDR, &r, 1) != 0) return false;
    // Store result in output parameter
    val = (uint8_t)r;
    return true;
}

// Read 16-bit signed integer from two consecutive registers
bool read_int16(uint8_t reg_low, int16_t &val) {
    uint8_t lo, hi;
    // Read low byte
    if (!read_reg(reg_low, lo)) return false;
    // Read high byte from next register
    if (!read_reg(reg_low + 1, hi)) return false;
    // Combine bytes into 16-bit value (little-endian)
    val = (int16_t)((hi << 8) | lo);
    return true;
}

// Initialize the LSM6DSL sensor
bool init_sensor() {
    uint8_t who;
    // Read WHO_AM_I register and verify it's 0x6A
    if (!read_reg(WHO_AM_I, who) || who != 0x6A) {
        printf("Sensor not found!\r\n");
        return false;
    }
    
    // Configure CTRL3_C: Block data update + auto-increment address
    write_reg(CTRL3_C, 0x44);
    // Configure accelerometer: 104 Hz, ±16g range
    write_reg(CTRL1_XL, 0x54);
    // Configure gyroscope: 104 Hz, ±250 dps range
    write_reg(CTRL2_G, 0x50);
    // Route data-ready signal to INT1 pin
    write_reg(INT1_CTRL, 0x03);
    // Enable pulsed data-ready mode (50μs pulses)
    write_reg(DRDY_PULSE_CFG, 0x80);
    
    // Wait for sensor to stabilize
    ThisThread::sleep_for(100ms);
    
    // Clear status register
    uint8_t dummy;
    read_reg(STATUS_REG, dummy);
    // Clear old data by reading all output registers
    int16_t temp;
    for (int i = 0; i < 6; i++) {
        read_int16(OUTX_L_XL + i*2, temp);
    }
    
    // Attach interrupt handler for rising edge on INT1
    int1.rise(&data_ready_isr);
    
    return true;
}

// Print function - called by event queue
void print_sample(ImuSample sample) {
    // Print in Teleplot format (>name:value)
    printf(">acc_x:%.3f\n>acc_y:%.3f\n>acc_z:%.3f\n>gyro_x:%.2f\n>gyro_y:%.2f\n>gyro_z:%.2f\n",
           sample.acc[0], sample.acc[1], sample.acc[2],
           sample.gyro[0], sample.gyro[1], sample.gyro[2]);
}

float estimate_frequency(float *x, int N, float fs) {
    int zero_crossings = 0;
    for (int i = 1; i < N; ++i) {
        if ((x[i-1] <= 0 && x[i] > 0) ||
            (x[i-1] >= 0 && x[i] < 0)) {
            zero_crossings++;
        }
    }
    float cycles = zero_crossings / 2.0f;      // 一个周期约两个零交叉
    float duration = N / fs;                   // 秒
    return cycles / duration;                  // Hz
}

void analyze_tremor(float *buf, int N) {
    const float fs = 104.0f;   // 你的采样率

    // 去掉平均值，避免 DC
    float mean = 0;
    for (int i = 0; i < N; ++i) mean += buf[i];
    mean /= N;
    for (int i = 0; i < N; ++i) buf[i] -= mean;

    float f = estimate_frequency(buf, N, fs);

    // 根据频率范围判断状态
    if (f > 3.0f && f < 5.0f) {
        printf(">state:1\n");  // 1 = tremor
    } else if (f > 5.0f && f < 7.0f) {
        printf(">state:2\n");  // 2 = dyskinesia
    } else {
        printf(">state:0\n");  // 0 = normal / other
    }
}


// Read sensor data and post to event queue
void read_sensor_data() {
    // Arrays to store raw 16-bit values
    int16_t acc[3], gyro[3];
    
    // Read all 3 axes for accelerometer and gyroscope
    for (int i = 0; i < 3; i++) {
        read_int16(OUTX_L_XL + i*2, acc[i]);
        read_int16(OUTX_L_G + i*2, gyro[i]);
    }
    
    // Create sample struct
    ImuSample sample;
    // Convert accelerometer raw values to g (±16g range: 0.488 mg/LSB)
    sample.acc[0] = acc[0] * 0.000488f;
    sample.acc[1] = acc[1] * 0.000488f;
    sample.acc[2] = acc[2] * 0.000488f;
    
    // Convert gyroscope raw values to dps (±250 dps range: 8.75 mdps/LSB)
    sample.gyro[0] = gyro[0] * 0.00875f;
    sample.gyro[1] = gyro[1] * 0.00875f;
    sample.gyro[2] = gyro[2] * 0.00875f;
    

    float ay = sample.acc[1];   // 例如 Y 轴

    acc_buf[buf_idx++] = ay;

    if (buf_idx >= N) {
        buf_idx = 0;
        // 这里调用一个函数做频率分析
        analyze_tremor(acc_buf, N);
    }
    // Post print event to queue with struct
    // print_queue.call(print_sample, sample);
    // static int counter = 0;

    // if (++counter >= 10) {  // 每10次采样打印一次
    //     counter = 0;
    //     print_queue.call(print_sample, sample);
    // }

}

// Acquisition task - reads sensor when data is ready
void acquisition_task() {
    while (true) {
        // Check if new data is ready
        if (data_ready) {
            // Clear flag
            data_ready = false;
            // Read and queue sensor data
            read_sensor_data();
        }
        // Short sleep to prevent busy-waiting
        ThisThread::sleep_for(1ms);
    }
}

// Print task - dispatches event queue
void print_task() {
    print_queue.dispatch_forever();
}

int main() {
    // Configure serial port
    static BufferedSerial pc(USBTX, USBRX, 115200);

    // Set I2C clock speed to 400 kHz (fast mode)
    i2c.frequency(400000);
    
    // Initialize sensor and halt if it fails
    if (!init_sensor()) {
        while(1) { ThisThread::sleep_for(1s); }
    }
    
    // Create acquisition thread
    Thread acq_thread;
    acq_thread.start(acquisition_task);
    
    // Create print thread
    Thread print_thread;
    print_thread.start(print_task);
    
    // Main thread does nothing, other threads handle everything
    while (true) {
        ThisThread::sleep_for(1s);
    }
}