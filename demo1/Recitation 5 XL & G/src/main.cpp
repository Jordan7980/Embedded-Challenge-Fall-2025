#include <cstdio>
#include <cmath>
#include <cstring>
#include "mbed.h"

// ==== BLE 相关 ====
#include "ble/BLE.h"
#include "ble/Gap.h"
#include "ble/gap/AdvertisingDataBuilder.h"
#include "ble/GattServer.h"
#include "ble/gatt/GattCharacteristic.h"
#include "ble/gatt/GattService.h"
#include "ble/UUID.h"
#include "events/EventQueue.h"

// Initialize I2C on pins PB_11 (SDA) and PB_10 (SCL)
I2C i2c(PB_11, PB_10);

// LSM6DSL I2C address (0x6A shifted left by 1 for Mbed's 8-bit addressing)
#define LSM6DSL_ADDR        (0x6A << 1)
#define WHO_AM_I            0x0F
#define CTRL1_XL            0x10
#define CTRL2_G             0x11
#define CTRL3_C             0x12
#define DRDY_PULSE_CFG      0x0B
#define INT1_CTRL           0x0D
#define STATUS_REG          0x1E
#define OUTX_L_G            0x22
#define OUTX_L_XL           0x28

// INT1 interrupt pin connected to PD_11
#define LSM6DSL_INT1_PIN    PD_11

// FOG detection states
typedef enum {ON_START, STATIONARY, WALKING, FREEZING} STATE;

// global Variables
const float FS_HZ = 52.0f;    // Sampling frequency
const int N = 156;          // 3-second window
float acc_buf[N];
int buf_idx = 0;

//FOG detection variables
int stepCounter = 0;
float currentTime = 0.0f;
float lastStepTime = 0.0f;
STATE CurrentState = ON_START;
STATE PreviousState = ON_START;

typedef struct {
    float acc[3];
    float gyro[3];
} ImuSample;

// Event queue for print task（目前没用，但保留）
EventQueue print_queue;

// INT1 中断
InterruptIn int1(LSM6DSL_INT1_PIN, PullDown);
volatile bool data_ready = false;

// 串口（USB 虚拟 COM），仍然保留 printf 输出
static BufferedSerial pc(USBTX, USBRX, 115200);

// ==== BLE 全局（注意：用的是 BLE，而不是 ble::BLE）====
static BLE &ble_inst = BLE::Instance();
static events::EventQueue ble_queue(16 * EVENTS_EVENT_SIZE);

// 自定义 “UART-like” Service / Characteristic UUID（NUS 风格）
static const UUID UART_SERVICE_UUID("6e400001-b5a3-f393-e0a9-e50e24dcca9e");
static const UUID UART_TX_CHAR_UUID("6e400003-b5a3-f393-e0a9-e50e24dcca9e");

// TX 特征 buffer
static uint8_t tx_buf[244];

static GattCharacteristic tx_char(
    UART_TX_CHAR_UUID,
    tx_buf,
    0,
    sizeof(tx_buf),
    GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY
);

static GattCharacteristic *uart_chars[] = { &tx_char };
static GattService uart_service(UART_SERVICE_UUID, uart_chars, sizeof(uart_chars) / sizeof(uart_chars[0]));

// ======== 把一行字符串发到 USB 串口 + BLE =========
void send_line(const char *s) {
    // 1) USB 串口照旧
    printf("%s", s);

    // 2) 尝试通过 BLE notify 发出去（即便没连上，失败也无所谓）
    uint16_t len = (uint16_t)strlen(s);
    if (len > sizeof(tx_buf)) {
        len = sizeof(tx_buf);
    }

    ble_error_t err = ble_inst.gattServer().write(
        tx_char.getValueHandle(),
        (const uint8_t *)s,
        len,
        /*local_only*/ false
    );
    (void)err; // 需要的话可以 printf 打出来调试
}

// ======== BLE 广播相关 =========
void start_advertising()
{
    ble::Gap &gap = ble_inst.gap();

    // 设置广播参数：可连接、100ms 间隔
    gap.setAdvertisingParameters(
        ble::LEGACY_ADVERTISING_HANDLE,
        ble::AdvertisingParameters(
            ble::advertising_type_t::CONNECTABLE_UNDIRECTED,
            ble::adv_interval_t(ble::millisecond_t(100))
        )
    );

    uint8_t adv_buffer[ble::LEGACY_ADVERTISING_MAX_SIZE];
    ble::AdvertisingDataBuilder adv_builder(adv_buffer);

    adv_builder.clear();
    adv_builder.setFlags();                 // 默认 flags
    adv_builder.setName("Tremor-STM32");   // 广播名：网页端用 namePrefix 过滤

    gap.setAdvertisingPayload(
        ble::LEGACY_ADVERTISING_HANDLE,
        adv_builder.getAdvertisingData()
    );

    gap.startAdvertising(ble::LEGACY_ADVERTISING_HANDLE);

    printf("BLE advertising as Tremor-STM32\r\n");
}

void ble_init_complete(BLE::InitializationCompleteCallbackContext *params)
{
    if (params->error != BLE_ERROR_NONE) {
        printf("BLE init failed: %d\r\n", params->error);
        return;
    }

    BLE &ble = params->ble;
    GattServer &gatt = ble.gattServer();

    ble_error_t err = gatt.addService(uart_service);
    if (err) {
        printf("addService error: %d\r\n", err);
    }

    start_advertising();
}

// BLE 事件调度 —— 按官方 demo 写法
void schedule_ble_events(BLE::OnEventsToProcessCallbackContext *context)
{
    ble_queue.call(Callback<void()>(&context->ble, &BLE::processEvents));
}

// ==== IMU / 信号处理部分 ====

// 数据就绪中断
void data_ready_isr() {
    data_ready = true;
}

bool write_reg(uint8_t reg, uint8_t val) {
    char buf[2] = {(char)reg, (char)val};
    return (i2c.write(LSM6DSL_ADDR, buf, 2) == 0);
}

bool read_reg(uint8_t reg, uint8_t &val) {
    char r = (char)reg;
    if (i2c.write(LSM6DSL_ADDR, &r, 1, true) != 0) return false;
    if (i2c.read(LSM6DSL_ADDR, &r, 1) != 0) return false;
    val = (uint8_t)r;
    return true;
}

bool read_int16(uint8_t reg_low, int16_t &val) {
    uint8_t lo, hi;
    if (!read_reg(reg_low, lo)) return false;
    if (!read_reg(reg_low + 1, hi)) return false;
    val = (int16_t)((hi << 8) | lo);
    return true;
}

bool init_sensor() {
    uint8_t who;
    if (!read_reg(WHO_AM_I, who) || who != 0x6A) {
        printf("Sensor not found! WHO_AM_I=0x%02X\r\n", who);
        return false;
    }

    // BDU + auto-increment
    write_reg(CTRL3_C, 0x44);
    // Accel: 104 Hz, ±16g
    write_reg(CTRL1_XL, 0x30);
    // Gyro:  104 Hz, ±250 dps
    write_reg(CTRL2_G, 0x30);
    // INT1: accel + gyro DRDY
    write_reg(INT1_CTRL, 0x03);
    // DRDY pulse
    write_reg(DRDY_PULSE_CFG, 0x80);

    ThisThread::sleep_for(100ms);

    uint8_t dummy;
    read_reg(STATUS_REG, dummy);
    int16_t temp;
    for (int i = 0; i < 6; i++) {
        read_int16(OUTX_L_XL + i*2, temp);
    }

    int1.rise(&data_ready_isr);

    return true;
}

float estimate_frequency(float *x, int N, float fs) {
    int zero_crossings = 0;
    for (int i = 1; i < N; ++i) {
        if ((x[i-1] <= 0 && x[i] > 0) ||
            (x[i-1] >= 0 && x[i] < 0)) {
            zero_crossings++;
        }
    }
    float cycles   = zero_crossings / 2.0f;
    float duration = N / fs;
    return cycles / duration;
}

#define MY_PI 3.14159265359f

// 简单的“定点频率扫描”，用于找主频
float estimate_frequency_fft_like(float *x, int N, float fs, float &best_mag_out) {
    // 扫描 0.5 ~ 10 Hz 的主频，FOG 可能在 0.5~3Hz 这一段
    const float f_min = 0.5f;
    const float f_max = 10.0f;

    float best_f   = 0.0f;
    float best_mag = 0.0f;

    for (float f = f_min; f <= f_max; f += 0.25f) {
        float re = 0.0f;
        float im = 0.0f;

        for (int n = 0; n < N; ++n) {
            float t     = n / fs;
            float angle = -2.0f * MY_PI * f * t;
            float c     = cosf(angle);
            float s     = sinf(angle);
            re += x[n] * c;
            im += x[n] * s;
        }

        float mag = sqrtf(re * re + im * im);

        if (mag > best_mag) {
            best_mag = mag;
            best_f   = f;
        }
    }

    best_mag_out = best_mag;
    return best_f;
}

// state 编码：
//   0 = Normal / Other
//   1 = Tremor-like (3–5 Hz)
//   2 = Dyskinesia-like (5–7 Hz)
//   3 = FOG-like (0.5–3 Hz, 振幅中等偏低)
void analyze_tremor(float *buf, int N) {
    const float fs = FS_HZ;

    // 1. 去 DC
    float mean = 0.0f;
    for (int i = 0; i < N; ++i) {
        mean += buf[i];
    }
    mean /= N;
    for (int i = 0; i < N; ++i) {
        buf[i] -= mean;
    }

    // 2. 计算 RMS
    float energy = 0.0f;
    for (int i = 0; i < N; ++i) {
        energy += buf[i] * buf[i];
    }
    float rms = sqrtf(energy / N);

    char line[64];

    // 输出 RMS
    snprintf(line, sizeof(line), ">rms:%.4f\n", rms);
    send_line(line);

    // 振幅阈值（可以后续调参）
    const float AMP_VERY_LOW = 0.008f;  // 几乎不动
    const float AMP_LOW      = 0.02f;   // 有一点抖动
    const float AMP_HIGH     = 0.08f;   // 太大了就当乱动，不判 FOG

    // 2.1 几乎没动：直接 Normal
    if (rms < AMP_VERY_LOW) {
        snprintf(line, sizeof(line), ">freq:0.00\n");
        send_line(line);
        snprintf(line, sizeof(line), ">state:0\n");
        send_line(line);
        return;
    }

    // 3. 频谱主频
    float mag = 0.0f;
    float f   = estimate_frequency_fft_like(buf, N, fs, mag);

    snprintf(line, sizeof(line), ">freq:%.2f\n", f);
    send_line(line);

    int state = 0;

    // ---- FOG-like: 0.5–3 Hz + 振幅中等 ----
    if (f >= 0.5f && f < 3.0f && rms >= AMP_LOW && rms <= AMP_HIGH) {
        state = 3;  // Freezing of Gait-like
    }
    // ---- Tremor-like: 3–5 Hz ----
    else if (f >= 3.0f && f < 5.0f) {
        state = 1;
    }
    // ---- Dyskinesia-like: 5–7 Hz ----
    else if (f >= 5.0f && f < 7.0f) {
        state = 2;
    }
    // 其余频段 / 振幅过大/乱，归 0
    else {
        state = 0;
    }

    snprintf(line, sizeof(line), ">state:%d\n", state);
    send_line(line);
}

//================STEP DETECTION=================
bool detect_steps(float acc_mag) {
    const float stepThreshold = 0.20f; //Threshold peakVal has to be, to be considered a step
    const float minStepInterval = 0.5f; //minimum time between steps to prevent misreadings

    static float last_mag = 0.0f;
    static float peakVal = 0.0f;
    static bool rising = false;

    if (acc_mag > last_mag) { //if acceleration is higher than the last recorded acceleration
        rising = true; //acceleration is rising (meaning a step is currently being taken)
        if (acc_mag > peakVal) {
            peakVal = acc_mag; //We will use this to help determine if a step was taken
        }
    }else if (rising && acc_mag < last_mag) {// checks if the person is finishing his step
        rising = false; //Step is now finished
        if (peakVal > stepThreshold && (currentTime - lastStepTime) > minStepInterval ) { //if the peak crosses the set threshold
            stepCounter++; //update steps
            lastStepTime = currentTime;
            peakVal = 0.0f;
            last_mag = acc_mag;
            return true; // Step has been detected
        }
        peakVal = 0.0f;
    }

    last_mag = acc_mag; //remember current mag as last_mag for the next sample

    return false;
}
//=================================================================


//==================Cadence=============
float findCadence() {
    if (currentTime == 0.0f || stepCounter == 0) return 0.0f;
    return (stepCounter / currentTime) * 60.0f;
}

//======================FOG STATE MACHINE==================
void analyzeFOG(bool stepDetected) {

    const float freezeTime = 2.0f; //4 seconds without steps
    const float minCadence = 20.0f; // need at least 20 steps/min to be considered walking

    float timeSinceLastStep = currentTime - lastStepTime;
    if (timeSinceLastStep < 0.0f) {
        timeSinceLastStep = 0.0f;
    }

    float cadence = findCadence();

    switch (CurrentState) {
        case ON_START:
            CurrentState = STATIONARY;
            PreviousState = CurrentState;
            break;
        case STATIONARY:
            PreviousState = CurrentState;
            if (stepDetected && cadence > minCadence) {
                CurrentState = WALKING;
            }
            break;
        case WALKING:
            PreviousState = CurrentState;
            if (timeSinceLastStep > freezeTime) {
                CurrentState = FREEZING;
                printf("FOG Detected\n");
            }
            break;
        case FREEZING:
            PreviousState = CurrentState;
            stepCounter = 0;
            if (stepDetected) {
                CurrentState = WALKING;
                printf("FOG Recovered\n");
            }
            break;
    }

}

void read_sensor_data() {
    int16_t acc[3], gyro[3];

    for (int i = 0; i < 3; i++) {
        read_int16(OUTX_L_XL + i*2, acc[i]);
        read_int16(OUTX_L_G + i*2,  gyro[i]);
    }

    ImuSample sample;
    sample.acc[0] = acc[0] * 0.000061f;
    sample.acc[1] = acc[1] * 0.000061f;
    sample.acc[2] = acc[2] * 0.000061f;

    sample.gyro[0] = gyro[0] * 0.00875f;
    sample.gyro[1] = gyro[1] * 0.00875f;
    sample.gyro[2] = gyro[2] * 0.00875f;

    currentTime += (1.0f/FS_HZ);

    float ax = sample.acc[0];
    float ay = sample.acc[1];   // For example, the Y-axis.
    float az = sample.acc[2];

    float mag = sqrtf(ax*ax + ay*ay + az*az);

    acc_buf[buf_idx++] = ay;

    float step = fabsf(mag - 1.0f);
    analyzeFOG(detect_steps(step));

    if (buf_idx >= N) {
        buf_idx = 0;
        analyze_tremor(acc_buf, N);

        float cadence = findCadence();       // steps per minute

        printf(">steps:%d\n", stepCounter);  // total steps so far
        printf(">cadence:%.2f\n", cadence);  // walking cadence in steps/min
        if(CurrentState == STATIONARY){
            printf(">is_Stationary:1\n");
        }else if(CurrentState == WALKING){
            printf(">is_Walking:1\n");
        }else if(CurrentState == FREEZING){
            printf(">is_FOG:1\n");
        }
        printf(">--------------------------\n");
    }
}

void acquisition_task() {
    while (true) {
        if (data_ready) {
            data_ready = false;
            read_sensor_data();
        }
        ThisThread::sleep_for(1ms);
    }
}

void print_task() {
    print_queue.dispatch_forever();
}

int main() {
    // I2C 设为 400kHz
    i2c.frequency(400000);

    if (!init_sensor()) {
        while (1) {
            ThisThread::sleep_for(1s);
        }
    }

    // ==== 初始化 BLE ====
    ble_inst.onEventsToProcess(schedule_ble_events);
    ble_inst.init(ble_init_complete);

    // BLE 事件线程
    Thread ble_thread;
    ble_thread.start(callback(&ble_queue, &events::EventQueue::dispatch_forever));

    // 采集线程
    Thread acq_thread;
    acq_thread.start(acquisition_task);

    // 打印线程（目前没用，但保留）
    Thread print_thread;
    print_thread.start(print_task);

    while (true) {
        ThisThread::sleep_for(1s);
    }
}
