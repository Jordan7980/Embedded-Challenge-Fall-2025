# Recitation 5 Accelometer & Gyroscope

## LSM6DSL Sensor 

### Sensor Ranges and Configuration

The LSM6DSL is a versatile 6-axis inertial measurement unit (IMU) that includes an accelerometer and gyroscope. One of its key features is the ability to operate at different measurement ranges to accommodate various applications.

#### Available Measurement Ranges

**Accelerometer Ranges: [acceleration due to gravity]**
- ±2g
- ±4g
- ±8g
- ±16g

**Gyroscope Ranges: [dps = degrees per second (angular velocity)]**
- ±125 dps
- ±250 dps 
- ±500 dps
- ±1000 dps
- ±2000 dps

### Accelerometer Range Configuration

Bits [3:2] in the CTRL1_XL register control the accelerometer's full-scale range:
- 00 = ±2g
- 10 = ±4g
- 11 = ±8g
- 01 = ±16g

```c
// 1. For ±2g range (default)
write_register(CTRL1_XL, 0x40);  // 0100 0000: ODR=104Hz, FS=±2g
const float ACC_SENSITIVITY = 0.061f;  // mg/LSB

// 2. For ±4g range
write_register(CTRL1_XL, 0x48);  // 0100 1000: ODR=104Hz, FS=±4g
const float ACC_SENSITIVITY = 0.122f;  // mg/LSB

// 3. For ±8g range
write_register(CTRL1_XL, 0x4C);  // 0100 1100: ODR=104Hz, FS=±8g
const float ACC_SENSITIVITY = 0.244f;  // mg/LSB

// 4. For ±16g range
write_register(CTRL1_XL, 0x44);  // 0100 0100: ODR=104Hz, FS=±16g
const float ACC_SENSITIVITY = 0.488f;  // mg/LSB
```

Example code to set ±8g range:

```c
// Configure accelerometer for ±8g range
write_register(CTRL1_XL, 0x4C);
printf("Accelerometer configured: 104 Hz, ±8g range\r\n");

// Use appropriate sensitivity for ±8g range
const float ACC_SENSITIVITY = 0.244f;  // mg/LSB
```

### Gyroscope Range Configuration

Bits [3:2] in the CTRL2_G register control the gyroscope's full-scale range:
- 00 = ±250 dps
- 01 = ±500 dps
- 10 = ±1000 dps
- 11 = ±2000 dps

Note: ±125 dps uses a different bit configuration (bit 1 = 1)

```c
// 1. For ±125 dps range
write_register(CTRL2_G, 0x42);  // 0100 0010: ODR=104Hz, FS=±125dps
const float GYRO_SENSITIVITY = 4.375f;  // mdps/LSB

// 2. For ±250 dps range (default)
write_register(CTRL2_G, 0x40);  // 0100 0000: ODR=104Hz, FS=±250dps
const float GYRO_SENSITIVITY = 8.75f;   // mdps/LSB

// 3. For ±500 dps range
write_register(CTRL2_G, 0x44);  // 0100 0100: ODR=104Hz, FS=±500dps
const float GYRO_SENSITIVITY = 17.5f;   // mdps/LSB

// 4. For ±1000 dps range
write_register(CTRL2_G, 0x48);  // 0100 1000: ODR=104Hz, FS=±1000dps
const float GYRO_SENSITIVITY = 35.0f;   // mdps/LSB

// 5. For ±2000 dps range
write_register(CTRL2_G, 0x4C);  // 0100 1100: ODR=104Hz, FS=±2000dps
const float GYRO_SENSITIVITY = 70.0f;   // mdps/LSB
```

Example code to set ±1000 dps range:

```c
// Configure gyroscope for ±1000 dps range
write_register(CTRL2_G, 0x48);
printf("Gyroscope configured: 104 Hz, ±1000 dps range\r\n");

// Use appropriate sensitivity for ±1000 dps range
const float GYRO_SENSITIVITY = 35.0f;  // mdps/LSB
```



### Why Sensor Values Keep Changing Even When the Board is at rest

When your board is at still position, you might see the accelerometer and gyroscope values jumping around a bit. Don't worry, this is totally normal! Here's why this happens:

1. **Sensor Noise**: These tiny MEMS sensors always have some random noise in them
2. **Temperature Stuff**: Even small temp changes in the room can make readings drift a bit
3. **Digital Conversion Issues**: When converting analog signals to digital, you lose some precision
4. **Gravity is Always There**: Your accelerometer is always feeling Earth's gravity (about 1g) even when still
5. **Tiny Vibrations**: People walking around, AC running, or your table vibrating slightly - sensors pick these up!

To make readings more stable, you could try:
- Adding a simple filter to smooth things out
- Calibrating the sensor when it's not moving
- Ignoring really small changes (like less than 0.02g)

### How I2C Works on This Board

The STM32 board talks to different sensors using something called I2C. It's pretty cool because it uses same communication lines to talk to multiple devices!

#### I2C Basics

1. **Shared bus**: All sensors connect to the same two pins (SDA = PB11, SCL = PB10)

2. **Each Sensor Has Its Own Address**: Like houses on a street, each sensor has a unique address:
   - Accelerometer/gyroscope: 0x6A (0xD4/0xD5 when reading/writing)
   - Humidity/temperature sensor: 0x5F (0xBE/0xBF)
   - Pressure sensor: 0x5D (0xBA/0xBB)
   - Magnetometer: 0x1E (0x3C/0x3D)
   - Distance sensor: 0x29 (0x52/0x53)

3. **Address Format**: We shift the address left by 1 bit and use the last bit to say if we're reading (1) or writing (0).

4. **Talking to One Sensor**: The microcontroller first sends the address of which sensor it wants to talk to. Only that sensor responds.

5. **Picking a Register**: After the sensor responds, the microcontroller tells it which specific register (like a memory location) it wants to read or change.

This way, all sensors can share the same communiation lines without getting mixed up!

Please Refer to the Datasheet of LSM6DSL datasheet in brighspace for more info about this sensor

**User Manual Page References**

| Section / Figure | Description        | Page No. |
| ---------------- | ------------------ | -------- |
| **7.12.4**       | Sensor Description | **26**   |
| **7.15**         | I²C Address        | **17**   |
| **Figure 22**    | Schematic Diagram  | **42**   |
| **Figure 27**    | Sensor Schematic   | **47**   |


**In LSM6DSL manual**

| Section | Description          | Page Range    |
| ------- | -------------------- | --------------|
| **6.3** | I²C Serial Interface | **38**        |
| **8**   | Register Mapping     | **48 – 51**   |

