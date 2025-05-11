#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>
#include <stdbool.h>

extern int mpu;
extern double dt;
extern volatile int data_ready;
extern double prevtime;
extern double currtime;

extern float *yaw_ptr;


// MPU6050 Register Addresses and Modes
#define MPU6050_REG_ACCEL_XOUT      0x3B // High: 3B, Low: 3C
#define MPU6050_REG_ACCEL_YOUT      0x3D // High: 3D, Low: 3E
#define MPU6050_REG_ACCEL_ZOUT      0x3F // High: 3F, Low: 40
#define MPU6050_REG_TEMP_OUT        0x41 // High: 41, Low: 42
#define MPU6050_REG_GYRO_XOUT       0x43 // High: 43, Low: 44
#define MPU6050_REG_GYRO_YOUT       0x45 // High: 45, Low: 46
#define MPU6050_REG_GYRO_ZOUT       0x47 // High: 47, Low: 48
#define MPU6050_REG_SAMPLE_RATE     0x19 // tần số lấy mẫu
#define MPU6050_REG_CONFIGURATION   0x1A // low-pass filter 
#define MPU6050_REG_GYRO_CONF       0x1B // Gyro 
#define MPU6050_REG_ACCEL_CONF      0x1C // Accel 
#define MPU6050_REG_INTERUPT        0x38 // ngắt
#define MPU6050_REG_PWR_MANAGE      0x6B // Đánh thức MPU

int16_t Mpu6050_read(uint8_t reg);
float Mpu6050_read_temperature();
double getTime();

/**
 * @brief 
 * Xem tại liệu Embebed
 */
void Mpu6050_init(uint8_t conf, uint8_t smplrt_div, uint8_t accel_conf, uint8_t gryo_conf, uint8_t interupt, uint8_t PWR);

/**
 * @brief 
 * @param FUll_Scale_Accel accelerometer 2g = 2, 4g = 4, 8g = 8, 16g = 16
 */
void Mpu6050_read_angles(float *angle_x, float *angle_y, int Full_Scale_Accel);

/**
 * @brief 
 * @param FUll_Scale_Accel accelerometer 2g = 2, 4g = 4, 8g = 8, 16g = 16
 */
float getAccelScale(int Full_Scale_Accel);

/**
 * @brief 
 * @param FUll_Scale_Gyro gyroscope 250°/s = 250, 500°/s = 500, 1000°/s = 1000, 2000°/s = 2000
 */
float getGyroScale(int Full_Scale_Gyro);

/**
 * @brief hàm xử lý ngắt
 */
void Mpu6050_int_handler(void);

/**
 * @brief hàm ngắt chân pin
 */
void Mpu6050_setup_interrupt(int int_pin);

/**
 * @brief hàm khởi tạo giá trị ban đầu (chỉ chạy 1 lần)
 */
void Mpu6050_initialize_value(int Full_Scale_Accel, int Full_Scale_Gyro);

/**
 * @brief 
 * @param calibration_mode TRUE khi muốn hiệu chuẩn, FALSE bỏ qua
 * @param so_lan số lần muốn hiệu chuẩn
 * @param FUll_Scale_Accel accelerometer 2g = 2, 4g = 4, 8g = 8, 16g = 16
 * @param FUll_Scale_Gyro gyroscope 250°/s = 250, 500°/s = 500, 1000°/s = 1000, 2000°/s = 2000
 */
void Mpu6050_read_angles_moves(bool calibration_mode, int so_lan, float *roll, float *pitch, float *yaw, int Full_Scale_Accel, int Full_Scale_Gyro);

// Hàm trả về trạng thái của xe:
// 'P' : xe đang chạy (nếu dao động gia tốc vượt ngưỡng)
// 'D' : xe đang đỗ
char Mpu6050_get_vehicle_state(void);

void Mpu6050_read_acceleration(float *ax, float *ay, float *az);

#endif // MPU6050_H