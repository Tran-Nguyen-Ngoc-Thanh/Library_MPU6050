#include "Mpu6050.h"
#include "Max7219.h"
#include <stdio.h>
#include <wiringPiI2C.h>
#include <math.h>
#include <unistd.h> 
#include <wiringPi.h>
#include <sys/time.h>

// define
//-----------------------------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------------------------

#define M_PI 3.14159265358979323846
#define MPU6050_ADDR 0x68

#define SAMPLE_WINDOW_SIZE    20
#define ACCEL_STD_THRESHOLD   1.0

double dt = 0.0;
volatile int data_ready = 0;
double prevtime;
double currtime;
int mpu;

float *yaw_ptr;

// Biến global
//-----------------------------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------------------------

static float gyroX_offset = 0, gyroY_offset = 0, gyroZ_offset = 0;
static float pitch_prev = 0, roll_prev = 0, yaw_prev = 0;
static float gyrox_prev = 0, gyroy_prev = 0, gyroz_prev = 0;
static int initialize_value  = 0;
// static float roll = 0, pitch = 0, yaw = 0a;
static float AccX = 0, AccY = 0, AccZ = 0;
static float gyrox = 0, gyroy = 0, gyroz = 0;
static float pitch_gyro = 0, roll_gyro = 0;
static float pitch_acc = 0, roll_acc = 0;
static int bien = 0;
static int state = 0;

static float acceleration_window[SAMPLE_WINDOW_SIZE];

static int sample_index = 0;
static bool window_is_full = false;

// Ngưỡng độ lệch chuẩn (standard deviation) của gia tốc để xác định xe đang chạy

//-----------------------------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------------------------

int16_t Mpu6050_read(uint8_t reg){
    uint8_t buffer[2];
    buffer[0]= wiringPiI2CReadReg8(mpu, reg);      //high
    buffer[1]= wiringPiI2CReadReg8(mpu, reg + 1);  //low
    return (int16_t)(buffer[0] << 8) | buffer[1];
}

//-----------------------------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------------------------

void Mpu6050_init(uint8_t conf, uint8_t smplrt_div, uint8_t accel_conf, uint8_t gryo_conf, uint8_t interupt, uint8_t PWR){
    mpu = wiringPiI2CSetup(MPU6050_ADDR);
    if (mpu == -1) {
        printf("❌ Lỗi: Không thể kết nối với DS3231!\n");
        return;
    }
    wiringPiI2CWriteReg8 (mpu, MPU6050_REG_SAMPLE_RATE, smplrt_div);   //500Hz                            1
    wiringPiI2CWriteReg8 (mpu, MPU6050_REG_CONFIGURATION, conf);      //không dùng xung ngoài, tắt DLF   0
    wiringPiI2CWriteReg8 (mpu, MPU6050_REG_GYRO_CONF, gryo_conf);     //Gyro +- 500 do/s                 0x08
    wiringPiI2CWriteReg8 (mpu, MPU6050_REG_ACCEL_CONF, accel_conf);   //Accel +- 8g                      0x10
    wiringPiI2CWriteReg8 (mpu, MPU6050_REG_INTERUPT, interupt);       //Mở interupt của data delay       1
    wiringPiI2CWriteReg8 (mpu, MPU6050_REG_PWR_MANAGE, PWR);          //nguồn xung MPU6050               0x01

    printf("✅ MPU6050 is ready!\n");
}

//-----------------------------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------------------------

float Mpu6050_read_temperature(){
    int16_t temp = Mpu6050_read(MPU6050_REG_TEMP_OUT);
    return temp/340.0 + 36.53;
}

//-----------------------------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------------------------

float getAccelScale(int Full_Scale_Accel){
    switch (Full_Scale_Accel) {
        case 2:  return 16384.0;
        case 4:  return 8192.0;
        case 8:  return 4096.0;
        case 16: return 2048.0;
        default: return -1; // Sai cấu hình
    }
}

//-----------------------------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------------------------

float getGyroScale(int Full_Scale_Gyro){
    switch (Full_Scale_Gyro) {
        case 250:  return 131.0;
        case 500:  return 65.5;
        case 1000: return 32.8;
        case 2000: return 16.4;
        default:   return -1; // Sai cấu hình
    }
}

//-----------------------------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------------------------

void Mpu6050_read_angles(float *angle_x, float *angle_y, int Full_Scale_Accel){
    float sum_x = 0;
    float sum_y = 0;
    float scaleA = getAccelScale(Full_Scale_Accel);
    
    for(int i = 0; i< 5; i++){
        int16_t acc_x = Mpu6050_read(MPU6050_REG_ACCEL_XOUT);
        int16_t acc_y = Mpu6050_read(MPU6050_REG_ACCEL_YOUT);
        int16_t acc_z = Mpu6050_read(MPU6050_REG_ACCEL_ZOUT);

        // Kiểm tra xem có lỗi không
        if (acc_x == 0 && acc_y == 0 && acc_z == 0) {
            printf("Lỗi đọc cảm biến MPU6050!\n");
            return;
        }
        if (scaleA < 0) {
            printf("Cấu hình Full Scale không hợp lệ!\n");
            return;
        }
    
        float ax = acc_x /scaleA;
        float ay = acc_y /scaleA;
        float az = acc_z /scaleA;
        
        sum_x += atan2(ay, sqrt(ax * ax + az * az)) * 180 / M_PI;
        sum_y += atan2(ax, sqrt(ay * ay + az * az)) * 180 / M_PI;

        usleep(10000);
    }
    *angle_x  = sum_x / 5;
    *angle_y = sum_y / 5;
}  

//-----------------------------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------------------------

double getTime(){
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec + tv.tv_usec / 1000000.0;
}

//-----------------------------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------------------------

void Mpu6050_int_handler(void){
    currtime = getTime();
    dt = currtime - prevtime; // dt tính theo giây
    prevtime = currtime;
    if (dt < 0.001) dt = 0.001;
    else if (dt > 0.1) dt = 0.1;
    float threshold = 0.1; 

    printf("Yaw = %.2f ", *yaw_ptr);

    if ((gyroz > -5) && (gyroz < 5)){
        printf("Dang dung \n",*yaw_ptr);
        
        Max7219_display_number_pos(0, 0, 0, 0x5B, 5, *yaw_ptr, 2);
    }
    else{
        printf("Dang xoay\n",*yaw_ptr);
          
        Max7219_display_number_pos(0, 0, 0, 0x67, 5, *yaw_ptr, 2);
    }

    data_ready = 1; 
}

//-----------------------------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------------------------

void Mpu6050_setup_interrupt(int int_pin){
    wiringPiSetup();                
    pinMode(int_pin, INPUT);        
    pullUpDnControl(int_pin, PUD_UP);
    wiringPiISR(int_pin, INT_EDGE_RISING, &Mpu6050_int_handler); // Thiết lập ngắt
}

//-----------------------------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------------------------

void Mpu6050_initialize_value(int Full_Scale_Accel, int Full_Scale_Gyro){
    if (initialize_value) return; // Đảm bảo chỉ chạy 1 lần

    float scaleA = getAccelScale(Full_Scale_Accel);
    float scaleG = getGyroScale(Full_Scale_Gyro);

    AccX = (float)Mpu6050_read(MPU6050_REG_ACCEL_XOUT) / scaleA;
    AccY = (float)Mpu6050_read(MPU6050_REG_ACCEL_YOUT) / scaleA;
    AccZ = (float)Mpu6050_read(MPU6050_REG_ACCEL_ZOUT) / scaleA;

    gyrox_prev = (float)Mpu6050_read(MPU6050_REG_GYRO_XOUT) / scaleG - gyroX_offset;  
    gyroy_prev = (float)Mpu6050_read(MPU6050_REG_GYRO_YOUT) / scaleG - gyroY_offset;  
    gyroz_prev = (float)Mpu6050_read(MPU6050_REG_GYRO_ZOUT) / scaleG - gyroZ_offset;

    roll_prev  = atan2(AccY, sqrt(AccX * AccX + AccZ * AccZ)) * 180 / M_PI;
    pitch_prev = atan2(AccX, sqrt(AccY * AccY + AccZ * AccZ)) * 180 / M_PI;

    prevtime = getTime();
    initialize_value = 1; // Đánh dấu đã khởi tạo
}

//-----------------------------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------------------------

void Mpu6050_read_angles_moves(bool calibration_mode, int so_lan, float *roll, float *pitch, float *yaw, int Full_Scale_Accel, int Full_Scale_Gyro){
    float scaleA = getAccelScale(Full_Scale_Accel);
    float scaleG = getGyroScale(Full_Scale_Gyro);

    if (calibration_mode){
        for (int i = 0; i < so_lan; i++)
        {
            gyroX_offset += (float)Mpu6050_read(MPU6050_REG_GYRO_XOUT) / scaleG;
            gyroY_offset += (float)Mpu6050_read(MPU6050_REG_GYRO_YOUT) / scaleG;
            gyroZ_offset += (float)Mpu6050_read(MPU6050_REG_GYRO_ZOUT) / scaleG;
        }
        gyroX_offset = gyroX_offset / so_lan;
        gyroY_offset = gyroY_offset / so_lan;
        gyroZ_offset = gyroZ_offset / so_lan;

        return;
    }
    else{
        AccX = (float)Mpu6050_read(MPU6050_REG_ACCEL_XOUT) / scaleA;
        AccY = (float)Mpu6050_read(MPU6050_REG_ACCEL_YOUT) / scaleA;
        AccZ = (float)Mpu6050_read(MPU6050_REG_ACCEL_ZOUT) / scaleA;

        gyrox = (float)Mpu6050_read(MPU6050_REG_GYRO_XOUT) / scaleG - gyroX_offset;
        gyroy = (float)Mpu6050_read(MPU6050_REG_GYRO_YOUT) / scaleG - gyroY_offset;
        gyroz = (float)Mpu6050_read(MPU6050_REG_GYRO_ZOUT) / scaleG - gyroZ_offset;

        if (AccX == 0 && AccY == 0 && AccZ == 0) {
            printf("Lỗi đọc cảm biến acc MPU6050!\n");
            return;
        }
        if (gyrox == 0 && gyroy == 0 && gyroz == 0) {
            printf("Lỗi đọc cảm biến gyro MPU6050!\n");
            return;
        }
        if (fabs(gyroz) < 0.5) gyroz = 0;

        pitch_gyro = pitch_prev + (gyroy_prev + gyroy) * 0.5 * dt;
        roll_gyro  = roll_prev + (gyrox_prev + gyrox) * 0.5 * dt;

        roll_acc  = atan2(AccY, sqrt(AccX * AccX + AccZ * AccZ)) * 180 / M_PI;
        pitch_acc = atan2(AccX, sqrt(AccY * AccY + AccZ * AccZ)) * 180 / M_PI;

        // Kết hợp hai nguồn đo (Complementary Filter)
        *pitch = 0.96 * pitch_gyro + 0.04 * pitch_acc;
        *roll  = 0.96 * roll_gyro + 0.04 * roll_acc;
        *yaw   = yaw_prev + (gyroz_prev + (gyroz)) * 0.5 * dt;

        if (*pitch > 90) *pitch = 90;
        else if (*pitch < -90) *pitch = -90;
        if (*roll > 90) *roll = 90;
        else if (*roll < -90) *roll = -90;

        gyrox_prev = gyrox;
        gyroy_prev = gyroy;
        gyroz_prev = gyroz;
        pitch_prev = *pitch;
        roll_prev  = *roll;
        yaw_prev   = *yaw;
        //printf("done\n");
    }
}

//-----------------------------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------------------------
