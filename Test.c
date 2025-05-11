#include "Max7219.h"
#include "Mpu6050.h"
#include "Ds3231.h"
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <wiringPi.h>

#define MPU6050_INT_PIN 7
float *yaw_ptr;
int main(){
    //--------------------khai báo--------------------
    wiringPiSetupPhys();
    Max7219_init(0, 7, 8);
    Mpu6050_init(0x00, 0x13, 0x10, 0x08, 0x01, 0x01);
    //--------------------biến--------------------
    float roll = 0.0;
    float pitch = 0.0;
    float yaw = 0.0;
    int acc_unit = 4;
    int gyro_unit = 1000;
    //---------------------------
    yaw_ptr = &yaw;
    //--------------------khởi tạo ban đầu--------------------
    //Max7219_display_flashing(3, 1);
    // Max7219_display_atoh();

    Mpu6050_read_angles_moves(TRUE, 5, &roll, &pitch, &yaw, acc_unit, gyro_unit);
    Mpu6050_initialize_value(acc_unit, gyro_unit);
    Mpu6050_setup_interrupt(MPU6050_INT_PIN);

    while (1) {
        if(data_ready){
            data_ready = 0;
            Mpu6050_read_angles_moves(FALSE, 0, &roll, &pitch, &yaw, acc_unit, gyro_unit);
            // if((yaw <=90) && (yaw >= -90)){
            //     Max7219_set_intensity(7);
            // }
            // else if(((yaw >90) && (yaw <= 120)) || ((yaw < -90) && (yaw >=-120))){
            //     Max7219_set_intensity(9);
        
            // }
            // else if(((yaw >120) && (yaw <= 150)) || ((yaw < -120) && (yaw >=-150))){
            //     Max7219_set_intensity(11);
        
            // }
            // else if(((yaw >150) && (yaw <= 180)) || ((yaw < -150) && (yaw >=-180))){
            //     Max7219_set_intensity(13);
        
            // }
            // else{
            //     Max7219_set_intensity(15);
            // }    
        }
    }

    return 0;

}

