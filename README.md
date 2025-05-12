# MODULE MPU6050

- [Giới thiệu](#giới-thiệu)
- [Thông số kỹ thuật](#thông-số-kỹ-thuật)
- [Sơ đồ kết nối](#Sơ-đồ-kết-nối)
- [Hướng dẫn sử dụng](#hướng-dẫn-sử-dụng)
  - [Bật chế độ I2C](#bật-chế-độ-I2C)
  - [Kiểm tra địa chỉ I2C](#kiểm-tra-địa-chỉ-I2C)
  - [Cài đặt và sử dụng](#cài-đặt-và-sử-dụng)
- [Tính năng của thư viện](#tính-năng-của-thư-viện)
- [Tài liệu tham khảo](#tài-liệu-tham-khảo)
- [Tổng hợp giá trị của từng thanh ghi trong cấu hình Mpu6050](#tổng-hợp-giá-trị-của-từng-thanh-ghi-trong-cấu-hình-Mpu6050)

# Giới thiệu
MPU6050 là một mô-đun cảm biến tích hợp được sản xuất bởi InvenSense. Nó kết hợp gia tốc kế 3 trục và con quay hồi chuyển 3 trục trong cùng một chip – tạo thành một hệ thống đo chuyển động 6 bậc tự do (6-DOF) chỉ trong một module nhỏ gọn. Không chỉ đơn thuần đo các giá trị gia tốc và quay, MPU6050 còn tích hợp một cảm biến nhiệt độ giúp nhận được dữ liệu nhiệt độ theo nhiệt độ môi trường và một bộ xử lý chuyển động số (DMP) giúp xử lý dữ liệu thô thành các giá trị góc nghiêng và chuyển động chính xác hơn. Module này thì được ứng dụng nhiều trong thực tế với một số ứng dụng tiêu biểu như sau:
- Robot tự cân bằng (self-balancing robot) và robot hình người
- Drone/quadcopter ổn định bay
- Thiết bị thể dục và chăm sóc sức khỏe
- Điều khiển cử chỉ và thiết bị chơi game
- Hệ thống định vị và theo dõi chuyển động
- Hệ thống báo động và an ninh

# Thông số kỹ thuật
- Giao tiếp:	I2C (tối đa 400 kHz)
- Địa chỉ I2C: mặc định	0x68 (hoặc 0x69 nếu chân AD0 = HIGH)
- Điện áp hoạt động:	3.3V – 5V (trên module đã tích hợp mạch chuyển mức logic và ổn áp)
- Dòng tiêu thụ:	~3.9 mA ở chế độ hoạt động, ~5 µA ở chế độ ngủ

# Sơ đồ kết nối
Phần cứng của MPU6050:  
- Pin 1: VCC (Nguồn cấp cho module (3.3V hoặc 5V tùy loại module))
- Pin 2: GND (Nối đất, tất cả điện áp đều được tham chiếu đến GND)  
- Pin 3: SCL (Serial Clock, Chân nhận xung (Clock) của giao tiếp I2C)  
- Pin 4: SDA (Serial Data, Dữ liệu truyền I2C)  
- Pin 5: XDA (Auxiliary SDA, Dành cho cảm biến phụ I2C (thường không dùng))  
- Pin 6: XCL (Auxiliary SCL, Xung clock I2C phụ (thường không dùng))
- Pin 7: AD0 (Address Select, Thay đổi địa chỉ I2C: GND → 0x68, VCC → 0x69)
- Pin 8: INT (Interrupt,	Xuất tín hiệu ngắt khi có dữ liệu sẵn)
  
Sơ đồ kết nối MAX7219 với rapsberry:    
- VCC -> 1(3.3V)  
- GND -> 6  
- SCL -> 5
- SDA -> 3 
- INT -> 7
  
# Hướng dẫn sử dụng
### Bật chế độ I2C 
```c
sudo raspi-config
```
Sau đó chọn: Interface option -> I2C -> Enable

### Kiểm tra địa chỉ I2C 
```c
sudo i2cdetect -y 1
```

### Cài đặt và sử dụng  
Bước 1:  
Tạo ra 2 file Mpu6050.c và file Mpu6050.h hoặc tải hoặc tham khảo 2 file code trên github. 

Bước 2:  
Chuyển file .c thành file .o:
```c
gcc -c -fPIC Mpu6050.c -o Mpu6050.o
```
Bước 3:  
Chuyển file .o thành file .so:
```c
gcc -shared -o libMpu6050.so Mpu6050.o
```
Bước 4:  
Thêm thư viện .so vào thư viện động trên raspberry:
```c
sudo cp libMpu6050.so /usr/lib/
```
Bước 5:  
Để dùng các chức năng người dùng cần thêm vào file code thư viện sau:    
```c
#include "Mpu6050.h"
```
Bước 6:  
Biên dịch và chạy file (file_code.c là file bạn đang viết):
```c
gcc file_code.c -o run -L. -lMpu6050 -lwiringPi -lm 
sudo ./run
```

# Tính năng của thư viện
Ở đây mình sẽ liệt kê chức năng tổng quan của một số hàm trong thư viện, bạn có thể xem chi tiết chức năng và cách khai báo giá trị của từng biến của hàm trong file Mpu6050.h).
- Hàm đọc dữ liệu từ thanh ghi trên Mpu6050
```c
int16_t Mpu6050_read(uint8_t reg)
```  

- Hàm cấu hình Mpu6050
 ```c
void Mpu6050_init(uint8_t conf, uint8_t smplrt_div, uint8_t accel_conf, uint8_t gryo_conf, uint8_t interupt, uint8_t PWR);
```  
-> [Tổng hợp giá trị của từng thanh ghi trong cấu hình Mpu6050](#tổng-hợp-giá-trị-của-từng-thanh-ghi-trong-cấu-hình-Mpu6050)

- Hàm đọc và trả về giá trị nhiệt độ
```c
float Mpu6050_read_temperature()
```  

- Hàm trả giá trị FUll_Scale_Accel
```c
float getAccelScale(int Full_Scale_Accel)
```  

- Hàm trả giá trị FUll_Scale_Gyro
```c
float getGyroScale(int Full_Scale_Gyro)
```  

- Hàm đọc góc nghiêng
```c
void Mpu6050_read_angles(float *angle_x, float *angle_y, int Full_Scale_Accel)
```

- Hàm đọc góc nghiêng khi có di chuyển
```c
void Mpu6050_read_angles_moves(bool calibration_mode, int so_lan, float *roll, float *pitch, float *yaw, int Full_Scale_Accel, int Full_Scale_Gyro){
```

## 📌📌Lưu ý: Một số hàm có thể có chức năng chưa hoàn toàn đúng, người dùng có thể viết lại hoặc tối ưu code để phù hợp với mục đích sử dụng 📌📌

# Tài liệu tham khảo
- [Datasheet MPU6050](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf)
- [Datasheet Register Map](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf)
  
# Tổng hợp giá trị của từng thanh ghi trong cấu hình Mpu6050
```c
MPU6050
////////////////////////// 0x19: sample rate //////////////////////////
Khi DLPF (Digital Low Pass Filter) được bật, Gyro Output Rate = 1 kHz.
Khi DLPF tắt, Gyro Output Rate = 8 kHz (không thường dùng).

Sample Rate(expected) = Gyroscope Output Rate / (1 + SMPLRT_DIV)

////////////////////////// 0x1A: configuration //////////////////////////
Tốc độ cao, ít trễ ➝ DLPF = 0 hoặc 1 (250 Hz, 184 Hz).
Cân bằng giữa độ nhiễu và phản hồi ➝ DLPF = 2 hoặc 3 (92 Hz, 41 Hz).
Độ nhiễu thấp, phản hồi chậm ➝ DLPF = 4 - 5 - 6 (20 Hz - 10Hz - 5 Hz).

DLPF = 0 or = 7, Gyro Output Rate = 8 kHz
DLPF = 1 - 6, Gyro Output Rate = 1 kHz	

////////////////////////// 0x1B: Gyroscope //////////////////////////
GYRO_CONFIG = (XG_ST × 2^7) + (YG_ST × 2^6) + (ZG_ST × 2^5) + (FS_SEL × 2^3)

Mặc định G_ST = 0, nếu bật trên trục nào thì G_ST trên trục đó = 1
FS_SEL   Full Scale Range      X,Y,ZG_ST = 0
  0        ± 250 °/s              0x00(0)
  1        ± 500 °/s              0x08(8)
  2        ± 1000 °/s             0x10(16)
  3        ± 2000 °/s             0x18(24)
  
////////////////////////// 0x1C: Accelerometer //////////////////////////
ACCEL_CONFIG = (XA_ST × 2^7) + (YA_ST × 2^6) + (ZA_ST × 2^5) + (AFS_SEL × 2^3)

Mặc định A_ST = 0, nếu bật trên trục nào thì A_ST trên trục đó = 1
AFS_SEL   Full Scale Range      X,Y,ZA_ST = 0
  0          ± 2g                  0x00(0)
  1          ± 4g                  0x08(8)
  2          ± 8g                  0x10(16)
  3          ± 16g                 0x18(24)
  
////////////////////////// 0x38: Interupt //////////////////////////
Muốn bật/tắt ngắt nào thì đặt bit tương ứng x=1 hoặc x=0.    x*2^bit
Bit	Tên	                  Chức năng
7	-	                Không sử dụng (luôn = 0)
6	-	                Không sử dụng (luôn = 0)
5	FIFO_OFLOW_EN	        Bật ngắt khi FIFO bị tràn (1 = Bật, 0 = Tắt)
4	I2C_MST_INT_EN	        Bật ngắt khi có lỗi từ I2C Master (1 = Bật, 0 = Tắt)
3	-	                Không sử dụng (luôn = 0)
2	DATA_RDY_EN	        Bật ngắt khi có dữ liệu mới (1 = Bật, 0 = Tắt)
1	-	                Không sử dụng (luôn = 0)
0	MOT_EN	                Bật ngắt khi phát hiện chuyển động (1 = Bật, 0 = Tắt) (***)


////////////////////////// 0x6B: PWR_manage //////////////////////////
PWR_MGMT_1 = (DEVICE_RESET × 2^7) + (SLEEP × 2^6) + (CYCLE × 2^5) + (TEMP_DIS × 2^3) + (CLKSEL × 2^0)

Trong đó:
    DEVICE_RESET (Bit 7) = 1 nếu reset, 0 nếu không.
    SLEEP (Bit 6) = 1 nếu vào chế độ ngủ, 0 nếu hoạt động bình thường.
    CYCLE (Bit 5) = 1 để bật chế độ đo chu kỳ (MPU tự động ngủ rồi đo).
    TEMP_DIS (Bit 3) = 1 để tắt cảm biến nhiệt độ, 0 để bật.
    CLKSEL (Bit 2-0) = Chọn xung nhịp (từ 0 đến 5, xem bảng). 
    
    CLKSEL   Giá trị (Dec)	      Chọn nguồn xung nhịp
    000	         0                 Xung nhịp bên trong 8MHz
    001	         1	           Gyroscope X  (***)
    010	         2	           Gyroscope Y
    011	         3	           Gyroscope Z
    100	         4	           Xung nhịp ngoài 32.768 kHz
    101	         5	           Xung nhịp ngoài 19.2 MHz
    
``` 
