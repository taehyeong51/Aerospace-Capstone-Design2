/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"
//#include "platform/mbed_thread.h"
#include "nRF24L01.h"
#include "RF24.h"
#include "RF24_config.h"
// I2C i2c(p9,p10); // i2c 통신 위한 pin 지정
I2C i2c(p28,p27); // i2c 통신 위한 pin 지정

PwmOut servo1(p21);
PwmOut servo2(p22);
PwmOut servo3(p23);
PwmOut servo4(p24);


RF24 NRF24L01(p5, p6, p7, p15, p8);     //PinName mosi, PinName miso, PinName sck, PinName _cepin, PinName _csnpin
Serial pc(USBTX, USBRX);
Ticker loops;
Ticker RF_loop;

const uint64_t pipe = 0x1212121212LL;
int8_t recv[30];
int16_t PITCH = 0, ROLL = 0, YAW = 0, THROTTLE = 0;
float throttle_offset = 0;
int8_t ackData[30];
int8_t flip1 = 1;
int BUT1, BUT2;
int rf_fail_count = 0;
uint32_t tick = 0;

#define MPU9250_ADDR 0x68 << 1 // 0b1101000, https://gksid102.tistory.com/90
#define MPU9250_WHOAMI 0x75 // 이렇게 정리해두면 나중에 data sheet 없어도 쉽게 확인 가능
#define MPU9250_ACCEL_X_OUT_H 0x3B
#define MPU9250_ACCEL_X_OUT_L 0x3C
#define MPU9250_ACCEL_Y_OUT_H 0x3D
#define MPU9250_ACCEL_Y_OUT_L 0x3E
#define MPU9250_ACCEL_Z_OUT_H 0x3F
#define MPU9250_ACCEL_Z_OUT_L 0x40
#define MPU9250_GYRO_X_OUT_H  0x43
#define MPU9250_GYRO_X_OUT_L  0x44
#define MPU9250_GYRO_Y_OUT_H  0x45
#define MPU9250_GYRO_Y_OUT_L  0x46
#define MPU9250_GYRO_Z_OUT_H  0x47
#define MPU9250_GYRO_Z_OUT_L  0x48

#define MPU9250_GYRO_CONFIG  0x1B
#define MPU9250_ACCEL_CONFIG 0x1C

#define GYRO_SCALE_FACTOR   16.4 // 16.4 for FS_SEL = 3
#define ACCEL_SCALE_FACTOR  2048 // 2048 for AFS_SEL = 3

#define PWR_MGMT_1       0x6B
#define PWR_MGMT_2       0x6C
#define INT_PIN_CFG      0x37
#define SMPLRT_DIV       0x19

#define CONFIG           0x1A
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define ACCEL_CONFIG2    0x1D

#define DLPF_CFG 0x03
#define A_scale 0x03
#define G_scale 0x03
float dt = 0.01;
float gyro_angle[3];
float roll_c, roll_accel, roll_g_n, roll_g_old, roll_a_n, roll_a_old;
float pitch_c, pitch_accel, pitch_g_n, pitch_g_old, pitch_a_n, pitch_a_old;
float tau = 3.0;
char cmd[2];  
char receive[2]; 
float x_acc,y_acc,z_acc,x_gyro,y_gyro,z_gyro;

int16_t x_acc_offset,y_acc_offset,z_acc_offset;
int16_t x_gyro_offset,y_gyro_offset,z_gyro_offset;
float acc_x_offset,acc_y_offset,acc_z_offset;
float gyro_x_offset,gyro_y_offset,gyro_z_offset;

// MPU9250 초기화에 사용되는 함수들 -------------------------------------------------------------------

void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
    char data_write[2];
    data_write[0] = subAddress;
    data_write[1] = data;
    i2c.write(address, data_write, 2, 0);
}

// MPU_readbyte : i2c 통신통해 slave의 지정된 주소에 해당하는 data 읽어와서 저장
uint8_t MPU_readbyte(uint8_t Address, uint8_t reg_addr)
{
    char reg_buff[2];
    char recv_buff[2];
    uint8_t ret;
    
    reg_buff[0] = reg_addr;     // 레지스터
    i2c.write(Address, reg_buff, 1, 0);     
    i2c.read(Address, recv_buff, 1, 1);

    /*
    ret = recv_buff[0];
    return ret;
    */
    return recv_buff[0];
}
void resetMPU9250()
{
    writeByte(MPU9250_ADDR, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
    wait(0.1);
}

void initMPU9250(uint8_t Ascale, uint8_t Gscale)
{
    resetMPU9250();
    wait(0.2);
 
    // char receive[2];
    uint8_t rt = 0;

    uint8_t gyro_config_data = 0b00011000;
    uint8_t acc_config_data = 0b00011000;
    uint8_t int_pin_cfg_data = 0b00100010;
    uint8_t acc_config_data2 = 0b00000101;
    writeByte(MPU9250_ADDR, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors
    wait(0.1); // Delay 100 ms for PLL to get established on x-axis gyro; should check for PLL ready interrupt
 
    writeByte(MPU9250_ADDR, PWR_MGMT_1, 0x01);  // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001

    uint8_t c = MPU_readbyte(MPU9250_ADDR, CONFIG);
    writeByte(MPU9250_ADDR, CONFIG, c | DLPF_CFG); // CONFIG 레지스터의 Bandwidth of Gyroscopes를 41Hz, 1KHz로 내부 샘플링하도록 설정
 
 
    cmd[0] = MPU9250_GYRO_CONFIG;       // 자이로 설정 어드레스
    cmd[1] = gyro_config_data;          // 자이로 설정 값
    i2c.write(MPU9250_ADDR, cmd, 2, 0); //GYRO_CONFIG 레지스터의 FS_SEL = 3으로 설정
    

    cmd[0] = MPU9250_ACCEL_CONFIG; // 가속도 설정 어드레스
    cmd[1] = acc_config_data;      // 가속도 설정 값
    i2c.write(MPU9250_ADDR, cmd, 2, 0); // ACCEL_CONFIG 레지스터의 AFS_SEL = 3으로 설정

    cmd[0] = ACCEL_CONFIG2; // 가속도 설정 어드레스
    cmd[1] = acc_config_data2;      // 가속도 설정 값
    i2c.write(MPU9250_ADDR, cmd, 2, 0); // ACCEL_CONFIG2 레지스터의 값을 0x05 로 설정

    cmd[0] = INT_PIN_CFG; // 가속도 설정 어드레스
    cmd[1] = 0x22;      // 가속도 설정 값
    i2c.write(MPU9250_ADDR, cmd, 2, 0); // INT_PIN_CFG 레지스터 값을 0x22로 설정

    cmd[0] = SMPLRT_DIV;
    cmd[1] = 0x04;// CONFIG 에서 1kHz internal sampling setting -> Interanl_Sample_Rate/(1 + SMPLRT_DIV) = SAMPLE_RATE   ====> SMPLRT_DIV = 4 : SAMPLE_RATE = 200Hz
    i2c.write(MPU9250_ADDR, cmd, 2, 0);

    cmd[0] = PWR_MGMT_2;
    cmd[1] = 0x00;
    i2c.write(MPU9250_ADDR, cmd, 2, 0);

    uint8_t acc_config = MPU_readbyte(MPU9250_ADDR, MPU9250_ACCEL_CONFIG);
    uint8_t acc2_config = MPU_readbyte(MPU9250_ADDR, ACCEL_CONFIG2);
    uint8_t gyro_config = MPU_readbyte(MPU9250_ADDR, MPU9250_GYRO_CONFIG);
    uint8_t int_pin_cfg = MPU_readbyte(MPU9250_ADDR, INT_PIN_CFG);
    uint8_t pwr_mgmt_2 = MPU_readbyte(MPU9250_ADDR, PWR_MGMT_2);
    pc.printf("ACC_CONFIG : %d \r\n",acc_config);
    pc.printf("ACC2_CONFIG : %d \r\n",acc2_config);
    pc.printf("GYRO_CONFIG : %d \r\n",gyro_config);
    pc.printf("INT_PIN_CONFIG : %d \r\n",int_pin_cfg);
    pc.printf("PWR_MGMT_2 : %d %x \r\n",pwr_mgmt_2, pwr_mgmt_2);

    
}
// ----------------------------------------------------------------------------------------------

// MPU9250 데이터 읽어오는데 사용되는 함수들-------------------------------------------------------------------
// Get_Value : MPU_readbyte 함수 이용해 원하는 가속도 또는 각속도 값 return
float Get_Value(uint8_t Address, uint8_t reg_addr_H, uint8_t reg_addr_L, float SCALE_FACTOR)
{
    int16_t result;
    float result_;
    result = MPU_readbyte(Address,reg_addr_H);
    result = result << 8;

    result += MPU_readbyte(Address,reg_addr_L);
    result_ = (float)result/SCALE_FACTOR;
    return result_;
}
// ----------------------------------------------------------------------------------------------

int constrain_int16(int16_t x, int min, int max)
{
    if (x > max)
        x = max;
    else if (x < min)
        x = min;
    return x;
}

void RF_READ()
{
    if (NRF24L01.available())
    {
        NRF24L01.read(recv, 10);

        // 스케일링 다른데서 곱하기
        ROLL = *(int16_t*)(&recv[0])- 3; //ROLL = - ROLL;           offset = 0
        PITCH = *(int16_t*)(&recv[2])- 7;//flip pitch and roll      offset = 6
        YAW = *(int16_t*)(&recv[4]) - 0; //                         offset = 2
        THROTTLE = *(int16_t*)(&recv[6]) - 0; //                    offset = 0
        BUT1 = recv[8];
        BUT2 = recv[9]; //should hold value here
        // pc.printf("\r RF_READ : ROLL : %d, PITCH : %d, YAW : %d, THROTTLE : %d \n\r", (int)ROLL, (int)PITCH, (int)YAW, (int)THROTTLE);

        // throttle offset 조절
        throttle_offset = (float)THROTTLE / (1159*10000) ;
        // throttle_offset = (float)THROTTLE / 1159 ;
        // pc.printf("\r THROTTLE_OFFSET : %f", (int)THROTTLE / 1159);
        // pc.printf("\r THROTTLE_OFFSET : %f", throttle_offset);
        rf_fail_count = 0;
    }

    else
    {
        rf_fail_count = rf_fail_count + 1;
                
        //printf(" rf_fail_count : %d\n\r", rf_fail_count);
        
        if (rf_fail_count >= 20 && rf_fail_count < 100)
        {
            printf(" rf_fail_count : %d\n\r", rf_fail_count);
            THROTTLE = THROTTLE - 2;
            THROTTLE = constrain_int16(THROTTLE, 0, 1023);
        }
        if (rf_fail_count >= 50)
        {
            THROTTLE = 0;
        }
        if (rf_fail_count >= 100)
        {
            rf_fail_count = 100;
        }
    }
}

void control_loop(void)
{
    RF_READ();
}

void timer_1ms(void)
{
    tick++;
}

int main()
{
    pc.baud(115200);
    // MPU9250 초기화 ------------------------------------------------------------------------
    i2c.frequency(400000);
    pc.baud(115200);
    // loops.attach_us(&timer_1ms, 1000);

    uint8_t gyro_config_data = 0b00000011;
    uint8_t acc_config_data = 0b00000011;
    uint8_t rt = 0;

    // WHOAMI 
    cmd[0] = MPU9250_WHOAMI;
    i2c.write(MPU9250_ADDR, cmd, 1, 0);     // cmd[0] 저장된 데이터를 보낸다.
    rt = i2c.read(MPU9250_ADDR, receive, 1, 1);

    pc.printf("rt: %d, WHOAMI: %x \r\n",rt, receive[0]);

    //initMPU9250 함수 통해 초기화
    initMPU9250(A_scale,G_scale);
    
    //Offset 구하기
    for(int i = 0; i<100; i++)
    {
        //------------------------가속도 offset을 위한 bias 구하기 ------------------------//

        x_acc_offset = MPU_readbyte(MPU9250_ADDR, MPU9250_ACCEL_X_OUT_H); // MPU9250_ACCEL_X_OUT_H 에 대한 MPU_readbyte 함수실행
        x_acc_offset = x_acc_offset << 8;                                        
        x_acc_offset += MPU_readbyte(MPU9250_ADDR, MPU9250_ACCEL_X_OUT_L); // 상위바이트 + 하위 바이트 
        acc_x_offset = (float)x_acc_offset/ACCEL_SCALE_FACTOR;                   // ACCEL scale factor 를 고려한 x축 ACC output 

        y_acc_offset = MPU_readbyte(MPU9250_ADDR, MPU9250_ACCEL_Y_OUT_H);
        y_acc_offset = y_acc_offset << 8;
        y_acc_offset += MPU_readbyte(MPU9250_ADDR, MPU9250_ACCEL_Y_OUT_L);
        acc_y_offset = (float)y_acc_offset/ACCEL_SCALE_FACTOR;

        z_acc_offset = MPU_readbyte(MPU9250_ADDR, MPU9250_ACCEL_Z_OUT_H);
        z_acc_offset = z_acc_offset << 8;
        z_acc_offset += MPU_readbyte(MPU9250_ADDR, MPU9250_ACCEL_Z_OUT_L);
        acc_z_offset = (float)z_acc_offset/ACCEL_SCALE_FACTOR;  

         //pc.printf("Accel_X_origin : %d, Accel_Y_origin : %d, Accel_Z_origin : %d \n\r", x_acc_offset, y_acc_offset, z_acc_offset);
         //pc.printf("Accel_X_scale : %f, Accel_Y_scale : %f, Accel_Z_scale : %f \n\r", acc_x_offset, acc_y_offset, acc_z_offset);

        //--------------자이로 offset을 위한 bias------------------//
        x_gyro_offset = MPU_readbyte(MPU9250_ADDR, MPU9250_GYRO_X_OUT_H);
        x_gyro_offset = x_gyro_offset << 8;
        x_gyro_offset += MPU_readbyte(MPU9250_ADDR, MPU9250_GYRO_X_OUT_L);
        gyro_x_offset = (float)x_gyro_offset/GYRO_SCALE_FACTOR;
                                             
        y_gyro_offset = MPU_readbyte(MPU9250_ADDR, MPU9250_GYRO_Y_OUT_H);
        y_gyro_offset = y_gyro_offset << 8;
        y_gyro_offset += MPU_readbyte(MPU9250_ADDR, MPU9250_GYRO_Y_OUT_L);
        gyro_y_offset = (float)y_gyro_offset/GYRO_SCALE_FACTOR;

        z_gyro_offset = MPU_readbyte(MPU9250_ADDR, MPU9250_GYRO_Z_OUT_H);
        z_gyro_offset = z_gyro_offset << 8;
        z_gyro_offset += MPU_readbyte(MPU9250_ADDR, MPU9250_GYRO_Z_OUT_L);
        gyro_z_offset = (float)z_gyro_offset/GYRO_SCALE_FACTOR;

        //pc.printf("Gyro_X_origin : %d, Gyro_Y_origin : %d, Gyro_Z_origin : %d \n\r", x_gyro_offset, y_gyro_offset, z_gyro_offset);
        //pc.printf("Gyro_X_scale : %f, Gyro_Y_scale : %f, Gyro_Z_scale : %f \r\n", gyro_x_offset, gyro_y_offset, gyro_z_offset);

        // ------ GYRO OFFSET SETTING ----- //
        gyro_x_offset = (float)gyro_x_offset + gyro_x_offset;
        gyro_y_offset = (float)gyro_y_offset + gyro_y_offset;
        gyro_z_offset = (float)gyro_z_offset + gyro_z_offset;

        // ------ ACCEL OFFSET SETTING ------ //
        acc_x_offset = (float)acc_x_offset + acc_x_offset;
        acc_y_offset = (float)acc_y_offset + acc_y_offset;
        acc_z_offset = (float)acc_z_offset + acc_z_offset;

    }
    /* for문을 통해서 100번 돌려서 100개의 데이터를 받는다. 
    예를 들어 gyro_x에 최신 값을 받고, 이전에 더해진 gyro_offset값과 더한 후 다시 gyro_offset에 저장한다.
    이를 100번 반복한다.*/
    // 각속도 offset 100개의 데이터를 받아서 평균내기
    gyro_x_offset = gyro_x_offset/100.0;
    gyro_y_offset = gyro_y_offset/100.0;
    gyro_z_offset = gyro_z_offset/100.0;

    // 가속도 offset 100개의 데이터를 받아서 평균내기
    acc_x_offset = acc_x_offset/100.0;
    acc_y_offset = acc_y_offset/100.0;
    acc_z_offset = acc_z_offset/100.0;
    pc.printf("1,2,3,4,5,6\r\n");
    // ------------------------------------------------------------------------
    // RF 모듈 초기화
    NRF24L01.begin();
    NRF24L01.setDataRate(RF24_2MBPS); //RF24_2MBPS
    NRF24L01.setChannel(10);  // set channel 10 20 30
    NRF24L01.setPayloadSize(28);
    NRF24L01.setAddressWidth(5);
    NRF24L01.setRetries(2, 4); //1,3 2,8 1,8
    NRF24L01.enableAckPayload();
    NRF24L01.openReadingPipe(0, pipe);
    NRF24L01.startListening();
    pc.printf("Code Start\r\n");
    //loops.attach_us(&control_loop, 2500); // ticker 2500
    loops.attach_us(&timer_1ms, 1000);       // 1ms period
    RF_loop.attach_us(&RF_READ, 2500);

    while (true) {
        // 0.01 - 10ms
        // 0.001 - 1ms
        servo1.period(0.00001);          // servo requires a 10KHz(0.0001sec = 10^-5sec)
        servo2.period(0.00001);          
        servo3.period(0.00001);          
        servo4.period(0.00001);          
        if(tick >= 10)      // 10ms loop  dt = 0.01
        {   
            // 아무런 우리 코드
            //control_loop();
            x_acc = Get_Value(MPU9250_ADDR, MPU9250_ACCEL_X_OUT_H,MPU9250_ACCEL_X_OUT_L,ACCEL_SCALE_FACTOR)-acc_x_offset;
            y_acc = Get_Value(MPU9250_ADDR, MPU9250_ACCEL_Y_OUT_H,MPU9250_ACCEL_Y_OUT_L,ACCEL_SCALE_FACTOR)-acc_y_offset;
            z_acc = Get_Value(MPU9250_ADDR, MPU9250_ACCEL_Z_OUT_H,MPU9250_ACCEL_Z_OUT_L,ACCEL_SCALE_FACTOR)-acc_z_offset;

            x_gyro = Get_Value(MPU9250_ADDR, MPU9250_GYRO_X_OUT_H,MPU9250_GYRO_X_OUT_L,GYRO_SCALE_FACTOR)-gyro_x_offset;
            y_gyro = Get_Value(MPU9250_ADDR, MPU9250_GYRO_Y_OUT_H,MPU9250_GYRO_Y_OUT_L,GYRO_SCALE_FACTOR)-gyro_y_offset;
            z_gyro = Get_Value(MPU9250_ADDR, MPU9250_GYRO_Z_OUT_H,MPU9250_GYRO_Z_OUT_L,GYRO_SCALE_FACTOR)-gyro_z_offset;
            // ----------------------------------------------------------------------------------------------
            gyro_angle[0] = gyro_angle[0] + (x_gyro)*dt;
            gyro_angle[1] = gyro_angle[1] + (y_gyro)*dt;

            roll_accel = atan(y_acc/z_acc)*180.0/3.14; 
            pitch_accel = asin(x_acc)*180.0/3.14;
            // pitch_accel = -atan(x_acc/sqrt(pow(y_acc,2)+pow(z_acc,2)))*180.0/3.14;
            // pitch_accel = atan(x_acc/z_acc)*180.0/3.14;

            roll_g_n = (1-dt/tau)*roll_g_old + dt*x_gyro;
            roll_a_n = (1-dt/tau)*roll_a_old + dt/tau*roll_accel;

            pitch_g_n = (1-dt/tau)*pitch_g_old + dt*y_gyro;
            pitch_a_n = (1-dt/tau)*pitch_a_old + dt/tau*pitch_accel;

            roll_g_old = roll_g_n;
            roll_a_old = roll_a_n;
            pitch_g_old = pitch_g_n;
            pitch_a_old = pitch_a_n;
            roll_c = roll_g_old + roll_a_old;
            pitch_c = pitch_g_old + pitch_a_old;
            // pc.printf("\r\nROLL_C : %f,PITCH_C : %f \r\nGYRO_ROLL : %f,GYRO_PITCH : %f\r\nROLL_ACCEL : %f, PITCH_ACCEL : %f\r\n",roll_c,pitch_c,gyro_angle[0],gyro_angle[1],roll_accel,pitch_accel);
            // 우리 코드 끝
            tick = 0;

            pc.printf("\r THROTTLE_OFFSET : %f", throttle_offset);
            servo1.pulsewidth(throttle_offset);
            servo2.pulsewidth(throttle_offset);
            servo3.pulsewidth(throttle_offset);
            servo4.pulsewidth(throttle_offset);
            
            // wait(0.25);

            
            // for(float offset=0.0; offset<0.001; offset+=0.0001) {
            // servo1.pulsewidth(0.001 + offset); // servo position determined by a pulsewidth between 1-2ms
            // servo2.pulsewidth(0.001 + offset); // servo position determined by a pulsewidth between 1-2ms
            // servo3.pulsewidth(0.001 + offset); // servo position determined by a pulsewidth between 1-2ms
            // servo4.pulsewidth(0.001 + offset); // servo position determined by a pulsewidth between 1-2ms
            // wait(0.25);
        
        }


    }
}