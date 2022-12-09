/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"
//#include "platform/mbed_thread.h"

#include "nRF24L01.h"
#include "RF24.h"
#include "RF24_config.h"
#include <cmath>
#include "rtos.h"

RF24 NRF24L01(p5, p6, p7, p15, p8);

I2C i2c(p28,p27); // i2c 통신 위한 pin 지정

PwmOut servo1(p21);
PwmOut servo2(p22);
PwmOut servo3(p23);
PwmOut servo4(p24);

int cnt = 0;

Ticker loops;
Ticker RF_loop;
Ticker pid_loop;
Ticker PWM_loop;
Ticker angle_bias_loop;

Serial pc(USBTX, USBRX);

int MPU9250_ADDRESS = (0x68<<1), //0b11010000
WHO_AM_I_MPU9250 = 0x75,
FIFO_EN = 0x23,
SMPLRT_DIV = 0x19,
CONFIG = 0x1A,
GYRO_CONFIG = 0x1B,
ACCEL_CONFIG = 0x1C,
ACCEL_CONFIG2 = 0x1D,
ACCEL_XOUT_H = 0x3B,
TEMP_OUT_H = 0x41,
GYRO_XOUT_H = 0x43,
PWR_MGMT_1 = 0x6B,
PWR_MGMT_2 = 0x6C,
INT_PIN_CFG = 0x37,
AK8963_ADDRESS = (0x0C<<1), //0b00001100 0x0C<<1
WHO_AM_I_AK8963 = 0x00, // should return 0x48
AK8963_ST1 = 0x02,
AK8963_XOUT_L = 0x03,
AK8963_CNTL = 0x0A,
AK8963_ASAX = 0x10;
float gyro_bias[3]={0.0,0.0,0.0};

int16_t gyro[3];
int16_t accel[3];
float dt=0.01;
float accel_f[3],gyro_f[3];
float gyro_angle[3]={0.0,0.0,0.0};
float roll_c=0,roll_accel=0, roll_g_n=0, roll_g_old=0, roll_a_n=0, roll_a_old=0;
float pitch_c=0,pitch_accel=0, pitch_g_n=0, pitch_g_old=0, pitch_a_n=0,pitch_a_old=0;
float tau=1.0/3.0;

// Variables for RF & PWM
const uint64_t pipe = 0x1212121212LL;
int8_t recv[30];
int16_t PITCH = 0, ROLL = 0, YAW = 0, THROTTLE = 0;
int16_t ROLL_in = 0, PITCH_in = 0, YAW_in = 0;
float throttle_offset = 0,ROLL_offset = 0,PITCH_offset=0,YAW_offset=0;
float PWM_Scale = 0;
int8_t ackData[30];
int8_t flip1 = 1;
int BUT1, BUT2;
int rf_fail_count = 0;
uint32_t tick = 0;

float angle_x = 0, angle_y = 0;

float target_roll=0,target_pitch=0,target_yaw;
float Roll_pid_in=0,Pitch_pid_in=0;
float pwm1=0,pwm2=0,pwm3=0,pwm4=0;

float tmp_bias_roll=0,tmp_bias_pitch=0;
float bias_roll=0,bias_pitch=0;
bool bias_check = false;

float tmp_kp=0,tmp_kd=0;


float Kp = 0,          // (P)roportional Tuning Parameter
      Ki = 0,         // (I)ntegral Tuning Parameter        
      Kd = 0,          // (D)erivative Tuning Parameter       
      pterm_roll,iterm_roll=0,dterm_roll,pterm_pitch,iterm_pitch=0,dterm_pitch,       // Used to accumulate error (integral)
      maxPID = 255,    // The maximum value that can be output
      oldValue_roll = 0,oldValue_pitch=0,    // The last sensor value
      error_roll = 0,error_pitch=0,
      result_roll = 0,result_pitch=0,
      dInput_roll = 0,dInput_pitch=0,
      Scale = 1;
     

// --------------------------------------------------------

void WHO_AM_I()
{
char cmd[2];
cmd[0] = WHO_AM_I_MPU9250;
i2c.write(MPU9250_ADDRESS, cmd, 1, 1);
i2c.read(MPU9250_ADDRESS|1, cmd, 1, 0);
uint8_t DEVICE_ID = cmd[0];
pc.printf("IMU device id is %d \n\r", DEVICE_ID);
}

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

void MPU9250_RESET() {
    // reset device
    char cmd[2];
    cmd[0] = PWR_MGMT_1; //status
    cmd[1] = 0x80;
    i2c.write(MPU9250_ADDRESS, cmd, 2, 0);
    // wait(0.1);
}

void MPU9250_INIT()
{
    char cmd[3];
    cmd[0] = PWR_MGMT_1; //reset
    cmd[1] = 0x80;
    i2c.write(MPU9250_ADDRESS, cmd, 2);
    pc.printf("MPU 1 \n\r");
    cmd[0] = PWR_MGMT_1; // Auto selects the best available clock source PLL if ready, else use the Internal

    cmd[1] = 0x01;
    i2c.write(MPU9250_ADDRESS, cmd, 2, 0);
    pc.printf("MPU 2 \n\r");

    cmd[0] = CONFIG;
    cmd[1] = 0x03;// 41Hz gyro bandwidth, 1kHz internal sampling
    i2c.write(MPU9250_ADDRESS, cmd, 2, 0);
    pc.printf("MPU 3 \n\r");

    // sample rate divisor for all sensors, 1000/(1+4)=200 Hz for Gyro
    cmd[0] = SMPLRT_DIV;
    cmd[1] = 0x04;//0x04
    i2c.write(MPU9250_ADDRESS, cmd, 2, 0);
    pc.printf("MPU 4 \n\r");


    cmd[0] = GYRO_CONFIG;
    cmd[1] = 0b00010000;// Gyro full scale 1000 deg/sec; Gyro DLPF Enable
    i2c.write(MPU9250_ADDRESS, cmd, 2, 0);
    pc.printf("MPU 6 \n\r");

    // Set accelerometer configuration
    // Accel fulll sacle range +/- 4g
    cmd[0] = ACCEL_CONFIG;
    cmd[1] = 0b00001000;// Accel
    i2c.write(MPU9250_ADDRESS, cmd, 2, 0);
    pc.printf("MPU 8 \n\r");

    // Set accelerometer sample rate configuration (Fast sampling)
    // It is possible to get a 1 kHz sample rate from the accelerometer by choosing 0 for
    // accel_fchoice_b bit [3], and 000 for bit[2-0] for A_DLPF_CFG; in this case the bandwidth is 218 Hz
    cmd[0] = ACCEL_CONFIG2;
    cmd[1] = 0b00000000; // 218 Hz bandwidth, 1kHz Sampling for accelerometer
    i2c.write(MPU9250_ADDRESS, cmd, 2, 0);
    pc.printf("MPU 10 \n\r");

    // XYZ Gyro accel enable (default)
    cmd[0] = PWR_MGMT_2;
    cmd[1] = 0x00;
    i2c.write(MPU9250_ADDRESS, cmd, 2, 0);
    pc.printf("MPU 11 \n\r");


    cmd[0] = INT_PIN_CFG;
    cmd[1] = 0x22; //0x02
    i2c.write(MPU9250_ADDRESS, cmd, 2, 0);
    pc.printf("MPU 12 \n\r");

}

void MPU9250_GET_GYRO(int16_t * destination)
{
char cmd[6];
cmd[0] = GYRO_XOUT_H;
i2c.write(MPU9250_ADDRESS, cmd, 1, 1);
i2c.read(MPU9250_ADDRESS|1, cmd, 6, 0);
destination[0] = (int16_t)(((int16_t)cmd[0] << 8) | cmd[1]);
destination[1] = (int16_t)(((int16_t)cmd[2] << 8) | cmd[3]);
destination[2] = (int16_t)(((int16_t)cmd[4] << 8) | cmd[5]);
//pc.printf("gyro_raw %d, %d, %d \n\r", destination[0], destination[1], destination[2]);
}

void MPU9250_GET_ACCEL(int16_t * destination)
{
char cmd[6];
cmd[0] = ACCEL_XOUT_H;
i2c.write(MPU9250_ADDRESS, cmd, 1, 1);
i2c.read(MPU9250_ADDRESS|1, cmd, 6, 0);
destination[0] = (int16_t)(((int16_t)cmd[0] << 8) | cmd[1]);
destination[1] = (int16_t)(((int16_t)cmd[2] << 8) | cmd[3]);
destination[2] = (int16_t)(((int16_t)cmd[4] << 8) | cmd[5]);
}

void gyro_bias_f(void){
    int16_t gyro1[3];
    pc.printf("Please keep still 5 seconds\n\r");
    for(int i=0;i<100;i++){
    MPU9250_GET_GYRO(gyro1);
    gyro_bias[0]=gyro_bias[0]+gyro1[0]/32.8;
    gyro_bias[1]=gyro_bias[1]+gyro1[1]/32.8;
    gyro_bias[2]=gyro_bias[2]+gyro1[2]/32.8;
    // pc.printf("bias finding i= %d\n\r",i);
    }
    gyro_bias[0]=gyro_bias[0]/100.0;
    gyro_bias[1]=gyro_bias[1]/100.0;
    gyro_bias[2]=gyro_bias[2]/100.0;
    pc.printf("bias finding completed %\n\r");
}




// RF 통신 코드 ----------------------------------------------------------
void initRF()
{
    NRF24L01.begin();
    NRF24L01.setDataRate(RF24_2MBPS); //RF24_2MBPS
    NRF24L01.setChannel(10);  // set channel 10 20 30
    NRF24L01.setPayloadSize(28);
    NRF24L01.setAddressWidth(5);
    NRF24L01.setRetries(2, 4); //1,3 2,8 1,8
    NRF24L01.enableAckPayload();
    NRF24L01.openReadingPipe(0, pipe);
    NRF24L01.startListening();
    pc.printf("RF module initialized");
}

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
        BUT1 = recv[8]; // Right Button
        BUT2 = recv[9]; // Left Button
        // 스케일링 다른데서 곱하기
        
        
        YAW = *(int16_t*)(&recv[4]) ; //                         offset = 2
        THROTTLE = *(int16_t*)(&recv[6]) ; //                    offset = 0
        ROLL = *(int16_t*)(&recv[0]); //ROLL = - ROLL;           offset = 0
        PITCH = *(int16_t*)(&recv[2]); //ROLL = - ROLL;           offset = 0

        // pc.printf("\r RF_READ : ROLL : %d, PITCH : %d, YAW : %d, THROTTLE : %d \n\r", (int)ROLL, (int)PITCH, (int)YAW, (int)THROTTLE);
        PWM_Scale = 0.5;
        int8_t D_Scale = 5; // Scale Factor for Dead Zone

        throttle_offset = (float)THROTTLE*100 / 1159 ;
        YAW_offset = PWM_Scale*(float)YAW*100 / 170;

        // Dead Zone 설정 - throttle & Yaw
        if(throttle_offset > -D_Scale && throttle_offset < D_Scale){throttle_offset = 0;} 
        if(YAW_offset > -D_Scale && YAW_offset < D_Scale){YAW_offset = 0;}

        if(throttle_offset < 10){
            if(BUT1 == 1 && BUT2 == 0){Kd += 0.01;}
            else if(BUT2 == 1 && BUT1 == 0){Kp += 0.01;}
            else if(BUT1 == 0 && BUT2 == 0){Kp = 0, Ki = 0, Kd = 0,Roll_pid_in=0,Pitch_pid_in=0;}
        }
        else{
            if(BUT2 == 1 && BUT1 == 0){Ki += 0.01;}
        }
        // if(BUT2==1 && BUT1 == 1){
            
        //     ROLL_offset = PWM_Scale*(float)ROLL*100 / 250;

            
        //     PITCH_offset = PWM_Scale*(float)PITCH*100 / 250;

        //     // Dead Zone 설정 - Roll & Pitch
        //     if(ROLL_offset > -D_Scale && ROLL_offset < D_Scale){ROLL_offset = 0;} 
        //     if(PITCH_offset > -D_Scale && PITCH_offset < D_Scale){PITCH_offset = 0;}
        // }
        // else if(BUT2==0 && BUT1 == 1){
        //     tmp_kp = PWM_Scale*(float)ROLL*100 / 250;
        //     ROLL_offset = 0;
        //     if(tmp_kp>40){Kp += 0.01;}
        //     else if(tmp_kp<-40){Kp -= 0.01;}
        // }
        // else if(BUT1==0 && BUT2 == 1){
        //     tmp_kd = PWM_Scale*(float)PITCH*100 / 250;
        //     PITCH_offset = 0;
        //     if(tmp_kd>40){Kd += 0.01;}
        //     else if(tmp_kd<-40){Kd -= 0.01;}
        // }

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
// ------------------------------------------------------------------------------


// PID 함수 --------------------------------------------------------------------
float pid_roll(float target, float current) {

    error_roll = target - current;

    //PID제어//
    pterm_roll = Kp * error_roll;
    iterm_roll += Ki * error_roll * dt;
    dterm_roll = Kd * (error_roll-oldValue_roll) / dt;

    // result_roll = pterm_roll + iterm_roll + dterm_roll;
    result_roll = pterm_roll + iterm_roll + dterm_roll;

    oldValue_roll = error_roll;
    // if (result > maxPID) result = maxPID;
	// else if (result < -maxPID) result = -maxPID;
    return result_roll;
}
float pid_pitch(float target, float current) {

    error_pitch = target - current;
    

    //PID제어//
    pterm_pitch = Kp * error_pitch;
    iterm_pitch += Ki * error_pitch * dt;
    dterm_pitch = Kd * (error_pitch - oldValue_pitch) / dt;

    // result_pitch = pterm_pitch + iterm_pitch + dterm_pitch;

    oldValue_pitch = error_pitch;
    result_pitch = pterm_pitch + iterm_pitch + dterm_pitch;

    // if (result > maxPID) result = maxPID;
	// else if (result < -maxPID) result = -maxPID;
    return result_pitch;
}

// ------------------------------------------------------------------------------


void activate(){
    // 1. 각도 획득하고 상보필터 거치는 부분 ---------------------------------------

    MPU9250_GET_GYRO(gyro); // Gyro 센서값 획득
    gyro_f[0]=gyro[0]/32.8- gyro_bias[0];
    gyro_f[1]=-(gyro[1]/32.8- gyro_bias[1]);
    gyro_f[2]=-(gyro[2]/32.8- gyro_bias[2]);
    gyro_angle[0]=gyro_angle[0]+(gyro_f[0])*dt;
    gyro_angle[1]=gyro_angle[1]+(gyro_f[1])*dt;

    MPU9250_GET_ACCEL(accel); // 가속도계 센서값 획득
    accel_f[0]=accel[0]/8192.0; 
    if(accel_f[0] > 0.9){accel_f[0]=0.9;}
    if(accel_f[0]<-0.9){accel_f[0]=-0.9;}
    accel_f[1]=-accel[1]/8192.0*(-1);
    accel_f[2]=-accel[2]/8192.0*(-1);
    roll_accel=atan(accel_f[1]/accel_f[2])*180.0/3.14;

    pitch_accel=asin(accel_f[0])*180.0/3.14;
    
    // 상보필터
    roll_g_n=(1-dt/tau)*roll_g_old+ dt*gyro_f[0];
    roll_a_n=(1-dt/tau)*roll_a_old+dt/tau*roll_accel;
    pitch_g_n=(1-dt/tau)*pitch_g_old+ dt*gyro_f[1];
    pitch_a_n=(1-dt/tau)*pitch_a_old+dt/tau*pitch_accel;



    roll_g_old=roll_g_n;
    roll_a_old=roll_a_n;
    pitch_g_old=pitch_g_n;
    pitch_a_old=pitch_a_n;
    roll_c=roll_g_old+roll_a_old-bias_roll;
    pitch_c=pitch_g_old+pitch_a_old-bias_pitch;
    
    tmp_bias_roll += roll_c;
    tmp_bias_pitch += pitch_c;
    if (cnt > 500 && bias_check) {
        pid_loop.detach();
        
        tmp_bias_roll = tmp_bias_roll / (float)cnt;
        tmp_bias_pitch = tmp_bias_pitch / (float)cnt;
        bias_check = false;
        pc.printf("\r\npid_loop detached,cnt:%d",cnt);
        pc.printf("\r\nbias check : ");
        pc.printf(bias_check ? "true" : "false");
        pc.printf("\r\nAngle_bias Terminated");
        }

    cnt += 1;
    // ------------------------------------------------------------------------------

    // 2. PID 함수 통해 Roll_pid_in,Pitch_pid_in Update ------------------------------

    // Single PID
    Roll_pid_in = pid_roll(target_roll,roll_c);
    Pitch_pid_in = pid_pitch(target_pitch,pitch_c);

    // 3. PWM 출력 신호 Update
    pwm1 = throttle_offset+ROLL_offset+PITCH_offset-YAW_offset-Roll_pid_in-Pitch_pid_in;
    pwm2 = throttle_offset-ROLL_offset+PITCH_offset+YAW_offset+Roll_pid_in-Pitch_pid_in;
    pwm3 = throttle_offset-ROLL_offset-PITCH_offset-YAW_offset+Roll_pid_in+Pitch_pid_in;
    pwm4 = throttle_offset+ROLL_offset-PITCH_offset+YAW_offset-Roll_pid_in+Pitch_pid_in;

    int int_pwm1 = (int) pwm1;
    int int_pwm2 = (int) pwm2;
    int int_pwm3 = (int) pwm3;
    int int_pwm4 = (int) pwm4;
    if (int_pwm1 < 0) {int_pwm1 = 0;}
    if (int_pwm2 < 0) {int_pwm2 = 0;}
    if (int_pwm3 < 0) {int_pwm3 = 0;}
    if (int_pwm4 < 0) {int_pwm4 = 0;}
    servo1.pulsewidth_us(int_pwm1);
    servo2.pulsewidth_us(int_pwm2);
    servo3.pulsewidth_us(int_pwm3);
    servo4.pulsewidth_us(int_pwm4);

    if (cnt > 600){

        pc.printf("\rKp %2.2f Ki %2.2f Kd %2.2f pterm_pitch %3.2f dterm_pitch % 3.2f pitch pid %3.f",Kp,Ki,Kd,pterm_pitch,pterm_roll,Pitch_pid_in);


    }

}

void angle_bias(void){
    pc.printf("\r\nAngle bias Initiated");
    bias_check = true;
    pid_loop.attach_us(&activate,10000);
}
int main() {
    
    int count=0;
    uint8_t rt = 0;

    pc.baud(115200);
    i2c.frequency(400000);
    pc.printf("\r\nStart");
    wait(0.3);

    pc.printf("\n\rReset");
    MPU9250_RESET();
    wait(0.1);

    pc.printf("\n\rInitialize");
    MPU9250_INIT();
    wait(0.1);

    pc.printf("\n\r%d th Who am I?",count);
    WHO_AM_I();
    wait(0.1);

    pc.printf("\n\rcount and accel outputs(x,y,z)");

    gyro_bias_f();
    pc.printf("\n\rgyro biases(deg/sec) %f %f %f \n\r",
    gyro_bias[0],gyro_bias[1],gyro_bias[2]);
    wait(0.1);
    
    initRF();
    

    angle_bias();
    wait_ms(1000);

    target_roll = roll_c - tmp_bias_roll;
    target_pitch = pitch_c - tmp_bias_pitch;

    bias_roll = tmp_bias_roll;
    bias_pitch = tmp_bias_pitch;

    if (bias_check==false && cnt > 500){
    pc.printf("\r\ntarget roll : %f, target pitch : %f",target_roll,target_pitch);

    pc.printf("\r\nLoop Start\r\n");
    servo1.period_us(100);          // servo requires a 10KHz(0.0001sec = 10^-5sec)
    servo2.period_us(100);          
    servo3.period_us(100);          
    servo4.period_us(100);

    RF_loop.attach_us(&RF_READ, 50000); // RF 통신 코드를 20 Hz(50000us) 주기로 실행
    pid_loop.attach_us(&activate,10000); // 1.각도추정, 2. PID, 3. PWM 이 포함된 함수 activate를 100Hz(10000us) 주기로 실행
    }

}