#ifndef __cpu
#define __cpu
#include "stm32f4xx.h"

#define aBin1ON     GPIO_SetBits(GPIOC , GPIO_Pin_4);
#define aBin1OFF    GPIO_ResetBits(GPIOC , GPIO_Pin_5);

                              

//#define A_MOTOR_FORWARD         \
//                                    aBin1ON; \
//                                    aBin1OFF
#define A_MOTOR_FORWARD         \
                                    GPIO_SetBits(GPIOC , GPIO_Pin_4);\
                                    GPIO_ResetBits(GPIOC , GPIO_Pin_5)
                                    
#define A_MOTOR_REVERSE         \
                                    GPIO_SetBits(GPIOC , GPIO_Pin_5);\
                                    GPIO_ResetBits(GPIOC , GPIO_Pin_4)                                      

                                                               
#define B_MOTOR_FORWARD     \
                                    GPIO_SetBits(GPIOB , GPIO_Pin_1);\
                                    GPIO_ResetBits(GPIOB , GPIO_Pin_0)
                                    
                                    
#define B_MOTOR_REVERSE     \
                                    GPIO_SetBits(GPIOB , GPIO_Pin_0);\
                                    GPIO_ResetBits(GPIOB , GPIO_Pin_1)
                                    
//mpu6500寄存器
#define MPU6050_RA_PWR_MGMT_1       0x6B
#define MPU6050_RA_PWR_MGMT_2       0x6C
#define MPU6050_RA_SMPLRT_DIV       0x19
#define MPU6050_RA_CONFIG           0x1A
#define MPU6050_RA_ACCEL_CONFIG     0x1C
#define MPU6050_RA_GYRO_CONFIG      0x1B
#define MPU6050_RA_WHO_AM_I         0x75
#define MPU6050_ACC_OUT             0x3B     //MPU6050加速度数据寄存器地址
#define MPU6050_GYRO_OUT        0x43     //MPU6050陀螺仪数据寄存器地址
#define MPU6050_RA_TEMP_OUT_H       0x41





#define iicW	0	//iic写操作位	
#define iicR	1	//iic读操作位	

#define MPU6050iicADDR    0xD2

#define mpuI2cScl1()  GPIO_SetBits(GPIOE, GPIO_Pin_7)	    //	MPU6050 IIC控制
#define mpuI2cScl0()  GPIO_ResetBits(GPIOE, GPIO_Pin_7)	    //	MPU6050 IIC控制
                                                         
#define mpuI2cSda1()  GPIO_SetBits(GPIOE, GPIO_Pin_8)	    //	MPU6050 IIC控制
#define mpuI2cSda0()  GPIO_ResetBits(GPIOE, GPIO_Pin_8) 	//	MPU6050 IIC控制

#define mpuI2cSdaRead()  GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_8)	/* 读SDA口线状态 */


// 定义卡尔曼滤波器结构体
typedef struct {
    float Q_angle;   // 过程噪声（角度）的协方差
    float Q_bias;    // 过程噪声（零偏）的协方差
    float R_measure; // 观测噪声（加速度计）的协方差
    
    float angle;     // 融合后的最优角度估计
    float bias;      // 估计的陀螺仪零偏
    float P[2][2];   // 误差协方差矩阵
} KalmanFilter_t;

typedef struct {
    // 原始数据
    int16_t Accel_X;
    int16_t Accel_Y;
    int16_t Accel_Z;
    int16_t Gyro_X;
    int16_t Gyro_Y;
    int16_t Gyro_Z;
    // 转换后的物理量（部分实现会有）
    float Ax; // 单位: g
    float Ay;
    float Az;
    float Gx; // 单位: °/s
    float Gy;
    float Gz;
} DataStruct1;

void cpuInit(void);
void delayMs(u16 value);
void delayUs(u16 value);


void iicStart(void);
void iicStop(void);
void iicSendOneByte(uint8_t data);

u8 iicWaitAck(void);
void iicAck(void);
void iicNAck(void);
u8 iicReadOneByte(void);
uint8_t i2c_CheckDevice(uint8_t _Address);
u32 mpuWrite(u8 reg_add,u8 reg_dat);
u32 mpuRead(u8 reg_add,unsigned char* Read,u8 num);
uint8_t MPU6050ReadID(void);
void MPU6500ReadAcc(short *accData);
void MPU6050ReadGyro(short *gyroData);
void MPU6050_ReturnTemp(float *Temperature);
float Calculate_Speed_RPM1(void);



float PID_Balance(float Angle, float Gyro , int xiuzheng);

float getAngleFromAccel(float ax, float ay, float az);
float Kalman_Update(KalmanFilter_t *kf, float newRate, float newAngle, float dt);
float Calculate_Speed_RPM(void);
uint8_t Sensor_Read_DMP(uint8_t slave_addr,uint8_t reg_add,uint8_t num,unsigned char* Read);
uint8_t Sensor_write_DMP(uint8_t slave_addr,uint8_t reg_add,uint8_t num,unsigned char* write);


#endif
