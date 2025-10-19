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
                                    
//mpu6500�Ĵ���
#define MPU6050_RA_PWR_MGMT_1       0x6B
#define MPU6050_RA_PWR_MGMT_2       0x6C
#define MPU6050_RA_SMPLRT_DIV       0x19
#define MPU6050_RA_CONFIG           0x1A
#define MPU6050_RA_ACCEL_CONFIG     0x1C
#define MPU6050_RA_GYRO_CONFIG      0x1B
#define MPU6050_RA_WHO_AM_I         0x75
#define MPU6050_ACC_OUT             0x3B     //MPU6050���ٶ����ݼĴ�����ַ
#define MPU6050_GYRO_OUT        0x43     //MPU6050���������ݼĴ�����ַ
#define MPU6050_RA_TEMP_OUT_H       0x41





#define iicW	0	//iicд����λ	
#define iicR	1	//iic������λ	

#define MPU6050iicADDR    0xD2

#define mpuI2cScl1()  GPIO_SetBits(GPIOE, GPIO_Pin_7)	    //	MPU6050 IIC����
#define mpuI2cScl0()  GPIO_ResetBits(GPIOE, GPIO_Pin_7)	    //	MPU6050 IIC����
                                                         
#define mpuI2cSda1()  GPIO_SetBits(GPIOE, GPIO_Pin_8)	    //	MPU6050 IIC����
#define mpuI2cSda0()  GPIO_ResetBits(GPIOE, GPIO_Pin_8) 	//	MPU6050 IIC����

#define mpuI2cSdaRead()  GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_8)	/* ��SDA����״̬ */


// ���忨�����˲����ṹ��
typedef struct {
    float Q_angle;   // �����������Ƕȣ���Э����
    float Q_bias;    // ������������ƫ����Э����
    float R_measure; // �۲����������ٶȼƣ���Э����
    
    float angle;     // �ںϺ�����ŽǶȹ���
    float bias;      // ���Ƶ���������ƫ
    float P[2][2];   // ���Э�������
} KalmanFilter_t;

typedef struct {
    // ԭʼ����
    int16_t Accel_X;
    int16_t Accel_Y;
    int16_t Accel_Z;
    int16_t Gyro_X;
    int16_t Gyro_Y;
    int16_t Gyro_Z;
    // ת�����������������ʵ�ֻ��У�
    float Ax; // ��λ: g
    float Ay;
    float Az;
    float Gx; // ��λ: ��/s
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
