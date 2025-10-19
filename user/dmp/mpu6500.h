#ifndef __mpu65
#define __mpu65


#define ERROR_MPU_INIT      -1 //错误代码所代表的含义
#define ERROR_SET_SENSOR    -2
#define ERROR_CONFIG_FIFO   -3
#define ERROR_SET_RATE      -4
#define ERROR_LOAD_MOTION_DRIVER    -5
#define ERROR_SET_ORIENTATION       -6
#define ERROR_ENABLE_FEATURE        -7
#define ERROR_SET_FIFO_RATE         -8
#define ERROR_SELF_TEST             -9
#define ERROR_DMP_STATE             -10
 
#define DEFAULT_MPU_HZ  100
#define Q30  1073741824.0f
 
int MPU6050_DMP_init(void);//DMP初始化函数
int MPU6050_DMP_Get_Date(float *pitch, float *roll, float *yaw);//DMP调用函数





















#endif



