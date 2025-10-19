#include "stm32f4xx.h"
#include "cpu.h"
#include <math.h>
#include "mpu6500.h"


u16 pwmval=0;    
u8 dir=1;
//u8 flag = 0;

typedef enum {FORWARD = 0, REVERSE = !FORWARD} DIRECTION;

int dire;
u8 ackk , success = 0;

short Acel[3];
short Gyro[3];
float Temp , pitch_rad , fused_angle , fused_angle_gz;

int16_t ax_raw, ay_raw, az_raw,
        gx_raw, gy_raw, gz_raw;



//short pitch;
const float M_PI = 3.14159265358979323846f;


// 初始化卡尔曼滤波器
void Kalman_Init(KalmanFilter_t *kf, float Q_angle, float Q_bias, float R_measure) {
    kf->Q_angle = Q_angle;
    kf->Q_bias = Q_bias;
    kf->R_measure = R_measure;
    
    kf->angle = 0.0;
    kf->bias = 0.0;
    kf->P[0][0] = 0.0;
    kf->P[0][1] = 0.0;
    kf->P[1][0] = 0.0;
    kf->P[1][1] = 0.0;
}

// 卡尔曼滤波计算
float Kalman_Update(KalmanFilter_t *kf, float newRate, float newAngle, float dt) {
    // 预测步骤：更新状态和误差协方差
    kf->angle += dt * (newRate - kf->bias);
    kf->P[0][0] += dt * (dt * kf->P[1][1] - kf->P[0][1] - kf->P[1][0] + kf->Q_angle);
    kf->P[0][1] -= dt * kf->P[1][1];
    kf->P[1][0] -= dt * kf->P[1][1];
    kf->P[1][1] += kf->Q_bias * dt;
    
    // 计算卡尔曼增益
    float S = kf->P[0][0] + kf->R_measure;
    float K[2];
    K[0] = kf->P[0][0] / S;
    K[1] = kf->P[1][0] / S;
    
    // 更新步骤：用观测值（加速度计角度）修正预测值
    float y = newAngle - kf->angle; // 观测残差
    kf->angle += K[0] * y;
    kf->bias += K[1] * y;
    
    // 更新误差协方差
    float P00_temp = kf->P[0][0];
    float P01_temp = kf->P[0][1];
    
    kf->P[0][0] -= K[0] * P00_temp;
    kf->P[0][1] -= K[0] * P01_temp;
    kf->P[1][0] -= K[1] * P00_temp;
    kf->P[1][1] -= K[1] * P01_temp;
    
    return kf->angle;
}

// 从加速度计原始数据计算角度（弧度）
float getAngleFromAccel(float ax, float ay, float az) {
    return atan2(ax, sqrt(ay * ay + az * az));
}

// 从加速度计数据计算滚转角（弧度），再转换为度
float getRollFromAccel(float ax, float ay, float az) {
    // 使用atan2函数计算滚转角，注意参数顺序和符号
    // 公式：roll = atan2(ay, sqrt(ax*ax + az*az)) 或 atan2(ay, az)
    // 具体公式需根据你的传感器安装方向确定[3](@ref)
    float roll_rad = atan2(ay, sqrt(ax * ax + az * az));
    // 将弧度转换为角度
    float roll_deg = roll_rad * 180.0f / M_PI;
    return roll_deg;
}


KalmanFilter_t kf_pitch;
KalmanFilter_t kf_pitch_gz;
DataStruct1 DataStruct;

// 速度环PI控制器
// 入口参数：Target_Speed - 目标速度（通常为0，以保持静止平衡）, encoder_left, encoder_right - 左右编码器测量的速度值
// 返回值：速度环计算出的PWM调整量
int Velocity(int Target_Speed, int encoder_left, int encoder_right)
{
    // 1. PI参数定义 - 这些是需要调试的核心参数
    float Velocity_Kp = 0.082;  // 比例系数
    float Velocity_Ki = 0.0041; // 积分系数
    int Velocity_I_Max = 1000; // 积分限幅，防止积分饱和
    
    // 2. 静态变量定义，用于保存状态
    static int Encoder_Err;          // 当前速度偏差
    static int EnC_Err_Lowout;       // 经过低通滤波后的偏差
    static int EnC_Err_Lowout_last;  // 上一次滤波后的偏差
    static int Encoder_S;            // 偏差的积分项
    static int PWM_out;              // PI输出
    
    float a = 0.7; // 低通滤波系数，用于平滑速度数据，减小突变干扰[1](@ref)
    
    // 3. 计算速度偏差：测量值 - 目标值
    // 注意：取左右编码器的平均值作为当前速度，使两轮速度一致[3](@ref)
    Encoder_Err = (encoder_left + encoder_right) - 2 * Target_Speed;
    
    // 4. 对偏差进行一阶低通滤波，使波形更平滑，防止速度突变干扰直立环[1](@ref)
    EnC_Err_Lowout = (1 - a) * Encoder_Err + a * EnC_Err_Lowout_last;
    EnC_Err_Lowout_last = EnC_Err_Lowout;
    
    // 5. 对滤波后的偏差进行积分，用于消除静差
    Encoder_S += EnC_Err_Lowout;
    // 积分限幅，防止积分项过大（积分饱和）导致系统失控[1,4](@ref)
    Encoder_S = Encoder_S > Velocity_I_Max ? Velocity_I_Max : (Encoder_S < (-Velocity_I_Max) ? (-Velocity_I_Max) : Encoder_S);
    
    // 6. PI控制器计算输出：P项 + I项
    PWM_out = Velocity_Kp * EnC_Err_Lowout + Velocity_Ki * Encoder_S;
    
    return PWM_out;
}

// 直立环PD控制器
// 输入：Angle - 当前角度（度），Gyro - 当前角速度（度/秒）
// 返回：直立环计算出的PWM控制量
float PID_Balance(float Angle, float Gyro , int xiuzheng) {
    // 1. 定义PID参数和中间变量
    float Balance_Kp = 32;   // 比例系数，需调试[4](@ref)
    float Balance_Kd = 40;     // 微分系数，需调试[4](@ref)
    float Target_Angle = 1.31;   // 目标平衡角度，通常为0度（竖直位置）
    
    // 2. 计算角度偏差
    // 注意：有些小车机械中值可能不是0度，需根据实际调整Target_Angle[2,6](@ref)
    float Bias = Target_Angle + xiuzheng - Angle;
    
    // 3. 计算PD输出
    // 公式：PWM = Kp * Bias + Kd * Gyro
    // 微分项直接使用陀螺仪测量的角速度，避免对角度求导带来的噪声[1,4](@ref)
    float Balance_PWM = Balance_Kp * Bias + Balance_Kd * Gyro;
    
    // 4. PWM输出限幅，防止过冲损坏硬件[1,2](@ref)
    #define PWM_MAX 1500  // 根据定时器ARR值设定
    #define PWM_MIN -1500
    if (Balance_PWM > PWM_MAX) Balance_PWM = PWM_MAX;
    if (Balance_PWM < PWM_MIN) Balance_PWM = PWM_MIN;
    
    return Balance_PWM;
}

// 转向环PD控制器,转向环的输出是一个用于产生差速的PWM修正量。你需要将它和??直立环??、??速度环??的输出叠加，最终决定左右电机的功率
float KP_TURN , KD_TURN;
float Turn_PD_Controller(float target_turn_rate, float current_turn_rate) {
    float error = target_turn_rate - current_turn_rate;
    static float last_error = 0;
    float derivative = error - last_error; // 简易微分项计算
    last_error = error;

    float output = KP_TURN * error + KD_TURN * derivative;
    return output;
}

// 假设你已经通过其他PID环计算出了基础PWM (balance_pwm) 和速度补偿 (velocity_pwm)
//float turn_pwm = Turn_PD_Controller(target_turn_rate, gyro_z_dps);

//// 将转向PWM差量分别施加到左右电机上
//motor_left  = balance_pwm + velocity_pwm - turn_pwm;
//motor_right = balance_pwm + velocity_pwm + turn_pwm;

//// 确保最终的PWM值在电机驱动允许的范围内
//motor_left  = constrain(motor_left, -MAX_PWM, MAX_PWM);
//motor_right = constrain(motor_right, -MAX_PWM, MAX_PWM);

// 全局变量
int32_t g_TotalCount = 0;    // 用于累计可能超过16位的总脉冲数，防止溢出
int16_t g_LastCount = 0;     // 上一次读取的计数值

// 假设：电机减速比为 30，采样时间间隔为 0.05秒 (50ms)
#define GEAR_RATIO      30
#define SAMPLE_TIME_S   0.002f

// 读取编码器计数值（16位有符号整数，可正可负）
int16_t Encoder_Get_Count(void)
{
    int16_t count;
    count = (int16_t)(TIM1->CNT); // 直接读取计数寄存器
    return count;
}



// 读取编码器计数值（16位有符号整数，可正可负）
int16_t Encoder_Get_Count1(void)
{
    int16_t count;
    count = (int16_t)(TIM5->CNT); // 直接读取计数寄存器
    return count;
}



volatile    float speed_rpm;

float Calculate_Speed_RPM(void)
{
    int16_t current_count;
    int16_t delta_count;


    current_count = Encoder_Get_Count(); // 读取当前计数值
    delta_count = current_count - g_LastCount; // 计算采样间隔内的脉冲数差

    // 处理计数器溢出情况（如果delta_count过大，可能发生了溢出）
    // 简单的处理：如果差值超过正负一半的周期（0x7FFF），则认为发生了溢出并修正
    if(delta_count > 0x7FFF) {
        delta_count -= 0xFFFF;
    } else if(delta_count < -0x7FFF) {
        delta_count += 0xFFFF;
    }

    g_TotalCount += delta_count; // 更新总脉冲数（如果需要）
    g_LastCount = current_count; // 更新上一次的计数值

    // 转速计算公式：RPM = (Δ脉冲数 / (4 * 编码器线数)) * (60 / 采样时间) / 减速比
    speed_rpm = ((float)delta_count / (4.0f * 500.0f)) * (60.0f / SAMPLE_TIME_S) / GEAR_RATIO;

    return speed_rpm;
}

volatile    float speed_rpm1;
// 全局变量
int32_t g_TotalCount1 = 0;    // 用于累计可能超过16位的总脉冲数，防止溢出
int16_t g_LastCount1 = 0;     // 上一次读取的计数值
float Calculate_Speed_RPM1(void)
{
    int16_t current_count;
    int16_t delta_count;


    current_count = Encoder_Get_Count1(); // 读取当前计数值
    delta_count = current_count - g_LastCount1; // 计算采样间隔内的脉冲数差

    // 处理计数器溢出情况（如果delta_count过大，可能发生了溢出）
    // 简单的处理：如果差值超过正负一半的周期（0x7FFF），则认为发生了溢出并修正
    if(delta_count > 0x7FFF) {
        delta_count -= 0xFFFF;
    } else if(delta_count < -0x7FFF) {
        delta_count += 0xFFFF;
    }

    g_TotalCount1 += delta_count; // 更新总脉冲数（如果需要）
    g_LastCount1 = current_count; // 更新上一次的计数值

    // 转速计算公式：RPM = (Δ脉冲数 / (4 * 编码器线数)) * (60 / 采样时间) / 减速比
    speed_rpm1 = ((float)delta_count / (4.0f * 500.0f)) * (60.0f / SAMPLE_TIME_S) / GEAR_RATIO;

    return speed_rpm1;
}

float pitch,roll,yaw;
float accel_angle = 0 , balancePwmValue = 0 , gyro_rate;
extern u8 cnt10msFlag , cnt50msFlag;
int angle_adjust=0;
//滚转角、俯仰角已经实现
//偏航角没有，使用Z轴角速度控制转向
int main()
 {
    float gyro_scale = 300.5; // 假设量程为 ±500 °/s, 灵敏度因子为 65.5 LSB/°/s
    
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
    cpuInit();
    
    Kalman_Init(&kf_pitch, 0.02, 0.003, 0.01);
    Kalman_Init(&kf_pitch_gz, 0.001, 0.003, 0.03);

    dire = FORWARD;
//     while(1)
//     {
//        if (MPU6050ReadID() == 1)
//        {
//            success = 1;
//        }
//        else
//        {
//            success = 0;
//        }
//     }
    
    int ret=0;
  do {
    ret=MPU6050_DMP_init();//DMP初始化
  } while (ret);
  while(1);
//	TIM_SetCompare1(TIM3,(u32)0);	//修改比较值，修改占空比  右侧   
//    TIM_SetCompare2(TIM3,(u32)0);	//修改比较值，修改占空比  左侧
  while(1)
  {
    if(cnt50msFlag == 1)
        {
            cnt50msFlag = 0;
            MPU6050_DMP_Get_Date(&pitch,&roll,&yaw);//DMP数据获取
//            sprintf(String,"pitch:%.1f",pitch);
//            sprintf(String,"roll:%.1f",roll);
//            sprintf(String,"yaw:%.1f",yaw);

        }
  }
	while(1)
	{
        MPU6500ReadAcc(Acel);
        MPU6050ReadGyro(Gyro);
        MPU6050_ReturnTemp(&Temp);
        
        ax_raw = Acel[0];
        ay_raw = Acel[1];
        az_raw = Acel[2];
        gx_raw = Gyro[0];
        gy_raw = Gyro[1];
        gz_raw = Gyro[2];
        
//        float ax_g = ax_raw / 16384.0;    //(需要C99编译器，C89编译器会报错，定义必须放在头位置)
//        float ay_g = ay_raw / 16384.0;
//        float az_g = az_raw / 16384.0;
        
        DataStruct.Ax = ax_raw / 8192.0;
        DataStruct.Ay = ay_raw / 8192.0;
        DataStruct.Az = az_raw / 8192.0;
        // 将原始数据转换为角速度（度/秒）
        DataStruct.Gx = gx_raw / gyro_scale;//x轴角速度
        DataStruct.Gy = gy_raw / gyro_scale;//y轴角速度
        DataStruct.Gz = gz_raw / gyro_scale;//z轴角速度
        
        // 计算加速度计角度（转换为角度制）
        accel_angle = getAngleFromAccel(DataStruct.Ax, DataStruct.Ay, DataStruct.Az) * 57.2958f;
        // 获取陀螺仪Y轴角速度（度/秒），注意：根据你的传感器安装方向，俯仰角可能对应Gy或Gx
        gyro_rate = DataStruct.Gy;
        // 计算采样时间间隔dt，例如0.01秒（100Hz）
        float dt = 0.0009f;
        
        

        
        
        
        
        // 计算加速度计滚转角（转换为角度制）
        float accel_roll = getRollFromAccel(DataStruct.Ax, DataStruct.Ay, DataStruct.Az);
        // 获取陀螺仪X轴角速度（度/秒）-- 注意：滚转角通常对应绕X轴的旋转，请根据你的传感器安装确认[3](@ref)
        float gyro_rate_gz = DataStruct.Gy; 
        
        // 应用卡尔曼滤波计算俯仰角
            fused_angle = Kalman_Update(&kf_pitch, gyro_rate, accel_angle, dt);
            // 应用卡尔曼滤波计算滚转角
            fused_angle_gz = Kalman_Update(&kf_pitch_gz, gyro_rate_gz, accel_roll, dt);
            
        if(cnt50msFlag == 1)
        {
            cnt50msFlag = 0;
            angle_adjust = Velocity(0 , (int)speed_rpm , (int)(-speed_rpm1));         //速度环
        }

        if(cnt10msFlag == 1)
        {
            cnt10msFlag = 0;
            
            
            
            
            balancePwmValue = PID_Balance(fused_angle , gyro_rate , angle_adjust);//直立环
            
            if(balancePwmValue > 0)
            {
                A_MOTOR_FORWARD;
                B_MOTOR_FORWARD;
                if((balancePwmValue > 15) && (balancePwmValue < 110))
                {
                    balancePwmValue = 110;
                } 
                TIM_SetCompare1(TIM3,(u32)balancePwmValue);	//修改比较值，修改占空比  右侧  定时器5   前进负值
                TIM_SetCompare2(TIM3,(u32)balancePwmValue);	//修改比较值，修改占空比  左侧   定时器1
            }
            else
            {
                balancePwmValue = -balancePwmValue;
                if((balancePwmValue > 15) && (balancePwmValue < 110))
                {
                    balancePwmValue = 110;
                }
                A_MOTOR_REVERSE;
                B_MOTOR_REVERSE;
                TIM_SetCompare1(TIM3,(u32)balancePwmValue);	//修改比较值，修改占空比  右侧   
                TIM_SetCompare2(TIM3,(u32)balancePwmValue);	//修改比较值，修改占空比  左侧
            }
        }
        

//        // 2. 使用公式计算俯仰角（弧度）
//        pitch_rad = atan2(ax_g, sqrt(ay_g * ay_g + az_g * az_g)) * 100;




    }

}


    //加减速测试代码
//        delayMs(2);
//        if(dir)
//        {
//            pwmval++;//dir==1 led0pwmval递增
//        }
//		else
//        {
//            pwmval--;	//dir==0 led0pwmval递减 
//        }            
// 		if(pwmval>15)
//        {
//            dir=0;//led0pwmval到达300后，方向为递减
//        }
//		if(pwmval==0)
//        {
//            dir=1;	//led0pwmval递减到0后，方向改为递增
//            dire = !dire;
//            if(dire == REVERSE )
//            {
//                A_MOTOR_REVERSE;
//                B_MOTOR_REVERSE;
//            }
//            else if(dire == FORWARD)
//            {
//                A_MOTOR_FORWARD;
//                B_MOTOR_FORWARD;
//            }
//            delayMs(1);
//        }
//        
// 
//		TIM_SetCompare1(TIM3,pwmval);	//修改比较值，修改占空比
//		TIM_SetCompare2(TIM3,pwmval);	//修改比较值，修改占空比





