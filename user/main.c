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


// ��ʼ���������˲���
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

// �������˲�����
float Kalman_Update(KalmanFilter_t *kf, float newRate, float newAngle, float dt) {
    // Ԥ�ⲽ�裺����״̬�����Э����
    kf->angle += dt * (newRate - kf->bias);
    kf->P[0][0] += dt * (dt * kf->P[1][1] - kf->P[0][1] - kf->P[1][0] + kf->Q_angle);
    kf->P[0][1] -= dt * kf->P[1][1];
    kf->P[1][0] -= dt * kf->P[1][1];
    kf->P[1][1] += kf->Q_bias * dt;
    
    // ���㿨��������
    float S = kf->P[0][0] + kf->R_measure;
    float K[2];
    K[0] = kf->P[0][0] / S;
    K[1] = kf->P[1][0] / S;
    
    // ���²��裺�ù۲�ֵ�����ٶȼƽǶȣ�����Ԥ��ֵ
    float y = newAngle - kf->angle; // �۲�в�
    kf->angle += K[0] * y;
    kf->bias += K[1] * y;
    
    // �������Э����
    float P00_temp = kf->P[0][0];
    float P01_temp = kf->P[0][1];
    
    kf->P[0][0] -= K[0] * P00_temp;
    kf->P[0][1] -= K[0] * P01_temp;
    kf->P[1][0] -= K[1] * P00_temp;
    kf->P[1][1] -= K[1] * P01_temp;
    
    return kf->angle;
}

// �Ӽ��ٶȼ�ԭʼ���ݼ���Ƕȣ����ȣ�
float getAngleFromAccel(float ax, float ay, float az) {
    return atan2(ax, sqrt(ay * ay + az * az));
}

// �Ӽ��ٶȼ����ݼ����ת�ǣ����ȣ�����ת��Ϊ��
float getRollFromAccel(float ax, float ay, float az) {
    // ʹ��atan2���������ת�ǣ�ע�����˳��ͷ���
    // ��ʽ��roll = atan2(ay, sqrt(ax*ax + az*az)) �� atan2(ay, az)
    // ���幫ʽ�������Ĵ�������װ����ȷ��[3](@ref)
    float roll_rad = atan2(ay, sqrt(ax * ax + az * az));
    // ������ת��Ϊ�Ƕ�
    float roll_deg = roll_rad * 180.0f / M_PI;
    return roll_deg;
}


KalmanFilter_t kf_pitch;
KalmanFilter_t kf_pitch_gz;
DataStruct1 DataStruct;

// �ٶȻ�PI������
// ��ڲ�����Target_Speed - Ŀ���ٶȣ�ͨ��Ϊ0���Ա��־�ֹƽ�⣩, encoder_left, encoder_right - ���ұ������������ٶ�ֵ
// ����ֵ���ٶȻ��������PWM������
int Velocity(int Target_Speed, int encoder_left, int encoder_right)
{
    // 1. PI�������� - ��Щ����Ҫ���Եĺ��Ĳ���
    float Velocity_Kp = 0.082;  // ����ϵ��
    float Velocity_Ki = 0.0041; // ����ϵ��
    int Velocity_I_Max = 1000; // �����޷�����ֹ���ֱ���
    
    // 2. ��̬�������壬���ڱ���״̬
    static int Encoder_Err;          // ��ǰ�ٶ�ƫ��
    static int EnC_Err_Lowout;       // ������ͨ�˲����ƫ��
    static int EnC_Err_Lowout_last;  // ��һ���˲����ƫ��
    static int Encoder_S;            // ƫ��Ļ�����
    static int PWM_out;              // PI���
    
    float a = 0.7; // ��ͨ�˲�ϵ��������ƽ���ٶ����ݣ���Сͻ�����[1](@ref)
    
    // 3. �����ٶ�ƫ�����ֵ - Ŀ��ֵ
    // ע�⣺ȡ���ұ�������ƽ��ֵ��Ϊ��ǰ�ٶȣ�ʹ�����ٶ�һ��[3](@ref)
    Encoder_Err = (encoder_left + encoder_right) - 2 * Target_Speed;
    
    // 4. ��ƫ�����һ�׵�ͨ�˲���ʹ���θ�ƽ������ֹ�ٶ�ͻ�����ֱ����[1](@ref)
    EnC_Err_Lowout = (1 - a) * Encoder_Err + a * EnC_Err_Lowout_last;
    EnC_Err_Lowout_last = EnC_Err_Lowout;
    
    // 5. ���˲����ƫ����л��֣�������������
    Encoder_S += EnC_Err_Lowout;
    // �����޷�����ֹ��������󣨻��ֱ��ͣ�����ϵͳʧ��[1,4](@ref)
    Encoder_S = Encoder_S > Velocity_I_Max ? Velocity_I_Max : (Encoder_S < (-Velocity_I_Max) ? (-Velocity_I_Max) : Encoder_S);
    
    // 6. PI���������������P�� + I��
    PWM_out = Velocity_Kp * EnC_Err_Lowout + Velocity_Ki * Encoder_S;
    
    return PWM_out;
}

// ֱ����PD������
// ���룺Angle - ��ǰ�Ƕȣ��ȣ���Gyro - ��ǰ���ٶȣ���/�룩
// ���أ�ֱ�����������PWM������
float PID_Balance(float Angle, float Gyro , int xiuzheng) {
    // 1. ����PID�������м����
    float Balance_Kp = 32;   // ����ϵ���������[4](@ref)
    float Balance_Kd = 40;     // ΢��ϵ���������[4](@ref)
    float Target_Angle = 1.31;   // Ŀ��ƽ��Ƕȣ�ͨ��Ϊ0�ȣ���ֱλ�ã�
    
    // 2. ����Ƕ�ƫ��
    // ע�⣺��ЩС����е��ֵ���ܲ���0�ȣ������ʵ�ʵ���Target_Angle[2,6](@ref)
    float Bias = Target_Angle + xiuzheng - Angle;
    
    // 3. ����PD���
    // ��ʽ��PWM = Kp * Bias + Kd * Gyro
    // ΢����ֱ��ʹ�������ǲ����Ľ��ٶȣ�����ԽǶ��󵼴���������[1,4](@ref)
    float Balance_PWM = Balance_Kp * Bias + Balance_Kd * Gyro;
    
    // 4. PWM����޷�����ֹ������Ӳ��[1,2](@ref)
    #define PWM_MAX 1500  // ���ݶ�ʱ��ARRֵ�趨
    #define PWM_MIN -1500
    if (Balance_PWM > PWM_MAX) Balance_PWM = PWM_MAX;
    if (Balance_PWM < PWM_MIN) Balance_PWM = PWM_MIN;
    
    return Balance_PWM;
}

// ת��PD������,ת�򻷵������һ�����ڲ������ٵ�PWM������������Ҫ������??ֱ����??��??�ٶȻ�??��������ӣ����վ������ҵ���Ĺ���
float KP_TURN , KD_TURN;
float Turn_PD_Controller(float target_turn_rate, float current_turn_rate) {
    float error = target_turn_rate - current_turn_rate;
    static float last_error = 0;
    float derivative = error - last_error; // ����΢�������
    last_error = error;

    float output = KP_TURN * error + KD_TURN * derivative;
    return output;
}

// �������Ѿ�ͨ������PID��������˻���PWM (balance_pwm) ���ٶȲ��� (velocity_pwm)
//float turn_pwm = Turn_PD_Controller(target_turn_rate, gyro_z_dps);

//// ��ת��PWM�����ֱ�ʩ�ӵ����ҵ����
//motor_left  = balance_pwm + velocity_pwm - turn_pwm;
//motor_right = balance_pwm + velocity_pwm + turn_pwm;

//// ȷ�����յ�PWMֵ�ڵ����������ķ�Χ��
//motor_left  = constrain(motor_left, -MAX_PWM, MAX_PWM);
//motor_right = constrain(motor_right, -MAX_PWM, MAX_PWM);

// ȫ�ֱ���
int32_t g_TotalCount = 0;    // �����ۼƿ��ܳ���16λ��������������ֹ���
int16_t g_LastCount = 0;     // ��һ�ζ�ȡ�ļ���ֵ

// ���裺������ٱ�Ϊ 30������ʱ����Ϊ 0.05�� (50ms)
#define GEAR_RATIO      30
#define SAMPLE_TIME_S   0.002f

// ��ȡ����������ֵ��16λ�з��������������ɸ���
int16_t Encoder_Get_Count(void)
{
    int16_t count;
    count = (int16_t)(TIM1->CNT); // ֱ�Ӷ�ȡ�����Ĵ���
    return count;
}



// ��ȡ����������ֵ��16λ�з��������������ɸ���
int16_t Encoder_Get_Count1(void)
{
    int16_t count;
    count = (int16_t)(TIM5->CNT); // ֱ�Ӷ�ȡ�����Ĵ���
    return count;
}



volatile    float speed_rpm;

float Calculate_Speed_RPM(void)
{
    int16_t current_count;
    int16_t delta_count;


    current_count = Encoder_Get_Count(); // ��ȡ��ǰ����ֵ
    delta_count = current_count - g_LastCount; // �����������ڵ���������

    // ��������������������delta_count���󣬿��ܷ����������
    // �򵥵Ĵ��������ֵ��������һ������ڣ�0x7FFF��������Ϊ���������������
    if(delta_count > 0x7FFF) {
        delta_count -= 0xFFFF;
    } else if(delta_count < -0x7FFF) {
        delta_count += 0xFFFF;
    }

    g_TotalCount += delta_count; // �������������������Ҫ��
    g_LastCount = current_count; // ������һ�εļ���ֵ

    // ת�ټ��㹫ʽ��RPM = (�������� / (4 * ����������)) * (60 / ����ʱ��) / ���ٱ�
    speed_rpm = ((float)delta_count / (4.0f * 500.0f)) * (60.0f / SAMPLE_TIME_S) / GEAR_RATIO;

    return speed_rpm;
}

volatile    float speed_rpm1;
// ȫ�ֱ���
int32_t g_TotalCount1 = 0;    // �����ۼƿ��ܳ���16λ��������������ֹ���
int16_t g_LastCount1 = 0;     // ��һ�ζ�ȡ�ļ���ֵ
float Calculate_Speed_RPM1(void)
{
    int16_t current_count;
    int16_t delta_count;


    current_count = Encoder_Get_Count1(); // ��ȡ��ǰ����ֵ
    delta_count = current_count - g_LastCount1; // �����������ڵ���������

    // ��������������������delta_count���󣬿��ܷ����������
    // �򵥵Ĵ��������ֵ��������һ������ڣ�0x7FFF��������Ϊ���������������
    if(delta_count > 0x7FFF) {
        delta_count -= 0xFFFF;
    } else if(delta_count < -0x7FFF) {
        delta_count += 0xFFFF;
    }

    g_TotalCount1 += delta_count; // �������������������Ҫ��
    g_LastCount1 = current_count; // ������һ�εļ���ֵ

    // ת�ټ��㹫ʽ��RPM = (�������� / (4 * ����������)) * (60 / ����ʱ��) / ���ٱ�
    speed_rpm1 = ((float)delta_count / (4.0f * 500.0f)) * (60.0f / SAMPLE_TIME_S) / GEAR_RATIO;

    return speed_rpm1;
}

float pitch,roll,yaw;
float accel_angle = 0 , balancePwmValue = 0 , gyro_rate;
extern u8 cnt10msFlag , cnt50msFlag;
int angle_adjust=0;
//��ת�ǡ��������Ѿ�ʵ��
//ƫ����û�У�ʹ��Z����ٶȿ���ת��
int main()
 {
    float gyro_scale = 300.5; // ��������Ϊ ��500 ��/s, ����������Ϊ 65.5 LSB/��/s
    
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//����ϵͳ�ж����ȼ�����2
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
    ret=MPU6050_DMP_init();//DMP��ʼ��
  } while (ret);
  while(1);
//	TIM_SetCompare1(TIM3,(u32)0);	//�޸ıȽ�ֵ���޸�ռ�ձ�  �Ҳ�   
//    TIM_SetCompare2(TIM3,(u32)0);	//�޸ıȽ�ֵ���޸�ռ�ձ�  ���
  while(1)
  {
    if(cnt50msFlag == 1)
        {
            cnt50msFlag = 0;
            MPU6050_DMP_Get_Date(&pitch,&roll,&yaw);//DMP���ݻ�ȡ
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
        
//        float ax_g = ax_raw / 16384.0;    //(��ҪC99��������C89�������ᱨ������������ͷλ��)
//        float ay_g = ay_raw / 16384.0;
//        float az_g = az_raw / 16384.0;
        
        DataStruct.Ax = ax_raw / 8192.0;
        DataStruct.Ay = ay_raw / 8192.0;
        DataStruct.Az = az_raw / 8192.0;
        // ��ԭʼ����ת��Ϊ���ٶȣ���/�룩
        DataStruct.Gx = gx_raw / gyro_scale;//x����ٶ�
        DataStruct.Gy = gy_raw / gyro_scale;//y����ٶ�
        DataStruct.Gz = gz_raw / gyro_scale;//z����ٶ�
        
        // ������ٶȼƽǶȣ�ת��Ϊ�Ƕ��ƣ�
        accel_angle = getAngleFromAccel(DataStruct.Ax, DataStruct.Ay, DataStruct.Az) * 57.2958f;
        // ��ȡ������Y����ٶȣ���/�룩��ע�⣺������Ĵ�������װ���򣬸����ǿ��ܶ�ӦGy��Gx
        gyro_rate = DataStruct.Gy;
        // �������ʱ����dt������0.01�루100Hz��
        float dt = 0.0009f;
        
        

        
        
        
        
        // ������ٶȼƹ�ת�ǣ�ת��Ϊ�Ƕ��ƣ�
        float accel_roll = getRollFromAccel(DataStruct.Ax, DataStruct.Ay, DataStruct.Az);
        // ��ȡ������X����ٶȣ���/�룩-- ע�⣺��ת��ͨ����Ӧ��X�����ת���������Ĵ�������װȷ��[3](@ref)
        float gyro_rate_gz = DataStruct.Gy; 
        
        // Ӧ�ÿ������˲����㸩����
            fused_angle = Kalman_Update(&kf_pitch, gyro_rate, accel_angle, dt);
            // Ӧ�ÿ������˲������ת��
            fused_angle_gz = Kalman_Update(&kf_pitch_gz, gyro_rate_gz, accel_roll, dt);
            
        if(cnt50msFlag == 1)
        {
            cnt50msFlag = 0;
            angle_adjust = Velocity(0 , (int)speed_rpm , (int)(-speed_rpm1));         //�ٶȻ�
        }

        if(cnt10msFlag == 1)
        {
            cnt10msFlag = 0;
            
            
            
            
            balancePwmValue = PID_Balance(fused_angle , gyro_rate , angle_adjust);//ֱ����
            
            if(balancePwmValue > 0)
            {
                A_MOTOR_FORWARD;
                B_MOTOR_FORWARD;
                if((balancePwmValue > 15) && (balancePwmValue < 110))
                {
                    balancePwmValue = 110;
                } 
                TIM_SetCompare1(TIM3,(u32)balancePwmValue);	//�޸ıȽ�ֵ���޸�ռ�ձ�  �Ҳ�  ��ʱ��5   ǰ����ֵ
                TIM_SetCompare2(TIM3,(u32)balancePwmValue);	//�޸ıȽ�ֵ���޸�ռ�ձ�  ���   ��ʱ��1
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
                TIM_SetCompare1(TIM3,(u32)balancePwmValue);	//�޸ıȽ�ֵ���޸�ռ�ձ�  �Ҳ�   
                TIM_SetCompare2(TIM3,(u32)balancePwmValue);	//�޸ıȽ�ֵ���޸�ռ�ձ�  ���
            }
        }
        

//        // 2. ʹ�ù�ʽ���㸩���ǣ����ȣ�
//        pitch_rad = atan2(ax_g, sqrt(ay_g * ay_g + az_g * az_g)) * 100;




    }

}


    //�Ӽ��ٲ��Դ���
//        delayMs(2);
//        if(dir)
//        {
//            pwmval++;//dir==1 led0pwmval����
//        }
//		else
//        {
//            pwmval--;	//dir==0 led0pwmval�ݼ� 
//        }            
// 		if(pwmval>15)
//        {
//            dir=0;//led0pwmval����300�󣬷���Ϊ�ݼ�
//        }
//		if(pwmval==0)
//        {
//            dir=1;	//led0pwmval�ݼ���0�󣬷����Ϊ����
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
//		TIM_SetCompare1(TIM3,pwmval);	//�޸ıȽ�ֵ���޸�ռ�ձ�
//		TIM_SetCompare2(TIM3,pwmval);	//�޸ıȽ�ֵ���޸�ռ�ձ�





