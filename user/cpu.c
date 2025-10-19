#include "cpu.h"
extern u16 p_msDelayValue;
extern u16 p_usDelayValue;

//ms延时
void delayMs(u16 value)
{
    p_msDelayValue = value;
    while(p_msDelayValue > 0);
}


//us延时
void delayUs(u16 value)
{
    p_usDelayValue = value;
    while(p_usDelayValue > 0);
}

//10us定时器
static void tim7Init(void)
{		 					 
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    
    TIM_TimeBaseStructure.TIM_Prescaler=84 - 1;  //定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period=10 - 1;   //自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM7,&TIM_TimeBaseStructure);//初始化定时器3
    
    TIM_ITConfig(TIM7,TIM_IT_Update,ENABLE); //允许定时器3更新中断
	TIM_Cmd(TIM7,ENABLE); //使能定时器3
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM7_IRQn; //定时器3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01; //抢占优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x03; //子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}  
//1us定时器
static void tim6Init(void)
{		 					 
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    
    TIM_TimeBaseStructure.TIM_Prescaler=8 - 1;  //定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period=10 - 1;   //自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM6,&TIM_TimeBaseStructure);//初始化定时器3
    
    TIM_ITConfig(TIM6,TIM_IT_Update,ENABLE); //允许定时器3更新中断
	TIM_Cmd(TIM6,ENABLE); //使能定时器3
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM6_DAC_IRQn; 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01; //抢占优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x03; //子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

//10K频率电机PWM
static void motorPwmInit(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
    
    TIM_TimeBaseStructure.TIM_Period=2100 - 1;   //自动重装载值
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Prescaler=2 - 1;  //定时器分频
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);//初始化定时器3
	
	//初始化TIM3 PWM模式	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性低
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM1 4OC1

	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);  //使能TIM3在CCR1上的预装载寄存器
	
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM1 4OC1

	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);  //使能TIM3在CCR1上的预装载寄存器
 
  TIM_ARRPreloadConfig(TIM3,ENABLE);//ARPE使能 
	
	TIM_Cmd(TIM3, ENABLE);  //使能TIM3
    
}
//编码器初始化
//TIM1 TIM5
static void encoderInit(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_ICInitTypeDef TIM_ICInitStruct;

    
    TIM_TimeBaseStructure.TIM_Prescaler=0;               
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; 
	TIM_TimeBaseStructure.TIM_Period=0xffff;   
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseStructure);//初始化定时器1
    
    TIM_TimeBaseStructure.TIM_Prescaler=0;  
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; 
	TIM_TimeBaseStructure.TIM_Period=0xffff;   
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM5,&TIM_TimeBaseStructure);//初始化定时器5
    
    TIM_EncoderInterfaceConfig(TIM1, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    TIM_EncoderInterfaceConfig(TIM5, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    
    TIM_ICInitStruct.TIM_Channel = TIM_Channel_1;
    TIM_ICInitStruct.TIM_ICPolarity = TIM_ICPolarity_Rising;   // 上升沿有效
    TIM_ICInitStruct.TIM_ICSelection = TIM_ICSelection_DirectTI; // 直接映射到TI1
    TIM_ICInitStruct.TIM_ICPrescaler = TIM_ICPSC_DIV1;        // 不分频
    TIM_ICInitStruct.TIM_ICFilter = 0x6;                      // 设置滤波器
    TIM_ICInit(TIM5, &TIM_ICInitStruct);

    // **新增：配置通道2**
    TIM_ICInitStruct.TIM_Channel = TIM_Channel_2;
    TIM_ICInitStruct.TIM_ICSelection = TIM_ICSelection_IndirectTI; // 映射到TI2
    // 极性、分频器和滤波器设置通常与通道1一致
    TIM_ICInitStruct.TIM_ICPolarity = TIM_ICPolarity_Rising;
    TIM_ICInitStruct.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStruct.TIM_ICFilter = 0x6;
    TIM_ICInit(TIM5, &TIM_ICInitStruct);

    // 6. 使能定时器计数器
    TIM_Cmd(TIM5, ENABLE);
    
    // 6. 输入捕获滤波器配置（根据信号质量调整）
    TIM_ICInitStruct.TIM_Channel = TIM_Channel_1;
    TIM_ICInitStruct.TIM_ICPolarity = TIM_ICPolarity_Rising;   // 捕获上升沿
    TIM_ICInitStruct.TIM_ICSelection = TIM_ICSelection_DirectTI; // 映射到TI1
    TIM_ICInitStruct.TIM_ICPrescaler = TIM_ICPSC_DIV1;        // 每个事件都捕获
    TIM_ICInitStruct.TIM_ICFilter = 0x6;                      // 设置滤波器，根据信号质量调整，0x6是一个常用值
    TIM_ICInit(TIM1, &TIM_ICInitStruct);
    // 配置通道2
    TIM_ICInitStruct.TIM_Channel = TIM_Channel_2;
    TIM_ICInitStruct.TIM_ICSelection = TIM_ICSelection_IndirectTI; // 映射到TI2
    TIM_ICInit(TIM1, &TIM_ICInitStruct);

    
    

    
//    TIM_ITConfig(TIM1,TIM_IT_Update,ENABLE); //允许定时器3更新中断
//    TIM_ITConfig(TIM5,TIM_IT_Update,ENABLE); //允许定时器3更新中断
    TIM_Cmd(TIM1, ENABLE);

//	NVIC_InitStructure.NVIC_IRQChannel=TIM1_UP_TIM10_IRQn; //定时器3中断
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01; //抢占优先级1
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x03; //子优先级3
//	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
    
//    NVIC_InitStructure.NVIC_IRQChannel=TIM5_IRQn; //定时器3中断
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01; //抢占优先级1
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x03; //子优先级3
//	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
//	NVIC_Init(&NVIC_InitStructure);


    TIM_CtrlPWMOutputs(TIM1, ENABLE);
}


//A0 A1 编码器
//A8 A9 编码器
//C4 C5 a电机方向
//B0 B1 b电机方向
//A6 A7 PWM引脚
//e7 e8 MPU6500 IIC
//PB6 PB7
static void gpioInit(void )
{
    GPIO_InitTypeDef GPIO_InitStructure;    
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;          //PWM
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;                    
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	            
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                  
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;                    
	GPIO_Init(GPIOA,&GPIO_InitStructure);                           
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;           //电机方向
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;                    
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;               
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                   
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;                     
	GPIO_Init(GPIOC,&GPIO_InitStructure);                            
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;           //电机方向
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;                   
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	                
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                  
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;                    
	GPIO_Init(GPIOB,&GPIO_InitStructure);     

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 ;                      //iic  scl
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;                   
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	                
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                  
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;                    
	GPIO_Init(GPIOE,&GPIO_InitStructure);   

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 ;                      //iic sda
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;                   
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	                
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;                  
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;                    
	GPIO_Init(GPIOE,&GPIO_InitStructure); 
    mpuI2cScl1();
    mpuI2cSda1();
    
     // 2. 配置GPIO：PA0(TIM5_CH1), PA1(TIM5_CH2)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        // 复用功能
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        // 上拉电阻
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 3. 引脚复用映射
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_TIM1);
     // 3. 引脚复用映射
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM5);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM5);
    
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource6,GPIO_AF_TIM3); //GPIOA6复用为定时器3
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_TIM3); //GPIOA6复用为定时器3
}

//定时器3 通道1  a6
//定时器3 通道2  a7
static void periphClockInit(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7,ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6,ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);  	//TIM3时钟使能   
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 	//使能PORTA时钟	
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); 	//使能PORTA时钟	
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); 	//使能PORTA时钟	
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE); 	//使能PORTA时钟	
	
	
}

//MPU6500 IIC 起始信号
void iicStart(void)
{
    mpuI2cSda1();
    mpuI2cScl1();
    delayUs(4);
    mpuI2cSda0();
    delayUs(4);
    mpuI2cScl0();
    delayUs(4);
}
//MPU6500 IIC 停止信号
void iicStop(void)
{
    mpuI2cSda0();
    mpuI2cScl1();
    delayUs(4);
    mpuI2cSda1();
}
//MPU6500  iic写一个字节
void iicSendOneByte(uint8_t data)
{
    u8 i = 0;
    //发送字节的高7位
    for ( i = 0; i < 8; i++)
	{		
		if (data & 0x80)
		{
			mpuI2cSda1();
		}
		else
		{
			mpuI2cSda0();
		}
		delayUs(8);
		mpuI2cScl1();
		delayUs(8);
		mpuI2cScl0();
		if (i == 7)
		{
			 mpuI2cSda1(); // 释放总线
		}
		data <<= 1;	/* 左移一个bit */
		delayUs(8);
	}
}
//MPU6500  IIC 读一个字节
u8 iicReadOneByte(void)
{
    uint8_t i;
	uint8_t value;

	/* 读到第1个bit为数据的bit7 */
	value = 0;
	for (i = 0; i < 8; i++)
	{
		value <<= 1;
		mpuI2cScl1();
		delayUs(8);
		if (mpuI2cSdaRead())
		{
			value++;
		}
		mpuI2cScl0();
		delayUs(8);
	}
	return value;
}
//mpu6500 iic等待应答信号
u8 iicWaitAck(void)
{
    uint8_t data;
    
    mpuI2cSda1();
    delayUs(8);
    mpuI2cScl1();
    delayUs(8);
    if (mpuI2cSdaRead())	/* CPU读取SDA口线状态 */
	{
		data = 1;
	}
	else
	{
		data = 0;
	}
    mpuI2cScl0();
    delayUs(8);
    
    return data;
}
//mpu6500 iic产生一个应答信号
void iicAck(void)
{
    mpuI2cSda0();
    delayUs(8);
    mpuI2cScl1();
    delayUs(8);
    mpuI2cScl0();
    delayUs(8);
    mpuI2cSda1();
}

//mpu6500 iic产生一个非应答信号
void iicNAck(void)
{
    mpuI2cSda1();
    delayUs(4);
    mpuI2cScl1();
    delayUs(4);
    mpuI2cScl0();
    delayUs(4);
}
//检测MPU6500是否存在 iic读取地址
uint8_t i2c_CheckDevice(uint8_t _Address)
{
    u8 temp;
    
    iicStart();
    iicSendOneByte(_Address | iicW);
    
    temp = iicWaitAck();
    iicStop();
    
    return temp;
    
}
//mpu6500 iic写入数据
u32 mpuWrite(u8 reg_add,u8 reg_dat)
{ 
	/*　第0步：发停止信号，启动内部写操作　*/
	iicStop();
	/* 第1步：发起I2C总线启动信号 */
	iicStart();
	/* 第2步：发起控制字节，高7bit是地址，bit0是读写控制位，0表示写，1表示读 */
	iicSendOneByte(MPU6050iicADDR | iicW);	/* 此处是写指令 */
	/* 第3步：发送一个时钟，判断器件是否正确应答 */
	if (iicWaitAck() != 0)
	{
	  goto cmd_fail;	/* EEPROM器件写超时 */
	}
	/* 第4步：开始写入寄存器地址 */
	iicSendOneByte(reg_add);

	/* 第5步：检查ACK */
	if (iicWaitAck() != 0)
	{
		goto cmd_fail;	/* 器件无应答 */
	}
	/* 第5步：开始写入数据 */
	iicSendOneByte(reg_dat);

	/* 第5步：检查ACK */
	if (iicWaitAck() != 0)
	{
		goto cmd_fail;	/* 器件无应答 */
	}	
	
	/* 命令执行成功，发送I2C总线停止信号 */
	iicStop();
	return 0;

cmd_fail: /* 命令执行失败后，切记发送停止信号，避免影响I2C总线上其他设备 */
	/* 发送I2C总线停止信号 */
	iicStop();
	return 1;
}

//mpu6500 iic读取数据
u32 mpuRead(u8 reg_add,unsigned char* Read,u8 num)
{
    /* 第0步：发停止信号，启动内部写操作　*/
	iicStop();
	/* 第1步：发起I2C总线启动信号 */
	iicStart();
	/* 第2步：发起控制字节，高7bit是地址，bit0是读写控制位，0表示写，1表示读 */
	iicSendOneByte(MPU6050iicADDR | iicW);	/* 此处是写指令 */
	/* 第3步：发送一个时钟，判断器件是否正确应答 */
	if (iicWaitAck() != 0)
	{
	  goto cmd_fail;	/* EEPROM器件写超时 */
	}
	/* 第4步：开始写入寄存器地址 */
	iicSendOneByte(reg_add);
	/* 第5步：检查ACK */
	if (iicWaitAck() != 0)
	{
		goto cmd_fail;	/* 器件无应答 */
	}
    /* 发送I2C总线停止信号 */
    iicStop();
	
    /* 第6步：发起I2C总线启动信号 */
	iicStart();
	
	/* 第7步：发起控制字节，高7bit是地址，bit0是读写控制位，0表示写，1表示读 */
	iicSendOneByte(MPU6050iicADDR | iicR);	/* 此处是读指令 */

	/* 第8步：检查ACK */
	if (iicWaitAck() != 0)
	{
		goto cmd_fail;	/* 器件无应答 */
	}
	while(num) 
   {

		*Read = iicReadOneByte();
    
		/* 读指针自增 */
		Read++; 
      
		if(num == 1)
		{
			iicNAck();	/* 最后1个字节读完后，CPU产生NACK信号(驱动SDA = 1) */
		}
		else
		{
			iicAck();	/* 中间字节读完后，CPU产生ACK信号(驱动SDA = 0) */  
		}				
		/*计数器自减 */
		num--;
  }

	/* 发送I2C总线停止信号 */
	iicStop();
	return 0;	/* 执行成功 */

cmd_fail: /* 命令执行失败后，切记发送停止信号，避免影响I2C总线上其他设备 */
	/* 发送I2C总线停止信号 */
	iicStop();
	return 1;
}
//mpu6500  初始化
static void mpu6500Init(void)
{
//    mpuWrite(MPU6050_RA_PWR_MGMT_1 , 0x80);     //复位6500
    delayMs(100);//延时100ms
    mpuWrite(MPU6050_RA_PWR_MGMT_1 , 0x00);     //解除休眠状态
    delayMs(100);//延时100ms
    mpuWrite(MPU6050_RA_SMPLRT_DIV , 0x01);     //设置陀螺仪采样率为500hz
//    mpuWrite(MPU6050_RA_CONFIG , 0x00);	
//    mpuWrite(MPU6050_RA_CONFIG , 0x06);	        //5hz低通滤波  
    mpuWrite(MPU6050_RA_ACCEL_CONFIG , 0x08);	//加速度计量程+-4g  灵敏度为2^16/8 = 8192(+-4所以除8)
    mpuWrite(MPU6050_RA_GYRO_CONFIG, 0x08);     //陀螺仪自检及测量范围，(不自检，陀螺仪量程+-500度/s)灵敏度为 2^16 / 1000 = 65.5
    delayUs(50);//延时50us
}
//读mpu6500的ID 验证IIC通讯是否成功
uint8_t MPU6050ReadID(void)
{
	unsigned char Re = 0;
    mpuRead(MPU6050_RA_WHO_AM_I,&Re,1);    //读器件地址
	if(Re != 0x70)
	{
//		MPU_ERROR("MPU6050 dectected error!\r\n检测不到MPU6050模块，请检查模块与开发板的接线");
		return 0;
	}
	else
	{
//		MPU_INFO("MPU6050 ID = %d\r\n",Re);
		return 1;
	}
		
}
//读加速度
void MPU6500ReadAcc(short *accData)
{
    uint8_t buf[6];
    mpuRead(MPU6050_ACC_OUT, buf, 6);
    accData[0] = (buf[0] << 8) | buf[1];
    accData[1] = (buf[2] << 8) | buf[3];
    accData[2] = (buf[4] << 8) | buf[5];
}
//读角加速度
void MPU6050ReadGyro(short *gyroData)
{
    uint8_t buf[6];
    mpuRead(MPU6050_GYRO_OUT,buf,6);
    gyroData[0] = (buf[0] << 8) | buf[1];
    gyroData[1] = (buf[2] << 8) | buf[3];
    gyroData[2] = (buf[4] << 8) | buf[5];
}
//读温度 转换为摄氏度
void MPU6050_ReturnTemp(float *Temperature)
{
	short temp3;
	uint8_t buf[2];
	
	mpuRead(MPU6050_RA_TEMP_OUT_H,buf,2);     //读取温度值
    temp3= (buf[0] << 8) | buf[1];	
	*Temperature=((double) temp3/340.0)+36.53;

}

/**
  * @brief  写入字节
  * @param   
  */
uint8_t Sensor_write_DMP(uint8_t slave_addr,uint8_t reg_add,uint8_t num,unsigned char* write)
{
	uint8_t i; 
	 /* 第1步：发起I2C总线启动信号 */
	iicStart(); 
	/* 第2步：发起控制字节，高7bit是地址，bit0是读写控制位，0表示写，1表示读 */
	iicSendOneByte((slave_addr<<1)|iicW);//发送器件地址+写命令	
	/* 第3步：发送一个时钟，判断器件是否正确应答 */
	if(iicWaitAck())	
	{
		/* 发送I2C总线停止信号 */
		iicStop();		 
		return 1;		
	}

	iicSendOneByte(reg_add);	
	/*等待ACK */
	iicWaitAck();		
	for(i=0;i<num;i++)
	{
		/* 第4步：开始写入数据 */
		iicSendOneByte(write[i]);	
		/* 第5步：检查ACK */
		if(iicWaitAck())	
		{
			/* 发送I2C总线停止信号 */
			iicStop();	 
			return 1;		 
		}		
	}    
	/* 发送I2C总线停止信号 */
	iicStop();	 
	return 0;	
} 
/**
  * @brief   读取6050寄存器
  */
uint8_t Sensor_Read_DMP(uint8_t slave_addr,uint8_t reg_add,uint8_t num,unsigned char* Read)
{ 
	/* 第1步：发起I2C总线启动信号 */
	iicStart(); 
	/* 第2步：发起控制字节，高7bit是地址，bit0是读写控制位，0表示写，1表示读 */
	iicSendOneByte((slave_addr<<1)|iicW);//发送器件地址+写命令	
	/* 第3步：发送一个时钟，判断器件是否正确应答 */
	if(iicWaitAck())	//等待应答
	{
		iicStop();		 
		return 1;		
	}
	/* 第4步：开始写入寄存器地址 */
	iicSendOneByte(reg_add);	
	/* 第5步：等待ACK */
	iicWaitAck();		//等待应答
  /* 第6步：发起I2C总线启动信号 */
	iicStart();
	/*  发起I2C总线启动信号  */
	iicSendOneByte((slave_addr<<1)|iicR);
	iicWaitAck();	
	while(num)
	{
		*Read=iicReadOneByte();	
        if(num == 1)
		{
			iicNAck();	/* 最后1个字节读完后，CPU产生NACK信号(驱动SDA = 1) */
		}
		else
		{
			iicAck();	/* 中间字节读完后，CPU产生ACK信号(驱动SDA = 0) */  
		}
		/*计数器自减 */
		num --;
		/* 读指针自增 */
		Read++; 
	}    
	/* 发送I2C总线停止信号 */
	iicStop();	
	return 0;	
}

// 清除编码器计数值
static void Encoder_Clear_Count(void)
{
    TIM1->CNT = 0;
}

static void Encoder_Clear_Count1(void)
{
    TIM5->CNT = 0;
}
extern u8 success;
void cpuInit(void)
{
    periphClockInit();
    
    gpioInit();
    tim7Init();
    tim6Init();
    encoderInit();
    // 清除编码器计数值
    Encoder_Clear_Count1();
    Encoder_Clear_Count();
    motorPwmInit();
    mpu6500Init();
    if (MPU6050ReadID() == 1)
    {
        success = 1;
    }
    else
    {
        success = 0;
    }
}




