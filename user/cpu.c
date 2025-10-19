#include "cpu.h"
extern u16 p_msDelayValue;
extern u16 p_usDelayValue;

//ms��ʱ
void delayMs(u16 value)
{
    p_msDelayValue = value;
    while(p_msDelayValue > 0);
}


//us��ʱ
void delayUs(u16 value)
{
    p_usDelayValue = value;
    while(p_usDelayValue > 0);
}

//10us��ʱ��
static void tim7Init(void)
{		 					 
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    
    TIM_TimeBaseStructure.TIM_Prescaler=84 - 1;  //��ʱ����Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_Period=10 - 1;   //�Զ���װ��ֵ
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM7,&TIM_TimeBaseStructure);//��ʼ����ʱ��3
    
    TIM_ITConfig(TIM7,TIM_IT_Update,ENABLE); //����ʱ��3�����ж�
	TIM_Cmd(TIM7,ENABLE); //ʹ�ܶ�ʱ��3
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM7_IRQn; //��ʱ��3�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01; //��ռ���ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x03; //�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}  
//1us��ʱ��
static void tim6Init(void)
{		 					 
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    
    TIM_TimeBaseStructure.TIM_Prescaler=8 - 1;  //��ʱ����Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_Period=10 - 1;   //�Զ���װ��ֵ
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM6,&TIM_TimeBaseStructure);//��ʼ����ʱ��3
    
    TIM_ITConfig(TIM6,TIM_IT_Update,ENABLE); //����ʱ��3�����ж�
	TIM_Cmd(TIM6,ENABLE); //ʹ�ܶ�ʱ��3
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM6_DAC_IRQn; 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01; //��ռ���ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x03; //�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

//10KƵ�ʵ��PWM
static void motorPwmInit(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
    
    TIM_TimeBaseStructure.TIM_Period=2100 - 1;   //�Զ���װ��ֵ
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_Prescaler=2 - 1;  //��ʱ����Ƶ
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);//��ʼ����ʱ��3
	
	//��ʼ��TIM3 PWMģʽ	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //�������:TIM����Ƚϼ��Ե�
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM1 4OC1

	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);  //ʹ��TIM3��CCR1�ϵ�Ԥװ�ؼĴ���
	
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM1 4OC1

	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);  //ʹ��TIM3��CCR1�ϵ�Ԥװ�ؼĴ���
 
  TIM_ARRPreloadConfig(TIM3,ENABLE);//ARPEʹ�� 
	
	TIM_Cmd(TIM3, ENABLE);  //ʹ��TIM3
    
}
//��������ʼ��
//TIM1 TIM5
static void encoderInit(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_ICInitTypeDef TIM_ICInitStruct;

    
    TIM_TimeBaseStructure.TIM_Prescaler=0;               
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; 
	TIM_TimeBaseStructure.TIM_Period=0xffff;   
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseStructure);//��ʼ����ʱ��1
    
    TIM_TimeBaseStructure.TIM_Prescaler=0;  
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; 
	TIM_TimeBaseStructure.TIM_Period=0xffff;   
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM5,&TIM_TimeBaseStructure);//��ʼ����ʱ��5
    
    TIM_EncoderInterfaceConfig(TIM1, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    TIM_EncoderInterfaceConfig(TIM5, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    
    TIM_ICInitStruct.TIM_Channel = TIM_Channel_1;
    TIM_ICInitStruct.TIM_ICPolarity = TIM_ICPolarity_Rising;   // ��������Ч
    TIM_ICInitStruct.TIM_ICSelection = TIM_ICSelection_DirectTI; // ֱ��ӳ�䵽TI1
    TIM_ICInitStruct.TIM_ICPrescaler = TIM_ICPSC_DIV1;        // ����Ƶ
    TIM_ICInitStruct.TIM_ICFilter = 0x6;                      // �����˲���
    TIM_ICInit(TIM5, &TIM_ICInitStruct);

    // **����������ͨ��2**
    TIM_ICInitStruct.TIM_Channel = TIM_Channel_2;
    TIM_ICInitStruct.TIM_ICSelection = TIM_ICSelection_IndirectTI; // ӳ�䵽TI2
    // ���ԡ���Ƶ�����˲�������ͨ����ͨ��1һ��
    TIM_ICInitStruct.TIM_ICPolarity = TIM_ICPolarity_Rising;
    TIM_ICInitStruct.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStruct.TIM_ICFilter = 0x6;
    TIM_ICInit(TIM5, &TIM_ICInitStruct);

    // 6. ʹ�ܶ�ʱ��������
    TIM_Cmd(TIM5, ENABLE);
    
    // 6. ���벶���˲������ã������ź�����������
    TIM_ICInitStruct.TIM_Channel = TIM_Channel_1;
    TIM_ICInitStruct.TIM_ICPolarity = TIM_ICPolarity_Rising;   // ����������
    TIM_ICInitStruct.TIM_ICSelection = TIM_ICSelection_DirectTI; // ӳ�䵽TI1
    TIM_ICInitStruct.TIM_ICPrescaler = TIM_ICPSC_DIV1;        // ÿ���¼�������
    TIM_ICInitStruct.TIM_ICFilter = 0x6;                      // �����˲����������ź�����������0x6��һ������ֵ
    TIM_ICInit(TIM1, &TIM_ICInitStruct);
    // ����ͨ��2
    TIM_ICInitStruct.TIM_Channel = TIM_Channel_2;
    TIM_ICInitStruct.TIM_ICSelection = TIM_ICSelection_IndirectTI; // ӳ�䵽TI2
    TIM_ICInit(TIM1, &TIM_ICInitStruct);

    
    

    
//    TIM_ITConfig(TIM1,TIM_IT_Update,ENABLE); //����ʱ��3�����ж�
//    TIM_ITConfig(TIM5,TIM_IT_Update,ENABLE); //����ʱ��3�����ж�
    TIM_Cmd(TIM1, ENABLE);

//	NVIC_InitStructure.NVIC_IRQChannel=TIM1_UP_TIM10_IRQn; //��ʱ��3�ж�
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01; //��ռ���ȼ�1
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x03; //�����ȼ�3
//	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
    
//    NVIC_InitStructure.NVIC_IRQChannel=TIM5_IRQn; //��ʱ��3�ж�
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01; //��ռ���ȼ�1
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x03; //�����ȼ�3
//	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
//	NVIC_Init(&NVIC_InitStructure);


    TIM_CtrlPWMOutputs(TIM1, ENABLE);
}


//A0 A1 ������
//A8 A9 ������
//C4 C5 a�������
//B0 B1 b�������
//A6 A7 PWM����
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
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;           //�������
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;                    
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;               
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                   
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;                     
	GPIO_Init(GPIOC,&GPIO_InitStructure);                            
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;           //�������
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
    
     // 2. ����GPIO��PA0(TIM5_CH1), PA1(TIM5_CH2)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        // ���ù���
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        // ��������
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 3. ���Ÿ���ӳ��
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_TIM1);
     // 3. ���Ÿ���ӳ��
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM5);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM5);
    
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource6,GPIO_AF_TIM3); //GPIOA6����Ϊ��ʱ��3
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_TIM3); //GPIOA6����Ϊ��ʱ��3
}

//��ʱ��3 ͨ��1  a6
//��ʱ��3 ͨ��2  a7
static void periphClockInit(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7,ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6,ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);  	//TIM3ʱ��ʹ��   
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 	//ʹ��PORTAʱ��	
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); 	//ʹ��PORTAʱ��	
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); 	//ʹ��PORTAʱ��	
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE); 	//ʹ��PORTAʱ��	
	
	
}

//MPU6500 IIC ��ʼ�ź�
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
//MPU6500 IIC ֹͣ�ź�
void iicStop(void)
{
    mpuI2cSda0();
    mpuI2cScl1();
    delayUs(4);
    mpuI2cSda1();
}
//MPU6500  iicдһ���ֽ�
void iicSendOneByte(uint8_t data)
{
    u8 i = 0;
    //�����ֽڵĸ�7λ
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
			 mpuI2cSda1(); // �ͷ�����
		}
		data <<= 1;	/* ����һ��bit */
		delayUs(8);
	}
}
//MPU6500  IIC ��һ���ֽ�
u8 iicReadOneByte(void)
{
    uint8_t i;
	uint8_t value;

	/* ������1��bitΪ���ݵ�bit7 */
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
//mpu6500 iic�ȴ�Ӧ���ź�
u8 iicWaitAck(void)
{
    uint8_t data;
    
    mpuI2cSda1();
    delayUs(8);
    mpuI2cScl1();
    delayUs(8);
    if (mpuI2cSdaRead())	/* CPU��ȡSDA����״̬ */
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
//mpu6500 iic����һ��Ӧ���ź�
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

//mpu6500 iic����һ����Ӧ���ź�
void iicNAck(void)
{
    mpuI2cSda1();
    delayUs(4);
    mpuI2cScl1();
    delayUs(4);
    mpuI2cScl0();
    delayUs(4);
}
//���MPU6500�Ƿ���� iic��ȡ��ַ
uint8_t i2c_CheckDevice(uint8_t _Address)
{
    u8 temp;
    
    iicStart();
    iicSendOneByte(_Address | iicW);
    
    temp = iicWaitAck();
    iicStop();
    
    return temp;
    
}
//mpu6500 iicд������
u32 mpuWrite(u8 reg_add,u8 reg_dat)
{ 
	/*����0������ֹͣ�źţ������ڲ�д������*/
	iicStop();
	/* ��1��������I2C���������ź� */
	iicStart();
	/* ��2������������ֽڣ���7bit�ǵ�ַ��bit0�Ƕ�д����λ��0��ʾд��1��ʾ�� */
	iicSendOneByte(MPU6050iicADDR | iicW);	/* �˴���дָ�� */
	/* ��3��������һ��ʱ�ӣ��ж������Ƿ���ȷӦ�� */
	if (iicWaitAck() != 0)
	{
	  goto cmd_fail;	/* EEPROM����д��ʱ */
	}
	/* ��4������ʼд��Ĵ�����ַ */
	iicSendOneByte(reg_add);

	/* ��5�������ACK */
	if (iicWaitAck() != 0)
	{
		goto cmd_fail;	/* ������Ӧ�� */
	}
	/* ��5������ʼд������ */
	iicSendOneByte(reg_dat);

	/* ��5�������ACK */
	if (iicWaitAck() != 0)
	{
		goto cmd_fail;	/* ������Ӧ�� */
	}	
	
	/* ����ִ�гɹ�������I2C����ֹͣ�ź� */
	iicStop();
	return 0;

cmd_fail: /* ����ִ��ʧ�ܺ��мǷ���ֹͣ�źţ�����Ӱ��I2C�����������豸 */
	/* ����I2C����ֹͣ�ź� */
	iicStop();
	return 1;
}

//mpu6500 iic��ȡ����
u32 mpuRead(u8 reg_add,unsigned char* Read,u8 num)
{
    /* ��0������ֹͣ�źţ������ڲ�д������*/
	iicStop();
	/* ��1��������I2C���������ź� */
	iicStart();
	/* ��2������������ֽڣ���7bit�ǵ�ַ��bit0�Ƕ�д����λ��0��ʾд��1��ʾ�� */
	iicSendOneByte(MPU6050iicADDR | iicW);	/* �˴���дָ�� */
	/* ��3��������һ��ʱ�ӣ��ж������Ƿ���ȷӦ�� */
	if (iicWaitAck() != 0)
	{
	  goto cmd_fail;	/* EEPROM����д��ʱ */
	}
	/* ��4������ʼд��Ĵ�����ַ */
	iicSendOneByte(reg_add);
	/* ��5�������ACK */
	if (iicWaitAck() != 0)
	{
		goto cmd_fail;	/* ������Ӧ�� */
	}
    /* ����I2C����ֹͣ�ź� */
    iicStop();
	
    /* ��6��������I2C���������ź� */
	iicStart();
	
	/* ��7������������ֽڣ���7bit�ǵ�ַ��bit0�Ƕ�д����λ��0��ʾд��1��ʾ�� */
	iicSendOneByte(MPU6050iicADDR | iicR);	/* �˴��Ƕ�ָ�� */

	/* ��8�������ACK */
	if (iicWaitAck() != 0)
	{
		goto cmd_fail;	/* ������Ӧ�� */
	}
	while(num) 
   {

		*Read = iicReadOneByte();
    
		/* ��ָ������ */
		Read++; 
      
		if(num == 1)
		{
			iicNAck();	/* ���1���ֽڶ����CPU����NACK�ź�(����SDA = 1) */
		}
		else
		{
			iicAck();	/* �м��ֽڶ����CPU����ACK�ź�(����SDA = 0) */  
		}				
		/*�������Լ� */
		num--;
  }

	/* ����I2C����ֹͣ�ź� */
	iicStop();
	return 0;	/* ִ�гɹ� */

cmd_fail: /* ����ִ��ʧ�ܺ��мǷ���ֹͣ�źţ�����Ӱ��I2C�����������豸 */
	/* ����I2C����ֹͣ�ź� */
	iicStop();
	return 1;
}
//mpu6500  ��ʼ��
static void mpu6500Init(void)
{
//    mpuWrite(MPU6050_RA_PWR_MGMT_1 , 0x80);     //��λ6500
    delayMs(100);//��ʱ100ms
    mpuWrite(MPU6050_RA_PWR_MGMT_1 , 0x00);     //�������״̬
    delayMs(100);//��ʱ100ms
    mpuWrite(MPU6050_RA_SMPLRT_DIV , 0x01);     //���������ǲ�����Ϊ500hz
//    mpuWrite(MPU6050_RA_CONFIG , 0x00);	
//    mpuWrite(MPU6050_RA_CONFIG , 0x06);	        //5hz��ͨ�˲�  
    mpuWrite(MPU6050_RA_ACCEL_CONFIG , 0x08);	//���ٶȼ�����+-4g  ������Ϊ2^16/8 = 8192(+-4���Գ�8)
    mpuWrite(MPU6050_RA_GYRO_CONFIG, 0x08);     //�������Լ켰������Χ��(���Լ죬����������+-500��/s)������Ϊ 2^16 / 1000 = 65.5
    delayUs(50);//��ʱ50us
}
//��mpu6500��ID ��֤IICͨѶ�Ƿ�ɹ�
uint8_t MPU6050ReadID(void)
{
	unsigned char Re = 0;
    mpuRead(MPU6050_RA_WHO_AM_I,&Re,1);    //��������ַ
	if(Re != 0x70)
	{
//		MPU_ERROR("MPU6050 dectected error!\r\n��ⲻ��MPU6050ģ�飬����ģ���뿪����Ľ���");
		return 0;
	}
	else
	{
//		MPU_INFO("MPU6050 ID = %d\r\n",Re);
		return 1;
	}
		
}
//�����ٶ�
void MPU6500ReadAcc(short *accData)
{
    uint8_t buf[6];
    mpuRead(MPU6050_ACC_OUT, buf, 6);
    accData[0] = (buf[0] << 8) | buf[1];
    accData[1] = (buf[2] << 8) | buf[3];
    accData[2] = (buf[4] << 8) | buf[5];
}
//���Ǽ��ٶ�
void MPU6050ReadGyro(short *gyroData)
{
    uint8_t buf[6];
    mpuRead(MPU6050_GYRO_OUT,buf,6);
    gyroData[0] = (buf[0] << 8) | buf[1];
    gyroData[1] = (buf[2] << 8) | buf[3];
    gyroData[2] = (buf[4] << 8) | buf[5];
}
//���¶� ת��Ϊ���϶�
void MPU6050_ReturnTemp(float *Temperature)
{
	short temp3;
	uint8_t buf[2];
	
	mpuRead(MPU6050_RA_TEMP_OUT_H,buf,2);     //��ȡ�¶�ֵ
    temp3= (buf[0] << 8) | buf[1];	
	*Temperature=((double) temp3/340.0)+36.53;

}

/**
  * @brief  д���ֽ�
  * @param   
  */
uint8_t Sensor_write_DMP(uint8_t slave_addr,uint8_t reg_add,uint8_t num,unsigned char* write)
{
	uint8_t i; 
	 /* ��1��������I2C���������ź� */
	iicStart(); 
	/* ��2������������ֽڣ���7bit�ǵ�ַ��bit0�Ƕ�д����λ��0��ʾд��1��ʾ�� */
	iicSendOneByte((slave_addr<<1)|iicW);//����������ַ+д����	
	/* ��3��������һ��ʱ�ӣ��ж������Ƿ���ȷӦ�� */
	if(iicWaitAck())	
	{
		/* ����I2C����ֹͣ�ź� */
		iicStop();		 
		return 1;		
	}

	iicSendOneByte(reg_add);	
	/*�ȴ�ACK */
	iicWaitAck();		
	for(i=0;i<num;i++)
	{
		/* ��4������ʼд������ */
		iicSendOneByte(write[i]);	
		/* ��5�������ACK */
		if(iicWaitAck())	
		{
			/* ����I2C����ֹͣ�ź� */
			iicStop();	 
			return 1;		 
		}		
	}    
	/* ����I2C����ֹͣ�ź� */
	iicStop();	 
	return 0;	
} 
/**
  * @brief   ��ȡ6050�Ĵ���
  */
uint8_t Sensor_Read_DMP(uint8_t slave_addr,uint8_t reg_add,uint8_t num,unsigned char* Read)
{ 
	/* ��1��������I2C���������ź� */
	iicStart(); 
	/* ��2������������ֽڣ���7bit�ǵ�ַ��bit0�Ƕ�д����λ��0��ʾд��1��ʾ�� */
	iicSendOneByte((slave_addr<<1)|iicW);//����������ַ+д����	
	/* ��3��������һ��ʱ�ӣ��ж������Ƿ���ȷӦ�� */
	if(iicWaitAck())	//�ȴ�Ӧ��
	{
		iicStop();		 
		return 1;		
	}
	/* ��4������ʼд��Ĵ�����ַ */
	iicSendOneByte(reg_add);	
	/* ��5�����ȴ�ACK */
	iicWaitAck();		//�ȴ�Ӧ��
  /* ��6��������I2C���������ź� */
	iicStart();
	/*  ����I2C���������ź�  */
	iicSendOneByte((slave_addr<<1)|iicR);
	iicWaitAck();	
	while(num)
	{
		*Read=iicReadOneByte();	
        if(num == 1)
		{
			iicNAck();	/* ���1���ֽڶ����CPU����NACK�ź�(����SDA = 1) */
		}
		else
		{
			iicAck();	/* �м��ֽڶ����CPU����ACK�ź�(����SDA = 0) */  
		}
		/*�������Լ� */
		num --;
		/* ��ָ������ */
		Read++; 
	}    
	/* ����I2C����ֹͣ�ź� */
	iicStop();	
	return 0;	
}

// �������������ֵ
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
    // �������������ֵ
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




