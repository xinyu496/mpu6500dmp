/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Templates/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @version V1.4.0
  * @date    04-August-2014
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
 #include "cpu.h"

/** @addtogroup Template_Project
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
 
}

u16 second;
static u32 cnt100000;
u16 p_msDelayValue;
u16 p_usDelayValue;
static u16 cnt100;
u32 df = 0 , df1=0;

// 全局变量
volatile int32_t p_tim1_overflows = 0;  // TIM1溢出次数（用于扩展计数范围）
volatile int32_t p_tim5_overflows = 0;

void TIM5_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET) {
        if (TIM5->CR1 & TIM_CR1_DIR) {
            p_tim5_overflows--;
        } else {
            p_tim5_overflows++;
        }
        TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
    }
}
void TIM1_UP_TIM10_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET) {
        // 根据计数方向更新溢出计数[8](@ref)
        if (TIM1->CR1 & TIM_CR1_DIR) {  // 向下计数（反转）
            p_tim1_overflows--;
        } else {                         // 向上计数（正转）
            p_tim1_overflows++;
        }
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
    }
}

//1us延时
void TIM6_DAC_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM6, TIM_IT_Update) == SET) //溢出中断
    {
        if(p_usDelayValue > 0)
        {
            p_usDelayValue--;
            
        }
        df++;
        if(df == 1000000)
        {
            df = 0;
            df1++;
        }
        TIM_ClearITPendingBit(TIM6, TIM_IT_Update); //清除中断标志位
    }
}
//10us中断
static u16 cnt10ms = 0 , cnt50ms = 0;
u8 cnt10msFlag = 0 , cnt50msFlag = 0;
void TIM7_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM7, TIM_IT_Update) == SET) //溢出中断
	{
        cnt50ms++;
        if(cnt50ms == 1000)
        {
            cnt50ms = 0;
            cnt50msFlag = 1;
            
        }
        cnt10ms++;
        //if(cnt10ms == 1000)
        if(cnt10ms == 200)
        {
            cnt10ms = 0;
            cnt10msFlag = 1;
            Calculate_Speed_RPM();
            Calculate_Speed_RPM1();
        }
        cnt100++;
        if(cnt100 == 100)
        {
            cnt100 = 0;
            if(p_msDelayValue > 0)
            {
                p_msDelayValue--;
                
            }
        }
        
        
        cnt100000++;
        if(cnt100000 == 99999)
        {
            cnt100000 = 0;
            second++;
        }
	}
	TIM_ClearITPendingBit(TIM7, TIM_IT_Update); //清除中断标志位
}

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
