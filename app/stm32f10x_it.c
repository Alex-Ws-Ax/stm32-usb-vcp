/**
  ******************************************************************************
  * @file    GPIO/IOToggle/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.6.0
  * @date    20-September-2021
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and peripherals
  *          interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2011 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "usb_vcp.h"
#include "usb_istr.h"
/** @addtogroup STM32F10x_StdPeriph_Examples
  * @{
  */

/** @addtogroup GPIO_IOToggle
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
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
  * @brief  This function handles PendSV_Handler exception.
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

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
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

/**
  * @}
  */
/*******************************************************************************
* Function Name  : USB_IRQHandler
* Description    : This function handles USB Low Priority interrupts
*                  requests.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USB_LP_CAN1_RX0_IRQHandler(void)
{
  USB_Istr();
}

#ifdef VCP_RX_BY_DMA

/*******************************************************************************
* Function Name  : DMAChannel5_IRQHandler
* Description    : This function handles DMA Stream 5 interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void VCP_RX_DMA_IRQHandler(void)
{
  VCP_RX_DMA_Channel_ISR();
}

#endif

/*******************************************************************************
* Function Name  : EVAL_COM1_IRQHandler
* Description    : This function handles EVAL_COM1 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USART2_IRQHandler(void)
{
#ifdef VCP_RX_BY_DMA
  /* Reset TIM counter */
  TIM_SetCounter(TIM2, 0);
  
  /* If overrun condition occurs, clear the ORE flag and recover communication */
  if (USART_GetFlagStatus(EVAL_COM1, USART_FLAG_ORE) != RESET)
  {
    (void)USART_ReceiveData(EVAL_COM1);
  }
  if (USART_GetITStatus(EVAL_COM1, USART_IT_RXNE) != RESET)
  {
    /* Send the received data to the PC Host*/
    USART_To_USB_Send_Data();
  }
#else
  if (USART_GetITStatus(VCP_PORT, USART_IT_RXNE) != RESET)
  {
    /* Send the received data to the PC Host*/
    USART_To_USB_Send_Data();
  }

  /* If overrun condition occurs, clear the ORE flag and recover communication */
  if (USART_GetFlagStatus(VCP_PORT, USART_FLAG_ORE) != RESET)
  {
    (void)USART_ReceiveData(VCP_PORT);
  }
}
#endif /* VCP_RX_BY_DMA */

#ifdef VCP_RX_BY_DMA

#if defined (USE_STM3210E_EVAL) || defined (USE_STM3210B_EVAL)
/*******************************************************************************
* Function Name  : DMA1_Channel4_IRQHandler
* Description    : This function handles DMA1 Channel7 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DMA1_Channel4_IRQHandler(void)
{
  /* Enable the receive of data on EP3 */
  SetEPRxValid(ENDP3);
  DMA_ClearITPendingBit(DMA1_IT_TC4);
}
#else
/*******************************************************************************
* Function Name  : DMA1_Channel7_IRQHandler
* Description    : This function handles DMA1 Channel7 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DMA1_Channel7_IRQHandler(void)
{
  /* Enable the receive of data on EP3 */
  SetEPRxValid(ENDP3);
  DMA_ClearITPendingBit(DMA1_IT_TC7);
}
#endif
#endif /* VCP_RX_BY_DMA */

/*******************************************************************************
* Function Name  : USB_FS_WKUP_IRQHandler
* Description    : This function handles USB WakeUp interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/


void USBWakeUp_IRQHandler(void)
{
  EXTI_ClearITPendingBit(EXTI_Line18);
}
#ifdef VCP_RX_BY_DMA
/**
  * @brief  This function handles TIM2 global interrupt request.
  * @param  None
  * @retval None
  */
void TIM2_IRQHandler(void)
{
  if(bDeviceState == CONFIGURED)
  {
    /* Check the data to be sent through IN pipe */
    Handle_USBAsynchXfer();
  }
  TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
}
#endif /* VCP_RX_BY_DMA */
