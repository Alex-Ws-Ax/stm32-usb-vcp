#ifndef __USB_VCP_H_
#define __USB_VCP_H_

#include <stdint.h>
#include "usb_type.h"
#include "stm32f10x.h"


#define         ID1                 (0x1FFFF7E8)
#define         ID2                 (0x1FFFF7EC)
#define         ID3                 (0x1FFFF7F0)

#define         VCP_PORT            USART2
#define         VCP_PORT_IRQ        USART2_IRQn
#define         VCP_PORT_PERIPH     RCC_APB1Periph_USART2

//#define         VCP_RX_BY_DMA       1

#if VCP_RX_BY_DMA
#define         VCP_RX_DMA_CHANNEL              DMA1_Channel5
#define         VCP_RX_DMA_IRQ                  DMA1_Channel5_IRQn
#define         VCP_RX_DMA_IT_TC                DMA1_IT_TC5
#define         VCP_RX_DMA_IRQHandler           DMA1_Channel5_IRQHandler
#define         VCP_RX_DMA_FLAG_GL              DMA1_FLAG_GL5
#define         VCP_RX_DMA_FLAG_TC              DMA1_FLAG_TC5

#define         USARTx_DR_ADDRESS               0x40013804
#define         USARTx_TX_DMA_CHANNEL           DMA1_Channel4
#define         USARTx_TX_DMA_FLAG_TC           DMA1_FLAG_TC4
#define         USARTx_TX_DMA_FLAG_GL           DMA1_FLAG_GL4
#define         USARTx_TX_DMA_IRQ               DMA1_Channel4_IRQn
#define         DMAx_CLK                        RCC_AHBPeriph_DMA1
#endif

void Set_System(void);
void Set_USBClock(void);
void Enter_LowPowerMode(void);
void Leave_LowPowerMode(void);
void USB_Interrupts_Config(void);
void USB_Cable_Config(FunctionalState NewState);
void USART_Config_Default(void);
bool USART_Config(void);
void USB_To_USART_Send_Data(uint8_t *data_buffer, uint8_t Nb_bytes);
void USART_To_USB_Send_Data(void);
void Handle_USBAsynchXfer(void);
void Get_SerialNum(void);
void VCP_RX_DMA_Channel_ISR(void);
void VCP_SendRxBufPacketToUsb(void);
void VCP_Data_InISR(void);


void USB_PreInit(void);




#endif

