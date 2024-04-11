/**
  ******************************************************************************
  * @file    uart_protocol_interface.h
  * @author  SRA
  * @brief   Header for uart_protocol_interface.c module
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0094, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0094
  *
  ******************************************************************************
  */


/* Define to prevent recursive inclusion ------------------------------------ */
#ifndef __UART_PROTOCOL_INTERFACE__H
#define __UART_PROTOCOL_INTERFACE__H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "cube_hal.h"
#include "STCmdP.h"
 
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void USARTConfig(void);  
void UART_SendMsg(TMsg *Msg);
int UART_ReceivedMSG(TMsg *Msg);


#ifdef __cplusplus
}
#endif

#endif /* __UART_PROTOCOL_INTERFACE__H */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
