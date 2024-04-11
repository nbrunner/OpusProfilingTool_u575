/**
  ******************************************************************************
  * @file    stcmdp_manager.c
  * @author  SRA 
  * @brief   STCmdP Protocol manager.
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

/* Includes ------------------------------------------------------------------*/
#include "stcmdp_manager.h"
#include "uart_protocol_interface.h"
#include "stcmdp_opt_command.h"
#include "opt_cmd_handlers.h"

/** @addtogroup DATA_LOG
* @{
*/

/** @defgroup STCMDP_INTERPRETER STCMDP_INTERPRETER
* @{
*/

/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
#define HOST_ADDRESS            0x32
#define UART_INTERFACE          0x01
#define DEV_ADDR                50

/* Private macros ------------------------------------------------------------*/

/** @defgroup STCMDP_INTERPRETER_Private_Variables STCMDP_INTERPRETER_Private_Variables
* @{
*/
/* Private variables ---------------------------------------------------------*/
static uint8_t PresentationString[] = {"OPT|NucleoU575ZI_Q|1.0.0"};
volatile uint8_t DataStreamingDest = 0x01;
volatile uint8_t senderInterface = 0;
TMsg Msg;

/**
  * @}
  */

/* Global variables ----------------------------------------------------------*/
uint32_t led_toggle_count = 0;

/* Private function prototypes -----------------------------------------------*/
static uint8_t HandleMSG(TMsg *Msg);
static void BUILD_REPLY_HEADER(TMsg *Msg);
static void BUILD_NACK_HEADER(TMsg *Msg);
static void Generic_SendMsg(TMsg *Msg);
static uint8_t Generic_ReceivedMSG(TMsg *Msg);

/* Functions Definition ------------------------------------------------------*/
/**
 * @brief  Control if a new message is received and handle it.
 * @param  None.
 * @retval None.
 */
void STCmdP_manager(void)
{
  if (Generic_ReceivedMSG((TMsg*) &Msg))
  {
    if (Msg.Data[0] == HOST_ADDRESS)
    {
      HandleMSG((TMsg*) &Msg);
      led_toggle_count = 200;
    }
  }
}

/**
  * @brief  Build reply msg header
  * @param  Msg pointer to the reply msg
  * @retval None
  */
static void BUILD_REPLY_HEADER(TMsg *Msg)
{
  Msg->Data[0] = Msg->Data[1];
  Msg->Data[1] = DEV_ADDR;
  Msg->Data[2] += CMD_Reply_Add;
  Msg->Len = 3;
}

/**
  * @brief  Build reply failed msg header
  * @param  Msg pointer to the reply msg
  * @retval None
  */
static void BUILD_NACK_HEADER(TMsg *Msg)
{
  Msg->Data[0] = Msg->Data[1];
  Msg->Data[1] = DEV_ADDR;
  Msg->Data[2] += CMD_NACK;
  Msg->Len = 3;
}

/**
  * @brief  Send masg through selected interface
  * @param  Msg pointer to the sending msg
  * @retval None
  */
static void Generic_SendMsg(TMsg *Msg)
{
  switch(senderInterface)
  {
    case UART_INTERFACE:
      UART_SendMsg(Msg);
      break;
  }
}

/**
  * @brief  Check if new msg is been received on the selected interface
  * @param  Msg pointer for the new msg
  * @retval 1 if a new message is received 0 otherwise
  */
static uint8_t Generic_ReceivedMSG(TMsg *Msg)
{
  if (UART_ReceivedMSG(Msg)) 
  {
    senderInterface = UART_INTERFACE;
    return 1;
  } 
  return 0;
}

/**
  * @brief  Handle the received msg
  * @param  Msg pointer to the new msg
  * @retval Error code (1=OK, 0=Error)
  */
static uint8_t HandleMSG(TMsg *Msg)
{
  const uint8_t *p1;
  uint8_t *p2;
  uint32_t i;

  if (Msg->Len < 2) return 0;
  if (Msg->Data[0] != DEV_ADDR) return 0;
  switch (Msg->Data[2]) /** CMD **/
  {
    case CMD_Ping:
      if (Msg->Len != 3) return 0;
      BUILD_REPLY_HEADER(Msg);
      Msg->Len = 3;
      Generic_SendMsg(Msg);
      return 1;
      
    case CMD_Read_PresString:
      if (Msg->Len != 3) return 0;
      BUILD_REPLY_HEADER(Msg);
      p1 = PresentationString;
      p2 = &Msg->Data[3];
      i = 0; //
      while (*p1)
      {
        *p2 = *p1;
        p1++;
        p2++;
        i++;
      }
      *p2 = 0;
      i++;
      Msg->Len = 3 + i;
      Generic_SendMsg(Msg);
      return 1;
      
    case CMD_Init_Opus_Encoder:
      if (Msg->Len < 3) return 0;
      if(Handle_CMD_InitOpusEncoder(Msg) == OPT_SUCCESS)
      {
        BUILD_REPLY_HEADER(Msg);
      }
      else
      {
        BUILD_NACK_HEADER(Msg);
      }
      Generic_SendMsg(Msg);
      return 1;
      
    case CMD_Init_Opus_Decoder:
      if (Msg->Len < 3) return 0;
      if(Handle_CMD_InitOpusDecoder(Msg) == OPT_SUCCESS)
      {
        BUILD_REPLY_HEADER(Msg);
      }
      else
      {
        BUILD_NACK_HEADER(Msg);
      }
      Generic_SendMsg(Msg);
      return 1;   
      
    case CMD_Reset_Opus:
      if (Msg->Len < 3) return 0;
      if(Handle_CMD_ResetOpus(Msg) == OPT_SUCCESS)
      {
        BUILD_REPLY_HEADER(Msg);
      }
      else
      {
        BUILD_NACK_HEADER(Msg);
      }
      Generic_SendMsg(Msg);
      return 1;   
      
    case CMD_EncDec_NoFile_Request:     
      if (Msg->Len < 3) return 0;
      BUILD_REPLY_HEADER(Msg);
      if(Handle_CMD_EncDec_NoFile_Request(Msg) == OPT_ERROR)
      {
        BUILD_NACK_HEADER(Msg);
      } 
      Generic_SendMsg(Msg);
      return 1; 
      
    case CMD_EncDec_File_Request:     
      if (Msg->Len < 3) return 0;
      BUILD_REPLY_HEADER(Msg);
      if(Handle_CMD_EncDec_File_Request(Msg) == OPT_ERROR)
      {
        BUILD_NACK_HEADER(Msg);
      } 
      Generic_SendMsg(Msg);
      return 1; 

    case CMD_Reset:
      NVIC_SystemReset();
      break;
      
    default:
      return 0;
  }
}


/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
