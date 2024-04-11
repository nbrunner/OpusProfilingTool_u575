/**
  ******************************************************************************
  * @file    opt_cmd_handlers.h
  * @author  SRA 
  * @brief   Header for opt_cmd_handlers.c module
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __OPT_CMD_HANDLERS
#define __OPT_CMD_HANDLERS

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "STCmdP.h"

/* Exported types ------------------------------------------------------------*/  
typedef enum
{
  OPT_SUCCESS = 1,                           
  OPT_ERROR = 0                          
} OPT_StatusTypeDef;

/* Exported constants --------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
OPT_StatusTypeDef Handle_CMD_InitOpusEncoder(TMsg *Msg);
OPT_StatusTypeDef Handle_CMD_InitOpusDecoder(TMsg *Msg);
OPT_StatusTypeDef Handle_CMD_ResetOpus(TMsg *Msg);
OPT_StatusTypeDef Handle_CMD_EncDec_NoFile_Request(TMsg *Msg);
OPT_StatusTypeDef Handle_CMD_EncDec_File_Request(TMsg *Msg);

#ifdef __cplusplus
}
#endif

#endif /* __OPT_CMD_HANDLERS */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
