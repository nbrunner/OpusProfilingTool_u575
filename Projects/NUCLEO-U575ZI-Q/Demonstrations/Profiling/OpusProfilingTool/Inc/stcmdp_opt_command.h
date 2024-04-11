/**
  ******************************************************************************
  * @file    stcmdp_opt_command.h
  * @author  SRA 
  * @brief   This file contains opt commands code
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
#ifndef __STCMDP_OPT_COMMAND_H
#define __STCMDP_OPT_COMMAND_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

/**********  GENERIC  CMD  (0x00 - 0x0F)  **********/
#define CMD_Ping                                        0x01
#define CMD_Read_PresString                             0x02
#define CMD_NACK                                        0x03
#define CMD_Reset                                       0x0F
#define CMD_Reply_Add                                   0x80
/****************************************************/

/*********** OPT  CMD  (0x3F - 0x4F)  ************/
#define CMD_Init_Opus_Encoder                           0x46
#define CMD_Init_Opus_Decoder                           0x47
#define CMD_Reset_Opus                                  0x48
#define CMD_EncDec_NoFile_Request                       0x49
#define CMD_EncDec_File_Request                         0x4A 
/****************************************************/

#ifdef __cplusplus
}
#endif

#endif /* __STCMDP_OPT_COMMAND_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
