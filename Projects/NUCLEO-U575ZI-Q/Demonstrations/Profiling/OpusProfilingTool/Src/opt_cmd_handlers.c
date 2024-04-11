/**
  ******************************************************************************
  * @file    opt_cmd_handlers.c
  * @author  SRA
  * @brief   This file provides a set of firmware functions to handle
  *          OPT custom commands.
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
#include "opt_cmd_handlers.h"
#include "opus_interface.h"
#include "stdlib.h"
#include "string.h"  

/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
#define SILK_MODE       0x00
#define HYBRID_MODE     0x01
#define CELT_MODE       0x02

/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
ENC_Opus_ConfigTypeDef EncConfigOpus;   /*!< opus encode configuration.*/
DEC_Opus_ConfigTypeDef DecConfigOpus;   /*!< opus decode configuration.*/

/* Global variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/* Functions Definition ------------------------------------------------------*/
/**
 * @brief  Handle init opus encoder command.
 * @param  Message received.
 * @retval OPT_SUCCESS in case of success OPT_ERROR otherwise.
 */
OPT_StatusTypeDef Handle_CMD_InitOpusEncoder(TMsg *Msg)
{
  Opus_Status status;
  
  if(ENC_Opus_IsConfigured())
  {
    return OPT_ERROR;
  }
  
  EncConfigOpus.application = (uint16_t) STCmdP_Deserialize(&Msg->Data[3], 2);
  EncConfigOpus.bitrate = STCmdP_Deserialize(&Msg->Data[5], 4);
  EncConfigOpus.channels = Msg->Data[16];
  EncConfigOpus.complexity = Msg->Data[9];
  EncConfigOpus.ms_frame = (float)Msg->Data[10];
  EncConfigOpus.ms_frame += (float)(Msg->Data[11]/10.0f);
  EncConfigOpus.sample_freq = STCmdP_Deserialize(&Msg->Data[12], 4);
  
  uint32_t enc_size = ENC_Opus_getMemorySize(&EncConfigOpus);
  
  EncConfigOpus.pInternalMemory = (uint8_t *)malloc(enc_size);
  
  int opus_err;
  status = ENC_Opus_Init(&EncConfigOpus, &opus_err);
  
  if(status != OPUS_SUCCESS)
  {
    return OPT_ERROR;
  }
  
  if(Msg->Data[17] == 0x01)
  {
    status = ENC_Opus_Set_VBR();
    if(status != OPUS_SUCCESS)
    {
      return OPT_ERROR;
    }
  }
  else if(Msg->Data[17] == 0x00)
  {
    status = ENC_Opus_Set_CBR(); 
    if(status != OPUS_SUCCESS)
    {
      return OPT_ERROR;
    }
  }

  if( ((uint16_t)STCmdP_Deserialize(&Msg->Data[18], 2)) == MODE_SILK_ONLY)
  {
    status = ENC_Opus_Force_SILKmode();
    if(status != OPUS_SUCCESS)
    {
      return OPT_ERROR;
    }
  }
  else if(((uint16_t)STCmdP_Deserialize(&Msg->Data[18], 2)) == MODE_CELT_ONLY)
  {
    status = ENC_Opus_Force_CELTmode();
    if(status != OPUS_SUCCESS)
    {
      return OPT_ERROR;
    }
  }
  
  return OPT_SUCCESS;
}

/**
 * @brief  Handle init opus decoder command.
 * @param  Message received.
 * @retval OPT_SUCCESS in case of success OPT_ERROR otherwise.
 */
OPT_StatusTypeDef Handle_CMD_InitOpusDecoder(TMsg *Msg)
{
  Opus_Status status;
  
  if(DEC_Opus_IsConfigured())
  {
    return OPT_ERROR;
  }
    
  DecConfigOpus.channels = Msg->Data[9];
  DecConfigOpus.ms_frame = (float)Msg->Data[3];
  DecConfigOpus.ms_frame += (float)(Msg->Data[4]/10.0f);
  DecConfigOpus.sample_freq = STCmdP_Deserialize(&Msg->Data[5], 4);

  uint32_t dec_size = DEC_Opus_getMemorySize(&DecConfigOpus);

  DecConfigOpus.pInternalMemory = (uint8_t *)malloc(dec_size);
  
  int opus_err;
  status = DEC_Opus_Init(&DecConfigOpus, &opus_err);
  
  if(status != OPUS_SUCCESS)
  {
    return OPT_ERROR;
  }
  
  return OPT_SUCCESS;
}

/**
 * @brief  Handle destroy opus ecoder and decoder command.
 * @param  Message received.
 * @retval OPT_SUCCESS in case of success OPT_ERROR otherwise.
 */
OPT_StatusTypeDef Handle_CMD_ResetOpus(TMsg *Msg)
{
  if(ENC_Opus_IsConfigured())
  {   
    ENC_Opus_Deinit();
    free(EncConfigOpus.pInternalMemory);
    memset(&EncConfigOpus, 0, sizeof(EncConfigOpus));
  }
  if(DEC_Opus_IsConfigured())
  {
    DEC_Opus_Deinit();
    free(DecConfigOpus.pInternalMemory); 
    memset(&DecConfigOpus, 0, sizeof(DecConfigOpus));
  }
  
  return OPT_SUCCESS;
}

/**
 * @brief  Handle encode and decode request, the command response will contain
 *         encoded and decoded data and profiling results.
 * @param  Message received.
 * @retval OPT_SUCCESS in case of success OPT_ERROR otherwise.
 */
OPT_StatusTypeDef Handle_CMD_EncDec_File_Request(TMsg *Msg)
{
  uint32_t numEncByte = 0;
  uint32_t enc_clock_count = 0;
  uint32_t numDecShort = 0;
  uint32_t dec_clock_count = 0;
  uint8_t toc_byte = 0;
  uint8_t mode = 0;

  /******** Encoder profiling ********/
  
  /* Enable trace and debug block */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

  __disable_irq();
  
  /* reset the counter */
  DWT->CYCCNT = 0;       
 
  /* enable the counter */
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; 
   
  numEncByte = ENC_Opus_Encode(&Msg->Data[3], EncConfigOpus.pInternalMemory);

  /* disable the counter */
  DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk; 
  
  enc_clock_count = DWT->CYCCNT;  
  
  __enable_irq();
  
  /***********************************/
  
  /* "RFC6716 - Definition of the Opus Audio Codec" for more details */
  toc_byte = EncConfigOpus.pInternalMemory[0] >> 3;
  
  if(toc_byte<12)
  {
    mode = SILK_MODE;
  }
  else if(11<toc_byte && toc_byte<16)
  {
    mode = HYBRID_MODE;
  }  
  else if(toc_byte>15)
  {
    mode = CELT_MODE;
  }
  
  Msg->Data[3] = mode;
  
  if(numEncByte>0)
  {
    STCmdP_Serialize(&Msg->Data[4], numEncByte, 2);
    STCmdP_Serialize(&Msg->Data[6], enc_clock_count, 4);
    memcpy(&Msg->Data[10], EncConfigOpus.pInternalMemory, numEncByte);
  }
  else
  {
    return OPT_ERROR;
  }
  
  /******** Decoder profiling ********/
 
  /* Enable trace and debug block */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  
  __disable_irq();
  
  /* reset the counter */
  DWT->CYCCNT = 0;       
 
  /* enable the counter */
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; 
  
  numDecShort = DEC_Opus_Decode(EncConfigOpus.pInternalMemory, numEncByte, DecConfigOpus.pInternalMemory);

  /* disable the counter */
  DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk; 
  
  dec_clock_count = DWT->CYCCNT;  
  
  __enable_irq();
  
  /***********************************/
  
  if(numDecShort>0)
  {
    numDecShort = numDecShort * DecConfigOpus.channels;
    STCmdP_Serialize(&Msg->Data[10+numEncByte], numDecShort, 2); 
    STCmdP_Serialize(&Msg->Data[10+numEncByte+2], dec_clock_count, 4);
    memcpy(&Msg->Data[10+numEncByte+2+4], DecConfigOpus.pInternalMemory, numDecShort*2);
  }
  else
  {
    return OPT_ERROR;
  }
 
  Msg->Len = 10 + numEncByte + 2 + 4 + numDecShort*2;
  
  return OPT_SUCCESS;
}

/**
 * @brief  Handle encode and decode request, the command response will contain
 *         only profiling results.
 * @param  Message received.
 * @retval OPT_SUCCESS in case of success OPT_ERROR otherwise.
 */
OPT_StatusTypeDef Handle_CMD_EncDec_NoFile_Request(TMsg *Msg)
{
  uint32_t numEncByte = 0;
  uint32_t enc_clock_count = 0;
  uint32_t numDecShort = 0;
  uint32_t dec_clock_count = 0;
  uint8_t toc_byte = 0;
  uint8_t mode = 0;

  /******** Encoder profiling ********/
  
  /* Enable trace and debug block */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  
   __disable_irq();
  
  /* reset the counter */
  DWT->CYCCNT = 0;       
 
  /* enable the counter */
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; 

  numEncByte = ENC_Opus_Encode(&Msg->Data[3], EncConfigOpus.pInternalMemory);  
  
   /* disable the counter */
  DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk; 
  
  enc_clock_count = DWT->CYCCNT;  
  
  __enable_irq();

  /***********************************/
  
  /* "RFC6716 - Definition of the Opus Audio Codec" for more details */
  toc_byte = EncConfigOpus.pInternalMemory[0] >> 3;
  
  if(toc_byte<12)
  {
    mode = SILK_MODE;
  }
  else if(11<toc_byte && toc_byte<16)
  {
    mode = HYBRID_MODE;
  }  
  else if(toc_byte>15)
  {
    mode = CELT_MODE;
  }
  
  Msg->Data[3] = mode;

  if(numEncByte>0)
  {
    STCmdP_Serialize(&Msg->Data[4], numEncByte, 2);
    STCmdP_Serialize(&Msg->Data[6], enc_clock_count, 4);
  }
  else
  {
    return OPT_ERROR;
  }
  
  /******** Decoder profiling ********/
  
  /* Enable trace and debug block */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
   
  __disable_irq();
  
  /* reset the counter */
  DWT->CYCCNT = 0;       
 
  /* enable the counter */
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; 
  
  numDecShort = DEC_Opus_Decode(EncConfigOpus.pInternalMemory, numEncByte, DecConfigOpus.pInternalMemory);
  
  /* disable the counter */
  DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk; 
  
  dec_clock_count = DWT->CYCCNT;  
  
  __enable_irq();
  
  /***********************************/
  
  if(numDecShort>0)
  {
    numDecShort = numDecShort * DecConfigOpus.channels;
    STCmdP_Serialize(&Msg->Data[10], numDecShort, 2); 
    STCmdP_Serialize(&Msg->Data[12], dec_clock_count, 4);
  }
  else
  {
    return OPT_ERROR;
  }
 
  Msg->Len = 16;
  
  return OPT_SUCCESS;
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
