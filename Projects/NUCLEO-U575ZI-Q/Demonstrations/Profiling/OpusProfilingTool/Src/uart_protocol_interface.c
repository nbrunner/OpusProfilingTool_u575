/**
  ******************************************************************************
  * @file    uart_protocol_interface.c
  * @author  SRA
  * @brief   This file contains the functions used for received and send message
  *          via UART and to configure it.
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

/*Include --------------------------------------------------------------------*/
#include "uart_protocol_interface.h"

/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
#define UART_RX_BUFFER_SIZE           (2*12288)
#define UART_TX_BUFFER_SIZE           (2*12288)

#define USARTx                           USART1
#define USARTx_CLK_ENABLE()              __HAL_RCC_USART1_CLK_ENABLE()
#define DMAx_CLK_ENABLE()                __HAL_RCC_GPDMA1_CLK_ENABLE()
#define USARTx_RX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()
#define USARTx_TX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()

#define USARTx_FORCE_RESET()             __HAL_RCC_USART1_FORCE_RESET()
#define USARTx_RELEASE_RESET()           __HAL_RCC_USART1_RELEASE_RESET()

#define USARTx_TX_PIN                    GPIO_PIN_9
#define USARTx_TX_GPIO_PORT              GPIOA
#define USARTx_TX_AF                     GPIO_AF7_USART1
#define USARTx_RX_PIN                    GPIO_PIN_10
#define USARTx_RX_GPIO_PORT              GPIOA
#define USARTx_RX_AF                     GPIO_AF7_USART1
#define USARTx_IRQn                      USART1_IRQn
 
/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef UartHandle;
DMA_HandleTypeDef hdma_rx;
DMA_HandleTypeDef hdma_tx;
DMA_NodeTypeDef Node_GPDMA1_Channel0;
DMA_QListTypeDef List_GPDMA1_Channel0;

uint16_t UART_StartOfMsg_idx = 0;
uint16_t UART_NewByte_idx = 0;

volatile uint8_t UART_RxBuffer[UART_RX_BUFFER_SIZE];
volatile uint8_t UART_TxBuffer[UART_TX_BUFFER_SIZE];

/* Global variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void USART_DMA_Configuration(void);
static void UART_PROTOCOL_SendData(uint16_t msg_size);
static uint16_t UART_PROTOCOL_CheckForNewData(void);
static uint8_t* UART_PROTOCOL_GetTxBuffer(void);
static uint8_t* UART_PROTOCOL_GetRxBuffer(void);

/**
  * @}
  */

/** @defgroup UART_PROTOCOL_INTERFACE_Exported_Functions UART_PROTOCOL_INTERFACE_Exported_Functions
* @{ 
*/

/* Functions Definition ------------------------------------------------------*/
/**
  * @brief  Send a Msg via UART
  * @param  Msg pointer to the msg
  * @retval None
  */
void UART_SendMsg(TMsg *Msg)
{  
  uint8_t *UART_TxBuffer;
  uint16_t CountOut;
  
  UART_TxBuffer = UART_PROTOCOL_GetTxBuffer();
  
  CountOut = STCmdP_Prepare_Msg((uint8_t*)UART_TxBuffer, Msg);
  
  UART_PROTOCOL_SendData(CountOut);
}

/**
  * @brief  Check if a message is received via UART.
  * @param  Msg pointer to the msg
  * @retval 1 if a msg is been received 0 otherwise
  */
int UART_ReceivedMSG(TMsg *Msg)
{
  uint8_t *UART_RxBuffer;
  uint16_t numByteRead = 0; 
 
  if (UART_NewByte_idx != UART_PROTOCOL_CheckForNewData())
  {
    UART_NewByte_idx = UART_PROTOCOL_CheckForNewData();
    UART_RxBuffer = UART_PROTOCOL_GetRxBuffer();
    
    if((numByteRead = STCmdP_Extract_Msg(UART_RxBuffer, UART_StartOfMsg_idx, UART_NewByte_idx, UART_RX_BUFFER_SIZE, Msg)) != 0)
    {
      UART_StartOfMsg_idx = (UART_StartOfMsg_idx+numByteRead)%UART_RX_BUFFER_SIZE;
      return 1;
    }
  }
  return 0;
}

/**
  * @brief  UART configuration
  * @param  None
  * @retval None
  */
void USARTConfig(void)
{
  UartHandle.Instance                    = USARTx;
  UartHandle.Init.BaudRate               = 921600;
  UartHandle.Init.WordLength             = UART_WORDLENGTH_8B;
  UartHandle.Init.StopBits               = UART_STOPBITS_1;
  UartHandle.Init.Parity                 = UART_PARITY_NONE;
  UartHandle.Init.HwFlowCtl              = UART_HWCONTROL_NONE;
  UartHandle.Init.Mode                   = UART_MODE_TX_RX;
  UartHandle.Init.OverSampling           = UART_OVERSAMPLING_16;  
  UartHandle.Init.Parity                 = UART_PARITY_NONE;
  UartHandle.Init.OneBitSampling         = UART_ONE_BIT_SAMPLE_DISABLE;
  UartHandle.Init.ClockPrescaler         = UART_PRESCALER_DIV1;
  UartHandle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  UartHandle.FifoMode                    = UART_FIFOMODE_DISABLE;
  
  UartHandle.pRxBuffPtr = (uint8_t*)UART_RxBuffer;
  UartHandle.RxXferSize = UART_RX_BUFFER_SIZE;
  UartHandle.pTxBuffPtr = (uint8_t*)UART_TxBuffer;
  UartHandle.TxXferSize = UART_TX_BUFFER_SIZE;
  UartHandle.ErrorCode  = HAL_UART_ERROR_NONE;
   
  if(HAL_UART_Init(&UartHandle) != HAL_OK)
  {
    while(1);
  }
  
  /* Enable the DMA transfer for the receiver request  */
  HAL_UART_Receive_DMA(&UartHandle, (uint8_t*)UART_RxBuffer, UART_RX_BUFFER_SIZE);
}

/**
 * @brief  UART configuration
 * @param  None
 * @retval None
 */
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    while(1);
  }
    
  USARTx_CLK_ENABLE();
  
  /* Enable GPIO TX/RX clock */
  USARTx_TX_GPIO_CLK_ENABLE();
  USARTx_RX_GPIO_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();
  
  DMAx_CLK_ENABLE();
  
  /* UART TX GPIO pin configuration  */
  GPIO_InitStruct.Pin       = USARTx_TX_PIN;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = USARTx_TX_AF;

  HAL_GPIO_Init(USARTx_TX_GPIO_PORT, &GPIO_InitStruct);

  /* UART RX GPIO pin configuration  */
  GPIO_InitStruct.Pin = USARTx_RX_PIN;
  GPIO_InitStruct.Alternate = USARTx_RX_AF;

  HAL_GPIO_Init(USARTx_RX_GPIO_PORT, &GPIO_InitStruct);

  USART_DMA_Configuration();  
    
  /* NVIC for USART, to catch the TX complete */
  HAL_NVIC_SetPriority(USARTx_IRQn, 0, 1);
  HAL_NVIC_EnableIRQ(USARTx_IRQn);
}

/**
 * @brief  Configure DMA for transmitter and receiver
 * @param  None
 * @retval None
 */
static void USART_DMA_Configuration(void)
{  
  DMA_NodeConfTypeDef NodeConfig= {0};

  /* USART1 DMA Init */
  /* GPDMA1_REQUEST_USART1_TX Init */
  hdma_tx.Instance = GPDMA1_Channel1;
  hdma_tx.Init.Request = GPDMA1_REQUEST_USART1_TX;
  hdma_tx.Init.BlkHWRequest = DMA_BREQ_SINGLE_BURST;
  hdma_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
  hdma_tx.Init.SrcInc = DMA_SINC_INCREMENTED;
  hdma_tx.Init.DestInc = DMA_DINC_FIXED;
  hdma_tx.Init.SrcDataWidth = DMA_SRC_DATAWIDTH_BYTE;
  hdma_tx.Init.DestDataWidth = DMA_DEST_DATAWIDTH_BYTE;
  hdma_tx.Init.Priority = DMA_LOW_PRIORITY_LOW_WEIGHT;
  hdma_tx.Init.SrcBurstLength = 1;
  hdma_tx.Init.DestBurstLength = 1;
  hdma_tx.Init.TransferAllocatedPort = DMA_SRC_ALLOCATED_PORT0|DMA_DEST_ALLOCATED_PORT0;
  hdma_tx.Init.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
  hdma_tx.Init.Mode = DMA_NORMAL;
  if (HAL_DMA_Init(&hdma_tx) != HAL_OK)
  {
    while(1);
  }

  HAL_NVIC_SetPriority(GPDMA1_Channel1_IRQn, 10, 0);
    HAL_NVIC_EnableIRQ(GPDMA1_Channel1_IRQn);

  __HAL_LINKDMA(&UartHandle, hdmatx, hdma_tx);

  if (HAL_DMA_ConfigChannelAttributes(&hdma_tx, DMA_CHANNEL_NPRIV) != HAL_OK)
  {
    while(1);
  }

  /* GPDMA1_REQUEST_USART1_RX Init */
  NodeConfig.NodeType = DMA_GPDMA_LINEAR_NODE;
  NodeConfig.Init.Request = GPDMA1_REQUEST_USART1_RX;
  NodeConfig.Init.BlkHWRequest = DMA_BREQ_SINGLE_BURST;
  NodeConfig.Init.Direction = DMA_PERIPH_TO_MEMORY;
  NodeConfig.Init.SrcInc = DMA_SINC_FIXED;
  NodeConfig.Init.DestInc = DMA_DINC_INCREMENTED;
  NodeConfig.Init.SrcDataWidth = DMA_SRC_DATAWIDTH_BYTE;
  NodeConfig.Init.DestDataWidth = DMA_DEST_DATAWIDTH_BYTE;
  NodeConfig.Init.SrcBurstLength = 1;
  NodeConfig.Init.DestBurstLength = 1;
  NodeConfig.Init.TransferAllocatedPort = DMA_SRC_ALLOCATED_PORT0|DMA_DEST_ALLOCATED_PORT1;
  NodeConfig.Init.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
  NodeConfig.Init.Mode = DMA_NORMAL;
  NodeConfig.TriggerConfig.TriggerPolarity = DMA_TRIG_POLARITY_MASKED;
  NodeConfig.DataHandlingConfig.DataExchange = DMA_EXCHANGE_NONE;
  NodeConfig.DataHandlingConfig.DataAlignment = DMA_DATA_RIGHTALIGN_ZEROPADDED;
  if (HAL_DMAEx_List_BuildNode(&NodeConfig, &Node_GPDMA1_Channel0) != HAL_OK)
  {
    while(1);
  }

  if (HAL_DMAEx_List_InsertNode(&List_GPDMA1_Channel0, NULL, &Node_GPDMA1_Channel0) != HAL_OK)
  {
    while(1);
  }

  if (HAL_DMAEx_List_SetCircularMode(&List_GPDMA1_Channel0) != HAL_OK)
  {
    while(1);
  }

  hdma_rx.Instance = GPDMA1_Channel0;
  hdma_rx.InitLinkedList.Priority = DMA_LOW_PRIORITY_LOW_WEIGHT;
  hdma_rx.InitLinkedList.LinkStepMode = DMA_LSM_FULL_EXECUTION;
  hdma_rx.InitLinkedList.LinkAllocatedPort = DMA_LINK_ALLOCATED_PORT1;
  hdma_rx.InitLinkedList.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
  hdma_rx.InitLinkedList.LinkedListMode = DMA_LINKEDLIST_CIRCULAR;
  if (HAL_DMAEx_List_Init(&hdma_rx) != HAL_OK)
  {
    while(1);
  }

  if (HAL_DMAEx_List_LinkQ(&hdma_rx, &List_GPDMA1_Channel0) != HAL_OK)
  {
    while(1);
  }

  __HAL_LINKDMA(&UartHandle, hdmarx, hdma_rx);

  if (HAL_DMA_ConfigChannelAttributes(&hdma_rx, DMA_CHANNEL_NPRIV) != HAL_OK)
  {
    while(1);
  }
}

/**
 * @brief  Check if new byte has been tranfered by the DMA
 * @param  None
 * @retval Number of byte received
 */
static uint16_t UART_PROTOCOL_CheckForNewData(void)
{
  return (uint16_t)( UartHandle.RxXferSize - ((uint16_t)__HAL_DMA_GET_COUNTER(UartHandle.hdmarx)));
}

/**
 * @brief  Send data via UART
 * @param  msg_size: number of byte to be sent
 * @retval None
 */
static void UART_PROTOCOL_SendData(uint16_t msg_size)
{
  UartHandle.TxXferSize = msg_size;
  UartHandle.TxXferCount = msg_size;
  HAL_UART_Transmit_DMA(&UartHandle, UartHandle.pTxBuffPtr, msg_size);
}

/**
 * @brief  Get pointer to the UART RX buffer
 * @param  None
 * @retval pointer to the UART RX buffer
 */
static uint8_t* UART_PROTOCOL_GetRxBuffer(void)
{
  return (uint8_t *)UART_RxBuffer;
}

/**
 * @brief  Get pointer to the UART TX buffer
 * @param  None
 * @retval pointer to the UART TX buffer
 */
static uint8_t* UART_PROTOCOL_GetTxBuffer(void)
{
  return (uint8_t *)UART_TxBuffer;
}



/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
