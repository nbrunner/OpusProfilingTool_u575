/**
  ******************************************************************************
  * @file    stcmdp_manager.h
  * @author  SRA 
  * @brief   Header for stcmdp_manager.c module
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
#ifndef __STCMDP_MANAGER_H
#define __STCMDP_MANAGER_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "STCmdP.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
extern uint32_t led_toggle_count;
/* Exported macros -----------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void STCmdP_manager(void);


#ifdef __cplusplus
}
#endif

#endif /* __STCMDP_MANAGER_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
