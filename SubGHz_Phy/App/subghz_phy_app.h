/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    subghz_phy_app.h
  * @author  MCD Application Team
  * @brief   Header of application of the SubGHz_Phy Middleware
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SUBGHZ_PHY_APP_H__
#define __SUBGHZ_PHY_APP_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/

/* USER CODE BEGIN EC */
/* MODEM type: one shall be 1 the other shall be 0 */
#define USE_MODEM_LORA  1
#define USE_MODEM_FSK   0

#define RF_FREQUENCY                                869525000 // EU standards, 10% duty cycle, 24 dBm max (in my case 14+2), middle of 869.4–869.65[Hz] so BW=250 is ok

#ifndef TX_OUTPUT_POWER
#define TX_OUTPUT_POWER                             14 // max of the Wio-E5 LE module [dBm]
#endif /* TX_OUTPUT_POWER */

#define CONFIGURATIONS_NUM									12	// 1 - 12 how many LoRa configurations there are
#define MEASUREMENTS_NUM									50	// po 10 pomiarow - ok 3min, zdecydowano na 50 bo sama komunikacja trwa wtedy 15 min w jednym miejscu, how many repetitions for collecting data for 1 configuration

#if (( USE_MODEM_LORA == 1 ) && ( USE_MODEM_FSK == 0 ))
//#define LORA_BANDWIDTH                              0         /* [0: 125 kHz, 1: 250 kHz, 2: 500 kHz, 3: Reserved] */
//#define LORA_SPREADING_FACTOR                       12        /* [SF7..SF12] */
//#define LORA_CODINGRATE                             4         /* [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8] */
//#define LORA_PREAMBLE_LENGTH                        8         /* Same for Tx and Rx, middleware sets min 12 for SF5 and 6 */
//#define LORA_SYMBOL_TIMEOUT                         5        /* Symbols */
//#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
//#define LORA_IQ_INVERSION_ON                        false
//#define PAYLOAD_LEN                                 64	// max 256

#elif (( USE_MODEM_LORA == 0 ) && ( USE_MODEM_FSK == 1 ))

#define FSK_FDEV                                    25000     /* Hz */
#define FSK_DATARATE                                50000     /* bps */
#define FSK_BANDWIDTH                               50000     /* Hz */
#define FSK_PREAMBLE_LENGTH                         5         /* Same for Tx and Rx */
#define FSK_FIX_LENGTH_PAYLOAD_ON                   false
#define PAYLOAD_LEN                                 32	// max 256

#else
#error "Please define a modem in the compiler subghz_phy_app.h."
#endif /* USE_MODEM_LORA | USE_MODEM_FSK */

#define MAX_PAYLOAD_LEN                                 255

/* USER CODE END EC */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Exported macros -----------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
/**
  * @brief  Init Subghz Application
  */
void SubghzApp_Init(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /*__SUBGHZ_PHY_APP_H__*/
