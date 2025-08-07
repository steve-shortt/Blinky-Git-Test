/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
extern int stdio_init   (void);
extern int app_main     (void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define A23_Pin GPIO_PIN_2
#define A23_GPIO_Port GPIOE
#define A19_Pin GPIO_PIN_3
#define A19_GPIO_Port GPIOE
#define A20_Pin GPIO_PIN_4
#define A20_GPIO_Port GPIOE
#define A21_Pin GPIO_PIN_5
#define A21_GPIO_Port GPIOE
#define A22_Pin GPIO_PIN_6
#define A22_GPIO_Port GPIOE
#define Key_Pin GPIO_PIN_13
#define Key_GPIO_Port GPIOC
#define A0_Pin GPIO_PIN_0
#define A0_GPIO_Port GPIOF
#define A1_Pin GPIO_PIN_1
#define A1_GPIO_Port GPIOF
#define A2_Pin GPIO_PIN_2
#define A2_GPIO_Port GPIOF
#define A3_Pin GPIO_PIN_3
#define A3_GPIO_Port GPIOF
#define A4_Pin GPIO_PIN_4
#define A4_GPIO_Port GPIOF
#define A5_Pin GPIO_PIN_5
#define A5_GPIO_Port GPIOF
#define SAI1_SDB_Pin GPIO_PIN_6
#define SAI1_SDB_GPIO_Port GPIOF
#define SAI1_MCKB_Pin GPIO_PIN_7
#define SAI1_MCKB_GPIO_Port GPIOF
#define SAI1_SCKB_Pin GPIO_PIN_8
#define SAI1_SCKB_GPIO_Port GPIOF
#define SAI1_FSB_Pin GPIO_PIN_9
#define SAI1_FSB_GPIO_Port GPIOF
#define DMCI_DATAIN_Pin GPIO_PIN_0
#define DMCI_DATAIN_GPIO_Port GPIOC
#define DFSM_CKOUT_Pin GPIO_PIN_2
#define DFSM_CKOUT_GPIO_Port GPIOC
#define VLCD_Pin GPIO_PIN_3
#define VLCD_GPIO_Port GPIOC
#define OpAmp1_INP_Pin GPIO_PIN_0
#define OpAmp1_INP_GPIO_Port GPIOA
#define OpAmp1_INM_Pin GPIO_PIN_1
#define OpAmp1_INM_GPIO_Port GPIOA
#define OpAmp1_OUT_Pin GPIO_PIN_3
#define OpAmp1_OUT_GPIO_Port GPIOA
#define ADC_DAC_Pin GPIO_PIN_4
#define ADC_DAC_GPIO_Port GPIOA
#define IDD_Measurement_Pin GPIO_PIN_5
#define IDD_Measurement_GPIO_Port GPIOA
#define SmartCard_IO_Pin GPIO_PIN_4
#define SmartCard_IO_GPIO_Port GPIOC
#define IDD_WAKEUP_Pin GPIO_PIN_5
#define IDD_WAKEUP_GPIO_Port GPIOC
#define SmartCard_CLK_Pin GPIO_PIN_0
#define SmartCard_CLK_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_2
#define LED1_GPIO_Port GPIOB
#define NFC_NSS_Pin GPIO_PIN_11
#define NFC_NSS_GPIO_Port GPIOF
#define A6_Pin GPIO_PIN_12
#define A6_GPIO_Port GPIOF
#define A7_Pin GPIO_PIN_13
#define A7_GPIO_Port GPIOF
#define A8_Pin GPIO_PIN_14
#define A8_GPIO_Port GPIOF
#define A9_Pin GPIO_PIN_15
#define A9_GPIO_Port GPIOF
#define A10_Pin GPIO_PIN_0
#define A10_GPIO_Port GPIOG
#define A11_Pin GPIO_PIN_1
#define A11_GPIO_Port GPIOG
#define D4_Pin GPIO_PIN_7
#define D4_GPIO_Port GPIOE
#define D5_Pin GPIO_PIN_8
#define D5_GPIO_Port GPIOE
#define D6_Pin GPIO_PIN_9
#define D6_GPIO_Port GPIOE
#define D7_Pin GPIO_PIN_10
#define D7_GPIO_Port GPIOE
#define D8_Pin GPIO_PIN_11
#define D8_GPIO_Port GPIOE
#define D9_Pin GPIO_PIN_12
#define D9_GPIO_Port GPIOE
#define D10_Pin GPIO_PIN_13
#define D10_GPIO_Port GPIOE
#define D11_Pin GPIO_PIN_14
#define D11_GPIO_Port GPIOE
#define D12_Pin GPIO_PIN_15
#define D12_GPIO_Port GPIOE
#define USBOTG_OVRCR_Pin GPIO_PIN_12
#define USBOTG_OVRCR_GPIO_Port GPIOB
#define NFC_SCK_Pin GPIO_PIN_13
#define NFC_SCK_GPIO_Port GPIOB
#define NFC_MISO_Pin GPIO_PIN_14
#define NFC_MISO_GPIO_Port GPIOB
#define NFC_MOSI_Pin GPIO_PIN_15
#define NFC_MOSI_GPIO_Port GPIOB
#define D13_Pin GPIO_PIN_8
#define D13_GPIO_Port GPIOD
#define D14_Pin GPIO_PIN_9
#define D14_GPIO_Port GPIOD
#define D15_Pin GPIO_PIN_10
#define D15_GPIO_Port GPIOD
#define A16_Pin GPIO_PIN_11
#define A16_GPIO_Port GPIOD
#define A17_Pin GPIO_PIN_12
#define A17_GPIO_Port GPIOD
#define A18_Pin GPIO_PIN_13
#define A18_GPIO_Port GPIOD
#define D0_Pin GPIO_PIN_14
#define D0_GPIO_Port GPIOD
#define D1_Pin GPIO_PIN_15
#define D1_GPIO_Port GPIOD
#define A12_Pin GPIO_PIN_2
#define A12_GPIO_Port GPIOG
#define A13_Pin GPIO_PIN_3
#define A13_GPIO_Port GPIOG
#define A14_Pin GPIO_PIN_4
#define A14_GPIO_Port GPIOG
#define A15_Pin GPIO_PIN_5
#define A15_GPIO_Port GPIOG
#define CODEC_INT_Pin GPIO_PIN_6
#define CODEC_INT_GPIO_Port GPIOG
#define LPUART_TX_Pin GPIO_PIN_7
#define LPUART_TX_GPIO_Port GPIOG
#define LPUART_RX_3V3_Pin GPIO_PIN_8
#define LPUART_RX_3V3_GPIO_Port GPIOG
#define USBOTG_PPWR_Pin GPIO_PIN_6
#define USBOTG_PPWR_GPIO_Port GPIOC
#define PT100_DATIN_Pin GPIO_PIN_7
#define PT100_DATIN_GPIO_Port GPIOC
#define uSD_D0_Pin GPIO_PIN_8
#define uSD_D0_GPIO_Port GPIOC
#define uSD_D1_Pin GPIO_PIN_9
#define uSD_D1_GPIO_Port GPIOC
#define uSD_DETECT_Pin GPIO_PIN_8
#define uSD_DETECT_GPIO_Port GPIOA
#define USBOTG_VBUS_Pin GPIO_PIN_9
#define USBOTG_VBUS_GPIO_Port GPIOA
#define USBOTG_ID_Pin GPIO_PIN_10
#define USBOTG_ID_GPIO_Port GPIOA
#define USBOTG_DM_Pin GPIO_PIN_11
#define USBOTG_DM_GPIO_Port GPIOA
#define USBOTG_DP_Pin GPIO_PIN_12
#define USBOTG_DP_GPIO_Port GPIOA
#define TMS_SWDIO_Pin GPIO_PIN_13
#define TMS_SWDIO_GPIO_Port GPIOA
#define TMS_SWCLK_Pin GPIO_PIN_14
#define TMS_SWCLK_GPIO_Port GPIOA
#define uSD_D2_Pin GPIO_PIN_10
#define uSD_D2_GPIO_Port GPIOC
#define uSD_D3_Pin GPIO_PIN_11
#define uSD_D3_GPIO_Port GPIOC
#define uSD_CLK_Pin GPIO_PIN_12
#define uSD_CLK_GPIO_Port GPIOC
#define D2_Pin GPIO_PIN_0
#define D2_GPIO_Port GPIOD
#define D3_Pin GPIO_PIN_1
#define D3_GPIO_Port GPIOD
#define uSD_CMD_Pin GPIO_PIN_2
#define uSD_CMD_GPIO_Port GPIOD
#define DFSDM_DATIN1_Pin GPIO_PIN_3
#define DFSDM_DATIN1_GPIO_Port GPIOD
#define FMC_NOE_Pin GPIO_PIN_4
#define FMC_NOE_GPIO_Port GPIOD
#define FMC_NWE_Pin GPIO_PIN_5
#define FMC_NWE_GPIO_Port GPIOD
#define SAI1_SDA_Pin GPIO_PIN_6
#define SAI1_SDA_GPIO_Port GPIOD
#define FMC_NE1_Pin GPIO_PIN_7
#define FMC_NE1_GPIO_Port GPIOD
#define FMC_NE2_Pin GPIO_PIN_9
#define FMC_NE2_GPIO_Port GPIOG
#define LCD_NE3_Pin GPIO_PIN_10
#define LCD_NE3_GPIO_Port GPIOG
#define USART1_CTS_3V3_Pin GPIO_PIN_11
#define USART1_CTS_3V3_GPIO_Port GPIOG
#define USART1_RTS_Pin GPIO_PIN_12
#define USART1_RTS_GPIO_Port GPIOG
#define I2C_SDA_Pin GPIO_PIN_13
#define I2C_SDA_GPIO_Port GPIOG
#define I2C_SCL_Pin GPIO_PIN_14
#define I2C_SCL_GPIO_Port GPIOG
#define IOExpander_INT_Pin GPIO_PIN_15
#define IOExpander_INT_GPIO_Port GPIOG
#define comp2_INP_Pin GPIO_PIN_4
#define comp2_INP_GPIO_Port GPIOB
#define Comp2_OUT_Pin GPIO_PIN_5
#define Comp2_OUT_GPIO_Port GPIOB
#define USART1_TX_Pin GPIO_PIN_6
#define USART1_TX_GPIO_Port GPIOB
#define USART1_IrDA_RX_3V3_Pin GPIO_PIN_7
#define USART1_IrDA_RX_3V3_GPIO_Port GPIOB
#define CAN_RX_Pin GPIO_PIN_8
#define CAN_RX_GPIO_Port GPIOB
#define CAN_TX_Pin GPIO_PIN_9
#define CAN_TX_GPIO_Port GPIOB
#define FMC_NBL0_Pin GPIO_PIN_0
#define FMC_NBL0_GPIO_Port GPIOE
#define FMC_NBL1_Pin GPIO_PIN_1
#define FMC_NBL1_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
