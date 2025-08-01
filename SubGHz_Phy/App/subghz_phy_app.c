/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    subghz_phy_app.c
 * @author  MCD Application Team
 * @brief   Application of the SubGHz_Phy Middleware
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

/* Includes ------------------------------------------------------------------*/
#include "platform.h"
#include "sys_app.h"
#include "subghz_phy_app.h"
#include "radio.h"

/* USER CODE BEGIN Includes */
#include "stm32_timer.h"
#include "stm32_seq.h"
#include "utilities_def.h"
#include "app_version.h"
#include "subghz_phy_version.h"
#include <stdio.h>
#include <math.h>
/* USER CODE END Includes */

/* External variables ---------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
	STATE_TX = 0,
	STATE_RX,

	STATE_ECHO_TX,
	STATE_ECHO_RX,

	STATE_NEXT_MEASUREMENT,
	STATE_END_MEASUREMENTS,
	STATE_NEXT_CONFIGURATION,
	STATE_END_CONFIGURATIONS
} SubGhz_State_t;

typedef enum
{
	NO_ERROR = 0,		// no error
	ERROR_TIMEOUT_TX,	// timeout tx
	ERROR_TIMEOUT_RX,	// timeout rx
	ERROR_RX,			// error rx, for example wrong crc
	ERROR_PAYLOAD		// data received not the same as transmitted
} Error_Code_t;

typedef struct
{
	uint32_t RoundTripTime; // could be cast to uint16_t == up to 65535 ms
	int16_t RssiValue; /* Last  Received packer Rssi*/
	int8_t SnrValue; /* Last  Received packer SNR (in Lora modulation)*/
	uint16_t TxBitRate; // equals: PayloadLen / TxTime
	Error_Code_t ErrorCode;	/* 0 - , 1 - , 2 - , 3 - , 4 -  */

} SubGhz_Measurements_t;	// last received data

typedef struct
{
	uint8_t PacketsReceived; // for PacketDeliveryRatio
	float PacketDeliveryRatio;
	SubGhz_Measurements_t Measurements[MEASUREMENTS_NUM];

} SubGhz_MeasurementsCollection_t;// all measurements collected for 1 configuration, for example 20 repetitions; SubGhz_MeasurementsCollection_t Collection[12] -> 12 configurations, each 20 repetitions

typedef struct
{
	uint32_t LORA_BANDWIDTH;
	uint32_t LORA_SPREADING_FACTOR;
	uint8_t LORA_CODINGRATE;
	uint16_t LORA_PREAMBLE_LENGTH;
	bool LORA_FIX_LENGTH_PAYLOAD_ON;
	bool LORA_IQ_INVERSION_ON;
	uint32_t TX_TIMEOUT_VALUE;
	uint16_t LORA_SYMBOL_TIMEOUT;
	uint8_t PAYLOAD_LEN;
} LoRaConfiguration_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Size of the payload to be sent */
/* Size must be greater of equal to sent payload*/
#define MAX_APP_BUFFER_SIZE          255

/* Afc bandwidth in Hz */
#define FSK_AFC_BANDWIDTH             83333

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* Radio events function pointer */
static RadioEvents_t RadioEvents;

/* USER CODE BEGIN PV */
static uint8_t BufferRx[MAX_APP_BUFFER_SIZE]; /* App Rx Buffer*/
static uint8_t BufferTx[MAX_APP_BUFFER_SIZE]; /* App Tx Buffer*/

uint16_t RxBufferSize = 0; /* Last  Received Buffer Size*/
static uint8_t rxMessage[MAX_PAYLOAD_LEN];
static const uint8_t txMessage[MAX_PAYLOAD_LEN] =
	{"Moja wiadomosc do przeprowadzania testow konfiguracji lora, ta wiadomosc jest przesylana z nadajnika do odbiornika i spowrotem jako echo w celu zbadania Round Trip Time. Dzieki temu w nadajniku mozliwe jest zbieranie danych wlasciwych dla nadajnika i odb"};

static uint32_t txTimestamp = 0; /* transmitted timestamp */

#if LORA_DIRECTION == LORA_TRANSMITER // for no warning messages
static uint32_t txTimestampEnd = 0; /* end of transmition timestamp */
static uint32_t rxTimestamp = 0; /* received timestamp */
#endif

static uint8_t ConfigurationNum = 0; // which LoRa configuration is tested now
static uint8_t MeasurementNum = 0; // which configurations measurement is tested now
static uint32_t CommTickCnt = 0; // num of received packages

static SubGhz_MeasurementsCollection_t Collection[CONFIGURATIONS_NUM];
static LoRaConfiguration_t LoRa;
static SubGhz_State_t State;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/*!
 * @brief Function to be executed on Radio Tx Done event
 */
static void OnTxDone(void);

/**
  * @brief Function to be executed on Radio Rx Done event
  * @param  payload ptr of buffer received
  * @param  size buffer size
  * @param  rssi
  * @param  LoraSnr_FskCfo
  */
static void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t LoraSnr_FskCfo);

/**
  * @brief Function executed on Radio Tx Timeout event
  */
static void OnTxTimeout(void);

/**
  * @brief Function executed on Radio Rx Timeout event
  */
static void OnRxTimeout(void);

/**
  * @brief Function executed on Radio Rx Error event
  */
static void OnRxError(void);

/* USER CODE BEGIN PFP */
/**
 * @brief Communication_Process
 */
static void Communication_Process(void);

/**
 * @brief Change LoRa configuration
 * @param Configuration index to set, check details of configurations in the definition of this function
 */
static void SetLoRaConfiguration(uint8_t NewConfigurationNum);

/**
 * @brief Calculate Packet Delivery Ratio of all configurations
 */
static void CalculatePDR(void);

/*
 * @brief Print measurements to UART in CSV format
 */
static void ExportMeasurementsAsCSV(void);

/* USER CODE END PFP */

/* Exported functions ---------------------------------------------------------*/
void SubghzApp_Init(void)
{
  /* USER CODE BEGIN SubghzApp_Init_1 */

  /* USER CODE END SubghzApp_Init_1 */

  /* Radio initialization */
  RadioEvents.TxDone = OnTxDone;
  RadioEvents.RxDone = OnRxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
  RadioEvents.RxTimeout = OnRxTimeout;
  RadioEvents.RxError = OnRxError;

  Radio.Init(&RadioEvents);

  /* USER CODE BEGIN SubghzApp_Init_2 */
	/* Radio Set frequency */
	Radio.SetChannel(RF_FREQUENCY);

	/* Radio configuration */
#if ((USE_MODEM_LORA == 1) && (USE_MODEM_FSK == 0))
	SetLoRaConfiguration(ConfigurationNum);

#elif ((USE_MODEM_LORA == 0) && (USE_MODEM_FSK == 1))
    APP_LOG(TS_OFF, VLEVEL_M, "---------------\n\r");
    APP_LOG(TS_OFF, VLEVEL_M, "FSK_MODULATION\n\r");
    APP_LOG(TS_OFF, VLEVEL_M, "FSK_BW=%d Hz\n\r", FSK_BANDWIDTH);
    APP_LOG(TS_OFF, VLEVEL_M, "FSK_DR=%d bits/s\n\r", FSK_DATARATE);

    Radio.SetTxConfig(MODEM_FSK, TX_OUTPUT_POWER, FSK_FDEV, 0,
                      FSK_DATARATE, 0,
                      FSK_PREAMBLE_LENGTH, FSK_FIX_LENGTH_PAYLOAD_ON,
                      true, 0, 0, 0, TX_TIMEOUT_VALUE);

    Radio.SetRxConfig(MODEM_FSK, FSK_BANDWIDTH, FSK_DATARATE,
                      0, FSK_AFC_BANDWIDTH, FSK_PREAMBLE_LENGTH,
                      0, FSK_FIX_LENGTH_PAYLOAD_ON, 0, true,
                      0, 0, false, true);

    Radio.SetMaxPayloadLength(MODEM_FSK, MAX_APP_BUFFER_SIZE);

#else
#error "Please define a modulation in the subghz_phy_app.h file."
#endif /* USE_MODEM_LORA | USE_MODEM_FSK */

	/*fills buffers*/
	memset(BufferTx, 0x0, MAX_APP_BUFFER_SIZE);
	memset(BufferRx, 0x0, MAX_APP_BUFFER_SIZE);

#if LORA_DIRECTION == LORA_TRANSMITER
	State = STATE_TX;
#elif LORA_DIRECTION == LORA_RECEIVER
	Radio.Rx(0);
	State = STATE_RX;
#endif

	/*register task to to be run in while(1) after Radio IT*/
	UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), UTIL_SEQ_RFU,
			Communication_Process);
	UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process),
			CFG_SEQ_Prio_0);
  /* USER CODE END SubghzApp_Init_2 */
}

/* USER CODE BEGIN EF */

/* USER CODE END EF */

/* Private functions ---------------------------------------------------------*/
static void OnTxDone(void)
{
  /* USER CODE BEGIN OnTxDone */
//	APP_LOG(TS_ON, VLEVEL_L, "OnTxDone\n\r");

#if LORA_DIRECTION == LORA_TRANSMITER
	txTimestampEnd = HAL_GetTick();
	Collection[ConfigurationNum].Measurements[MeasurementNum].TxBitRate =
			(LoRa.PAYLOAD_LEN * 8 * 1000) / (txTimestampEnd - txTimestamp); // *8 -> byte to bit, *1000 -> ms to s

	// listen for echo
	Radio.Rx((txTimestampEnd - txTimestamp) + 500); // 500 ms margin
#elif LORA_DIRECTION == LORA_RECEIVER
	Radio.Rx(0);
#endif

	UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process),
			CFG_SEQ_Prio_0);

  /* USER CODE END OnTxDone */
}

static void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t LoraSnr_FskCfo)
{
  /* USER CODE BEGIN OnRxDone */
	printf("OnRxDone\n\r");

	/* Clear BufferRx*/
	memset(BufferRx, 0, MAX_APP_BUFFER_SIZE);
	/* Record payload size*/
	RxBufferSize = size;
	if (RxBufferSize <= MAX_APP_BUFFER_SIZE)
	{
		memcpy(BufferRx, payload, RxBufferSize);
	}

#if LORA_DIRECTION == LORA_TRANSMITER
	rxTimestamp = HAL_GetTick();
	Collection[ConfigurationNum].Measurements[MeasurementNum].RoundTripTime =
			rxTimestamp - txTimestamp;
	Collection[ConfigurationNum].Measurements[MeasurementNum].RssiValue = rssi;
	Collection[ConfigurationNum].Measurements[MeasurementNum].SnrValue =
			LoraSnr_FskCfo;
#endif

//	/* Record payload content*/
//	APP_LOG(TS_ON, VLEVEL_H, "payload. size=%d \n\r", size);
//	for (int32_t i = 0; i < PAYLOAD_LEN; i++)
//	{
//		APP_LOG(TS_OFF, VLEVEL_H, "%02X", BufferRx[i]);
//		if (i % 16 == 15)
//		{
//			APP_LOG(TS_OFF, VLEVEL_H, "\n\r");
//		}
//	}
//	APP_LOG(TS_OFF, VLEVEL_H, "\n\r");

	/* Run Communication process in background*/
	UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process),
			CFG_SEQ_Prio_0);
  /* USER CODE END OnRxDone */
}

static void OnTxTimeout(void)
{
  /* USER CODE BEGIN OnTxTimeout */
	APP_LOG(TS_ON, VLEVEL_L, "OnTxTimeout\n\r");

#if LORA_DIRECTION == LORA_TRANSMITER
	Collection[ConfigurationNum].Measurements[MeasurementNum].ErrorCode = ERROR_TIMEOUT_TX;

	State = STATE_NEXT_MEASUREMENT;
#elif LORA_DIRECTION == LORA_RECEIVER
	Radio.Rx(0);
	State = STATE_RX;
#endif

	/* Run Communication process in background*/
	UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process),
			CFG_SEQ_Prio_0);
  /* USER CODE END OnTxTimeout */
}

static void OnRxTimeout(void)
{
  /* USER CODE BEGIN OnRxTimeout */
	APP_LOG(TS_ON, VLEVEL_L, "OnRxTimeout\n\r");

#if LORA_DIRECTION == LORA_TRANSMITER
	Collection[ConfigurationNum].Measurements[MeasurementNum].ErrorCode = ERROR_TIMEOUT_RX;

	State = STATE_NEXT_MEASUREMENT;
#elif LORA_DIRECTION == LORA_RECEIVER
	Radio.Rx(0);
	State = STATE_RX;
#endif

	/* Run Communication process in background*/
	UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process),
			CFG_SEQ_Prio_0);
  /* USER CODE END OnRxTimeout */
}

static void OnRxError(void)
{
  /* USER CODE BEGIN OnRxError */
	APP_LOG(TS_ON, VLEVEL_L, "OnRxError\n\r");

#if LORA_DIRECTION == LORA_TRANSMITER
	Collection[ConfigurationNum].Measurements[MeasurementNum].ErrorCode = ERROR_RX;

	State = STATE_NEXT_MEASUREMENT;
#elif LORA_DIRECTION == LORA_RECEIVER
	Radio.Rx(0);
	State = STATE_RX;
#endif

	/* Run Communication process in background*/
	UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process),
			CFG_SEQ_Prio_0);
  /* USER CODE END OnRxError */
}

/* USER CODE BEGIN PrFD */
static void Communication_Process(void) // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
{
	/* States for changing TX and RX (because RTT feature) and changing configurations */
	switch (State)
	{
	case STATE_TX:
		if(ConfigurationNum == 0 && MeasurementNum == 0)
		{
			printf("Waiting on input to start measurements...\n\r");
			while(1)
			{
				HAL_Delay(50);
				if(HAL_GPIO_ReadPin(BUTTON_SW1_GPIO_PORT, BUTTON_SW1_PIN) == GPIO_PIN_RESET)
				{
					while(1)
					{
						HAL_Delay(50);
						if(HAL_GPIO_ReadPin(BUTTON_SW1_GPIO_PORT, BUTTON_SW1_PIN) == GPIO_PIN_SET)
							break;
					}

					printf("Program started\n\r");
					break;
				}
			}
		}

		memset(&Collection[ConfigurationNum].Measurements[MeasurementNum], 0,
				sizeof(Collection[0].Measurements[0]));

		memset(BufferTx, 0, MAX_APP_BUFFER_SIZE);
		memcpy(BufferTx, txMessage, LoRa.PAYLOAD_LEN);

		HAL_Delay(Radio.GetWakeupTime());

		HAL_GPIO_TogglePin(LED1_GPIO_PORT, LED1_PIN);
		txTimestamp = HAL_GetTick();  // start transmission

		Radio.Send(BufferTx, LoRa.PAYLOAD_LEN);

		State = STATE_ECHO_RX;
		break;

	case STATE_RX:
		if (RxBufferSize > 0)
		{
			HAL_GPIO_TogglePin(LED1_GPIO_PORT, LED1_PIN);

			memcpy(rxMessage, BufferRx, MAX_APP_BUFFER_SIZE);

//			printf("Acquired data: \"%s\"\n\r", BufferRx);
			printf("Received data nr %lu", ++CommTickCnt);

			memset(BufferRx, 0, MAX_APP_BUFFER_SIZE);
			RxBufferSize = 0;

			State = STATE_ECHO_TX;
		}
		else
		{
			if(HAL_GPIO_ReadPin(BUTTON_SW1_GPIO_PORT, BUTTON_SW1_PIN) == GPIO_PIN_RESET) // change configuration
			{
				while (1)
				{
					HAL_Delay(50);
					if (HAL_GPIO_ReadPin(BUTTON_SW1_GPIO_PORT, BUTTON_SW1_PIN) == GPIO_PIN_SET)
						break;
				}
				HAL_GPIO_TogglePin(LED1_GPIO_PORT, LED1_PIN);
				SetLoRaConfiguration(++ConfigurationNum);
				Radio.Rx(0);
			}
		}
		break;

	case STATE_ECHO_TX:
		printf("Waiting for Radio.GetWakeupTime(): %lu\n\r", Radio.GetWakeupTime());
		HAL_Delay(Radio.GetWakeupTime());

		memset(BufferTx, 0, MAX_APP_BUFFER_SIZE);
		memcpy(BufferTx, rxMessage, LoRa.PAYLOAD_LEN);

		Radio.Send(BufferTx, LoRa.PAYLOAD_LEN);

		State = STATE_RX;
		break;

	case STATE_ECHO_RX:
		if (RxBufferSize > 0)
		{
//			printf("Acquired data: \"%s\"\n\r", BufferRx);

			uint8_t payloadIsAccurate = 1;
			for (uint16_t index = 0; index < LoRa.PAYLOAD_LEN; index++)
			{
				if (BufferRx[index] != BufferTx[index])
					payloadIsAccurate = 0;
			}

			if (payloadIsAccurate)
			{
				Collection[ConfigurationNum].PacketsReceived++;
			}
			else
			{
				Collection[ConfigurationNum].Measurements[MeasurementNum].ErrorCode = ERROR_PAYLOAD;
			}

			memset(BufferRx, 0, MAX_APP_BUFFER_SIZE);
			RxBufferSize = 0;

			State = STATE_NEXT_MEASUREMENT;
		}
		break;

	case STATE_NEXT_MEASUREMENT:
		if (++MeasurementNum >= MEASUREMENTS_NUM)
		{
			State = STATE_END_MEASUREMENTS;
		}
		else
		{
			State = STATE_TX;
		}
		break;

	case STATE_END_MEASUREMENTS:
		MeasurementNum = 0;

		printf("Configuration %u done, SET RECEIVER IN NEXT CONFIG, waiting on input for new configuration...\n\r", ConfigurationNum + 1);
		while (1)
		{
			HAL_Delay(50);
			if (HAL_GPIO_ReadPin(BUTTON_SW1_GPIO_PORT, BUTTON_SW1_PIN) == GPIO_PIN_RESET)
			{
				while (1)
				{
					HAL_Delay(50);
					if (HAL_GPIO_ReadPin(BUTTON_SW1_GPIO_PORT, BUTTON_SW1_PIN) == GPIO_PIN_SET)
						break;
				}
				State = STATE_NEXT_CONFIGURATION;
				break;
			}
		}
		break;

	case STATE_NEXT_CONFIGURATION:
		if (++ConfigurationNum >= CONFIGURATIONS_NUM)
		{
			State = STATE_END_CONFIGURATIONS;
		}
		else
		{
			SetLoRaConfiguration(ConfigurationNum);
			State = STATE_TX;
		}
		break;

	case STATE_END_CONFIGURATIONS:
		ConfigurationNum = 0;

		CalculatePDR();
		ExportMeasurementsAsCSV();

		printf("END OF PROGRAM\n\r");
		while (1)
		{
			HAL_Delay(500);
			HAL_GPIO_TogglePin(LED1_GPIO_PORT, LED1_PIN);
		}
		break;

	default:
		break;
	}

	// Set the next step
	UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process),
			CFG_SEQ_Prio_0);
}


static void SetLoRaConfiguration(uint8_t NewConfigurationNum)
{
	printf("New configuration number: %u\n\r", (ConfigurationNum + 1));

	switch (NewConfigurationNum)
	{
	case 0:
		LoRa.LORA_BANDWIDTH = 0;
		LoRa.LORA_SPREADING_FACTOR = 7;
		LoRa.LORA_CODINGRATE = 1;
		LoRa.LORA_PREAMBLE_LENGTH = 8;
		LoRa.LORA_SYMBOL_TIMEOUT = 5;
		LoRa.LORA_FIX_LENGTH_PAYLOAD_ON = false;
		LoRa.LORA_IQ_INVERSION_ON = false;
		LoRa.PAYLOAD_LEN = 64;
		LoRa.TX_TIMEOUT_VALUE = 15000;
		break;

	case 1:
		LoRa.LORA_BANDWIDTH = 1;
		LoRa.LORA_SPREADING_FACTOR = 7;
		LoRa.LORA_CODINGRATE = 1;
		LoRa.LORA_PREAMBLE_LENGTH = 8;
		LoRa.LORA_SYMBOL_TIMEOUT = 5;
		LoRa.LORA_FIX_LENGTH_PAYLOAD_ON = false;
		LoRa.LORA_IQ_INVERSION_ON = false;
		LoRa.PAYLOAD_LEN = 64;
		LoRa.TX_TIMEOUT_VALUE = 15000;
		break;

	case 2:
		LoRa.LORA_BANDWIDTH = 0;
		LoRa.LORA_SPREADING_FACTOR = 8;
		LoRa.LORA_CODINGRATE = 1;
		LoRa.LORA_PREAMBLE_LENGTH = 8;
		LoRa.LORA_SYMBOL_TIMEOUT = 5;
		LoRa.LORA_FIX_LENGTH_PAYLOAD_ON = false;
		LoRa.LORA_IQ_INVERSION_ON = false;
		LoRa.PAYLOAD_LEN = 64;
		LoRa.TX_TIMEOUT_VALUE = 15000;
		break;

	case 3:
		LoRa.LORA_BANDWIDTH = 0;
		LoRa.LORA_SPREADING_FACTOR = 9;
		LoRa.LORA_CODINGRATE = 1;
		LoRa.LORA_PREAMBLE_LENGTH = 8;
		LoRa.LORA_SYMBOL_TIMEOUT = 5;
		LoRa.LORA_FIX_LENGTH_PAYLOAD_ON = false;
		LoRa.LORA_IQ_INVERSION_ON = false;
		LoRa.PAYLOAD_LEN = 64;
		LoRa.TX_TIMEOUT_VALUE = 15000;
		break;

	case 4:
		LoRa.LORA_BANDWIDTH = 0;
		LoRa.LORA_SPREADING_FACTOR = 10;
		LoRa.LORA_CODINGRATE = 1;
		LoRa.LORA_PREAMBLE_LENGTH = 8;
		LoRa.LORA_SYMBOL_TIMEOUT = 5;
		LoRa.LORA_FIX_LENGTH_PAYLOAD_ON = false;
		LoRa.LORA_IQ_INVERSION_ON = false;
		LoRa.PAYLOAD_LEN = 64;
		LoRa.TX_TIMEOUT_VALUE = 15000;
		break;

	case 5:
		LoRa.LORA_BANDWIDTH = 0;
		LoRa.LORA_SPREADING_FACTOR = 11;
		LoRa.LORA_CODINGRATE = 1;
		LoRa.LORA_PREAMBLE_LENGTH = 8;
		LoRa.LORA_SYMBOL_TIMEOUT = 5;
		LoRa.LORA_FIX_LENGTH_PAYLOAD_ON = false;
		LoRa.LORA_IQ_INVERSION_ON = false;
		LoRa.PAYLOAD_LEN = 64;
		LoRa.TX_TIMEOUT_VALUE = 15000;
		break;

	case 6:
		LoRa.LORA_BANDWIDTH = 0;
		LoRa.LORA_SPREADING_FACTOR = 12;
		LoRa.LORA_CODINGRATE = 1;
		LoRa.LORA_PREAMBLE_LENGTH = 8;
		LoRa.LORA_SYMBOL_TIMEOUT = 5;
		LoRa.LORA_FIX_LENGTH_PAYLOAD_ON = false;
		LoRa.LORA_IQ_INVERSION_ON = false;
		LoRa.PAYLOAD_LEN = 64;
		LoRa.TX_TIMEOUT_VALUE = 15000;
		break;

	case 7:
		LoRa.LORA_BANDWIDTH = 0;
		LoRa.LORA_SPREADING_FACTOR = 7;
		LoRa.LORA_CODINGRATE = 2;
		LoRa.LORA_PREAMBLE_LENGTH = 8;
		LoRa.LORA_SYMBOL_TIMEOUT = 5;
		LoRa.LORA_FIX_LENGTH_PAYLOAD_ON = false;
		LoRa.LORA_IQ_INVERSION_ON = false;
		LoRa.PAYLOAD_LEN = 64;
		LoRa.TX_TIMEOUT_VALUE = 15000;
		break;

	case 8:
		LoRa.LORA_BANDWIDTH = 0;
		LoRa.LORA_SPREADING_FACTOR = 7;
		LoRa.LORA_CODINGRATE = 3;
		LoRa.LORA_PREAMBLE_LENGTH = 8;
		LoRa.LORA_SYMBOL_TIMEOUT = 5;
		LoRa.LORA_FIX_LENGTH_PAYLOAD_ON = false;
		LoRa.LORA_IQ_INVERSION_ON = false;
		LoRa.PAYLOAD_LEN = 64;
		LoRa.TX_TIMEOUT_VALUE = 15000;
		break;

	case 9:
		LoRa.LORA_BANDWIDTH = 0;
		LoRa.LORA_SPREADING_FACTOR = 7;
		LoRa.LORA_CODINGRATE = 4;
		LoRa.LORA_PREAMBLE_LENGTH = 8;
		LoRa.LORA_SYMBOL_TIMEOUT = 5;
		LoRa.LORA_FIX_LENGTH_PAYLOAD_ON = false;
		LoRa.LORA_IQ_INVERSION_ON = false;
		LoRa.PAYLOAD_LEN = 64;
		LoRa.TX_TIMEOUT_VALUE = 15000;
		break;

	case 10:
		LoRa.LORA_BANDWIDTH = 0;
		LoRa.LORA_SPREADING_FACTOR = 7;
		LoRa.LORA_CODINGRATE = 1;
		LoRa.LORA_PREAMBLE_LENGTH = 8;
		LoRa.LORA_SYMBOL_TIMEOUT = 5;
		LoRa.LORA_FIX_LENGTH_PAYLOAD_ON = false;
		LoRa.LORA_IQ_INVERSION_ON = false;
		LoRa.PAYLOAD_LEN = 8;
		LoRa.TX_TIMEOUT_VALUE = 15000;
		break;

	case 11:
		LoRa.LORA_BANDWIDTH = 0;
		LoRa.LORA_SPREADING_FACTOR = 7;
		LoRa.LORA_CODINGRATE = 1;
		LoRa.LORA_PREAMBLE_LENGTH = 8;
		LoRa.LORA_SYMBOL_TIMEOUT = 5;
		LoRa.LORA_FIX_LENGTH_PAYLOAD_ON = false;
		LoRa.LORA_IQ_INVERSION_ON = false;
		LoRa.PAYLOAD_LEN = 255;
		LoRa.TX_TIMEOUT_VALUE = 15000;
		break;

	default:
		printf("Exceeded configuration limit!\n\r");
		LoRa.LORA_BANDWIDTH = 0;
		LoRa.LORA_SPREADING_FACTOR = 7;
		LoRa.LORA_CODINGRATE = 1;
		LoRa.LORA_PREAMBLE_LENGTH = 8;
		LoRa.LORA_SYMBOL_TIMEOUT = 5;
		LoRa.LORA_FIX_LENGTH_PAYLOAD_ON = false;
		LoRa.LORA_IQ_INVERSION_ON = false;
		LoRa.PAYLOAD_LEN = 64;
		LoRa.TX_TIMEOUT_VALUE = 15000;
		break;
	}

	Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LoRa.LORA_BANDWIDTH,
			LoRa.LORA_SPREADING_FACTOR, LoRa.LORA_CODINGRATE,
			LoRa.LORA_PREAMBLE_LENGTH, LoRa.LORA_FIX_LENGTH_PAYLOAD_ON,
			true, 0, 0, LoRa.LORA_IQ_INVERSION_ON, LoRa.TX_TIMEOUT_VALUE);

	if (LoRa.LORA_FIX_LENGTH_PAYLOAD_ON == false)
	{
		Radio.SetRxConfig(MODEM_LORA, LoRa.LORA_BANDWIDTH,
				LoRa.LORA_SPREADING_FACTOR, LoRa.LORA_CODINGRATE, 0,
				LoRa.LORA_PREAMBLE_LENGTH, LoRa.LORA_SYMBOL_TIMEOUT,
				LoRa.LORA_FIX_LENGTH_PAYLOAD_ON, 0, true, 0, 0,
				LoRa.LORA_IQ_INVERSION_ON, true);
	}
	else
	{
		Radio.SetRxConfig(MODEM_LORA, LoRa.LORA_BANDWIDTH,
				LoRa.LORA_SPREADING_FACTOR, LoRa.LORA_CODINGRATE, 0,
				LoRa.LORA_PREAMBLE_LENGTH, LoRa.LORA_SYMBOL_TIMEOUT,
				LoRa.LORA_FIX_LENGTH_PAYLOAD_ON, LoRa.PAYLOAD_LEN, true, 0, 0,
				LoRa.LORA_IQ_INVERSION_ON, true);
	}

	Radio.SetMaxPayloadLength(MODEM_LORA, MAX_APP_BUFFER_SIZE);

	memset(BufferTx, 0x0, MAX_APP_BUFFER_SIZE);
	memset(BufferRx, 0x0, MAX_APP_BUFFER_SIZE);
	RxBufferSize = 0;
}


static void CalculatePDR(void)
{
	for (uint8_t configurationIndex = 0;
			configurationIndex < CONFIGURATIONS_NUM; configurationIndex++)
	{
		Collection[configurationIndex].PacketDeliveryRatio = ((float)(Collection[configurationIndex].PacketsReceived) / MEASUREMENTS_NUM);
	}
}


static void ExportMeasurementsAsCSV(void)
{
	printf("\nExported data:\n\n\r");

	// first CSV
	printf("CSV:PDR_data\n\r");
	printf("Config,PacketsReceived,PacketDeliveryRatio\n\r");
	for (uint8_t config = 0; config < CONFIGURATIONS_NUM; config++)
	{
		printf("%u,%u,%u\n\r", config, Collection[config].PacketsReceived,
				(uint8_t)(Collection[config].PacketDeliveryRatio * 100));
	}
	printf("\n\r");

	// second CSV
	printf("CSV:Measurements_data\n\r");
	printf("Config,Measurement,RTT_ms,RSSI_dBm,SNR_dB,Bitrate_bps,ErrorCode\n\r");
	for (uint8_t config = 0; config < CONFIGURATIONS_NUM; config++)
	{
		for (uint8_t i = 0; i < MEASUREMENTS_NUM; i++)
		{
			SubGhz_Measurements_t *m = &Collection[config].Measurements[i];

			printf("%u,%u,%lu,%d,%d,%u,%u\n\r", config, i,
					(unsigned long )m->RoundTripTime, m->RssiValue, m->SnrValue,
					m->TxBitRate, m->ErrorCode);
		}
	}
	printf("\n\r");
}


/* USER CODE END PrFD */
