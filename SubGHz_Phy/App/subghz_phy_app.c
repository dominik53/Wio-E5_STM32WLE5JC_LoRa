/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    subghz_phy_app.c
 * @author  Dominik Kijak
 * @brief   Taking measurements of LoRa communication, LoRa app layer
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

#if ENCRYPTION == ENCRYPTION_AES128_CTR_CMAC
#include "p2p_encryption.h"
#endif
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
	Error_Code_t ErrorCode;	/* 0 - 4  */

} SubGhz_Measurements_t;	// last received data

typedef struct
{
	uint8_t PacketsReceived; // for PacketDeliveryRatio
	float PacketDeliveryRatio;
	SubGhz_Measurements_t Measurements[MEASUREMENTS_NUM];

} SubGhz_MeasurementsCollection_t;// all measurements collected for 1 configuration, for example 20 repetitions; SubGhz_MeasurementsCollection_t Collection[12] -> 12 configurations, each 20 repetitions

typedef struct
{
	uint32_t LORA_BANDWIDTH;		// [0: 125 kHz, 1: 250 kHz, 2: 500 kHz, 3: Reserved]
	uint32_t LORA_SPREADING_FACTOR;	// [SF7..SF12]
	uint8_t LORA_CODINGRATE;		// [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
	uint16_t LORA_PREAMBLE_LENGTH;	// Same for Tx and Rx, middleware sets min 12 for SF5 and 6
	bool LORA_FIX_LENGTH_PAYLOAD_ON;
	bool LORA_IQ_INVERSION_ON;
	uint32_t TX_TIMEOUT_VALUE;
	uint16_t LORA_SYMBOL_TIMEOUT;	// Symbols
	uint8_t PAYLOAD_LEN;			// 8 - 256
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

#if LOG_RX_DATA
static uint32_t CommTickCnt = 0; // num of received packages
#endif

static SubGhz_MeasurementsCollection_t Collection[CONFIGURATIONS_NUM];
static LoRaConfiguration_t LoRa;
static SubGhz_State_t State;

#if DEBUG_TRANSMITTER
uint8_t txSemaphore = 1;
#endif

#if ENCRYPTION == ENCRYPTION_AES128_CTR_CMAC
// encryption
p2penc_ctx_t ctx;	// personal info
uint32_t fcnt;		// frame counter
size_t out_len;		// encrypted frame length
size_t used_payload_len;	// payload lenth taken for encryption

// decryption
size_t decrypt_len;
uint32_t decrypt_fcnt;
uint8_t decrypt_fport;
#endif

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
#if ENCRYPTION == ENCRYPTION_AES128_CTR_CMAC
		p2penc_init(&ctx, DEVICE_ADDRESS, P2PENC_DIR_UPLINK, nwk_s_key, app_s_key);
#endif
  /* USER CODE END SubghzApp_Init_1 */

  /* Radio initialization */
  RadioEvents.TxDone = OnTxDone;
  RadioEvents.RxDone = OnRxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
  RadioEvents.RxTimeout = OnRxTimeout;
  RadioEvents.RxError = OnRxError;

  Radio.Init(&RadioEvents);

  /* USER CODE BEGIN SubghzApp_Init_2 */
#if DEBUG_LORAWAN && LORA_DIRECTION == LORA_RECEIVER
  Radio.SetPublicNetwork(true);
#endif
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
#if DEBUG_TRANSMITTER
	printf("TX time: %lu\n\r", (txTimestampEnd - txTimestamp));
	txSemaphore = 1;
	return;
#endif

#if ENCRYPTION == ENCRYPTION_AES128_CTR_CMAC
	Collection[ConfigurationNum].Measurements[MeasurementNum].TxBitRate =
			(used_payload_len * 8 * 1000) / (txTimestampEnd - txTimestamp); // *8 -> byte to bit, *1000 -> ms to s
#else
	Collection[ConfigurationNum].Measurements[MeasurementNum].TxBitRate =
			(LoRa.PAYLOAD_LEN * 8 * 1000) / (txTimestampEnd - txTimestamp); // *8 -> byte to bit, *1000 -> ms to s
#endif

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
	else
	{
		printf("RxBufferSize too large\n\r");
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
#if DEBUG_TRANSMITTER
	txSemaphore = 1;
	return;
#endif
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
#if DEBUG_TRANSMITTER
		if(txSemaphore)
		{
			txSemaphore = 0;
			txTimestamp = HAL_GetTick();  // start transmission
			Radio.Send(BufferTx, LoRa.PAYLOAD_LEN);
		}
		else
			memcpy(BufferTx, txMessage, LoRa.PAYLOAD_LEN);

		break;
#endif

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

#if ENCRYPTION == ENCRYPTION_AES128_CTR_CMAC
		int rc = p2penc_build_frame(&ctx, fcnt, F_PORT, txMessage, LoRa.PAYLOAD_LEN, LoRa.PAYLOAD_LEN, BufferTx, &out_len, &used_payload_len);
		printf("Encryption result: %d, frame_cnt: %lu, encrypted frame len: %u, payload used: %u/%u \n\r", rc, fcnt, out_len, used_payload_len, LoRa.PAYLOAD_LEN);
#else
		memcpy(BufferTx, txMessage, LoRa.PAYLOAD_LEN);
#endif

		HAL_Delay(Radio.GetWakeupTime());

		HAL_GPIO_TogglePin(LED1_GPIO_PORT, LED1_PIN);
		txTimestamp = HAL_GetTick();  // start transmission

#if ENCRYPTION == ENCRYPTION_AES128_CTR_CMAC
		Radio.Send(BufferTx, out_len);
		fcnt += 2;	// plus 1 for this frame, another +1 for echo frame
#else
		Radio.Send(BufferTx, LoRa.PAYLOAD_LEN);
#endif

		State = STATE_ECHO_RX;
		break;

	case STATE_RX:
		if (RxBufferSize > 0)
		{
			HAL_GPIO_TogglePin(LED1_GPIO_PORT, LED1_PIN);

#if ENCRYPTION == ENCRYPTION_AES128_CTR_CMAC
			decrypt_len = sizeof(rxMessage);
			int result = p2penc_parse_and_decrypt(&ctx, BufferRx, LoRa.PAYLOAD_LEN, rxMessage, &decrypt_len, &decrypt_fcnt, &decrypt_fport);
			printf("Decryption result: %d, received data len: %u\n\r", result, RxBufferSize);
#else
			memcpy(rxMessage, BufferRx, MAX_APP_BUFFER_SIZE);
#endif


#if LOG_RX_DATA
			// print received data
			printf("Acquired data %lu: \"", ++CommTickCnt);
#if ENCRYPTION == ENCRYPTION_AES128_CTR_CMAC
			for(uint16_t i = 0; i < decrypt_len; i++)
#else
			for(uint16_t i = 0; i < RxBufferSize; i++)
#endif
			{
#if DEBUG_LORAWAN == 1
				if(i != 0)
					printf(" ");
				printf("0x%02X", rxMessage[i]);
			}
			printf("\"\n\r");
#else
				printf("%c", (unsigned char)rxMessage[i]);
			}
			printf("\"\n\r");
#endif
#endif

			memset(BufferRx, 0, MAX_APP_BUFFER_SIZE);
			RxBufferSize = 0;

#if DEBUG_LORAWAN == 0 && LORA_DIRECTION == LORA_RECEIVER
			State = STATE_ECHO_TX;
#endif
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
//		printf("Waiting for Radio.GetWakeupTime(): %lu\n\r", Radio.GetWakeupTime());
		HAL_Delay(Radio.GetWakeupTime());

		memset(BufferTx, 0, MAX_APP_BUFFER_SIZE);

#if ENCRYPTION == ENCRYPTION_AES128_CTR_CMAC
		int results = p2penc_build_frame(&ctx, (decrypt_fcnt + 1), F_PORT, rxMessage, LoRa.PAYLOAD_LEN, LoRa.PAYLOAD_LEN, BufferTx, &out_len, &used_payload_len);
		printf("Encryption result: %d, frame_cnt: %lu, encrypted frame len: %u, payload used: %u/%u \n\r", results, (decrypt_fcnt + 1), out_len, used_payload_len, LoRa.PAYLOAD_LEN);
		Radio.Send(BufferTx, out_len);
#else
		memcpy(BufferTx, rxMessage, LoRa.PAYLOAD_LEN);
		Radio.Send(BufferTx, LoRa.PAYLOAD_LEN);
#endif

		State = STATE_RX;
		break;

	case STATE_ECHO_RX:
		if (RxBufferSize > 0)
		{
#if ENCRYPTION == ENCRYPTION_AES128_CTR_CMAC
			decrypt_len = sizeof(rxMessage);
			int resulte = p2penc_parse_and_decrypt(&ctx, BufferRx, LoRa.PAYLOAD_LEN, rxMessage, &decrypt_len, &decrypt_fcnt, &decrypt_fport);
			printf("Decryption result: %d, decrypt_len: %u\n\r", resulte, decrypt_len);
#endif

//			printf("Acquired data: \"%s\"\n\r", BufferRx);

			uint8_t payloadIsAccurate = 1;

#if ENCRYPTION == ENCRYPTION_AES128_CTR_CMAC
			for (uint16_t index = 0; index < decrypt_len; index++)
			{
				if (rxMessage[index] != txMessage[index])
					payloadIsAccurate = 0;
			}
#else
			for (uint16_t index = 0; index < LoRa.PAYLOAD_LEN; index++)
			{
				if (BufferRx[index] != BufferTx[index])
					payloadIsAccurate = 0;
			}
#endif

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


static void SetLoRaConfiguration(uint8_t NewConfigurationNum) // todo zmienic konfiguracje i pomyslec czy reszta zostaje bez zmian np czy dodac obsluge zmiany konfiguracji
{
	printf("New configuration number: %u\n\r", (ConfigurationNum + 1));

	switch (NewConfigurationNum)
	{
	case 0:

#if DEBUG_LORAWAN
		// RX1, FREQ: 868500000
		LoRa.LORA_BANDWIDTH = 0;
		LoRa.LORA_SPREADING_FACTOR = 12;
		LoRa.LORA_CODINGRATE = 1;
		LoRa.LORA_PREAMBLE_LENGTH = 8;
		LoRa.LORA_SYMBOL_TIMEOUT = 5;
		LoRa.LORA_FIX_LENGTH_PAYLOAD_ON = false;
		LoRa.LORA_IQ_INVERSION_ON = true;
		LoRa.PAYLOAD_LEN = 33;
		LoRa.TX_TIMEOUT_VALUE = 15000;

		// RX2, FREQ: 869525000
//		LoRa.LORA_BANDWIDTH = 0;
//		LoRa.LORA_SPREADING_FACTOR = 12;
//		LoRa.LORA_CODINGRATE = 1;
//		LoRa.LORA_PREAMBLE_LENGTH = 8;
//		LoRa.LORA_SYMBOL_TIMEOUT = 5;
//		LoRa.LORA_FIX_LENGTH_PAYLOAD_ON = false;
//		LoRa.LORA_IQ_INVERSION_ON = true;
//		LoRa.PAYLOAD_LEN = 33;
//		LoRa.TX_TIMEOUT_VALUE = 15000;
#else
		LoRa.LORA_BANDWIDTH = 1;
		LoRa.LORA_SPREADING_FACTOR = 7;
		LoRa.LORA_CODINGRATE = 1;
		LoRa.LORA_PREAMBLE_LENGTH = 8;
		LoRa.LORA_SYMBOL_TIMEOUT = 5;
		LoRa.LORA_FIX_LENGTH_PAYLOAD_ON = false;
		LoRa.LORA_IQ_INVERSION_ON = false;
		LoRa.PAYLOAD_LEN = 255;
		LoRa.TX_TIMEOUT_VALUE = 15000;
#endif

//		LoRa.LORA_BANDWIDTH = 1;
//		LoRa.LORA_SPREADING_FACTOR = 7;
//		LoRa.LORA_CODINGRATE = 1;
//		LoRa.LORA_PREAMBLE_LENGTH = 8;
//		LoRa.LORA_SYMBOL_TIMEOUT = 5;
//		LoRa.LORA_FIX_LENGTH_PAYLOAD_ON = false;
//		LoRa.LORA_IQ_INVERSION_ON = false;
//		LoRa.PAYLOAD_LEN = 8;
//		LoRa.TX_TIMEOUT_VALUE = 15000;

//		LoRa.LORA_BANDWIDTH = 0;
//		LoRa.LORA_SPREADING_FACTOR = 12;
//		LoRa.LORA_CODINGRATE = 4;
//		LoRa.LORA_PREAMBLE_LENGTH = 8;
//		LoRa.LORA_SYMBOL_TIMEOUT = 5;
//		LoRa.LORA_FIX_LENGTH_PAYLOAD_ON = false;
//		LoRa.LORA_IQ_INVERSION_ON = false;
//		LoRa.PAYLOAD_LEN = 8;
//		LoRa.TX_TIMEOUT_VALUE = 15000;


		break;

	case 1:
		LoRa.LORA_BANDWIDTH = 1;
		LoRa.LORA_SPREADING_FACTOR = 7;
		LoRa.LORA_CODINGRATE = 1;
		LoRa.LORA_PREAMBLE_LENGTH = 8;
		LoRa.LORA_SYMBOL_TIMEOUT = 5;
		LoRa.LORA_FIX_LENGTH_PAYLOAD_ON = false;
		LoRa.LORA_IQ_INVERSION_ON = false;
		LoRa.PAYLOAD_LEN = 16;
		LoRa.TX_TIMEOUT_VALUE = 15000;
		break;

	case 2:
		LoRa.LORA_BANDWIDTH = 0;
		LoRa.LORA_SPREADING_FACTOR = 12;
		LoRa.LORA_CODINGRATE = 4;
		LoRa.LORA_PREAMBLE_LENGTH = 8;
		LoRa.LORA_SYMBOL_TIMEOUT = 5;
		LoRa.LORA_FIX_LENGTH_PAYLOAD_ON = false;
		LoRa.LORA_IQ_INVERSION_ON = false;
		LoRa.PAYLOAD_LEN = 16;
		LoRa.TX_TIMEOUT_VALUE = 15000;
		break;

	default:
		printf("Soft error, exceeded configuration limit!\n\r");
		return;
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
