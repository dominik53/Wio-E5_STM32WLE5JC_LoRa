/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    lora_app.c
  * @author  MCD Application Team
  * @brief   Application of the LRWAN Middleware
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
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
#include "lora_app.h"
#include "stm32_seq.h"
#include "stm32_timer.h"
#include "utilities_def.h"
#include "app_version.h"
#include "lorawan_version.h"
#include "subghz_phy_version.h"
#include "lora_info.h"
#include "LmHandler.h"
#include "adc_if.h"
#include "CayenneLpp.h"
#include "sys_sensors.h"
#include "flash_if.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* External variables ---------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Private typedef -----------------------------------------------------------*/
/**
  * @brief LoRa State Machine states
  */
typedef enum TxEventType_e
{
  /**
    * @brief Appdata Transmission issue based on timer every TxDutyCycleTime
    */
  TX_ON_TIMER,
  /**
    * @brief Appdata Transmission external event plugged on OnSendEvent( )
    */
  TX_ON_EVENT
  /* USER CODE BEGIN TxEventType_t */

  /* USER CODE END TxEventType_t */
} TxEventType_t;

/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/**
  * LEDs period value of the timer in ms
  */
#define LED_PERIOD_TIME 500

/**
  * Join switch period value of the timer in ms
  */
#define JOIN_TIME 2000

/*---------------------------------------------------------------------------*/
/*                             LoRaWAN NVM configuration                     */
/*---------------------------------------------------------------------------*/
/**
  * @brief LoRaWAN NVM Flash address
  * @note last 2 sector of a 128kBytes device
  */
#define LORAWAN_NVM_BASE_ADDRESS                    ((void *)0x0803F000UL)

/* USER CODE BEGIN PD */
static const char *slotStrings[] = { "1", "2", "C", "C_MC", "P", "P_MC" };

#define MAX_PHY_PAYLOAD_LEN	255
#define TX_RETRY_PERIOD	100
#define MEASUREMENT_PERIOD	16000	// ile czakamy na nastepny cykl tx-rx
#define FIRST_MEASUREMENT_PERIOD	60000	// pierwszy raz czekamy dluzej zeby join i ustalanie RXC zeszlo z kolejki
#define CONFIGURATIONS_NUM	1

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

static const uint8_t txMessage[MAX_PHY_PAYLOAD_LEN] =
	{"Moja wiadomosc do przeprowadzania testow konfiguracji lora, ta wiadomosc jest przesylana z nadajnika do odbiornika i spowrotem jako echo w celu zbadania Round Trip Time. Dzieki temu w nadajniku mozliwe jest zbieranie danych wlasciwych dla nadajnika i odb"};
static uint8_t rxMessage[MAX_PHY_PAYLOAD_LEN];
static uint8_t rxMessageLen = 0;

static UTIL_TIMER_Object_t TxRetryTimer;
static UTIL_TIMER_Object_t measurementTimer;
static uint8_t loggedBusy = 0;	// flaga zeby ograniczyc spam na uart
static uint8_t rxTimeoutCnt = 0;	// info ktory rx obslugujemy, w klasach a i b przed rozpoczeciem nowego pomiaru czegamy na RX2
static uint8_t echoMode = 0;	// only send echo to gate
static uint8_t startMeasurements = 0; // LORA_TRANSMITER flag to start measuring
static uint8_t waitingForNode2rx = 0;	// czekamy na odp z drugiego urzadzenia
static uint8_t receivedNode2Frame = 0;	// czy dostalismy odp z node
static uint8_t firstMeasurement = 1;	// wykkonujemy pierwszy pomiar

#if LORA_DIRECTION == LORA_RECEIVER
static uint8_t txPending = 0;	// after rx for echo tx
#endif

static uint32_t txTimestamp = 0; /* transmitted timestamp */
static uint32_t txTimestampEnd = 0; /* end of transmition timestamp */
static uint32_t rxTimestamp = 0; /* received timestamp */

static uint8_t ConfigurationNum = 0; // which LoRa configuration is tested now
static uint8_t MeasurementNum = 0; // which configurations measurement is tested now

static SubGhz_MeasurementsCollection_t Collection[CONFIGURATIONS_NUM];


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private function prototypes -----------------------------------------------*/
/**
  * @brief  LoRa End Node send request
  */
static void SendTxData(void);

/**
  * @brief  TX timer callback function
  * @param  context ptr of timer context
  */
static void OnTxTimerEvent(void *context);

/**
  * @brief  join event callback function
  * @param  joinParams status of join
  */
static void OnJoinRequest(LmHandlerJoinParams_t *joinParams);

/**
  * @brief callback when LoRaWAN application has sent a frame
  * @brief  tx event callback function
  * @param  params status of last Tx
  */
static void OnTxData(LmHandlerTxParams_t *params);

/**
  * @brief callback when LoRaWAN application has received a frame
  * @param appData data received in the last Rx
  * @param params status of last Rx
  */
static void OnRxData(LmHandlerAppData_t *appData, LmHandlerRxParams_t *params);

/**
  * @brief callback when LoRaWAN Beacon status is updated
  * @param params status of Last Beacon
  */
static void OnBeaconStatusChange(LmHandlerBeaconParams_t *params);

/**
  * @brief callback when system time has been updated
  */
static void OnSysTimeUpdate(void);

/**
  * @brief callback when LoRaWAN application Class is changed
  * @param deviceClass new class
  */
static void OnClassChange(DeviceClass_t deviceClass);

/**
  * @brief  LoRa store context in Non Volatile Memory
  */
static void StoreContext(void);

/**
  * @brief  stop current LoRa execution to switch into non default Activation mode
  */
static void StopJoin(void);

/**
  * @brief  Join switch timer callback function
  * @param  context ptr of Join switch context
  */
static void OnStopJoinTimerEvent(void *context);

/**
  * @brief  Notifies the upper layer that the NVM context has changed
  * @param  state Indicates if we are storing (true) or restoring (false) the NVM context
  */
static void OnNvmDataChange(LmHandlerNvmContextStates_t state);

/**
  * @brief  Store the NVM Data context to the Flash
  * @param  nvm ptr on nvm structure
  * @param  nvm_size number of data bytes which were stored
  */
static void OnStoreContextRequest(void *nvm, uint32_t nvm_size);

/**
  * @brief  Restore the NVM Data context from the Flash
  * @param  nvm ptr on nvm structure
  * @param  nvm_size number of data bytes which were restored
  */
static void OnRestoreContextRequest(void *nvm, uint32_t nvm_size);

/**
  * Will be called each time a Radio IRQ is handled by the MAC layer
  *
  */
static void OnMacProcessNotify(void);

/**
  * @brief Change the periodicity of the uplink frames
  * @param periodicity uplink frames period in ms
  * @note Compliance test protocol callbacks
  */
static void OnTxPeriodicityChanged(uint32_t periodicity);

/**
  * @brief Change the confirmation control of the uplink frames
  * @param isTxConfirmed Indicates if the uplink requires an acknowledgement
  * @note Compliance test protocol callbacks
  */
static void OnTxFrameCtrlChanged(LmHandlerMsgTypes_t isTxConfirmed);

/**
  * @brief Change the periodicity of the ping slot frames
  * @param pingSlotPeriodicity ping slot frames period in ms
  * @note Compliance test protocol callbacks
  */
static void OnPingSlotPeriodicityChanged(uint8_t pingSlotPeriodicity);

/**
  * @brief Will be called to reset the system
  * @note Compliance test protocol callbacks
  */
static void OnSystemReset(void);

/* USER CODE BEGIN PFP */
static void logInitData(void); // log software data
static void waitForInput(const char *message1, const char *message2);
//static GPIO_PinState checkInput2(void);
static void OnTxRetryTimerEvent(void *context);
static void OnMeasurementTimerEvent(void *context);
static void ExportMeasurementsAsCSV(void);
static void CalculatePDR(void);
static void nextMeasurement(uint8_t incMeasurement);


/* USER CODE END PFP */

/* Private variables ---------------------------------------------------------*/
/**
  * @brief LoRaWAN default activation type
  */
static ActivationType_t ActivationType = LORAWAN_DEFAULT_ACTIVATION_TYPE;

/**
  * @brief LoRaWAN force rejoin even if the NVM context is restored
  */
static bool ForceRejoin = LORAWAN_FORCE_REJOIN_AT_BOOT;

/**
  * @brief LoRaWAN handler Callbacks
  */
static LmHandlerCallbacks_t LmHandlerCallbacks =
{
  .GetBatteryLevel =              GetBatteryLevel,
  .GetTemperature =               GetTemperatureLevel,
  .GetUniqueId =                  GetUniqueId,
  .GetDevAddr =                   GetDevAddr,
  .OnRestoreContextRequest =      OnRestoreContextRequest,
  .OnStoreContextRequest =        OnStoreContextRequest,
  .OnMacProcess =                 OnMacProcessNotify,
  .OnNvmDataChange =              OnNvmDataChange,
  .OnJoinRequest =                OnJoinRequest,
  .OnTxData =                     OnTxData,
  .OnRxData =                     OnRxData,
  .OnBeaconStatusChange =         OnBeaconStatusChange,
  .OnSysTimeUpdate =              OnSysTimeUpdate,
  .OnClassChange =                OnClassChange,
  .OnTxPeriodicityChanged =       OnTxPeriodicityChanged,
  .OnTxFrameCtrlChanged =         OnTxFrameCtrlChanged,
  .OnPingSlotPeriodicityChanged = OnPingSlotPeriodicityChanged,
  .OnSystemReset =                OnSystemReset,
};

/**
  * @brief LoRaWAN handler parameters
  */
static LmHandlerParams_t LmHandlerParams =
{
  .ActiveRegion =             ACTIVE_REGION,
  .DefaultClass =             LORAWAN_DEFAULT_CLASS,
  .AdrEnable =                LORAWAN_ADR_STATE,
  .IsTxConfirmed =            LORAWAN_DEFAULT_CONFIRMED_MSG_STATE,
  .TxDatarate =               LORAWAN_DEFAULT_DATA_RATE,
  .TxPower =                  LORAWAN_DEFAULT_TX_POWER,
  .PingSlotPeriodicity =      LORAWAN_DEFAULT_PING_SLOT_PERIODICITY,
  .RxBCTimeout =              LORAWAN_DEFAULT_CLASS_B_C_RESP_TIMEOUT
};

/**
  * @brief Type of Event to generate application Tx
  */
static TxEventType_t EventType = TX_ON_EVENT;

/**
  * @brief Timer to handle the application Tx
  */
static UTIL_TIMER_Object_t TxTimer;

/**
  * @brief Tx Timer period
  */
static UTIL_TIMER_Time_t TxPeriodicity = APP_TX_DUTYCYCLE;

/**
  * @brief Join Timer period
  */
static UTIL_TIMER_Object_t StopJoinTimer;

/* USER CODE BEGIN PV */
/**
  * @brief User application buffer
  */
static uint8_t AppDataBuffer[LORAWAN_APP_DATA_BUFFER_MAX_SIZE];

/**
  * @brief User application data structure
  */
static LmHandlerAppData_t AppData = { 0, 0, AppDataBuffer };

/* USER CODE END PV */

/* Exported functions ---------------------------------------------------------*/
/* USER CODE BEGIN EF */

/* USER CODE END EF */

void LoRaWAN_Init(void)
{
  /* USER CODE BEGIN LoRaWAN_Init_LV */
	APP_LOG(TS_ON, VLEVEL_M, "PROGRAM BEGAN\n\r");
  /* USER CODE END LoRaWAN_Init_LV */

  /* USER CODE BEGIN LoRaWAN_Init_1 */
	logInitData(); /* Get LoRaWAN APP version*/

	if (FLASH_IF_Init(NULL) != FLASH_IF_OK)
	{
		Error_Handler();
	}

  /* USER CODE END LoRaWAN_Init_1 */

  UTIL_TIMER_Create(&StopJoinTimer, JOIN_TIME, UTIL_TIMER_ONESHOT, OnStopJoinTimerEvent, NULL);

  UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_LmHandlerProcess), UTIL_SEQ_RFU, LmHandlerProcess);

  UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_LoRaSendOnTxTimerOrButtonEvent), UTIL_SEQ_RFU, SendTxData);
  UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_LoRaStoreContextEvent), UTIL_SEQ_RFU, StoreContext);
  UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_LoRaStopJoinEvent), UTIL_SEQ_RFU, StopJoin);

  /* Init Info table used by LmHandler*/
  LoraInfo_Init();

  /* Init the Lora Stack*/
  LmHandlerInit(&LmHandlerCallbacks, APP_VERSION);

  LmHandlerConfigure(&LmHandlerParams);

  /* USER CODE BEGIN LoRaWAN_Init_2 */
	UTIL_TIMER_Create(&TxRetryTimer, TX_RETRY_PERIOD, UTIL_TIMER_ONESHOT, OnTxRetryTimerEvent, NULL);
	UTIL_TIMER_Create(&measurementTimer, FIRST_MEASUREMENT_PERIOD, UTIL_TIMER_ONESHOT, OnMeasurementTimerEvent, NULL);

	LmHandlerJoin(ActivationType, ForceRejoin);
//    if(LORAWAN_DEFAULT_CLASS == CLASS_A)
//    {
//    	APP_LOG(TS_OFF, VLEVEL_M, "default class A\r\n");
//
//    }
//    else
//    {
//    	LmHandlerErrorStatus_t status = LmHandlerRequestClass(LORAWAN_DEFAULT_CLASS);
//    	APP_LOG(TS_OFF, VLEVEL_M, "requested to change class to %c, status: %d\r\n", "ABC"[LORAWAN_DEFAULT_CLASS], status);
////    	LoRaMacStart();
//    }

	if (EventType == TX_ON_TIMER)
	{
		/* send every time timer elapses */
		UTIL_TIMER_Create(&TxTimer, 5000, UTIL_TIMER_ONESHOT, OnTxTimerEvent, NULL);
		UTIL_TIMER_Start(&TxTimer);
	}
	else if (EventType == TX_ON_EVENT)
	{

	}

	return;
  /* USER CODE END LoRaWAN_Init_2 */

  LmHandlerJoin(ActivationType, ForceRejoin);

  if (EventType == TX_ON_TIMER)
  {
    /* send every time timer elapses */
    UTIL_TIMER_Create(&TxTimer, TxPeriodicity, UTIL_TIMER_ONESHOT, OnTxTimerEvent, NULL);
    UTIL_TIMER_Start(&TxTimer);
  }
  else
  {
    /* USER CODE BEGIN LoRaWAN_Init_3 */

    /* USER CODE END LoRaWAN_Init_3 */
  }

  /* USER CODE BEGIN LoRaWAN_Init_Last */

  /* USER CODE END LoRaWAN_Init_Last */
}

/* USER CODE BEGIN PB_Callbacks */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
//  switch (GPIO_Pin)
//  {
//    case  BUT1_Pin:
//      /* Note: when "EventType == TX_ON_TIMER" this GPIO is not initialized */
//      if (EventType == TX_ON_EVENT)
//      {
//        UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_LoRaSendOnTxTimerOrButtonEvent), CFG_SEQ_Prio_0);
//      }
//      break;
//    case  BUT2_Pin:
//      UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_LoRaStopJoinEvent), CFG_SEQ_Prio_0);
//      break;
//    case  BUT3_Pin:
//      UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_LoRaStoreContextEvent), CFG_SEQ_Prio_0);
//      break;
//    default:
//      break;
//  }
}

/* USER CODE END PB_Callbacks */

/* Private functions ---------------------------------------------------------*/
/* USER CODE BEGIN PrFD */

/* USER CODE END PrFD */

static void OnRxData(LmHandlerAppData_t *appData, LmHandlerRxParams_t *params)
{
  /* USER CODE BEGIN OnRxData_1 */
#if APP_LOG_EVENTS_ENABLED == 1
	APP_LOG(TS_ON, VLEVEL_M, "OnRxData\n\r");
#endif

	rxTimestamp = HAL_GetTick();
	Collection[ConfigurationNum].Measurements[MeasurementNum].RoundTripTime = rxTimestamp - txTimestamp;

	uint8_t RxPort = 0;

	if (params != NULL)
	{
		if (params->IsMcpsIndication)
		{
			if (appData != NULL)
			{
				if(LORAWAN_DEFAULT_CLASS != CLASS_C)
				{
					Collection[ConfigurationNum].PacketsReceived++;
				}

				RxPort = appData->Port;
				if (appData->Buffer != NULL)
				{
					switch (appData->Port)
					{
					case LORAWAN_SWITCH_CLASS_PORT:
						/*this port switches the class*/
						if (appData->BufferSize == 1)
						{
							APP_LOG(TS_ON, VLEVEL_M, "Received switch class to %d\n\r", appData->Buffer[0]);
							LmHandlerErrorStatus_t changeClassStatus;

							switch (appData->Buffer[0])
							{
							case 0:
							{
								changeClassStatus = LmHandlerRequestClass(CLASS_A);
								break;
							}
							case 1:
							{
								changeClassStatus = LmHandlerRequestClass(CLASS_B);
								break;
							}
							case 2:
							{
								changeClassStatus = LmHandlerRequestClass(CLASS_C);
								break;
							}
							default:
								APP_LOG(TS_ON, VLEVEL_M, "ERROR default class\n\r");
								break;
							}

							APP_LOG(TS_ON, VLEVEL_M, "Class change status: %d\n\r", changeClassStatus);

							DeviceClass_t myClass;
							LmHandlerErrorStatus_t classStatus = LmHandlerGetCurrentClass(&myClass);
							APP_LOG(TS_ON, VLEVEL_M, "Class status: %d\n\r", classStatus);
							APP_LOG(TS_OFF, VLEVEL_M, "*** MY NEW CLASS: %d ***\n\r", myClass);
						}
						else
							APP_LOG(TS_ON, VLEVEL_M, "Received switch class but BufferSize != 1 !!!!!!!\n\r");

						break;
					case LORAWAN_USER_APP_PORT:
						if (appData->BufferSize > 0)
						{
#if LORA_DIRECTION == LORA_RECEIVER
							if(echoMode)
							{
								memcpy(rxMessage, appData->Buffer, appData->BufferSize);
								rxMessageLen = appData->BufferSize;
								txPending = 1;
							}
#else
							if(LORAWAN_DEFAULT_CLASS == CLASS_C)
							{
								rxTimestamp = HAL_GetTick();
								Collection[ConfigurationNum].Measurements[MeasurementNum].RoundTripTime = rxTimestamp - txTimestamp;
								Collection[ConfigurationNum].PacketsReceived++;
								APP_LOG(TS_ON, VLEVEL_M, "### OnRx ###\n\r");
								receivedNode2Frame = 1;
								if (appData->BufferSize <= APP_PAYLOAD_LEN)
								{
									uint8_t payloadIsAccurate = 1;
									for (uint16_t index = 0; index < APP_PAYLOAD_LEN - 1; index++)
									{
										if (appData->Buffer[index + 1] != txMessage[index]) // pierwszy bajt == 0
											payloadIsAccurate = 0;
									}

									if (!payloadIsAccurate)
									{
										Collection[ConfigurationNum].Measurements[MeasurementNum].ErrorCode = ERROR_PAYLOAD;
									}
								}
								else
								{
									APP_LOG(TS_ON, VLEVEL_M,
											"!!! appData->BufferSize <= APP_PAYLOAD_LEN NOT TRUE !!!\n\r");
									Collection[ConfigurationNum].Measurements[MeasurementNum].ErrorCode =
											ERROR_PAYLOAD;
								}
							}
#endif

							appData->Buffer[appData->BufferSize] = 0;
							APP_LOG(TS_ON, VLEVEL_M, "Received data: %s\n\r", appData->Buffer);
						}
						else
							APP_LOG(TS_ON, VLEVEL_M, "Received data, bufferSize: 0\n\r");
						break;

					default:
						APP_LOG(TS_ON, VLEVEL_M, "Received data on unknown port\n\r");
						break;
					}
				}
				else
					APP_LOG(TS_ON, VLEVEL_M, "Received appData->Buffer == NULL\n\r");
			}
			else
				APP_LOG(TS_ON, VLEVEL_M, "Received appData == NULL\n\r");
		}
		else
			APP_LOG(TS_ON, VLEVEL_M, "Received IsMcpsIndication == NULL\n\r");

		if (params->RxSlot < RX_SLOT_NONE)
		{
			Collection[ConfigurationNum].Measurements[MeasurementNum].RssiValue = params->Rssi;
			Collection[ConfigurationNum].Measurements[MeasurementNum].SnrValue = params->Snr;

			APP_LOG(TS_OFF, VLEVEL_H,
					"###### D/L FRAME:%04d | PORT:%d | DR:%d | SLOT:%s | RSSI:%d | SNR:%d\r\n",
					params->DownlinkCounter, RxPort, params->Datarate,
					slotStrings[params->RxSlot], params->Rssi, params->Snr);
		}
		else
		{
			Collection[ConfigurationNum].Measurements[MeasurementNum].RssiValue = 4;
			Collection[ConfigurationNum].Measurements[MeasurementNum].SnrValue = 4;
		}
	}
	else
		APP_LOG(TS_ON, VLEVEL_M, "Received params == NULL\n\r");

	nextMeasurement(1);
  /* USER CODE END OnRxData_1 */
}

static void SendTxData(void)
{
  /* USER CODE BEGIN SendTxData_1 */
	LmHandlerErrorStatus_t status = LORAMAC_HANDLER_ERROR;
	UTIL_TIMER_Time_t nextTxIn = 0;

	if (LmHandlerIsBusy() == false)
	{
		loggedBusy = 0;
		AppData.Port = LORAWAN_USER_APP_PORT;

		if(echoMode) // receiver w trybie echo
		{
			uint8_t i;
			for (i = 0; i < rxMessageLen; i++)
			{
				AppData.Buffer[i] = rxMessage[i];
			}
			AppData.BufferSize = i;
		}
		else // transmitter
		{
			uint8_t i;
			for (i = 0; i < APP_PAYLOAD_LEN; i++)
			{
				if (i != 0)
				{
					AppData.Buffer[i] = txMessage[i - 1];
				}
				else	// i == 0, pierwszy bajt to 0, tak bylo w przykladzie i mi nie przeszkadza
				{
					AppData.Buffer[i] = 0u;
				}
			}
			AppData.BufferSize = i;
		}

		if(startMeasurements)
		{
			waitingForNode2rx = 1;
			UTIL_TIMER_Start(&measurementTimer);
		}


		APP_LOG(TS_ON, VLEVEL_M, "### OnTimerStart ###\n\r");

		if(firstMeasurement)
			firstMeasurement = 0;
		else
			MeasurementNum++;

		txTimestamp = HAL_GetTick();
		status = LmHandlerSend(&AppData, LmHandlerParams.IsTxConfirmed, false);
		if (LORAMAC_HANDLER_SUCCESS == status)
		{
//			APP_LOG(TS_ON, VLEVEL_M, "SEND REQUEST\r\n");
			APP_LOG(TS_ON, VLEVEL_M, "### OnTx ###\n\r");
		}
		else if (LORAMAC_HANDLER_DUTYCYCLE_RESTRICTED == status)
		{
			nextTxIn = LmHandlerGetDutyCycleWaitTime();
			if (nextTxIn > 0)
			{
				APP_LOG(TS_ON, VLEVEL_L, "Next Tx in  : ~%d second(s)\r\n", (nextTxIn / 1000));
				// todo moze byc tutaj potrzebne: UTIL_TIMER_Start(&TxRetryTimer);
			}
		}
		else
			APP_LOG(TS_ON, VLEVEL_M, "ERROR: send status: %d\r\n", status);
	}
	else
	{
		if(!loggedBusy)
		{
			loggedBusy = 1;
			APP_LOG(TS_ON, VLEVEL_M, "LmHandler is busy!!!!!\r\n");
		}

		if(!waitingForNode2rx)
			UTIL_TIMER_Start(&TxRetryTimer);
	}

  /* USER CODE END SendTxData_1 */
}

static void OnTxTimerEvent(void *context)
{
  /* USER CODE BEGIN OnTxTimerEvent_1 */
#if APP_LOG_EVENTS_ENABLED == 1
	APP_LOG(TS_ON, VLEVEL_M, "OnTxTimerEvent\n\r");
#endif
	return;
  /* USER CODE END OnTxTimerEvent_1 */
  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_LoRaSendOnTxTimerOrButtonEvent), CFG_SEQ_Prio_0);

  /*Wait for next tx slot*/
  UTIL_TIMER_Start(&TxTimer);
  /* USER CODE BEGIN OnTxTimerEvent_2 */

  /* USER CODE END OnTxTimerEvent_2 */
}

/* USER CODE BEGIN PrFD_LedEvents */
// PRIVATE FUNCTIONS
static void logInitData(void)
{
#if APP_LOG_SOFTWARE_ENABLED
	uint32_t feature_version = 0UL;

	APP_LOG(TS_OFF, VLEVEL_M, "APPLICATION_VERSION: V%X.%X.%X\r\n",
			(uint8_t)(APP_VERSION_MAIN), (uint8_t)(APP_VERSION_SUB1),
			(uint8_t)(APP_VERSION_SUB2));

	/* Get MW LoRaWAN info */
	APP_LOG(TS_OFF, VLEVEL_M, "MW_LORAWAN_VERSION:  V%X.%X.%X\r\n",
			(uint8_t)(LORAWAN_VERSION_MAIN), (uint8_t)(LORAWAN_VERSION_SUB1),
			(uint8_t)(LORAWAN_VERSION_SUB2));

	/* Get MW SubGhz_Phy info */
	APP_LOG(TS_OFF, VLEVEL_M, "MW_RADIO_VERSION:    V%X.%X.%X\r\n",
			(uint8_t)(SUBGHZ_PHY_VERSION_MAIN),
			(uint8_t)(SUBGHZ_PHY_VERSION_SUB1),
			(uint8_t)(SUBGHZ_PHY_VERSION_SUB2));

	/* Get LoRaWAN Link Layer info */
	LmHandlerGetVersion(LORAMAC_HANDLER_L2_VERSION, &feature_version);
	APP_LOG(TS_OFF, VLEVEL_M, "L2_SPEC_VERSION:     V%X.%X.%X\r\n",
			(uint8_t )(feature_version >> 24),
			(uint8_t )(feature_version >> 16), (uint8_t )(feature_version >> 8));

	/* Get LoRaWAN Regional Parameters info */
	LmHandlerGetVersion(LORAMAC_HANDLER_REGION_VERSION, &feature_version);
	APP_LOG(TS_OFF, VLEVEL_M, "RP_SPEC_VERSION:     V%X-%X.%X.%X\r\n",
			(uint8_t )(feature_version >> 24),
			(uint8_t )(feature_version >> 16), (uint8_t )(feature_version >> 8),
			(uint8_t )(feature_version));
#endif
}


static void waitForInput(const char *message1, const char *message2)
{
	APP_LOG(TS_ON, VLEVEL_M, message1);
	while (1)
	{
		HAL_Delay(50);
		if (HAL_GPIO_ReadPin(BUTTON_SW1_GPIO_PORT, BUTTON_SW1_PIN)
				== GPIO_PIN_RESET)
		{
			while (1)
			{
				HAL_Delay(50);
				if (HAL_GPIO_ReadPin(BUTTON_SW1_GPIO_PORT, BUTTON_SW1_PIN)
						== GPIO_PIN_SET)
					break;
			}

			APP_LOG(TS_ON, VLEVEL_M, message2);
			break;
		}
	}
}


//static GPIO_PinState checkInput2(void)
//{
//	return HAL_GPIO_ReadPin(BUTTON_SW2_GPIO_PORT, BUTTON_SW2_PIN);
//}


static void OnTxRetryTimerEvent(void *context)
{
	nextMeasurement(0);
}

static void OnMeasurementTimerEvent(void *context)
{
	// start timer, w obsludze sprawdzmy czy przyszla wiadomosc, jesli nie to errorCode timeout i next, jesli tak to next
	APP_LOG(TS_ON, VLEVEL_M, "### OnMeasurementTimer ###\n\r");
	waitingForNode2rx = 0;

	if(receivedNode2Frame)
	{
		receivedNode2Frame = 0;
		APP_LOG(TS_OFF, VLEVEL_M, "Frame received\n\r");
	}
	else
	{
		APP_LOG(TS_OFF, VLEVEL_M, "Frame not received\n\r");
		Collection[ConfigurationNum].Measurements[MeasurementNum].ErrorCode = ERROR_TIMEOUT_RX;
	}

	UTIL_TIMER_Stop(&measurementTimer);
	UTIL_TIMER_SetPeriod(&measurementTimer, MEASUREMENT_PERIOD);

	nextMeasurement(1);
}

static void ExportMeasurementsAsCSV(void)
{
	APP_LOG(TS_OFF, VLEVEL_M, "\nExported data:\n\n\r");

	// first CSV
	APP_LOG(TS_OFF, VLEVEL_M, "CSV:PDR_data\n\r");
	APP_LOG(TS_OFF, VLEVEL_M, "Config,PacketsReceived,PacketDeliveryRatio\n\r");
	for (uint8_t config = 0; config < CONFIGURATIONS_NUM; config++)
	{
		APP_LOG(TS_OFF, VLEVEL_M, "%u,%u,%u\n\r", config, Collection[config].PacketsReceived,
				(uint8_t)(Collection[config].PacketDeliveryRatio * 100));
	}
	APP_LOG(TS_OFF, VLEVEL_M, "\n\r");

	// second CSV
	APP_LOG(TS_OFF, VLEVEL_M, "CSV:Measurements_data\n\r");
	APP_LOG(TS_OFF, VLEVEL_M, "Config,Measurement,RTT_ms,RSSI_dBm,SNR_dB,Bitrate_bps,ErrorCode\n\r");
	for (uint8_t config = 0; config < CONFIGURATIONS_NUM; config++)
	{
		for (uint8_t i = 0; i < MEASUREMENTS_NUM; i++)
		{
			SubGhz_Measurements_t *m = &Collection[config].Measurements[i];

			APP_LOG(TS_OFF, VLEVEL_M, "%d,%d,%d,%d,%d,%d,%d\n\r", config, i,
					(unsigned long )m->RoundTripTime, m->RssiValue, m->SnrValue,
					m->TxBitRate, m->ErrorCode);

			HAL_Delay(15);
		}
	}
	APP_LOG(TS_OFF, VLEVEL_M, "\n\r");
}


static void CalculatePDR(void)
{
	for (uint8_t configurationIndex = 0;
			configurationIndex < CONFIGURATIONS_NUM; configurationIndex++)
	{
		Collection[configurationIndex].PacketDeliveryRatio = ((float)(Collection[configurationIndex].PacketsReceived) / MEASUREMENTS_NUM);
	}
}


static void nextMeasurement(uint8_t incMeasurement)
{
	// reset timera na tx retry, zajmiemy sie tu obsluga tx
	UTIL_TIMER_Stop(&TxRetryTimer);
	UTIL_TIMER_SetPeriod(&TxRetryTimer, TX_RETRY_PERIOD);

	// zlecamy kolejny pomiar (w przypadku problemow z wczesniejszym zleceniem)
	if(incMeasurement == 0)
	{
		APP_LOG(TS_ON, VLEVEL_M, "### OnNextMeasurement0 ###\n\r");
		UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_LoRaSendOnTxTimerOrButtonEvent), CFG_SEQ_Prio_0);
		return;
	}



	// inkrementujemy numer pomiaru i sprwadzamy koniec
	if(MeasurementNum >= MEASUREMENTS_NUM && echoMode == 0)	// koniec badania
	{
		if(!waitingForNode2rx)
		{
			CalculatePDR();
			ExportMeasurementsAsCSV();
			APP_LOG(TS_ON, VLEVEL_M, "END OF PROGRAM\n\r");
		}
	}
	else	// kolejny pomiar
	{
		if(((MeasurementNum >= 5 || echoMode > 0) && LORAWAN_DEFAULT_CLASS == CLASS_C) || startMeasurements)
		{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET); // LED on

#if LORA_DIRECTION == LORA_TRANSMITER
			if (!startMeasurements)
			{
				APP_LOG(TS_ON, VLEVEL_M, "***** STOP CONTINOUS TX *****\n\r");

				MeasurementNum = 0;
				startMeasurements = 1;
			}

			if(waitingForNode2rx) // pomiar w trakcie, czekamy
			{
				return;
			}
#else
			echoMode = 1;
			MeasurementNum = 0;
			if(txPending)
			{
				UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_LoRaSendOnTxTimerOrButtonEvent), CFG_SEQ_Prio_0);
			}
			return;
#endif
		}
		APP_LOG(TS_ON, VLEVEL_M, "***** MEASUREMENT NUM %d *****\n\r", MeasurementNum);
		UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_LoRaSendOnTxTimerOrButtonEvent), CFG_SEQ_Prio_0);
	}
}






/* USER CODE END PrFD_LedEvents */

static void OnTxData(LmHandlerTxParams_t *params)
{
  /* USER CODE BEGIN OnTxData_1 */
#if APP_LOG_EVENTS_ENABLED == 1
	APP_LOG(TS_ON, VLEVEL_M, "OnTxData\n\r");
#endif

	txTimestampEnd = HAL_GetTick();
	Collection[ConfigurationNum].Measurements[MeasurementNum].TxBitRate = (APP_PAYLOAD_LEN * 8 * 1000) / (txTimestampEnd - txTimestamp); // *8 -> byte to bit, *1000 -> ms to s

  if ((params != NULL))
  {
    /* Process Tx event only if its a mcps response to prevent some internal events (mlme) */
    if (params->IsMcpsConfirm != 0)
    {
//      APP_LOG(TS_OFF, VLEVEL_M, "\r\n###### ========== MCPS-Confirm =============\r\n");
      APP_LOG(TS_OFF, VLEVEL_H, "###### U/L FRAME:%04d | PORT:%d | DR:%d | PWR:%d", params->UplinkCounter,
              params->AppData.Port, params->Datarate, params->TxPower);

      APP_LOG(TS_OFF, VLEVEL_H, " | MSG TYPE:");
      if (params->MsgType == LORAMAC_HANDLER_CONFIRMED_MSG)
      {
        APP_LOG(TS_OFF, VLEVEL_H, "CONFIRMED [%s]\r\n", (params->AckReceived != 0) ? "ACK" : "NACK");
      }
      else
      {
        APP_LOG(TS_OFF, VLEVEL_H, "UNCONFIRMED\r\n");
      }
    }
  }
  /* USER CODE END OnTxData_1 */
}

static void OnJoinRequest(LmHandlerJoinParams_t *joinParams)
{
  /* USER CODE BEGIN OnJoinRequest_1 */
#if APP_LOG_EVENTS_ENABLED == 1
	APP_LOG(TS_ON, VLEVEL_M, "OnJoinRequest\n\r");
#endif

	if (joinParams != NULL)
	{
		if (joinParams->Status == LORAMAC_HANDLER_SUCCESS)
		{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET); // LED on
			APP_LOG(TS_OFF, VLEVEL_M, "\r\n###### = JOINED = ");
			if (joinParams->Mode == ACTIVATION_TYPE_ABP)
			{
				APP_LOG(TS_OFF, VLEVEL_M, "ABP ======================\r\n");
			}
			else
			{
				APP_LOG(TS_OFF, VLEVEL_M, "OTAA =====================\r\n");
			}
		}
		else
		{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET); // LED off
			APP_LOG(TS_OFF, VLEVEL_M,
					"\r\n###### = JOIN FAILED, status: %d\r\n",
					joinParams->Status);
		}

		APP_LOG(TS_OFF, VLEVEL_H, "###### U/L FRAME:JOIN | DR:%d | PWR:%d\r\n",
				joinParams->Datarate, joinParams->TxPower);

		waitForInput("Press button1 to start...\n\r", "Started measurements\n\r");
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET); // LED off

		DeviceClass_t myClass;
		LmHandlerErrorStatus_t classStatus = LmHandlerGetCurrentClass(&myClass);
		APP_LOG(TS_ON, VLEVEL_M, "Class status: %d, my class: %d\n\r", classStatus, myClass);

		MeasurementNum = 0;
		nextMeasurement(0);
	}
  /* USER CODE END OnJoinRequest_1 */
}

static void OnBeaconStatusChange(LmHandlerBeaconParams_t *params)
{
  /* USER CODE BEGIN OnBeaconStatusChange_1 */
#if APP_LOG_EVENTS_ENABLED == 1
	APP_LOG(TS_ON, VLEVEL_M, "OnBeaconStatusChange\n\r");
#endif

  if (params != NULL)
  {
    switch (params->State)
    {
      default:
      case LORAMAC_HANDLER_BEACON_LOST:
      {
        APP_LOG(TS_OFF, VLEVEL_M, "\r\n###### BEACON LOST\r\n");
        break;
      }
      case LORAMAC_HANDLER_BEACON_RX:
      {
        APP_LOG(TS_OFF, VLEVEL_M,
                "\r\n###### BEACON RECEIVED | DR:%d | RSSI:%d | SNR:%d | FQ:%d | TIME:%d | DESC:%d | "
                "INFO:02X%02X%02X %02X%02X%02X\r\n",
                params->Info.Datarate, params->Info.Rssi, params->Info.Snr, params->Info.Frequency,
                params->Info.Time.Seconds, params->Info.GwSpecific.InfoDesc,
                params->Info.GwSpecific.Info[0], params->Info.GwSpecific.Info[1],
                params->Info.GwSpecific.Info[2], params->Info.GwSpecific.Info[3],
                params->Info.GwSpecific.Info[4], params->Info.GwSpecific.Info[5]);
        break;
      }
      case LORAMAC_HANDLER_BEACON_NRX:
      {
        APP_LOG(TS_OFF, VLEVEL_M, "\r\n###### BEACON NOT RECEIVED\r\n");
        break;
      }
    }
  }
  /* USER CODE END OnBeaconStatusChange_1 */
}

static void OnSysTimeUpdate(void)
{
  /* USER CODE BEGIN OnSysTimeUpdate_1 */
#if APP_LOG_EVENTS_ENABLED == 1
	APP_LOG(TS_ON, VLEVEL_M, "OnSysTimeUpdate\n\r");
#endif
  /* USER CODE END OnSysTimeUpdate_1 */
}

static void OnClassChange(DeviceClass_t deviceClass)
{
  /* USER CODE BEGIN OnClassChange_1 */
#if APP_LOG_EVENTS_ENABLED == 1
	APP_LOG(TS_ON, VLEVEL_M, "OnClassChange\n\r");
#endif

  APP_LOG(TS_OFF, VLEVEL_M, "Switch to Class %c done\r\n", "ABC"[deviceClass]);

//  LmHandlerJoin(ActivationType, ForceRejoin);

  /* USER CODE END OnClassChange_1 */
}

static void OnMacProcessNotify(void)
{
  /* USER CODE BEGIN OnMacProcessNotify_1 */
#if APP_LOG_EVENTS_ENABLED == 1
	APP_LOG(TS_ON, VLEVEL_M, "OnMacProcessNotify\n\r");
#endif
  /* USER CODE END OnMacProcessNotify_1 */
  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_LmHandlerProcess), CFG_SEQ_Prio_0);

  /* USER CODE BEGIN OnMacProcessNotify_2 */
  	if(lmHandler_getRxTimeout() > 0)
  	{
  		if(echoMode)
  		  	return;

  		if(LORAWAN_DEFAULT_CLASS == CLASS_A || LORAWAN_DEFAULT_CLASS == CLASS_B)
  		{
  			if(++rxTimeoutCnt >= 2)
  				rxTimeoutCnt = 0;
//  			else
//  				return;	// jeszcze nie konczymy pomiaru
  		}

  		// to koniec tego pomiaru
  		if (LORAWAN_DEFAULT_CONFIRMED_MSG_STATE == LORAMAC_HANDLER_UNCONFIRMED_MSG)
		{
			if (LORAWAN_DEFAULT_CLASS != CLASS_C)
			{
				rxTimestamp = HAL_GetTick();
				Collection[ConfigurationNum].Measurements[MeasurementNum].RoundTripTime = rxTimestamp - txTimestamp;
				Collection[ConfigurationNum].PacketsReceived++;
				APP_LOG(TS_ON, VLEVEL_M, "App RX timeout\n\r");
			}
		}
		else
		{
			Collection[ConfigurationNum].Measurements[MeasurementNum].ErrorCode = ERROR_TIMEOUT_RX;
			APP_LOG(TS_ON, VLEVEL_M, "App RX timeout on conf\n\r");
		}

  		nextMeasurement(1);
  	}
  	else
  	{
  		APP_LOG(TS_ON, VLEVEL_M, "No app RX timeout\n\r");
  	}

  /* USER CODE END OnMacProcessNotify_2 */
}

static void OnTxPeriodicityChanged(uint32_t periodicity)
{
  /* USER CODE BEGIN OnTxPeriodicityChanged_1 */
#if APP_LOG_EVENTS_ENABLED == 1
	APP_LOG(TS_ON, VLEVEL_M, "OnTxPeriodicityChanged\n\r");
#endif
  /* USER CODE END OnTxPeriodicityChanged_1 */
  TxPeriodicity = periodicity;

  if (TxPeriodicity == 0)
  {
    /* Revert to application default periodicity */
    TxPeriodicity = APP_TX_DUTYCYCLE;
  }

  /* Update timer periodicity */
  UTIL_TIMER_Stop(&TxTimer);
  UTIL_TIMER_SetPeriod(&TxTimer, TxPeriodicity);
  UTIL_TIMER_Start(&TxTimer);
  /* USER CODE BEGIN OnTxPeriodicityChanged_2 */

  /* USER CODE END OnTxPeriodicityChanged_2 */
}

static void OnTxFrameCtrlChanged(LmHandlerMsgTypes_t isTxConfirmed)
{
  /* USER CODE BEGIN OnTxFrameCtrlChanged_1 */
#if APP_LOG_EVENTS_ENABLED == 1
	APP_LOG(TS_ON, VLEVEL_M, "OnTxFrameCtrlChanged\n\r");
#endif
  /* USER CODE END OnTxFrameCtrlChanged_1 */
  LmHandlerParams.IsTxConfirmed = isTxConfirmed;
  /* USER CODE BEGIN OnTxFrameCtrlChanged_2 */

  /* USER CODE END OnTxFrameCtrlChanged_2 */
}

static void OnPingSlotPeriodicityChanged(uint8_t pingSlotPeriodicity)
{
  /* USER CODE BEGIN OnPingSlotPeriodicityChanged_1 */
#if APP_LOG_EVENTS_ENABLED == 1
	APP_LOG(TS_ON, VLEVEL_M, "OnPingSlotPeriodicityChanged\n\r");
#endif
  /* USER CODE END OnPingSlotPeriodicityChanged_1 */
  LmHandlerParams.PingSlotPeriodicity = pingSlotPeriodicity;
  /* USER CODE BEGIN OnPingSlotPeriodicityChanged_2 */

  /* USER CODE END OnPingSlotPeriodicityChanged_2 */
}

static void OnSystemReset(void)
{
  /* USER CODE BEGIN OnSystemReset_1 */
#if APP_LOG_EVENTS_ENABLED == 1
	APP_LOG(TS_ON, VLEVEL_M, "OnSystemReset\n\r");
#endif
  /* USER CODE END OnSystemReset_1 */
  if ((LORAMAC_HANDLER_SUCCESS == LmHandlerHalt()) && (LmHandlerJoinStatus() == LORAMAC_HANDLER_SET))
  {
    NVIC_SystemReset();
  }
  /* USER CODE BEGIN OnSystemReset_Last */

  /* USER CODE END OnSystemReset_Last */
}

static void StopJoin(void)
{
  /* USER CODE BEGIN StopJoin_1 */
#if APP_LOG_EVENTS_ENABLED == 1
	APP_LOG(TS_ON, VLEVEL_M, "StopJoin\n\r");
#endif
  /* USER CODE END StopJoin_1 */

  UTIL_TIMER_Stop(&TxTimer);

  if (LORAMAC_HANDLER_SUCCESS != LmHandlerStop())
  {
    APP_LOG(TS_OFF, VLEVEL_M, "LmHandler Stop on going ...\r\n");
  }
  else
  {
    APP_LOG(TS_OFF, VLEVEL_M, "LmHandler Stopped\r\n");
    if (LORAWAN_DEFAULT_ACTIVATION_TYPE == ACTIVATION_TYPE_ABP)
    {
      ActivationType = ACTIVATION_TYPE_OTAA;
      APP_LOG(TS_OFF, VLEVEL_M, "LmHandler switch to OTAA mode\r\n");
    }
    else
    {
      ActivationType = ACTIVATION_TYPE_ABP;
      APP_LOG(TS_OFF, VLEVEL_M, "LmHandler switch to ABP mode\r\n");
    }
    LmHandlerConfigure(&LmHandlerParams);
    LmHandlerJoin(ActivationType, true);
    UTIL_TIMER_Start(&TxTimer);
  }
  UTIL_TIMER_Start(&StopJoinTimer);
  /* USER CODE BEGIN StopJoin_Last */

  /* USER CODE END StopJoin_Last */
}

static void OnStopJoinTimerEvent(void *context)
{
  /* USER CODE BEGIN OnStopJoinTimerEvent_1 */
#if APP_LOG_EVENTS_ENABLED == 1
	APP_LOG(TS_ON, VLEVEL_M, "OnStopJoinTimerEvent\n\r");
#endif
  /* USER CODE END OnStopJoinTimerEvent_1 */
  if (ActivationType == LORAWAN_DEFAULT_ACTIVATION_TYPE)
  {
    UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_LoRaStopJoinEvent), CFG_SEQ_Prio_0);
  }
  /* USER CODE BEGIN OnStopJoinTimerEvent_Last */

  /* USER CODE END OnStopJoinTimerEvent_Last */
}

static void StoreContext(void)
{
  LmHandlerErrorStatus_t status = LORAMAC_HANDLER_ERROR;

  /* USER CODE BEGIN StoreContext_1 */

  /* USER CODE END StoreContext_1 */
  status = LmHandlerNvmDataStore();

  if (status == LORAMAC_HANDLER_NVM_DATA_UP_TO_DATE)
  {
    APP_LOG(TS_OFF, VLEVEL_M, "NVM DATA UP TO DATE\r\n");
  }
  else if (status == LORAMAC_HANDLER_ERROR)
  {
    APP_LOG(TS_OFF, VLEVEL_M, "NVM DATA STORE FAILED\r\n");
  }
  /* USER CODE BEGIN StoreContext_Last */

  /* USER CODE END StoreContext_Last */
}

static void OnNvmDataChange(LmHandlerNvmContextStates_t state)
{
  /* USER CODE BEGIN OnNvmDataChange_1 */
#if APP_LOG_EVENTS_ENABLED == 1
	APP_LOG(TS_ON, VLEVEL_M, "OnNvmDataChange\n\r");
#endif
  /* USER CODE END OnNvmDataChange_1 */
  if (state == LORAMAC_HANDLER_NVM_STORE)
  {
    APP_LOG(TS_OFF, VLEVEL_M, "NVM DATA STORED\r\n");
  }
  else
  {
    APP_LOG(TS_OFF, VLEVEL_M, "NVM DATA RESTORED\r\n");
  }
  /* USER CODE BEGIN OnNvmDataChange_Last */

  /* USER CODE END OnNvmDataChange_Last */
}

static void OnStoreContextRequest(void *nvm, uint32_t nvm_size)
{
  /* USER CODE BEGIN OnStoreContextRequest_1 */
#if APP_LOG_EVENTS_ENABLED == 1
	APP_LOG(TS_ON, VLEVEL_M, "OnStoreContextRequest\n\r");
#endif
  /* USER CODE END OnStoreContextRequest_1 */
  /* store nvm in flash */
  if (FLASH_IF_Erase(LORAWAN_NVM_BASE_ADDRESS, FLASH_PAGE_SIZE) == FLASH_IF_OK)
  {
    FLASH_IF_Write(LORAWAN_NVM_BASE_ADDRESS, (const void *)nvm, nvm_size);
  }
  /* USER CODE BEGIN OnStoreContextRequest_Last */

  /* USER CODE END OnStoreContextRequest_Last */
}

static void OnRestoreContextRequest(void *nvm, uint32_t nvm_size)
{
  /* USER CODE BEGIN OnRestoreContextRequest_1 */
#if APP_LOG_EVENTS_ENABLED == 1
	APP_LOG(TS_ON, VLEVEL_M, "OnRestoreContextRequest\n\r");
#endif
  /* USER CODE END OnRestoreContextRequest_1 */
  FLASH_IF_Read(nvm, LORAWAN_NVM_BASE_ADDRESS, nvm_size);
  /* USER CODE BEGIN OnRestoreContextRequest_Last */

  /* USER CODE END OnRestoreContextRequest_Last */
}

