/*
 * app_config.h
 *
 *  Created on: Apr 28, 2025
 *      Author: Dominik
 */

#ifndef INC_APP_CONFIG_H_
#define INC_APP_CONFIG_H_

#define LORA_TRANSMITER 0
#define LORA_RECEIVER	1

#define ENCRYPTION_NONE	0
#define ENCRYPTION_AES128_CTR_CMAC	1


/************ APP CONFIGURATION ************/
/*******************************************/
#define LORA_DIRECTION	LORA_RECEIVER

#define ENCRYPTION	ENCRYPTION_AES128_CTR_CMAC


#define APP_LOG_ENABLED	1
#define DEBUG_LORAWAN	0		// nasłuchiwanie ramek, receiver only
#define DEBUG_TRANSMITTER	0	// ciągłe wysyłanie ramek, transmiter only
#define LOG_RX_DATA	0			// czy wypisywac na uart odebrane dane

/*******************************************/
/*******************************************/


#endif /* INC_APP_CONFIG_H_ */
