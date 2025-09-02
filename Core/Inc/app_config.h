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




/************ APP CONFIGURATION ************/
/*******************************************/
#define LORA_DIRECTION LORA_TRANSMITER	// which of the two devices I am

//MAX APP PAYLOAD
//DR0 = 23
//DR1 = 23
//DR2 = 23
//DR3 = 87
//DR4 = 194
//DR5 = 194
#define APP_PAYLOAD_LEN		23	// ile bajtow danych przesylamy
#define MEASUREMENTS_NUM	50	// how many repetitions


#define APP_LOG_EVENTS_ENABLED		0 // log on app events
#define APP_LOG_SOFTWARE_ENABLED	0 // log software on app init

/*******************************************/
/*******************************************/


#endif /* INC_APP_CONFIG_H_ */
