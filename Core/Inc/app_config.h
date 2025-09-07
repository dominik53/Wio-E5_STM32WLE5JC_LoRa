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
//DR0 = 51	\	23	/ 51
//DR1 = 51	\	23	/ 51
//DR2 = 51	\	23	/ 51
//DR3 = 115	\	87	/ 51
//DR4 = 222	\	194	/ 51
//DR5 = 222	\	194	/ 51
#define APP_PAYLOAD_LEN		51	// ile bajtow danych przesylamy
#define MEASUREMENTS_NUM	50	// how many repetitions


#define APP_LOG_EVENTS_ENABLED		0 // log on app events
#define APP_LOG_SOFTWARE_ENABLED	0 // log software on app init

/*******************************************/
/*******************************************/


#endif /* INC_APP_CONFIG_H_ */
