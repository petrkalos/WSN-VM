/* 
* Petros Kalos Mina Vouronikoy 
* Group 2
* Assignment 5
*/

#ifndef RADIO_SENSE_TO_LEDS_H
#define RADIO_SENSE_TO_LEDS_H

#define TOSH_DATA_LENGTH 100
#define MAXAPPSIZE		82
#define MAXAPP 			3

#define BUFFSIZE 		5
#define GROUP_ID 		2

#define NON				3
#define MAXREG			10
#define MSG_DELAY		10

#define AGGR			80
//Size of msg queue
#define QSIZE 			30

//#define SIM				//use code for simulation only

typedef nx_struct {
	nx_uint8_t app_id;
	nx_uint8_t comm[MAXAPPSIZE];
} vm_msg;

typedef nx_struct radio_msg {
	nx_uint8_t 	group_id;		//group id
	nx_uint8_t 	parent_id;		//source id
	nx_uint8_t 	msg_id;			//msg id
	nx_uint8_t 	sender_id;		//sender id
	nx_uint8_t	hop;
	nx_uint8_t 	flag;
	
	nx_uint8_t	sampler_id;
	
	nx_uint16_t data_cnt;
	nx_uint16_t data[2];
	
	vm_msg		app;
} radio_msg;

enum {
	AM_SERIAL_MSG=10,
	AM_RADIO_MSG = 11
};

#endif
