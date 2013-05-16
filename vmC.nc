/* 
* Petros Kalos Mina Vouronikoy 
* Group 2
* Assignment 5
*/

#include "vm.h"

configuration vmC {}

implementation {
	components MainC, vm as App, LedsC;

	components new TimerMilliC() as Timer0;
	components new TimerMilliC() as Timer1;
	components new TimerMilliC() as Timer2;

	components new TimerMilliC() as SendTimerC;

	components new DemoSensorC() as Sensor;

	components SerialActiveMessageC as SAM;
	components ActiveMessageC as AM;
	
	
	App.Boot -> MainC.Boot;
	App.Leds -> LedsC;

	App.Timer[0] -> Timer0;
	App.Timer[1] -> Timer1;
	App.Timer[2] -> Timer2;

	App.SendTimer -> SendTimerC;	

	App.SControl -> SAM;
  	App.SReceive -> SAM.Receive[AM_SERIAL_MSG];
  	App.SAMSend -> SAM.AMSend[AM_SERIAL_MSG];
	App.SPacket -> SAM;
	
	App.RadioControl -> AM;
	App.Receive -> AM.Receive[AM_RADIO_MSG];
	App.AMSend -> AM.AMSend[AM_RADIO_MSG];
	App.AMPacket -> AM;
	App.Packet -> AM;

	App.Read -> Sensor;
}

