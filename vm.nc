/* 
* Petros Kalos Mina Vouronikoy 
* Group 2
* Assignment 5
*/

#include "vm.h"

module vm {
	uses {
		interface Leds;
		interface Boot;

		interface Timer<TMilli> as Timer[int i];
		interface Timer<TMilli> as SendTimer;

		interface SplitControl as SControl;
  		interface Receive as SReceive;
  		interface AMSend as SAMSend;
		interface Packet as SPacket;
		
		interface SplitControl as RadioControl;
		interface Receive as Receive;
		interface AMSend as AMSend;
		interface Packet as AMPacket;
		interface Packet;

		interface Read<uint16_t>;
	}
}


implementation {
	void sendmsg(radio_msg *msg);
	bool msg_insert(radio_msg *msg);
	
	//Message queue
	radio_msg queue[QSIZE];
	uint8_t q_in=0,q_out=0;
	
	//Serial message
	message_t spacket;
	bool slocked = FALSE;
	
	
	//Rember and manage active queries in network	
	radio_msg buffer[BUFFSIZE];
	
	uint8_t next_app = 0;			//runs through app buffer 
	
	uint8_t msg_id = 1;

	typedef struct{ 			//app state
		nx_uint8_t app_id;
		unsigned char comm[MAXAPPSIZE];
		char r[MAXREG];
		unsigned char pc;
		unsigned char init_len; 
		unsigned char timer_len;
		unsigned char msg_len;
		unsigned char state;	//idle=2,active=1,halt=0
		unsigned char sensor;
		bool aggr_active;
		int aggr_delay;
		char msg_counter;
	}state;
	
	state app[MAXAPP];
	#ifdef SIM
	#define wait 10
	nx_uint8_t sapp1[0x0C]	= {0x0C,0x03,0x05,0x00,0xE0,wait,0x00,0xD7,0xF0,0xE0,wait,0x00};
	nx_uint8_t sapp2[0x10]	= {0x10,0x03,0x05,0x04,0xE0,wait,0x00,0xD7,0xF0,0xE0,wait,0x00,0x27,0x09,0xF0,0x00};
	nx_uint8_t sapp3[0x1C]	= {0x1C,0x07,0x0C,0x05,0x17,0x00,0x18,0x7F,0xE1,wait,0x00,0xD1,0x57,0x88,0x01,0xF1,0x17,0x00,0x18,0x7F,0xE1,wait,0x00,0x37,0x09,0x88,0x0A,0x00};
	#endif
	double waitled(int k){
		int i;
		double t = 3.14;
		if(k==2){	
			call Leds.led2Toggle();
			for(i=0;i<20000;i++)
				t *= t;
			
			call Leds.led2Toggle();
		}else if(k==1){	
			call Leds.led1Toggle();
			for(i=0;i<20000;i++)
				t *= t;
			
			call Leds.led1Toggle();
		}else if(k==0){	
			call Leds.led0Toggle();
			for(i=0;i<20000;i++)
				t *= t;
			
			call Leds.led0Toggle();
		}
		return t;
	}

	void serial_sendmsg(vm_msg *msg){
		if(slocked){
			return;
		}else{
			vm_msg* rcm = (vm_msg*)call SPacket.getPayload(&spacket, sizeof(vm_msg));
			if (rcm == NULL) {return;}

			if (call SPacket.maxPayloadLength() < sizeof(vm_msg)) {
				return;
			}
			
			memcpy(rcm,msg,sizeof(vm_msg));
			
			if (call SAMSend.send(AM_BROADCAST_ADDR, &spacket, sizeof(vm_msg)) == SUCCESS) {
				slocked = TRUE;
			}
		}
	}

	bool q_add(radio_msg *msg){
		if((q_in+1)%QSIZE == q_out) return 0;
		
		memcpy(&queue[q_in],msg,sizeof(radio_msg));
		q_in= (q_in+1)%QSIZE;
		
		if(call SendTimer.isRunning()==0){
			call SendTimer.startOneShot((TOS_NODE_ID+1)*MSG_DELAY);
		}
		return 1;
	}
	
	event void SendTimer.fired() {
		if(q_out != q_in ){
			sendmsg(&queue[q_out]);
			q_out = (q_out+1)%QSIZE;
			call SendTimer.startOneShot((TOS_NODE_ID+1)*MSG_DELAY);
		}
	}

	bool disassembler(uint8_t n_app){
		uint8_t pc= app[n_app].pc;
		//dbg("vm","%s.App %u, Dissasebler[%d] = %x\n",sim_time_string(),n_app,pc,app[n_app].comm[pc]);
		switch (app[n_app].comm[pc]){
			case 0x00 :{																		//ends handler execution
				if(call Timer.isRunning[n_app]()){
					if(app[n_app].msg_counter==0){			//check if messages waiting to serve
						app[n_app].state = 2;				//waiting for timer state
					}else{
						app[n_app].pc=app[n_app].init_len+app[n_app].timer_len;
						app[n_app].msg_counter = 0;
					}
				}
				else{
					if(app[n_app].timer_len==0)
						app[n_app].state = 0;			//application end
				}
				
				break;
			}
			case 0x11: case 0x12: case 0x13: case 0x14: case 0x15: case 0x16:case  0x17 :case  0x18:{					//rx = val
				uint8_t reg = (app[n_app].comm[pc] & 0x0F)-1;
				uint8_t val = app[n_app].comm[pc+1];
				
				app[n_app].r[reg] = val;
				app[n_app].pc+=2;
				break;
			}		
			case 0x21 :case  0x22 :case 0x23 :case  0x24 :case  0x25 :case   0x26 :case  0x27 :case  0x28:{			//rx = ry
				uint8_t reg1 = (app[n_app].comm[pc] & 0x0F)-1;
				uint8_t reg2 = app[n_app].comm[pc+1]-1;
				
				app[n_app].r[reg1] = app[n_app].r[reg2];
				app[n_app].pc+=2;
				break;
			}	
			case 0x31 :case  0x32 :case 0x33 :case  0x34 :case  0x35 :case   0x36 :case  0x37 :case  0x38:{		//rx = rx + ry
				uint8_t reg1 = (app[n_app].comm[pc] & 0x0F)-1;
				uint8_t reg2 = app[n_app].comm[pc+1]-1;
				
				app[n_app].r[reg1] += app[n_app].r[reg2];
				app[n_app].pc+=2;
				break;
			}	
			case 0x41 :case  0x42 :case 0x43 :case  0x44 :case  0x45 :case   0x46 :case  0x47 :case  0x48:{		//rx = rx - ry
				uint8_t reg1 = (app[n_app].comm[pc] & 0x0F)-1;
				uint8_t reg2 = app[n_app].comm[pc+1]-1;
				
				app[n_app].r[reg1] -= app[n_app].r[reg2];
				app[n_app].pc+=2;
				break;
			}
			case 0x51 :case  0x52 :case 0x53 :case  0x54 :case  0x55 :case   0x56 :case  0x57 :case  0x58:{			//rx = rx +1
				uint8_t reg = (app[n_app].comm[pc] & 0x0F)-1;
				app[n_app].r[reg]++;
				app[n_app].pc+=1;
				break;
			}	
			case 0x61 :case  0x62 :case 0x63 :case  0x64 :case  0x65 :case   0x66 :case  0x67 :case  0x68:{			//rx = rx – 1
				uint8_t reg = (app[n_app].comm[pc] & 0x0F)-1;
				app[n_app].r[reg]--;
				app[n_app].pc+=1;
				break;
			}
			case 0x71 :case  0x72 :case 0x73 :case  0x74 :case  0x75 :case   0x76 :case  0x77 :case  0x78:{			//rx = max(rx,ry)
				uint8_t reg1 = (app[n_app].comm[pc] & 0x0F)-1;
				uint8_t reg2 = app[n_app].comm[pc+1]-1;
				
				if(app[n_app].r[reg2]>app[n_app].r[reg1])
					app[n_app].r[reg1] = app[n_app].r[reg2];
		
				app[n_app].pc+=2;
				
				break;
			}	
			case 0x81 :case  0x82 :case 0x83 :case  0x84 :case  0x85 :case   0x86 :case  0x87 :case  0x88:{		//rx = min(rx,ry)
				uint8_t reg1 = (app[n_app].comm[pc] & 0x0F)-1;
				uint8_t reg2 = app[n_app].comm[pc+1]-1;
				
				if(app[n_app].r[reg2]<app[n_app].r[reg1])
					app[n_app].r[reg1] = app[n_app].r[reg2];
				
				app[n_app].pc+=2;
				
				break;
			}	
			case 0x91 :case  0x92 :case 0x93 :case  0x94 :case  0x95 :case   0x96 :case  0x97 :case  0x98:{			//if ( rx > 0 ) pc = pc + off
				
				uint8_t reg1 = (app[n_app].comm[pc] & 0x0F)-1;
				uint8_t off = app[n_app].comm[pc+1]+1;
				
				if(app[n_app].r[reg1]>0) app[n_app].pc+=off;
				else app[n_app].pc+=2;
				
				break;
			}	
			case 0xA1 :case  0xA2 :case 0xA3 :case  0xA4 :case  0xA5 :case   0xA6 :case  0xA7 :case  0xA8:{			//if ( rx == 0 ) pc = pc + off
				
				uint8_t reg1 = (app[n_app].comm[pc] & 0x0F)-1;
				uint8_t off = app[n_app].comm[pc+1]+1;
				if(app[n_app].r[reg1]==0){
					app[n_app].pc+=off;
				}
				else app[n_app].pc+=2;
				
				break;
			}
			case 0xB0 :{													//pc = pc + off
				uint8_t off = app[n_app].comm[pc+1]+1;
				app[n_app].pc+=off;
				break;
			}
			case 0xC0 :case 0xC1:{												//if ( val != 0 ) turn led on else turn led off
				uint8_t val = (app[n_app].comm[pc] & 0x0F);
				if(val != 0 ){ 
					if(n_app == 0) call Leds.led0On();
					else if(n_app ==1 ) call Leds.led1On();
					else call Leds.led2On();
					dbg("vm","%s.App %u, LedOn\n",sim_time_string(),n_app);
				}
				else{ 
					if(n_app == 0) call Leds.led0Off();
					else if(n_app ==1 ) call Leds.led1Off();
					else call Leds.led2Off();
					dbg("vm","%s.App %u, LedOff\n",sim_time_string(),n_app);
				}
				app[n_app].pc+=1;
				break;
			}
			case 0xD1 :case  0xD2 :case 0xD3 :case  0xD4 :case  0xD5 :case   0xD6 :case  0xD7 :case  0xD8:{					//	rx = current brightness value
				uint8_t reg1 = (app[n_app].comm[pc] & 0x0F);
				app[n_app].sensor = reg1;
				call Read.read();
				//dbg("vm","%s.App %u, Read sensor\n",sim_time_string(),n_app);
				//waitled(0);
				return 1;
			}
			case 0xE0 :{															//set timer to expire after val seconds (0 cancels the timer)
				uint8_t val = app[n_app].comm[pc+1];
				if(val!=0)
					call Timer.startOneShot[n_app](val*1000);
				else
					call Timer.stop[n_app]();
				app[n_app].aggr_active=FALSE;
				app[n_app].pc+=2;
				break;
			}
			case 0xE1 :{															//set timer to expire after (NON-hop) seconds
				uint8_t val = app[n_app].comm[pc+1];
				call Timer.startOneShot[n_app](val*1000+app[n_app].aggr_delay);
				//dbg("vm","%20s.Set aggregation timer to %d\n",sim_time_string(),val*1000+app[n_app].aggr_delay);
				app[n_app].aggr_active=TRUE;
				app[n_app].pc+=2;
				break;
			}
			case 0xF0 :case  0xF1:{													//send contents of r7-r8 towards the application sink;at the root, this instruction should send the message over serial to the PC 
				int i;
				for(i=0;i<BUFFSIZE;i++){
					if(buffer[i].app.app_id==app[n_app].app_id){
						if(app[n_app].comm[pc]== 0xF0){
							buffer[i].data[0] = app[n_app].r[6];
							buffer[i].data_cnt=1;
						}else{
							buffer[i].data[0] = app[n_app].r[6];
							buffer[i].data[1] = app[n_app].r[7];
							buffer[i].data_cnt=2;
						}
						buffer[i].flag = 1;
						buffer[i].sampler_id=TOS_NODE_ID;
						q_add(&buffer[i]);
						if(buffer[i].parent_id == TOS_NODE_ID){
							vm_msg temp;
							temp.app_id = app[n_app].app_id;
							temp.comm[0] = buffer[i].data_cnt;
							temp.comm[1] = buffer[i].data[0];
							temp.comm[2] = buffer[i].data[1];
							temp.comm[3] = buffer[i].sampler_id;
							serial_sendmsg(&temp);
							dbg("vm","DIS %20s.App %u, Message received from %d with Data[%d] %d-%d\n",sim_time_string(),
							buffer[i].app.app_id,buffer[i].sampler_id,buffer[i].data_cnt,buffer[i].data[0],buffer[i].data[1]);
						}

						break;
					}
				}
				if(i==BUFFSIZE) dbg("vm","Cannot find reversed path\n");
				app[n_app].pc+=1;
				break;
			}
		}
		return 0;
	}
		
	task void runTask() {
		int i;
		
		if(disassembler(next_app)) return;
		
		//search for active application
		for(i=(next_app+1)%MAXAPP; app[i].state != 1; i =(i+1)%MAXAPP){
			if(i==next_app){	//if no active applications
				return;
			}
		}
		
		next_app = i;
		
		post  runTask();
	}
	
	bool app_add(nx_uint8_t hop,nx_uint8_t app_id,nx_uint8_t *msg){
		int i;
		for(i=0;i<MAXAPP;i++){
			if(app[i].state==0 || app[i].app_id==app_id){
				app[i].pc = 0;
				app[i].app_id = app_id;
				app[i].init_len = msg[1];
				app[i].timer_len = msg[2];
				app[i].msg_len = msg[3];
				app[i].msg_counter = 0;
				app[i].aggr_delay = (TOS_NODE_ID+1)*MSG_DELAY+(NON-hop)*AGGR;
				
				memcpy(app[i].comm,&msg[4],msg[1]+msg[2]+msg[3]);
				app[i].state = 1;
				memset(app[i].r,0,MAXREG);
				next_app = i;
				post runTask();
				return 1;
			}
		}
		return 0;
	}
	
	event void Timer.fired[int i](){
		next_app = i;
		app[i].state=1;
		app[i].pc = app[i].init_len;
		post runTask();
	}
	
	event void Boot.booted(){
		int i;
		
		for(i=0;i<MAXAPP;i++){
			app[i].pc = 0;
			app[i].init_len = 0;
			app[i].timer_len = 0;
			app[i].state = 0;
			app[i].sensor = 0;
		}
		
		for(i=0;i<BUFFSIZE;i++){
			buffer[i].parent_id = 0; 	//Empty buffer
			buffer[i].msg_id = 0; 	//Empty buffer
		}
		#ifdef SIM
		{
			
		}
		#else
		{
			
		}
		#endif
		call SControl.start();
		call RadioControl.start();
	}

		
	/* Serial Communication */
	
	event void SAMSend.sendDone(message_t* bufPtr, error_t error) {
		if (&spacket == bufPtr){
			slocked = FALSE;
		}
	}
	
	event message_t* SReceive.receive(message_t* bufPtr, void* payload, uint8_t len) {
		if (len != sizeof(vm_msg)) {return bufPtr;}
		else {
			radio_msg msg;
			vm_msg* rcm = (vm_msg*)payload;

			msg.group_id = GROUP_ID;
			msg.parent_id = msg.sender_id = TOS_NODE_ID;
			msg.msg_id = msg_id++;
			msg.flag = 0;
			msg.hop = 0;
			msg.app.app_id = rcm->app_id;
			memcpy(msg.app.comm,rcm->comm,rcm->comm[0]);
			
			msg_insert(&msg);
			q_add(&msg);
			
			return bufPtr;
		}
	}
	
	event void SControl.startDone(error_t err) {
		vm_msg t;
		t.app_id = 0xAA;
		memset(&t.comm,0xFF,MAXAPPSIZE);
		t.comm[0] = 0;
		t.comm[1] = 0;
		t.comm[2] = 0;
		t.comm[3] = TOS_NODE_ID;
		

		serial_sendmsg(&t);
		
	}
	
	event void SControl.stopDone(error_t err) {
	}

	event void Read.readDone(error_t result, uint16_t data){
		if (result == SUCCESS){
			int i;
			for(i=0;i<MAXAPP;i++){
				if(app[i].sensor!=0){
					app[i].r[app[i].sensor-1] = (data>>9);

					app[i].pc+=1;
					app[i].sensor = 0;
				}
			}
				
			post runTask();
		}
	}
	
	default command void Timer.startPeriodic[int](uint32_t dt) {}
	default command void Timer.startOneShot[int](uint32_t dt) {}
	default command bool Timer.isRunning[int]() {return 0;}
	default command void Timer.stop[int]() {}
	
	//================================ Flooding Section =============================
	
	message_t packet;
	bool locked = FALSE;
	
	void sendmsg(radio_msg *msg){
		if (locked) {
			return;
		}else {
			radio_msg* tmsg = (radio_msg*)call Packet.getPayload(&packet, sizeof(radio_msg));
			memcpy(tmsg,msg,sizeof(radio_msg));
			
			if (call AMSend.send(AM_BROADCAST_ADDR, &packet, sizeof(radio_msg)) == SUCCESS) {
				locked = TRUE;
			}
		}
	}
	
	uint8_t msg_sender_id(radio_msg *msg){
		int i;
		for(i=0;i<BUFFSIZE;i++){
			if(buffer[i].parent_id == msg->parent_id
				&& buffer[i].msg_id == msg->msg_id) return buffer[i].sender_id;
		}
		return 255;
	}
	
	bool msg_insert(radio_msg *msg){
		uint8_t i;
		waitled(1);
		for(i=0;i<BUFFSIZE;i++){
			if(buffer[i].parent_id==0) break;
		}
		if(i==BUFFSIZE) return 0;
		
		app_add(msg->hop,msg->app.app_id,msg->app.comm);
		
		dbg("vm","App[%d] received and stored, hop %d\n",msg->app.app_id,msg->hop);
		
		memcpy(&buffer[i],msg,sizeof(radio_msg));
		return 1;
	}
	
	bool msg_exists(radio_msg *msg){
		int i;
		if(msg->parent_id == TOS_NODE_ID) return 1; //if it's my message
			
		for(i=0;i<BUFFSIZE;i++)
			if(buffer[i].parent_id == msg->parent_id
				&& buffer[i].msg_id == msg->msg_id) return 1;
			
			return 0;
	}
	
	event void RadioControl.startDone(error_t err) {
		#ifdef SIM
		if (err == SUCCESS) {
			if(TOS_NODE_ID==1){
				radio_msg msg;
				dbg("vm","\n\n======= Parent %u send query =======\n",TOS_NODE_ID);
				msg.group_id = GROUP_ID;
				msg.parent_id = msg.sender_id = TOS_NODE_ID;
				msg.msg_id = msg_id++;
				msg.flag = 0;
				msg.hop = 1;
				msg.app.app_id = 1;
				memcpy(msg.app.comm,sapp1,sapp1[0]);
				
				msg_insert(&msg);
				q_add(&msg);
			}
		}
		#endif
	}
	
	event void RadioControl.stopDone(error_t err) {}
	
	event void AMSend.sendDone(message_t* bufPtr, error_t error) {
		if (&packet == bufPtr){
			locked = FALSE;
		}else{
			dbg("vm","Send fail\n");
		}
	}
	
	int check_app(radio_msg *msg){
		int i;
		for(i=0;i<MAXAPP;i++){
			if(app[i].app_id == msg->app.app_id && app[i].msg_len!=0){ //if application exist and msg handler exist
				//dbg("vm","%20s.State %u, TEST\n",sim_time_string(),app[i].state);
				if(app[i].state==2){
					app[i].pc=app[i].init_len+app[i].timer_len;
					app[i].r[8] = msg->data[0];
					app[i].r[9] = msg->data[1];
					app[i].state = 1;
					next_app = i;
					post runTask();
				}else if(app[i].state==1){
					app[i].r[8] = msg->data[0];
					app[i].r[9] = msg->data[1];
					app[i].msg_counter = 1;
					dbg("vm","%20s.App %u, WAIT MESSAGE\n",sim_time_string(),i);
				}
				return i;
			}
		}
		return -1;
	}
	
	event message_t* Receive.receive(message_t* bufPtr, void* payload, uint8_t len) {
		if (len != sizeof(radio_msg)) { return bufPtr;}
		else {
			radio_msg* msg = (radio_msg*)payload;
			if (msg == NULL) {return bufPtr;}
			if(msg->group_id == GROUP_ID){
				if (msg->flag == 1){//Response message
					if (TOS_NODE_ID != msg->sender_id) return bufPtr;
					
					msg->sender_id = msg_sender_id(msg);//get current sender id
					
					if(msg->parent_id != TOS_NODE_ID){
						//decide if forward the message
						if(check_app(msg)==-1)
							q_add(msg);
					}else{
						int i;
						for(i=0;i<MAXAPP;i++)
							if(app[i].app_id == msg->app.app_id) break;
							
						if(app[i].aggr_active){
							check_app(msg);
						}else{
							vm_msg temp;
							temp.app_id = msg->app.app_id;
							temp.comm[0] = msg->data_cnt;
							temp.comm[1] = msg->data[0];
							temp.comm[2] = msg->data[1];
							temp.comm[3] = msg->sampler_id;
							
							serial_sendmsg(&temp);
							dbg("vm","REC %20s.App %u, Message received from %d with Data[%d] %d-%d\n",sim_time_string(),
								msg->app.app_id,msg->sampler_id,msg->data_cnt,msg->data[0],msg->data[1]);	
								
						}
					}
					return bufPtr;
				}
				
				if(msg_exists(msg)){
					return bufPtr;
				}
				
				msg->hop++;
				
				if(!msg_insert(msg)) dbg("vm","Message dropped\n");
				
				call Read.read();
				
				msg->sender_id = TOS_NODE_ID;
				
				q_add(msg);
			}
			return bufPtr;
		}
	}
	
}
