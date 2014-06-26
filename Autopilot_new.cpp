/*
 * Autopilot_new.cpp
 *
 *  Created on: Jun 26, 2014
 *      Author: Tanmay, Caleb
 *
 *
 *      New Version of Quadrotor autopilot code. Restarting with everything
 *      Step1:Read correct data from IMU
 *      Step2:Read receiver inputs
 */


///included
#include "predef.h"
#include <stdio.h>
#include <stdlib.h>
#include <basictypes.h>
#include <ucos.h>
#include <ctype.h>
#include <startnet.h>
#include <autoupdate.h>
#include <dhcpclient.h>
#include <taskmon.h>
#include <smarttrap.h>
#include <effs_fat/fat.h>
#include <effs_fat/multi_drive_mmc_mcf.h>
#include <effs_fat/effs_utils.h>
#include <sim.h>
#include <pins.h>
#include <ucosmcfc.h>
#include <pinconstant.h>
#include <HiResTimer.h>
#include <utils.h>
#include <constants.h>
#include <cfinter.h>
#include <math.h>
#include <serial.h>
#include <dspi.h> //needed for IMU communication
#include "PINS_Definations.h"

//IMU Commands Send the hex value to send the corresponding IMU values
#define xghigh 0x12
#define xglow  0x10
#define yghigh 0x16
#define yglow  0x14
#define zghigh 0x1A
#define	zglow  0x18
#define	xahigh 0x1E
#define xalow  0x1C
#define	yahigh 0x22
#define	yalow  0x20
#define	zahigh 0x26
#define	zalow  0x24
#define	xm     0x28
#define	ym     0x2A
#define	zm     0x2C
#define	barhigh 0x30
#define	barlow  0x2E
#define command_size 12

//Inner loop gains
#define innerloop_gain_pitch 10.0f
#define innerloop_gain_roll 10.0f
#define innerloop_gain_yaw 30.0f


extern "C"{
	void UserMain( void * pd);
	void SetIntc( int intc, long func, int source, int level);
}

//Global Variables Warning: Think twice before defining any global variable, Do you need it absolutely?

/**********HiResTimer************/
HiResTimer* ppm_timer;

/**********PPM Variables*********/
uint8_t ppmsync=0;
double pilot_channel[8]={0,0,0,0,0,0,0,0};
double current_time,previous_time,time_width;
uint8_t pulse_count=0;

/******Structures********/
//angle structure
typedef struct{
	double roll;
	double pitch;
	double yaw;
}ANGLE;


/*************************************Autopilot_loop*******************************************/

void Autopilot_loop(void *){ //runs inner loop and controls motor comamnds according to the control algorithm

	//IMU variables
	BYTE IMU_command[12]={xahigh,00,yahigh,00,zahigh,00,xghigh,00,yghigh,00,zghigh,00,};
	double IMU_data[6]={0,0,0,0,0,0};//0-Gyro Z,1-Accel X,2-Accel Y,3-Accel Z,4 Gyro X,5 Gyro Y;
	uint8_t IMU_raw[12]={0,0,0,0,0,0,0,0,0,0,0,0};

	uint16_t pilot_throttle=0;
	int16_t pilot_roll=0,pilot_yaw=0,pilot_pitch=0;

	ANGLE rate_error,attitude_control;
	uint16_t Auto1,Auto2,Auto3,Auto4;
	uint16_t pwmr=0;

	while(1){

		DSPIStart(3,IMU_command,IMU_raw,12,NULL);//start talking to IMU
		while(!DSPIdone(3)){};//wait for DSPI to finish

		IMU_data[0]=0.02*(short int)(IMU_raw[0]*256+IMU_raw[1]);//Z Gyro
		IMU_data[1]=0.00025*(short int)(IMU_raw[2]*256+IMU_raw[3]);//X Accel
		IMU_data[2]=0.00025*(short int)(IMU_raw[4]*256+IMU_raw[5]);//Y Accel
		IMU_data[3]=0.00025*(short int)(IMU_raw[6]*256+IMU_raw[7]);//Z Accel
		IMU_data[4]=0.02*(short int)(IMU_raw[8]*256+IMU_raw[9]);// X Gyro
		IMU_data[5]=0.02*(short int)(IMU_raw[10]*256+IMU_raw[11]);// Y Gyro


		//Parsing Pilot command
		pilot_throttle=pilot_channel[1]*P_FACTOR;
		if(pilot_throttle<T_LIMIT)
			pilot_throttle=T_LIMIT;
		if(pilot_throttle>T_LIMIT)
			pilot_throttle=1.2*pilot_throttle;

		pilot_roll=0.8*(0.0015-pilot_channel[0])*P_FACTOR;
		pilot_yaw=1*(0.0015-pilot_channel[3])*P_FACTOR;
		pilot_pitch=0.8*(0.0015-pilot_channel[2])*P_FACTOR;

		//Attitude rate error
		rate_error.pitch=-IMU_data[5];
		rate_error.roll=-IMU_data[4];
		rate_error.yaw=-IMU_data[0];

		//attitude control
		attitude_control.pitch=innerloop_gain_pitch*rate_error.pitch;///pitch control
		attitude_control.roll=innerloop_gain_roll*rate_error.roll;//roll control
		attitude_control.yaw=innerloop_gain_yaw*rate_error.yaw;//yaw control


		//X configuration Pitch control
		Auto1=pilot_throttle + attitude_control.pitch + pilot_pitch+attitude_control.yaw - attitude_control.roll + pilot_roll;
		Auto2=pilot_throttle + attitude_control.pitch + pilot_pitch-attitude_control.yaw + attitude_control.roll - pilot_roll;
		Auto3=pilot_throttle - attitude_control.pitch - pilot_pitch+attitude_control.yaw + attitude_control.roll - pilot_roll;
		Auto4=pilot_throttle - attitude_control.pitch - pilot_pitch-attitude_control.yaw - attitude_control.roll + pilot_roll;

		if (pilot_channel[4] > 0.0013)  //Gear Switch, Up kills the motor
		{
			if(Auto1<P_END)
				sim1.mcpwm.sm[0].val[3]=P_END;
			else if(Auto1>P_MAX2)
				sim1.mcpwm.sm[0].val[3]=P_MAX2;
			else
				//sim1.mcpwm.sm[0].val[3]=Auto1;
				sim1.mcpwm.sm[0].val[3]=Auto1;

			if(Auto2<P_END)
				sim1.mcpwm.sm[1].val[3]=P_END;
			else if(Auto2>P_MAX2)
				sim1.mcpwm.sm[1].val[3]=P_MAX2;
			else
				sim1.mcpwm.sm[1].val[3]=Auto2;

			if(Auto3<P_END)
				sim1.mcpwm.sm[2].val[3]=P_END;
			else if(Auto3>P_MAX2)
				sim1.mcpwm.sm[2].val[3]=P_MAX2;
			else
				sim1.mcpwm.sm[2].val[3]=Auto3;

			if(Auto4<P_END)
				sim1.mcpwm.sm[0].val[5]=P_END;
			else if(Auto4>P_MAX2)
				sim1.mcpwm.sm[0].val[5]=P_MAX2;
			else
				sim1.mcpwm.sm[0].val[5]=Auto4;
		}

		else{

			sim1.mcpwm.sm[0].val[3]=P_END;
			sim1.mcpwm.sm[1].val[3]=P_END;
			sim1.mcpwm.sm[2].val[3]=P_END;
			sim1.mcpwm.sm[0].val[5]=P_END;
		}

			pwmr=sim1.mcpwm.mcr;
			sim1.mcpwm.mcr |=LDOK;




	}//Autopilot_loop while loop

}//Autopilot_loop task main bracket

/**********************Interrupt for reading receiver data******************************/

INTERRUPT(PPM_read_interrupt,0x2200){

	sim2.eport.epfr = 0x04;//clearing interrupt flag

	/*
	 * 0-Roll Left->Right 1 ms->1.9 ms
	 * 1-Throttle Bottom->Top
	 * 2-Pitch Bottom->Top
	 * 3-Yaw Left->Right
	 * 4-Gear Up=1.2 ms Down-1.5 ms
	 * 5-Throttle cut Down=1 ms Up=1.9 ms
	 * 6-Pitch Trim Minus sign=1.9 Plus sign=1 ms
	 * 7- Default value=1.5 ms not mapped to anything
	 */

	current_time=ppm_timer->readTime();
	time_width=current_time-previous_time;
	J2[48]=J2[48]^J2[48];

	if((time_width>=0.006) && ppmsync==0)
	{
			ppmsync=1;
			pulse_count=0;
	}

	if(ppmsync==2)
	{
		pilot_channel[pulse_count]=time_width;
		pulse_count=pulse_count+1;
	}

	if(pulse_count==8)
	{
		pulse_count=0;
		ppmsync=0;
	}

	previous_time=current_time;
	if(ppmsync==1)
		ppmsync=2;

}//PPM_read_interrupt bracket


/*****************************UserMain***************************************************/

void UserMain( void* pd ){

	/////Usual Routine
	InitializeStack();
	OSChangePrio( MAIN_PRIO );
    EnableAutoUpdate();
	EnableTaskMonitor();
	EnableSmartTraps();

	initPINS();

	/**********************Start Serial Port**************/
	int fdDebug;//Serial fd variable
	SerialClose(7);
	fdDebug = OpenSerial( 7, 115200, 1, 8, eParityNone );//opening

	ReplaceStdio( 0, fdDebug ); // stdin via UART 7
	ReplaceStdio( 1, fdDebug ); // stdout via UART 7
	ReplaceStdio( 2, fdDebug ); // sterr via UART 7

	//Starting the Code
	iprintf("***********************Autopilot New Version 1 code************************\n");

	DSPIInit(3,2000000,16,0x01,1,1,1,0,0,0);//Initializing the Hardware to talk to IMU

	OSSimpleTaskCreate(Autopilot_loop,MAIN_PRIO+1);//Creating an Autopilot Task

	//Configuring and starting the timer to read receiver signal
	ppm_timer=HiResTimer::getHiResTimer(1);//timer for ppm
	ppm_timer->init();
	ppm_timer->start();

	SetIntc(0,(long)&PPM_read_interrupt,2,1);// Configuring external pins to interrupt and read receiver data
	sim2.eport.epier |= 0x04;// enabling the interrupt

	while(1){

		OSTimeDly(100);


	}//Main While Bracket
}///Main Bracket

