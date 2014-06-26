/*
 * PINS_Definations.cpp
 *
 *  Created on: Jun 26, 2014
 *      Author: Tanmay, Caleb
 */

#include "PINS_Definations.h"
#include <basictypes.h>
#include <sim.h>
#include <pins.h>

void initPINS()
{
	uint16_t pwmr=0;

	J2[48].function(0);//led D2,GPIO
	J2[48]=0;

	J2[16].function(2);//uart7 RX
	J2[20].function(2);//uart7 TX
	J2[39].function(2);//RX 8
	J2[42].function(2);//TX 8

	J2[21].function(1);//SPI3 Input
	J2[22].function(1);//SPI3 Out
	J2[23].function(1);//SPI3 chip select 0
	J2[24].function(1);//SPI3 clock



	J2[25].function(2);//PWMA0-1
	J2[30].function(2);//PWMA1-2
	J2[35].function(2);//PWMA2-3
	J2[31].function(2);//PWMA3-4
	J2[27].function(2);//PWMB0-5
	J2[40].function(2);//PWMB1-6
	J2[28].function(2);//PWMB2-7
	J2[19].function(2);//PWMB3-8

	//AND and Multiplexer pins
	J2[15].function(0);//CH7 Enable
	J2[17].function(0);//CH1 Enable
	J2[18].function(0);//CH2 Enable
	J2[37].function(0);//CH3 Enable
	//J2[43].function(0);//CH4 Enable
	J2[45].function(0);//CH5 Enable
	J2[47].function(0);//CH6 Enable
	J1[5].function(0);//CH8 Enable
	J1[13].function(0);//AP_OUT

	//USRF pins
	J2[7].function(2);

	J2[43].function(3);//External interrupt 2 reading PPM
	sim2.eport.eppar |= 0x0020;

	//ipin=J2[43];


	J2[15]=1;J2[17]=1;J2[18]=1;J2[37]=1;J1[5]=1;J1[13]=1;//J2[43]=1;///Running everything on Autopilot.
	J2[45]=1;J2[47]=1;

	//PWM Registers
	//Submodule 0
	sim1.mcpwm.sm[0].cr2=CR2;
	sim1.mcpwm.sm[0].cr1=CR1;
	sim1.mcpwm.sm[0].ocr=OCR;
	sim1.mcpwm.sm[0].dismap=DISMAP;
	//Edge Aligned PWM
	sim1.mcpwm.sm[0].init=P_INIT;
	sim1.mcpwm.sm[0].val[1]=P_MAX;
	sim1.mcpwm.sm[0].val[2]=P_START;
	sim1.mcpwm.sm[0].val[3]=P_END;
	sim1.mcpwm.sm[0].val[4]=P_START;
	sim1.mcpwm.sm[0].val[5]=P_END;

	//Submodule 1
	sim1.mcpwm.sm[1].cr2=CR2;
	sim1.mcpwm.sm[1].cr1=CR1;
	sim1.mcpwm.sm[1].ocr=OCR;
	sim1.mcpwm.sm[1].dismap=DISMAP;
	//Edge Aligned PWM
	sim1.mcpwm.sm[1].init=P_INIT;
	sim1.mcpwm.sm[1].val[1]=P_MAX;
	sim1.mcpwm.sm[1].val[2]=P_START;
	sim1.mcpwm.sm[1].val[3]=P_END;
	sim1.mcpwm.sm[1].val[4]=P_START;
	sim1.mcpwm.sm[1].val[5]=P_END;

	//Submodule 2
	sim1.mcpwm.sm[2].cr2=CR2;
	sim1.mcpwm.sm[2].cr1=CR1;
	sim1.mcpwm.sm[2].ocr=OCR;
	sim1.mcpwm.sm[2].dismap=DISMAP;
	//Edge Aligned PWM
	sim1.mcpwm.sm[2].init=P_INIT;
	sim1.mcpwm.sm[2].val[1]=P_MAX;
	sim1.mcpwm.sm[2].val[2]=P_START;
	sim1.mcpwm.sm[2].val[3]=P_END;
	sim1.mcpwm.sm[2].val[4]=P_START;
	sim1.mcpwm.sm[2].val[5]=P_END;

	//Submodule 3
	sim1.mcpwm.sm[3].cr2=CR2;
	sim1.mcpwm.sm[3].cr1=CR1;
	sim1.mcpwm.sm[3].ocr=OCR;
	sim1.mcpwm.sm[3].dismap=DISMAP;
	//Edge Aligned PWM
	sim1.mcpwm.sm[3].init=P_INIT;
	sim1.mcpwm.sm[3].val[1]=P_MAX;
	sim1.mcpwm.sm[3].val[2]=P_START;
	sim1.mcpwm.sm[3].val[3]=P_END;
	sim1.mcpwm.sm[3].val[4]=P_START;
	sim1.mcpwm.sm[3].val[5]=P_END;

	//enabling PWM Outputs
	sim1.mcpwm.outen=OUTEN;
	//starting PWM
	pwmr=sim1.mcpwm.mcr;
	sim1.mcpwm.mcr=LDOK;
	sim1.mcpwm.mcr |=P_RUN;
}
