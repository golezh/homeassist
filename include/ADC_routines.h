//******************************************************
//**** ADC ROUTINES - HEADER FILE **************
//******************************************************
//Controller		: ATmega32 (Clock: 8 Mhz-internal)
//Compiler			: AVR-GCC (winAVR with AVRStudio-4)
//Project Version	: DL_1.0
//Author			: CC Dharmani, Chennai (India)
//			  		  www.dharmanitech.com
//Date				: 10 May 2011
//******************************************************

#ifndef _ADC_ROUTINES_H_
#define _ADC_ROUTINES_H_


#define ADC_ENABLE 					    ADCSRA |= (1<<ADEN)
#define ADC_DISABLE 				    ADCSRA &= 0x7F
#define ADC_START_CONVERSION	    	ADCSRA |= (1<<ADSC)


//battery discharge indicating port
#define BInd_PORT        PORTC
#define BInd_DDR         DDRC
#define BInd_PIN         PINC
#define BInd_BIT         3
#define BattaryNorm      BInd_PORT &=~(1<<BInd_BIT); //0
#define BattaryLow       BInd_PORT |=(1<<BInd_BIT);  //1


void ADC_init(void);
unsigned int ADC_read(void);
void readTemperature(unsigned char);
void readVoltage(unsigned char);
void batteryCheck(void);
void bat_init(void);
unsigned int ADC_Read(void);

#endif
