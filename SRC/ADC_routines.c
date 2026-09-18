//******************************************************
//     **** ADC ROUTINES - SOURCE FILE *****
//******************************************************
//Controller		: ATmega32 (Clock: 8 Mhz-internal)
//Compiler			: AVR-GCC (winAVR with AVRStudio-4)
//Project Version	: DL_1.0
//Author			: CC Dharmani, Chennai (India)
//			  		  www.dharmanitech.com
//Date				: 10 May 2011
//******************************************************

#include <avr/io.h>
#include "ADC_routines.h"
#include "UART_routines.h"
#include <util/delay.h>

volatile unsigned char temperature[7];
volatile unsigned char voltage[7];

void ADC_init(void);
//******************************************************
//Purpose : Initialize the ADC
//Conversion time: 52uS
//******************************************************
void ADC_init(void)
{
  ADCSRA = 0x00;  //disable adc
  ADMUX  = 0x40;  //select adc input 0, ref:AVCC
  ADCSRA = 0x80;  //prescaler:4, single conversion mode
  ADC_ENABLE;
}

void bat_init(void)
{
  BInd_DDR |=(1<<BInd_BIT);   //pin as output
  BattaryNorm;
}
//********************************************************************
//Purpose : Do an Analog to Digital Conversion
//Paramtr :	none
//return  : intger temperature value
//********************************************************************
unsigned int ADC_read(void)
{
    char i;
    unsigned int ADC_temp, ADCH_temp;
    unsigned int ADC_var = 0;
    
            
    for(i=0;i<8;i++)             // do the ADC conversion 8 times for better accuracy 
    {
	 	    ADC_START_CONVERSION;
        while(!(ADCSRA & 0x10)); // wait for conversion done, ADIF flag active
        ADCSRA|=(1<<ADIF);
		
        ADC_temp = ADCL;         // read out ADCL register
        ADCH_temp = ADCH;        // read out ADCH register        
        ADC_temp +=(ADCH_temp << 8);
        ADC_var += ADC_temp;      // accumulate result (8 samples) for later averaging
    }

    ADC_var = ADC_var >> 3;       // average the 8 samples

    if(ADC_var > 1023) ADC_var = 1023;
	
  return ADC_var;
}


//********************************************************************
//Purpose : Read temperature from LM35 connected to the ADC
//Paramtr : unsigned char ADC channel number
//returns : None (modifies the global string 'temperature')
//********************************************************************
void readTemperature(unsigned char channel)
{
   unsigned int value;
   float volt;

   ADMUX = 0x40 | channel;
   //ADMUX = 0x40 | 8;
   value = ADC_read();

   volt = (float)(value * 5.0)/ 1024.0;
   value = (unsigned int)(volt * 1000);

   temperature[6] = 'C';  //centigrade
   temperature[5] = 0xb0; //ascii value for degree symbol
   temperature[4] = (value % 10) | 0x30;
   temperature[3] = '.';  
   value = value / 10;
   temperature[2] = (value % 10) | 0x30;
   value = value / 10;
   temperature[1] = (value % 10) | 0x30;
   value = value / 10;
   temperature[0] = value | 0x30;   
}  


//********************************************************************
//Purpose : Read voltage from ADC channels
//Paramtr : unsigned char ADC channel number
//returns : None (modifies the global string 'voltage')
//********************************************************************
void readVoltage(unsigned char channel)
{
  unsigned int value;
  float volt;
  ADC_ENABLE;
  _delay_ms(100);
  ADMUX = (1 << REFS0) | channel;   // ADC0, PA0 if channel = 0, ADC1, PA1 if channel = 1, etc
  //ADMUX = 0x40 | 8;
  value = ADC_read();

  volt = (float)(value * 5.0)/ 1024.0;
  value = (unsigned int)(volt * 1000);

  voltage[6] = 'V';  //V for voltage
  voltage[5] = ' '; 
  voltage[4] = (value % 10) | 0x30;
  value = value / 10;
  voltage[3] = (value % 10) | 0x30;
  value = value / 10;
  voltage[2] = (value % 10) | 0x30;
  voltage[1] = '.';  
  value = value / 10;
  voltage[0] = value | 0x30;
}  

//
void batteryCheck(void)
{
  unsigned int value;
  ADC_ENABLE;
  _delay_ms(100);
  ADMUX = 0x40 | 0;
  //ADMUX = 0x40 | 8;
  value = ADC_read();
  if (value>0xFC)
  {
   BattaryNorm;
  }
  else
  {
   BattaryLow;
  }
  if (value > 0xFA)
   value = value - 0xFA;
  else
   value = 0;
  if (value == 0)
  {
    voltage[2] = '%';
    voltage[1] = '0';
    voltage[0] = '0';
  }else{
    voltage[3] = '%'; 
    voltage[2] = (value % 10) | 0x30;
    value = value / 10;
    if (value > 0)  voltage[1] = (value % 10) | 0x30;
    else voltage[1] = ' ';
    value = value / 10;
    if (value > 0) voltage[0] = value | 0x30;
    else voltage[0] = '=';
  }

  ADC_DISABLE;
} 
unsigned int ADC_Read(void)
{
  unsigned int value;
  ADC_START_CONVERSION;  
  ADMUX = 0x40 | 0;
  //ADMUX = 0x40 | 8;
  value = ADC_read();
  return value;
} 
