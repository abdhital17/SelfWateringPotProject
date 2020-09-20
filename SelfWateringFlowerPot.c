/*
 *Name: Abhishek Dhital
 *ID:   1001548204
 */


//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL with LCD/Temperature Sensor
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz
// Stack:           4096 bytes (needed for sprintf)

// Hardware configuration:
// LM60 Temperature Sensor:
//   AIN3/PE0 is driven by the sensor
//   (V = 424mV + 6.25mV / degC with +/-2degC uncalibrated error)
// UART Interface:
//   U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port
//   Configured to 115,200 baud, 8N1

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include "tm4c123gh6pm.h"
#include "wait.h"
#include "uart0.h"
#include "adc0.h"



//History offset

#define MOISTURE_BLOCK 0
#define WATER_BLOCK 1
#define LIGHT_BLOCK 2

// Bitband aliases
#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))

#define DEINT        (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 6*4)))
#define SPEAKER      (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 1*4)))
#define MOTOR        (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 2*4)))


// PortF masks
#define GREEN_LED_MASK 8
#define RED_LED_MASK 2

//PortC masks
#define COMP_MASK 128

//PortD masks
#define DEINT_MASK 64
#define SPEAKER_MASK 2
#define MOTOR_MASK 4

// PortE masks
#define AIN3_MASK 1 //E0
#define AIN2_MASK 2 //E1
#define AIN1_MASK 4 //E2


#define MAX_CHARS 80
#define MAX_FIELDS 5


typedef struct _USER_DATA
{
char buffer[MAX_CHARS+1];
uint8_t fieldCount;
uint8_t fieldPosition[MAX_FIELDS];
char fieldType[MAX_FIELDS];
} USER_DATA;

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
void initHw()
{
	// Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    // Note UART on port A must use APB
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable clocks
    SYSCTL_RCGCTIMER_R |=SYSCTL_RCGCTIMER_R1| SYSCTL_RCGCTIMER_R2;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R4;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5| SYSCTL_RCGCGPIO_R2| SYSCTL_RCGCGPIO_R3;
    SYSCTL_RCGCHIB_R |=1;
    SYSCTL_RCGCACMP_R |= 0x00000001;
    _delay_cycles(3);


    // Configure LED pins
    GPIO_PORTF_DIR_R |= GREEN_LED_MASK | RED_LED_MASK;  // bits 1 and 3 are outputs
    GPIO_PORTF_DR2R_R |= GREEN_LED_MASK | RED_LED_MASK; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R |= GREEN_LED_MASK | RED_LED_MASK;  // enable LEDs


    //Configure DEINT pins, speaker pins, and motor pins on port D
    GPIO_PORTD_DIR_R |= DEINT_MASK;
    GPIO_PORTD_DIR_R |= SPEAKER_MASK| MOTOR_MASK; //speaker and motor output
    GPIO_PORTD_DEN_R |=DEINT_MASK | SPEAKER_MASK| MOTOR_MASK;
    GPIO_PORTD_DR2R_R |=DEINT_MASK | SPEAKER_MASK| MOTOR_MASK;

    //COnfigure comparator C0- pins
    GPIO_PORTC_DIR_R &= ~COMP_MASK;
    GPIO_PORTC_DEN_R &=~COMP_MASK;
    GPIO_PORTC_AFSEL_R |= COMP_MASK;
    GPIO_PORTC_AMSEL_R |=COMP_MASK;

    COMP_ACREFCTL_R=0x0000020F;
    COMP_ACCTL0_R = 0x0000040C;


   // Configure AIN3 as an analog input
	GPIO_PORTE_AFSEL_R |= AIN3_MASK| AIN2_MASK| AIN1_MASK;                 // select alternative functions for AN3 (PE0)
    GPIO_PORTE_DEN_R &= ~(AIN3_MASK| AIN2_MASK| AIN1_MASK);                  // turn off digital operation on pin PE0
    GPIO_PORTE_AMSEL_R |= AIN3_MASK|AIN2_MASK| AIN1_MASK;                 // turn on analog operation on pin PE0

   //Configure the Hibernation module
    HIB_CTL_R |= 0x80000041;

    // Configure Timer 1 as the time base
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)

    TIMER1_TAILR_R =40000000;                         // set load value to 40E6 for 1 Hz interrupt rate
    TIMER1_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
    NVIC_EN0_R |= 1 << (INT_TIMER1A-16);            // turn-on interrupt 37 (TIMER1A)
    TIMER1_CTL_R |= TIMER_CTL_TAEN;                // turn-on timer


    // Configure Timer 2 as the time base
    TIMER2_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER2_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
    TIMER2_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)

    TIMER2_TAILR_R = 40000000;                        // set load value to 40E6 for 1 Hz interrupt rate
    TIMER2_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
    NVIC_EN0_R |= 1 << (INT_TIMER2A-16);             // turn-on interrupt 39 (TIMER2A)
    //TIMER2_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer
}


void initEEPROM()
{
    SYSCTL_RCGCEEPROM_R |= 1;

    _delay_cycles(6);

    while((EEPROM_EEDONE_R & 1));
    if((EEPROM_EESUPP_R & 4 ) && (EEPROM_EESUPP_R & 8))
        return;

    SYSCTL_SREEPROM_R = 1;
    SYSCTL_SREEPROM_R =0;

    _delay_cycles(6);
    while((EEPROM_EEDONE_R & 1) !=0 );

    if((EEPROM_EESUPP_R & 4 ) && (EEPROM_EESUPP_R & 8))
        return;
}


void enablePump()
{
    MOTOR=1;
}


void disablePump()
{
    MOTOR=0;
}


void update_History(int8_t block, int8_t offset, int16_t data)
{
  while(EEPROM_EEDONE_R & 1);
  EEPROM_EEBLOCK_R = block;
  EEPROM_EEOFFSET_R = offset;
  EEPROM_EERDWR_R   = data;
}


void print_History(int8_t offset[])
{
    uint8_t i=0;
    char text[5];
    for(i=0; i<3; i++)
    {
      if(i==0)
          putsUart0("moisture: ");
      else if(i==1)
          putsUart0("water level: ");
      else
          putsUart0("light level: ");
      uint8_t j;

      while(EEPROM_EEDONE_R & 1);
      EEPROM_EEBLOCK_R = i;
      for(j=0; j <  16; j++)
      {
        while(EEPROM_EEDONE_R & 1);
        EEPROM_EEOFFSET_R = j;
         if(EEPROM_EERDWRINC_R != 0x11111111)
         {
             sprintf(text, "%i", EEPROM_EERDWRINC_R);
             putsUart0(text);

             if(i==0)
                 putsUart0("%, ");
             else if(i==1)
                 putsUart0("ml, ");
             else
                 putsUart0("%, ");
         }

      }
      putcUart0('\n');
    }
}



void playWaterLowAlert()
{
    //notes of twinkle-twinkle-little-star
    //uint32_t notes[]={76445,76445,51021,51021,45454,45454,51021,51021,57269,57269,60674,60674,68104,68104,60674,76445,51021,51021,57269,57269,60674,60674,68104,68104};
    uint32_t notes[]={76445,50000,76445,50000,76445,50000,76445,50000};

    uint8_t i=0;
    for(i=0;i<sizeof(notes)/sizeof(notes[0]);i++)
    {
        TIMER2_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
        TIMER2_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
        TIMER2_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)

        TIMER2_TAILR_R = notes[i];                          // set load value equal to the current element of notes[] array
        TIMER2_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
        NVIC_EN0_R |= 1 << (INT_TIMER2A-16);             // turn-on interrupt 39 (TIMER2A)
        TIMER2_CTL_R |= TIMER_CTL_TAEN;                 // turn-on timer
        waitMicrosecond(500000);

        TIMER2_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before next iteration
    }
}

void playBatteryLowAlert()
{
        uint32_t notes[]={6000,7000,6000,7000,6000,7000};
        uint8_t i=0;
        for(i=0;i<sizeof(notes)/sizeof(notes[0]);i++)
        {
         TIMER2_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
         TIMER2_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
         TIMER2_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)

         TIMER2_TAILR_R = notes[i];                          // set load value equal to the parameter passed to the function
         TIMER2_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
         NVIC_EN0_R |= 1 << (INT_TIMER2A-16);             // turn-on interrupt 39 (TIMER2A)
         TIMER2_CTL_R |= TIMER_CTL_TAEN;                 // turn-on timer
         waitMicrosecond(500000);

         TIMER2_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before next iteration

        }
}


int getVolume()
{
  DEINT=1;
  waitMicrosecond(100000); //to let the capacitor deintegrate fully
  TIMER1_TAV_R=0;
  DEINT=0;
  while((COMP_ACSTAT0_R && COMP_ACSTAT0_OVAL)!=0);
  uint32_t t= 40000000- TIMER1_TAV_R;
  return (t*0.57339)-146.21;
}


void timer1Isr()
{
    TIMER1_TAV_R=0;
    TIMER1_ICR_R = TIMER_ICR_TATOCINT;               // clear interrupt flag
}


//Interrupt service routine for Timer 2
//Used to toggle the GPIO pin for the speaker schematic
void timer2Isr()
{
    if(SPEAKER)
        SPEAKER=0;
    else
        SPEAKER=1;

    TIMER2_ICR_R = TIMER_ICR_TATOCINT;               // clear interrupt flag
}




void getsUart0(USER_DATA* d)
{
  uint8_t c=0; //counter variable
  char ch;
  while (1)  //loop starts
  {
    ch=getcUart0();
    if ((ch==8 || ch==127) && c>0) c--;

    else if (ch==13)
        {
         d->buffer[c]=0;
         putcUart0(ch);
         return;
        }
    else if (ch>=32)
     {
        d->buffer[c]=ch;
        putcUart0(ch);
        c++;
        if (c==MAX_CHARS)
        {
            d->buffer[c]='\0';
            return;
        }
     }
     else continue;
  }
}


void parseFields(USER_DATA* d)
{
    uint8_t i=0;
    char prev=0;
    d->fieldCount=0;
    while(d->buffer[i]!='\0')
    {
        if((d->fieldCount)>=MAX_FIELDS)
        {
            break;
        }

        char temp=d->buffer[i];

        if(((temp>=97 && temp<=122) || (temp>=65&&temp<=90)) && prev!='a' )
        {
            prev='a';
            d->fieldType[(d->fieldCount)]='a';
            d->fieldPosition[(d->fieldCount)]=i;
            d->fieldCount+=1;
        }

        else if ((temp>=48 && temp<=57) && prev!='n')
           {
                prev='n';
                d->fieldType[d->fieldCount]='n';
                d->fieldPosition[d->fieldCount]=i;
                d->fieldCount+=1;
            }
        else if(!((temp>=97 && temp<=122) || (temp>=65&&temp<=90)) && !(temp>=48 && temp<=57) )
           {
             prev=0;
             d->buffer[i]='\0';
           }
        i++;
   }
}


char* getFieldString(USER_DATA* data, uint8_t fieldNumber)
{
  if(fieldNumber<=data->fieldCount)
      return &(data->buffer[data->fieldPosition[fieldNumber]]);

  else
      return NULL;
}


int32_t getFieldInteger(USER_DATA* data, uint8_t fieldNumber)
{
    if (fieldNumber<=data->fieldCount && data->fieldType[fieldNumber]=='n')
    {
        //return (&data->buffer[data->fieldPosition[fieldNumber]]);
        return atoi(getFieldString(data, fieldNumber));
    }
    else
        return 0;
}

bool isCommand(USER_DATA* data, const char strCommand[], uint8_t minArguments)
{
 if(strcmp(strCommand,getFieldString(data,0))==0 && (data->fieldCount)>minArguments)
     return true;
 return false;
}

float getLightPercentage()
{
    // Use AIN3 input with N=4 hardware sampling
    setAdc0Ss3Mux(3);
    setAdc0Ss3Log2AverageCount(4);
    // Read sensor for the light
    uint16_t raw;
    float instantLight;

    raw = readAdc0Ss3();
    instantLight = (((raw+0.5) / 4096.0 * 3.3));
    return instantLight*100/3.3;
}

float getMoisturePercentage()
{
    setAdc0Ss3Mux(2);
    setAdc0Ss3Log2AverageCount(4);
    uint16_t raw;
    float moist;
    raw=readAdc0Ss3();
    moist= (((raw+0.5) / 4096.0 * 3.3));
    return moist* 100/3.3;

}

float getBatteryVoltage()
{
    setAdc0Ss3Mux(1);
    setAdc0Ss3Log2AverageCount(4);
    uint16_t raw;
    float battery;
    raw=readAdc0Ss3();
    battery=(((raw+0.5)/4096 *3.3));
    return (battery/47)*147;
}

uint32_t getCurrentSeconds()
{
  return HIB_RTCC_R;
}

bool isWateringAllowed(uint32_t s1, uint32_t s2)
{
    uint32_t currentTime= getCurrentSeconds();
    if (currentTime >= s1 && currentTime< s2)
        return true;
    return false;
}


//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------
int main(void)
{
    int8_t offset_count[3]= {0, 0, 0} ;  // index 0 will have the offset count for block 0 - moisture
                                     // index 1 will have the offset count for block 1 - water level
                                    // index 2 will have the offset count for block 2 - light level


    int moisture_level= 50;
    int light_level = 10;
    uint32_t s1= 0;
    uint32_t s2= 0;

    USER_DATA data;

    // Initialize hardware
    initUart0();
    initHw();
    initAdc0Ss3();
    initEEPROM();

   while(1)
  {
    if(kbhitUart0())
    {
      getsUart0(&data);
      putsUart0("\n");

      parseFields(&data);

      #ifdef DEBUG
      uint8_t i;
      for(i=0; i< data.fieldCount; i++)
      {
       putcUart0(data.fieldType[i]);
       putcUart0('\n');
       putsUart0(&data.buffer[(data.fieldPosition[i])]);
       putcUart0('\n');
      }
      #endif

      bool valid=false;

      if(isCommand(&data,"set",2))
      {
        int32_t add=getFieldInteger(&data,1);
        int32_t d=getFieldInteger(&data,2);
        valid=true;
      }


      if(isCommand(&data,"status",0))
      {
        char text [50];

        //for the volume in container
        int vol;
        vol=getVolume();
        sprintf(text,"volume: %d ml\t",vol);   //y=(0.57339x)-146.21
        putsUart0(text);
        update_History(WATER_BLOCK , offset_count[WATER_BLOCK], vol);
        offset_count[WATER_BLOCK] = (offset_count[WATER_BLOCK]+1) % 16;

        //light
        float instantLight=getLightPercentage();
        sprintf(text,"light: %.3f %% \t",instantLight);
        putsUart0(text);
        update_History(LIGHT_BLOCK , offset_count[LIGHT_BLOCK], instantLight);
        offset_count[LIGHT_BLOCK] = (offset_count[LIGHT_BLOCK]+1) % 16;

        //moisture
        float moisture=getMoisturePercentage();
        sprintf(text,"moisture: %f %% \t",moisture);
        putsUart0(text);
        update_History(MOISTURE_BLOCK, offset_count[MOISTURE_BLOCK], moisture);
        offset_count[MOISTURE_BLOCK] = (offset_count[MOISTURE_BLOCK] + 1) % 16;



        //battery voltage
        float battery=getBatteryVoltage();
        sprintf(text,"battery: %.3f V \n",battery);
        putsUart0(text);

        valid=true;
      }

      if(isCommand(&data,"alert",1))
      {
          light_level = getFieldInteger(&data, 1);
          valid=true;
      }

      if(isCommand(&data,"pump",1))
      {
          char* str=getFieldString(&data,1);
          if(strcmp(str,"ON")==0)
          {
             enablePump();
             valid=true;
          }
          else if(strcmp(str,"OFF")==0)
          {
              disablePump();
              valid=true;
          }
      }

      if(isCommand(&data, "time", 2))
      {
        uint32_t seconds= getFieldInteger(&data, 1) * 3600 + getFieldInteger(&data,2) * 60;
        HIB_RTCLD_R = seconds;
        valid=true;
      }

      if(isCommand(&data, "water", 4))
      {
        s1=getFieldInteger(&data,1)*3600 + getFieldInteger(&data, 2)*60;
        s2=getFieldInteger(&data,3)*3600 + getFieldInteger(&data, 4)*60;
        valid= true;
      }

      if(isCommand(&data, "level", 1))
      {
        moisture_level= getFieldInteger(&data, 1);
        valid= true;
      }

      if(isCommand(&data, "history", 0))
      {
          print_History(offset_count);
          valid=true;
      }


     if(!valid)
         putsUart0("Invalid command\n");
    }

    else if(!kbhitUart0())
    {
        if(moisture_level != -1)
        {
            if(getMoisturePercentage()< moisture_level)
            {
                if(isWateringAllowed(s1,s2))
                {
                   enablePump();
                   waitMicrosecond(5000000);
                   disablePump();
                   waitMicrosecond(30000000);
                }
            }
        }

        if(getVolume()< 200 && getLightPercentage() > light_level)
        {
            playWaterLowAlert();
            waitMicrosecond(3000000);
        }

        if(getBatteryVoltage()<3)
        {
            playBatteryLowAlert();
            waitMicrosecond(3000000);
        }
    }

  }
}
