// Color emitter Project
// Ulysses Aguilar
// Bashar Al Atom
//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL with LCD/Temperature Sensor
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// Red LED:
//   M0PWM3 (PB5) driven by an NPN transistor that powers the red LED
// Green LED:
//   M0PWM5 (PE5) driven by an NPN transistor that powers the green LED
// Blue LED:
//   M0PWM4 (PE4) driven by an NPN transistor that powers the blue LED
// Push button:
//   (PF4) configured to pull up
// board LED:
//   (PF3) indicator
// phototransistor:
//   AN0/PE3 is driven by the sensor
// UART Interface:
//   U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port
//   Configured to 115,200 baud, 8N1
//
//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "tm4c123gh6pm.h"

#define  MAX_CHARS 100
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))
#define SREEPROM (*((volatile uint32_t *)(0x42000000 + (0x400FE558-0x40000000)*32 ))
#define PREEPROM (*((volatile uint32_t *)(0x42000000 + (0x400FEA58-0x40000000)*32 ))
#define PUSH_BUTTON  (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 4*4)))
#define PUSH_BUTTON_MASK 16


//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------
// RGB values for trigger(calibrate output)
uint16_t pwm_r,pwm_g,pwm_b;
// is the initial memmory address of the EEPROM
uint32_t curraddr = 0x0000;
// used for led sample, 0 means sample is off
uint8_t led=0;
// temporary place holders for RGB readout
uint16_t r,g,b;
// struct used to store colors into EEPROM
struct color{
    uint16_t r;
    uint16_t g;
    uint16_t b;
    uint16_t valid;
};
struct color tempColor;
// D and avg is used for delta -1 indicated D hasn't been set.
// E is for match command, -1 indicates E hasn't been set.
int32_t D=-1,avg=0,E = -1;
// array to store input
char strinput[MAX_CHARS];
// array of char* to tokenize string
char* tokens[10];
// this is used to store the number of arguments
uint8_t NArgs = 0;
//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------
// Initialize Hardware
void initHw()
{
    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S)
            | SYSCTL_RCC_USEPWMDIV | SYSCTL_RCC_PWMDIV_2;
    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    // Note UART on port A must use APB
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable clocks
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOE | SYSCTL_RCGC2_GPIOB|SYSCTL_RCGC2_GPIOA|SYSCTL_RCGC2_GPIOF;
    SYSCTL_RCGCEEPROM_R |= 1;     //turn-on EEPROM clock
    SYSCTL_RCGC0_R |= SYSCTL_RCGC0_PWM0;             // turn-on PWM0 module
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;       // turn-on timer


    // Configure three backlight LEDs
    GPIO_PORTB_DIR_R |= 0x20;   // make bit5 an output
    GPIO_PORTB_DR2R_R |= 0x20;  // set drive strength to 2mA
    GPIO_PORTB_ODR_R |= 0x20;
    GPIO_PORTB_DEN_R |= 0x20;   // enable bit5 for digital
    GPIO_PORTB_AFSEL_R |= 0x20; // select auxilary function for bit 5
    GPIO_PORTB_PCTL_R = GPIO_PCTL_PB5_M0PWM3; // enable PWM on bit 5

    // Configure LEDs
    GPIO_PORTF_DIR_R = 0x08;  // bits 3 and 2 are outputs, other pins are inputs
    GPIO_PORTF_DR2R_R = 0x08; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R = 0x08;  // enable LEDs and pushbuttons

    GPIO_PORTE_DIR_R |= 0x30;   // make bits 4 and 5 outputs
    GPIO_PORTE_DR2R_R |= 0x30;  // set drive strength to 2mA
    GPIO_PORTE_DEN_R |= 0x30;   // enable bits 4 and 5 for digital
    GPIO_PORTE_ODR_R |= 0x30;   // turns led off completely
    GPIO_PORTE_AFSEL_R |= 0x30; // select auxilary function for bits 4 and 5
    GPIO_PORTE_PCTL_R |= GPIO_PCTL_PE4_M0PWM4 | GPIO_PCTL_PE5_M0PWM5; // enable PWM on bits 4 and 5

    // Configure PWM module0 to drive RGB backlight
    // RED   on M0PWM3 (PB5), M0PWM1b
    // BLUE  on M0PWM4 (PE4), M0PWM2a
    // GREEN on M0PWM5 (PE5), M0PWM2b
    __asm(" NOP");                                   // wait 3 clocks
    __asm(" NOP");
    __asm(" NOP");
    SYSCTL_SRPWM_R = SYSCTL_SRPWM_R0;                // reset PWM0 module
    SYSCTL_SRPWM_R = 0;                              // leave reset state
    PWM0_1_CTL_R = 0;                                // turn-off PWM0 generator 1
    PWM0_2_CTL_R = 0;                                // turn-off PWM0 generator 2
    PWM0_1_GENB_R = PWM_0_GENB_ACTCMPBD_ZERO | PWM_0_GENB_ACTLOAD_ONE;
                                                     // output 3 on PWM0, gen 1b, cmpb
    PWM0_2_GENA_R = PWM_0_GENA_ACTCMPAD_ZERO | PWM_0_GENA_ACTLOAD_ONE;
                                                     // output 4 on PWM0, gen 2a, cmpa
    PWM0_2_GENB_R = PWM_0_GENB_ACTCMPBD_ZERO | PWM_0_GENB_ACTLOAD_ONE;
                                                     // output 5 on PWM0, gen 2b, cmpb
    PWM0_1_LOAD_R = 1024;                            // set period to 40 MHz sys clock / 2 / 1024 = 19.53125 kHz
    PWM0_2_LOAD_R = 1024;

    PWM0_1_CMPB_R = 0;                               // red off (0=always low, 1023=always high)
    PWM0_2_CMPB_R = 0;                               // green off
    PWM0_2_CMPA_R = 0;                               // blue off
    PWM0_1_CTL_R = PWM_0_CTL_ENABLE;                 // turn-on PWM0 generator 1
    PWM0_2_CTL_R = PWM_0_CTL_ENABLE;                 // turn-on PWM0 generator 2
    PWM0_ENABLE_R = PWM_ENABLE_PWM3EN | PWM_ENABLE_PWM4EN | PWM_ENABLE_PWM5EN;// enable outputs

                                                     // turn on GPIO ports A and E
    SYSCTL_RCGCADC_R |= 1;                           // turn on ADC module 0 clocking
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;         // turn-on UART0, leave other UARTs in same status

    // Configure AN0 as an analog input
    GPIO_PORTE_AFSEL_R |= 0x08;                      // select alternative functions for AN0 (PE3)
    GPIO_PORTE_DEN_R &= ~0x08;                       // turn off digital operation on pin PE3
    GPIO_PORTE_AMSEL_R |= 0x08;                      // turn on analog operation on pin PE3

    // Configure ADC
    ADC0_CC_R = ADC_CC_CS_SYSPLL;                    // select PLL as the time base (not needed, since default value)
    ADC0_ACTSS_R &= ~ADC_ACTSS_ASEN3;                // disable sample sequencer 3 (SS3) for programming
    ADC0_EMUX_R = ADC_EMUX_EM3_PROCESSOR;            // select SS3 bit in ADCPSSI as trigger
    ADC0_SSMUX3_R = 0;                               // set first sample to AN0
    ADC0_SSCTL3_R = ADC_SSCTL3_END0;                 // mark first sample as the end
    ADC0_ACTSS_R |= ADC_ACTSS_ASEN3;                 // enable SS3 for operation

    // Configure UART0 pins
    GPIO_PORTA_DIR_R |= 2;                           // enable output on UART0 TX pin: default, added for clarity
    GPIO_PORTA_DEN_R |= 3;                           // enable digital on UART0 pins: default, added for clarity
    GPIO_PORTA_AFSEL_R |= 3;                         // use peripheral to drive PA0, PA1: default, added for clarity
    GPIO_PORTA_PCTL_R &= 0xFFFFFF00;                 // set fields for PA0 and PA1 to zero
    GPIO_PORTA_PCTL_R |= GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;
                                                     // select UART0 to drive pins PA0 and PA1: default, added for clarity

    // Configure UART0 to 115200 baud, 8N1 format
    UART0_CTL_R = 0;                                 // turn-off UART0 to allow safe programming
    UART0_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
    UART0_IBRD_R = 21;                               // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART0_FBRD_R = 45;                               // round(fract(r)*64)=45
    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN; // configure for 8N1 w/ 16-level FIFO
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN;
                                                     // enable TX, RX, and module
    GPIO_PORTF_DEN_R |= PUSH_BUTTON_MASK;// enable LEDs and pushbuttons
    GPIO_PORTF_PUR_R |= PUSH_BUTTON_MASK; // enable internal pull-up for push button
    __asm(" NOP");
    __asm(" NOP");
    __asm(" NOP");
    __asm(" NOP");

    // Configure Timer 1 for periodic service
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
    TIMER1_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
    NVIC_EN0_R |= 1 << (INT_TIMER1A-16);             // turn-on interrupt 37 (TIMER1A)
    //Configer EEPROM
    waitMicrosecond(3);   // ensure ready to config
    while(EEPROM_EEDONE_R&EEPROM_EEDONE_WORKING);//wait for EEPROM READY
    //do it again, data sheets indicates the need for reset did not work
    // so we took it out
    waitMicrosecond(3);
    while(EEPROM_EEDONE_R&EEPROM_EEDONE_WORKING);


}
// Reads from EEPROM
// derived from EEPROM library code
uint32_t EEPROM_READ(uint32_t* Data,uint32_t add,uint32_t count)
{
    while(EEPROM_EEDONE_R&EEPROM_EEDONE_WORKING); //wait for ready
    EEPROM_EEBLOCK_R= add>>6;   //get right block address
    EEPROM_EEOFFSET_R = (add>>2)&0x0F;   //get right offset to block address

    count/=4;  //pointer arithmetic

    while(count) //iterates through the block
    {
        *Data=EEPROM_EERDWRINC_R; // getting data in integer size

        Data++;//move into the next integer address in struct
        count--; //done reading this block
        // if there is still data to read & we reach the end of the block move to the next block
        if(count&&(EEPROM_EEOFFSET_R==0))
        {
            EEPROM_EEBLOCK_R+=1;
        }
    }
    return EEPROM_EEDONE_R; //status return
}

// Reads from EEPROM
// derived from EEPROM library code
uint32_t EEPROM_WRITE(uint32_t* Data,uint32_t add,uint32_t count){
    while(EEPROM_EEDONE_R&EEPROM_EEDONE_WORKING);//wait for ready
    EEPROM_EEBLOCK_R= add>>6;      //get right block address
    EEPROM_EEOFFSET_R = (add>>2)&0x0F;    //get right offset to block address

    count/=4;  //pointer arithmetic

    while(count)
    {
        EEPROM_EERDWRINC_R = *Data; // write data
        waitMicrosecond(3);
        while(EEPROM_EEDONE_R&EEPROM_EEDONE_WORKING); // wait for write to be done

        Data++;// move to next integer address in struct
        count--; // done reading the address
        // if there is still data to read & we reach the end of the block move to the next block
        if(count&&(EEPROM_EEOFFSET_R==0))
        {
            EEPROM_EEBLOCK_R+=1;
        }
    }
    return EEPROM_EEDONE_R;  //status return
}
// for button command waits for push button to be pressed
void waitPbPress()
{
    while(PUSH_BUTTON);
}
// function that reads AN value
int16_t readAdc0Ss3()
{
    ADC0_PSSI_R |= ADC_PSSI_SS3;                     // set start bit
    while (ADC0_ACTSS_R & ADC_ACTSS_BUSY);           // wait until SS3 is not busy
    return ADC0_SSFIFO3_R;                           // get single result from the FIFO
}
// utility function used for delta command, returns absolute value
int32_t abs(int32_t a)
{
    if(a<0)
        return -1*a;
    return a;
}
// sets the CMP values for RGB
void setRgbColor(uint16_t red, uint16_t green, uint16_t blue)
{
    PWM0_1_CMPB_R = red;
    PWM0_2_CMPA_R = blue;
    PWM0_2_CMPB_R = green;
}

// Blocking function that writes a serial character when the UART buffer is not full
void putcUart0(char c)
{
    while (UART0_FR_R & UART_FR_TXFF);               // wait if uart0 tx fifo full
    UART0_DR_R = c;                                  // write character to fifo
}

// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char* str)
{
    uint8_t i;
    for (i = 0; i < strlen(str); i++)
      putcUart0(str[i]);
}

// Blocking function that returns with serial data once the buffer is not empty
char getcUart0()
{
    while (UART0_FR_R & UART_FR_RXFE);               // wait if uart0 rx fifo empty
    return UART0_DR_R & 0xFF;                        // get character from fifo
}
//derived from from c strtok function to tokenize string without including the library
char * strtokk(char * str, char *comp)
{
    static int pos;
    static char *s;
    int i =0, start = pos;

    // Copying the string for further calls of strtok
    if(str!=NULL)
        s = str;

    i = 0;
    int j = 0;
    //While not end of string
    while(s[pos] != '\0')
    {
        j = 0;
        //Comparing of one of the delimiter matches the character in the string
        while(comp[j] != '\0')
        {
            //Pos point to the next location in the string that we have to read
            if(s[pos] == comp[j])
            {
                //Replace the delimter by \0 to break the string
                s[pos] = '\0';
                pos = pos+1;
                //Checking for the case where there is no relevant string before the delimeter.
                //start specifies the location from where we have to start reading the next character
                if(s[start] != '\0')
                    return (&s[start]);
                else
                {
                    // Move to the next string after the delimiter
                    start = pos;
                    // Decrementing as it will be incremented at the end of the while loop
                    pos--;
                    break;
                }
            }
            j++;
        }
        pos++;
    }//End of Outer while
    s[pos] = '\0';
    if(s[start] == '\0')
        return NULL;
    else
        return &s[start];
}
//function that recives user input from uart
void getsUart0(char * str, uint8_t maxChars)
{
    // count for the number of characters in the string
    int count =0;

    // temp is used to be a temporary place holder for the incoming character
    char temp;
    while(count<maxChars-1)
    {
        // get the character from the RX end
        temp = getcUart0();
        if (temp==8&& count!=0)
        {
            // if it is a backspace take one of the count
            // (as if we are deleting the last char)
            count--;
            continue;
        }
        else if(temp ==13)
        {
            // if we get a carriage return, then this is the end of the string
            // null terminate the string and return
            temp ='\0';
            str[count]=temp;
            return;
        }
        else if(temp >=' ')
        {
            // else if the character has an ascii code above 32
            if(temp>='A'&&temp<='Z')
            {
                // by adding 32 to a capital letter we convert it into a smaller one
                temp+=32;
            }

            // store the character after editing it into the string.
            str[count]=temp;
        }
        // if we reached this point then we added a character to the string
        count++;
    }
    // null terminate the string and return, as this is the end.
    str[count]='\0';
    return;
}


void trigger(uint16_t pwm_r,uint16_t pwm_g,uint16_t pwm_b){
    if(led==1)
    GREEN_LED = 1;
    char strr[60];       // str is used to be able to print the raw value to
                       // the UART
    int16_t v;
    waitMicrosecond(10000);
    setRgbColor(pwm_r,0,0);
    waitMicrosecond(10000);
    r = readAdc0Ss3();
    waitMicrosecond(10000);
    setRgbColor(0,pwm_g,0);
    waitMicrosecond(10000);
    g = readAdc0Ss3();
    waitMicrosecond(10000);
    setRgbColor(0,0,pwm_b);
    waitMicrosecond(10000);
    b = readAdc0Ss3();
    v = sqrt(r*r+b*b+g*g);

    if(D==-1){ // if D wasn't set
    sprintf(strr, "Color(%d, %d, %d)\r\n",r,g,b);
    putsUart0(strr);
    sprintf(strr, "RGB(%.1f, %.1f, %.1f)\r\n",r*0.063,g*0.063,b*0.063);
    putsUart0(strr);
    }else if(D!=-1){    // if D is set

        avg = 0.9*avg+0.1*v;
        if(abs(avg-v)>D){
           sprintf(strr, "Color(%d, %d, %d)\r\n",r,g,b);
           putsUart0(strr);
           sprintf(strr, "RGB(%.1f, %.1f, %.1f)\r\n",r*0.063,g*0.063,b*0.063);
           putsUart0(strr);
        }
    }
    if(E!=-1){
        uint16_t i=0;
        uint32_t v;
        for(i=0;i<255;++i){ // size change?
            readEEPROM(i);
            if(tempColor.valid==1){
                v = sqrt(((r-tempColor.r)*(r-tempColor.r))+((g-tempColor.g)*(g-tempColor.g))+((b-tempColor.b)*(b-tempColor.b)));
                if(v<E){
                    sprintf(strr, "match at N: %d\r\n",i);
                    putsUart0(strr);
                    sprintf(strr, "Color(%d, %d, %d)\r\n",tempColor.r,tempColor.g,tempColor.b);
                    putsUart0(strr);
                }
            }
        }
    }


    setRgbColor(0,0,0);
    waitMicrosecond(100);

    if(led==1)
    GREEN_LED=0;
}

void periodicIsr(){
    trigger(pwm_r,pwm_g,pwm_b);
   TIMER1_ICR_R = TIMER_ICR_TATOCINT;
}

void parse_string(){

   // Returns first token
    tokens[0] = strtok(strinput, " ,");
    NArgs =0;
    // Keep printing tokens while one of the
    // delimiters present in str[].
    while (tokens[NArgs] != NULL) {
        NArgs++;
        tokens[NArgs] = strtok(NULL, " ,");
    }

}

// returns are the value of the nth argument
uint16_t getValue(uint8_t argNumber){
    return atoi(tokens[argNumber+1]);
}

// returns the nth argument as a string
char * getString(uint8_t argNumber){
    return tokens[argNumber+1];
}

// checks if the strinput is a command with minArgs number of arguments
bool isCommand(const char * strcmd, uint8_t minArgs){
    if(strcmp(tokens[0],strcmd)==0 && NArgs>=minArgs ){
        return 1;
    }

    return 0;
}

void readEEPROM(uint16_t N){
    EEPROM_READ((uint32_t *)&tempColor, curraddr+N*sizeof(struct color), sizeof(tempColor)); //Write struct to EEPROM start from 0x0000
}

// function to wait for a period of time
void waitMicrosecond(uint32_t us)
{
    __asm("WMS_LOOP0:   MOV  R1, #6");          // 1
    __asm("WMS_LOOP1:   SUB  R1, #1");          // 6
    __asm("             CBZ  R1, WMS_DONE1");   // 5+1*3
    __asm("             NOP");                  // 5
    __asm("             NOP");                  // 5
    __asm("             B    WMS_LOOP1");       // 5*2 (speculative, so P=1)
    __asm("WMS_DONE1:   SUB  R0, #1");          // 1
    __asm("             CBZ  R0, WMS_DONE0");   // 1
    __asm("             NOP");                  // 1
    __asm("             B    WMS_LOOP0");       // 1*2 (speculative, so P=1)
    __asm("WMS_DONE0:");                        // ---
                                                // 40 clocks/us + error
}


int main(void)
{
    // Initialize hardware
    initHw();
    int16_t i = 0;
    uint16_t raw; // r,g, and b are used to set the color of the leds
                       // raw is used to get the value of the anolog input
    uint16_t T=3500;
    uint16_t j;
    char str[60];       // str is used to be able to print the raw value to
                       // the UART

    while(1)
    {
        getsUart0(&strinput,100); // get the input from the UART
        putsUart0("\r\n");
        parse_string(); // tokenize the input

        if(isCommand("rgb",3))  // if the command is an rgb command
        {
            // get the arguments and set the leds to the given values
            r = getValue(0);
            g = getValue(1);
            b = getValue(2);
            setRgbColor(r,g,b);
        }else if(isCommand("light",0)){
            // read the sensor value and print it to the UART
            raw = readAdc0Ss3();
            sprintf(str, "Raw ADC:        %u\r\n", raw);
            putsUart0(str);
        }else if(isCommand("test",0)){
            TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                  // turn-off timer
            for(j=0;j<1023;++j)
            {
                waitMicrosecond(15000);
                setRgbColor(j,0,0);
                waitMicrosecond(15000);
                r = readAdc0Ss3();
                waitMicrosecond(15000);
                setRgbColor(0,j,0);
                waitMicrosecond(15000);
                g = readAdc0Ss3();
                waitMicrosecond(15000);
                setRgbColor(0,0,j);
                waitMicrosecond(15000);
                b = readAdc0Ss3();
                sprintf(str, "%d\t%d\t%d\t%d\r\n", j,r,g,b);
                putsUart0(str);
                setRgbColor(0,0,0);
            }
        }else if(isCommand("calibrate",0)){
            TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                  // turn-off timer
            //red
            j=0;
            raw=0;
            while(j<=1023&&raw<T){
                waitMicrosecond(10000);
                setRgbColor(j,0,0);
                waitMicrosecond(10000);
                raw = readAdc0Ss3();
                j++;
            }
            if(raw<T){
                putsUart0("Error\r\n");
                continue;
            }
            pwm_r=j;
            //green
            j=0;
            raw=0;
            while(j<=1023&&raw<T){
                waitMicrosecond(10000);
                setRgbColor(0,j,0);
                waitMicrosecond(10000);
                raw = readAdc0Ss3();
                j++;
            }
            if(raw<T){
                putsUart0("Error\r\n");
                continue;
            }
            pwm_g=j;
            //blue
            j=0;
            raw=0;
            while(j<=1023&&raw<T){
                waitMicrosecond(1000);
                setRgbColor(0,0,j);
                waitMicrosecond(1000);
                raw = readAdc0Ss3();
                j++;
            }
            if(raw<T){
                putsUart0("Error\r\n");
                continue;
            }
            pwm_b=j;
            sprintf(str, "R: %d\tG: %d\tB: %d\r\n", pwm_r,pwm_g,pwm_b);
            putsUart0(str);
            setRgbColor(0,0,0);
        }else if(isCommand("trigger",0)){
            TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                  // turn-off timer
            trigger(pwm_r,pwm_g,pwm_b);
            setRgbColor(0,0,0);
        }else if(isCommand("button",0)){
            TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                  // turn-off timer
            waitMicrosecond(100000);
            waitPbPress();
            trigger(pwm_r,pwm_g,pwm_b);
            setRgbColor(0,0,0);
        }else if(isCommand("periodic",1)){
            if (getValue(0)==0){
                TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                  // turn-off timer
            }else{
                TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                  // turn-off timer
                uint32_t value = getValue(0);
                uint32_t T = 4000000*getValue(0);
                TIMER1_TAILR_R = T;          // set load value to 2e5 for 200 Hz interrupt rate
                TIMER1_CTL_R |= TIMER_CTL_TAEN;
                sprintf(str, "%d\r\n", TIMER1_TAILR_R);
                putsUart0(str);
            }
        }else if(isCommand("delta",1)){
            if(strcmp(getString(0),"off")!=0){
            D = getValue(0);
            }else{
                D=-1;
            }
        }else if(isCommand("match",1)){
            if(strcmp(getString(0),"off")!=0){
                E = getValue(0);
                }else{
                    E=-1;
                }
        }else if(isCommand("color",1)){
            TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                  // turn-off timer
            trigger(pwm_r,pwm_g,pwm_b);
            setRgbColor(0,0,0);
            tempColor.valid = 1;
            tempColor.r = r;
            tempColor.b = b;
            tempColor.g = g;
            EEPROM_WRITE((uint32_t *)&tempColor, curraddr+getValue(0)*sizeof(struct color), sizeof(tempColor)); //Write struct to EEPROM start from 0x0000
        }else if(isCommand("erase",1)){
            if(strcmp(getString(0),"all")==0){
                uint16_t i=0;
                for(i=0;i<255;++i){ // size change?
                    tempColor.valid = 0;
                    tempColor.r = 0;
                    tempColor.b = 0;
                    tempColor.g = 0;
                    EEPROM_WRITE((uint32_t *)&tempColor, curraddr+i*sizeof(struct color), sizeof(tempColor)); //Write struct to EEPROM start from 0x0000
                }
            }else{
            tempColor.valid = 0;
            tempColor.r = 0;
            tempColor.b = 0;
            tempColor.g = 0;
            EEPROM_WRITE((uint32_t *)&tempColor, curraddr+getValue(0)*sizeof(struct color), sizeof(tempColor)); //Write struct to EEPROM start from 0x0000
            }

        }else if(isCommand("led",1)){
            if(strcmp(getString(0),"on")==0){
                GREEN_LED=1;
            }else if(strcmp(getString(0),"off")==0){
                GREEN_LED=0;
            }else if (strcmp(getString(0),"sample")==0){
                led=1;
            }
        }else{
            char strprint[60];
            sprintf(strprint, "Command not found\r\n");
            putsUart0(strprint);
        }

        // wait for 10 milliseconds to make sure the light is on
        waitMicrosecond(10000);


        // nullify the input string to take other inputs
        for(j =0;j<MAX_CHARS;++j)
        {
            strinput[j]='\0';
        }

    }
}


