#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>
#include "tm4c123gh6pm.h"
#include "UART0.h"
// function to initilise the uart
void initUART0(){

    SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOA;
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;         // turn-on UART0, leave other UARTs in same status
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
