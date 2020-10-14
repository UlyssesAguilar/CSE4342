// Gyroscope project
// Ulysses Aguilar
// Bashar Al Atom

/*
 * This is the main module where most of the uart commands handling happens.
 */
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "tm4c123gh6pm.h"
#include <string.h>
#include "UART0.h"
#include "RTC.h"
#include "Utility.h"
#include "I2C0.h"
#include "MPU.h"
#define  MAX_CHARS 100

//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------

// array to store input
char strinput[MAX_CHARS];
// array of char* to tokenize string
char* tokens[10];
// this is used to store the number of arguments
uint8_t NArgs = 0;

// not used anymore
uint32_t RTC_Old=0;

// not used anymore
uint16_t currPage;

// variables used to store the current and starting states for the lfsr random
// number generator
uint32_t state=0,startingState=0;

// variable used to store the periodic time period
uint32_t T=1;

// variable used to store the number of samples left
uint32_t NSamples=0;

// variable used to store the days of each month. It is mainly used to handle the
// the time and data functions
uint16_t daysOfEachMonth[12] = {31,28,31,30,31,30,31,31,30,31,30,31};

// variable used to store the hour minute and second, from which the rtc counts
struct time storedTime;

// variable used to store the day month year, from which the rtc counts
struct date storedDate;

// Para is used to store the parameter that we are gating/hysteresis
// LTflag is used to store the flag of the GT|LT of the gating function
// Hflag is used to store the status of hysteresis
// Trigflag is used to identify if there is a trigger ongoing
uint32_t Para=4,LTflag,Hflag=0,Trigflag = 1;


// level is used to store the gating level value
// H is used to store the hysteresis value
uint32_t level,H=0;

// logmask is used to store the logging flags. This include the leveling flag,
// as well as the sleeping flag.
uint32_t logMask=0;

// not used anymore
uint8_t sleepflag;

// variable used to store the current offset in the current page.
uint32_t currOffset=0x00;

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



    // start the clocking for the GPIO port F
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOF;

    // unlock it and make sure the interrupt flag is cleared for PF0
    GPIO_PORTF_LOCK_R = 0x4C4F434B;
    GPIO_PORTF_CR_R|=0x01;
    GPIO_PORTF_IM_R&=~0x01;
    GPIO_PORTF_LOCK_R = 0;

    // wait for a bit
    waitMicrosecond(10000);


    // initialise the other peripherals
    initUART0();
    initI2c0();
    initRTC();
    initTemp();
    initMPU9250();
}

// function called to parse the command string
void parse_string()
{
    // Returns first token
    tokens[0] = strtok(strinput, " ,");
    NArgs =0;
    // Keep printing tokens while one of the
    // delimiters present in str[].
    while (tokens[NArgs] != NULL)
    {
        NArgs++;
        tokens[NArgs] = strtok(NULL, " ,");
    }

}
// returns are the value of the nth argument
uint16_t getValue(uint8_t argNumber)
{
    return atoi(tokens[argNumber+1]);
}

// returns the nth argument as a string
char * getString(uint8_t argNumber)
{
    return tokens[argNumber+1];
}

// checks if the strinput is a command with minArgs number of arguments
bool isCommand(const char * strcmd, uint8_t minArgs)
{
    if(strcmp(tokens[0],strcmd)==0 && NArgs>=minArgs )
    {
        return 1;
    }
    return 0;
}

// function that edits the stored date variable to the new data by adding a
// specfic number of days.
void newDateFromDay(uint16_t daysToAdd){
        storedDate.day += daysToAdd;
        while(storedDate.day>daysOfEachMonth[storedDate.mth-1]){
            if((storedDate.yr%4==0)^(storedDate.yr%100==0)^(storedDate.yr%400==0)){
                        if(storedDate.mth == 2 &&storedDate.day>daysOfEachMonth[1]+1){
                            storedDate.mth+=1;
                            storedDate.day -=daysOfEachMonth[1]+1;
                        }
            }else{
                storedDate.day-=daysOfEachMonth[storedDate.mth-1];
                storedDate.mth+=1;
            }

            if(storedDate.mth>=12){
                storedDate.yr+=1;
                storedDate.mth-=12;
            }
        }
}

// function used to set the stored time and date variables
void setStoredTimeAndDate(uint32_t RTC){
    RTC += (storedTime.hrs*3600+storedTime.min*60+storedTime.sec);
    storedTime.hrs = (RTC/3600)%24;
    storedTime.min = (RTC/60)%60;
    storedTime.sec = (RTC)%60;
    uint16_t days =  (RTC/86400);
    newDateFromDay(days);
}

// function used to print the current time.
void printTime(uint32_t RTC){
    char str[60];
    RTC += (storedTime.hrs*3600+storedTime.min*60+storedTime.sec);
    sprintf(str, "%d:%d:%d\n",(RTC/3600)%24,(RTC/60)%60,(RTC)%60);
    putsUart0(str);
}

// function used to print the date
void printDate(uint32_t RTC){
    char str[60];
    uint16_t day,mth,yr;
    RTC += (storedTime.hrs*3600+storedTime.min*60+storedTime.sec);
    day = storedDate.day+ RTC/86400;
    mth = storedDate.mth;
    yr = storedDate.yr;
    while(day>daysOfEachMonth[mth-1]){
        if((yr%4==0)^(yr%100==0)^(yr%400==0)){
                    if(mth == 2 &&day>daysOfEachMonth[1]+1){
                        mth+=1;
                        day -=daysOfEachMonth[1]+1;
                    }
        }else{
            day-=daysOfEachMonth[mth-1];
            mth+=1;
        }

        if(mth>=12){
            yr+=1;
            mth-=12;
        }
    }
    sprintf(str,"%d / %d / %d\n",mth,day,yr);
    putsUart0(str);
}

// main function
int main(void)
 {

    // setup hardware
    initHw();
    char str[60];       // str is used to be able to print the raw value to
                       // the UART

    // variables used for the I2c and loops
    int16_t i = 0,j;
    uint8_t add,reg,data;

    // setup a starting value for the stored date and time variables
    storedDate.day = 3;
    storedDate.mth = 10;
    storedDate.yr = 2019;
    storedTime.sec = 0;
    storedTime.min = 48;
    storedTime.hrs = 11;

    while(1) // infinite loop
    {
        putsUart0("\t\t********\t\t\r\n");
        getsUart0(&strinput,100); // get the input from the UART
        putsUart0("\r\n");
        parse_string(); // tokenize the input
        if (isCommand("time",3))
        {
            // time hr min sec command
            if (validateTime(atoi(tokens[1]),atoi(tokens[2]),atoi(tokens[3])))
            {
            storedTime.hrs  = atoi(tokens[1]);
            storedTime.min  = atoi(tokens[2]);
            storedTime.sec  = atoi(tokens[3]);
            resetRTC();
            }
            else
            {
                sprintf(str, "Input time is wrong. Please ensure that the time is correct.\n");
                putsUart0(str);
            }
        }
        else if (isCommand("time",0)){
            // time command
            printTime(HIB_RTCC_R);

        }
        else if(isCommand("date",3))  // if the command is an rgb command
        {
            // date month day year command

            if (validateDate(atoi(tokens[1]),atoi(tokens[2]),atoi(tokens[3])))
            {
            storedDate.mth = atoi(tokens[1]);
            storedDate.day = atoi(tokens[2]);
            storedDate.yr = atoi(tokens[3]);
            resetRTC();
            }
            else
            {
                sprintf(str, "Input date is wrong. Please ensure that the date is correct.\n");
                putsUart0(str);
            }
        }
        else if(isCommand("date",0)){
            // date command
            printDate(HIB_RTCC_R);
        }
        else if(isCommand("reset",0)){
            // reset command
            NVIC_APINT_R = NVIC_APINT_SYSRESETREQ|NVIC_APINT_VECTKEY;

        }
        else if (isCommand("write",3))
        {
            // write address register data
            // this is used to write to the i2c
            add = asciiToUint8(tokens[1]);
            reg = asciiToUint8(tokens[2]);
            data = asciiToUint8(tokens[3]);
            writeI2c0Register(add, reg, data);
            sprintf(str, "Writing 0x%02hhx to address 0x%02hhx, register 0x%02hhx\n", data, add, reg);
            putsUart0(str);
        }
        else if (isCommand("read",2))
        {
            // read address register
            // this is used to read from the i2c
            add = asciiToUint8(tokens[1]);
            reg = asciiToUint8(tokens[2]);
            data = readI2c0Register(add, reg);
            sprintf(str, "Read 0x%02hhx from address 0x%02hhx, register 0x%02hhx\n", data, add, reg);
            putsUart0(str);
        }
        else if (isCommand("poll",0))
        {
            // poll command
            // lists devices on the i2c
            putsUart0("Devices found: ");
            for (i = 4; i < 119; i++)
            {
                if (pollI2c0Address(i))
                {
                    sprintf(str, "0x%02x ", i);
                    putsUart0(str);
                }
            }
            putsUart0("\n");
        }
        else if (isCommand("help",0))
        {
            // help command
            putsUart0("poll\n");
            putsUart0("read ADD REG\n");
            putsUart0("write ADD REG DATA\n");
        }
        else if(isCommand("temp",0)){
            // temp command
            // this simply gets the temprature value from the chip
            uint32_t g = getTemp();
            sprintf(str, "%d\n", g);
            putsUart0(str);
        }
        else if(isCommand("read",0)){
            // read command
            // this function reads the sensor data
            int16_t values[3];
            readAccelData(values);
            sprintf(str, "Accel data - > %d   %d   %d\r\n", values[0], values[1], values[2]);
            putsUart0(str);
            readGyroData(values);
            sprintf(str, "Gyro data - > %d   %d   %d\r\n", values[0], values[1], values[2]);
            putsUart0(str);
            readMagData(values);
            sprintf(str, "Mag data - > %d   %d   %d\r\n", values[0], values[1], values[2]);
            putsUart0(str);
            uint32_t g = getTemp();
            sprintf(str, "Temprature - > %d\n", g);
            putsUart0(str);
        }
        else if(isCommand("periodic",1)){
            // periodic T command
            // setup the T value
            T = asciiToUint32(tokens[1]);
            if(NSamples >0 && validateInput()){
                // if the input is valid and the number of samples is bigger than zero start
                storeData();
                sprintf(str, "Periodic sampling at %d ms\r\n",T);
                putsUart0(str);
                startMatch();
            }else{
                sprintf(str, "Please check all the input parameters are correct!\r\n");
                putsUart0(str);
            }

        }
        else if(isCommand("samples",1)){
            // samples N command

            // sets the number of samples
            NSamples = asciiToUint8(tokens[1]);
            sprintf(str, "# of Samples parameter: %d \r\n",NSamples);
            storeData();
            putsUart0(str);
        }
        else if(isCommand("trigger",0))
        {
            // trigger command
            if(NSamples >0 && validateInput()){
                // if the input is valid and the number of samples is bigger than zero start
                storeData();
                Trigflag = 1;
                startTrigger();
                readI2c0Register(MPU9250, 0x3A);
            }else{
                sprintf(str, "Please check all the input parameters are correct!\r\n");
                putsUart0(str);
            }
        }
        else if (isCommand("gating",3)){
            // gating PARAMETER GT|LT level
            SetGating();
            storeData();
            sprintf(str, "Gating parameters: %d %d %d\r\n",Para,LTflag,level);
            putsUart0(str);
        }
        else if(isCommand("log",1)){
            // log PARAMETER command
            setLogging();
            storeData();
            sprintf(str, "Logging mask: %d \r\n",logMask);
            putsUart0(str);
        }else if(isCommand("hyst",1))
        {
            // hyst H command
            // this is the Hysteresis command

            Hflag = 0;
            H = asciiToUint32(tokens[1]);
            storeData();
            sprintf(str, "Hysterisis parameter: %d \r\n",H);
            putsUart0(str);
        }else if(isCommand("stop",0)){
            // stop command
            // stops periodic and trigger commands
            endMatch();
            stopTrigger();
        }else if(isCommand("leveling",1)){
            // leveling on|off command
            if(strcmp(tokens[1],"on")==0){
            // if on setup the flag in logmask
            logMask|=0x10;
            // get a starting state and ensure it is after 64k bytes to
            // ensure not to write over critical data
            state = (getRandomStart()<<2)|(0x010000);                    //asciiToUint32(tokens[1]);
            sprintf(str, "Leveling is on! Starting state is: %d \r\n",state);
            putsUart0(str);
            // save the starting state
            startingState = state;
            // current offset in this page is zero
            currOffset = 0x0000;
            // store vital data
            storeData();

            // erase the page to be able to write
            eraseFlash(state&0x3FC00);
            }else if(strcmp(tokens[1],"off")==0){
            // if off clear leveling flag
            logMask&=~0x10;
            // start at 65k page
            state =(0x010000);

            sprintf(str, "Leveling is off! Starting state is: %d \r\n",state);
            putsUart0(str);
            // save starting state
            startingState = state;
            // current offset in this page is zero
            currOffset = 0x0000;
            // save vital data
            storeData();
            // erase the page to be able to write
            eraseFlash(state&0x3FC00);
            }

        }else if(isCommand("next",0)){
            // next command
            // used to skip to the next state
            nextPage();
            sprintf(str, "next parameter: %u pp\t %d \r\n",state,state);
            putsUart0(str);
        }else if(isCommand("data",0)){
            // data command, prints all the logged data
            printData();
        }else if(isCommand("states",1)){
            // states G command
            // sets the state to G
            state = asciiToUint32(tokens[1]);
            sprintf(str, "State is: %d \r\n",state);
            putsUart0(str);
            startingState = state;
            currOffset = 0x0000;
            eraseFlash(state&0x3FC00);
            storeData();

        }else if(isCommand("sampless",0)){

            // sampless command
            // print the number of samples left
            sprintf(str, "# of Samples parameter: %d \r\n",NSamples);
                       putsUart0(str);
        }else if(isCommand("sleep",0)){
            // sleep command
            // do a hibernation request
            HibSleep();
            sprintf(str, "Going to sleep...\r\n");
            putsUart0(str);

        }

        // nullify the input string to take other inputs
        for(j =0;j<MAX_CHARS;++j)
        {
            strinput[j]='\0';
        }

    }

}
