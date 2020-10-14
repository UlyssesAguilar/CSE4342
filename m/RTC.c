/*
 *
 * This module is used to handle all the functionalities related to the hibernation peripheral.
 *
 */

#include "tm4c123gh6pm.h"
#include "RTC.h"
#include <stdint.h>
#include <stdio.h>
#include "UART0.h"
#include "Utility.h"
#include "Temp.h"
#include "MPU.h"


/*
 *
 *  Function to initialise the hibernation peripheral with the rtc functionality,
 *  it also enables the hibernation interrupt.
 *
 *
 */
void initRTC(){
    // Configure RTC
    HIB_CTL_R = HIB_CTL_CLK32EN | HIB_CTL_RTCEN;
    NVIC_EN1_R |= 1 << (INT_HIBERNATE - 16 - 32);
    while(!HIB_CTL_WRC&HIB_CTL_R);
    return;
}

/*
 * This function is used to load the RTC register with a count of 0.
 * It is mainly used whenever the user changes the time and date.
 */
void resetRTC(){
    while(!HIB_CTL_WRC&HIB_CTL_R);
    HIB_RTCLD_R = 0;
}

/*
 * This function sets up the match functionality in the hibernation peripheral.
 */
void startMatch(){

    // Set match seconds value
    HIB_RTCM0_R = HIB_RTCC_R + T/1000;
    while (!(HIB_CTL_R & HIB_CTL_WRC));

    // Set the match subseconds value
    HIB_RTCSS_R = (uint16_t)(T%1000)*32768/1000 << HIB_RTCSS_RTCSSM_S;
    while (!(HIB_CTL_R & HIB_CTL_WRC));

    // Interrupt mask for alarm
    HIB_IM_R = HIB_IM_RTCALT0;
    while (!(HIB_CTL_R & HIB_CTL_WRC));

    // Load the current RTC value
    HIB_RTCLD_R = HIB_RTCC_R;
    while(!HIB_CTL_WRC&HIB_CTL_R);

    // Set the trim value (recomended)
    HIB_RTCT_R = 0x7FFF;
    while (!(HIB_CTL_R & HIB_CTL_WRC));

    // Call the startRTC function when done
    startRTC();
}

/*
 * function used to end the match functionality in the hibernation peripheral.
 */
void endMatch(){
   // Turn off the interrupt mask for alarm
   HIB_IM_R &= ~HIB_IM_RTCALT0;
   while (!(HIB_CTL_R & HIB_CTL_WRC));

   // Load the current rtc value
   HIB_RTCLD_R = HIB_RTCC_R;
   while(!HIB_CTL_WRC&HIB_CTL_R);

   // Set the trim value
   HIB_RTCT_R = 0x7FFF;
   while (!(HIB_CTL_R & HIB_CTL_WRC));

   // Call the startRTC function when done
   startRTC();
}

/*
 * The store data function is used to store the main data into the
 * 16 * 32 bit words in the battery backed memory in the hibernation peripheral
 *
 * it stores the following variables:
 *  Hflag -- Hysteresis flag
 *  Ltflag -- less than flag (gating)
 *  Trigflag -- trigger flag (0 no trigger / 1 trigger is ongoing)
 *  currOffset -- offset for the current page (flash)
 *  NSamples -- the number of samples left
 *  Para -- parameter for gating and hysteresis
 *  level -- the value for the gating function
 *  H -- the hysteresis value
 *  storedTime -- this is set using the uart
 *  storedDate -- this is set using the uart
 *  T -- time used for the periodic function
 *  logMask -- variable used to store logging flags, leveling flag, and  sleeping flags.
 *  state -- variable that stores the current state for the lfsr random number generator
 *  startingState -- variable that stores the starting state for the lfsr random number
 *                   generator.
 */
void storeData(){
    // go through each of the 16 words
    uint32_t i =0;
    for(i=0;i<16;++i){

        // for each i store one of the variables
        // we use HWREG to be able to get the register that is used.
        switch(i){
        case 0:
           HIB_DATA_R = currOffset;
           while (!(HIB_CTL_R & HIB_CTL_WRC));
           break;
        case 1:
            HWREG(0x400FC030 + (i*4))= (uint32_t)NSamples;
            while (!(HIB_CTL_R & HIB_CTL_WRC));

            break;
        case 2:
            HWREG(0x400FC030 + (i*4))= (uint32_t)Para;
            while (!(HIB_CTL_R & HIB_CTL_WRC));

            break;
        case 3:
            HWREG(0x400FC030 + (i*4))= (uint32_t)level;
            while (!(HIB_CTL_R & HIB_CTL_WRC));

            break;
        case 4:
            HWREG(0x400FC030 + (i*4))= (uint32_t)H;
            while (!(HIB_CTL_R & HIB_CTL_WRC));

            break;
        case 5:
            HWREG(0x400FC030 + (i*4)) = (uint32_t)storedTime.hrs;
            while (!(HIB_CTL_R & HIB_CTL_WRC));

            break;
        case 6:
            HWREG(0x400FC030 + (i*4)) = (uint32_t)storedTime.min;
            while (!(HIB_CTL_R & HIB_CTL_WRC));

            break;
        case 7:
            HWREG(0x400FC030 + (i*4)) = (uint32_t)storedTime.sec;
            while (!(HIB_CTL_R & HIB_CTL_WRC));

            break;
        case 8:
            HWREG(0x400FC030 + (i*4)) = (uint32_t)storedDate.day;
            while (!(HIB_CTL_R & HIB_CTL_WRC));

            break;
        case 9:
            HWREG(0x400FC030 +(i*4)) =(uint32_t)storedDate.mth;
            while (!(HIB_CTL_R & HIB_CTL_WRC));

            break;
        case 10:
            HWREG(0x400FC030 + (i*4)) = (uint32_t)storedDate.yr;
            while (!(HIB_CTL_R & HIB_CTL_WRC));

            break;
        case 11:
            HWREG(0x400FC030 + (i*4)) =(uint32_t) T;
            while (!(HIB_CTL_R & HIB_CTL_WRC));

            break;
        case 12:
            HWREG(0x400FC030 + (i*4)) = (uint32_t)logMask | (LTflag<<6)|(Hflag<<7)|(Trigflag<<8);
            while (!(HIB_CTL_R & HIB_CTL_WRC));

            break;
        case 13:
            HWREG(0x400FC030 + (i*4)) = (uint32_t)state;
            while (!(HIB_CTL_R & HIB_CTL_WRC));

            break;
        case 14:
            HWREG(0x400FC030 + (i*4)) = (uint32_t)startingState;
            while (!(HIB_CTL_R & HIB_CTL_WRC));

            break;
        }

    }

    return;
}
/*
 * The retrieve data function is used to retrieve the main data from the
 * 16 * 32 bit words in the battery backed memory in the hibernation peripheral
 *
 * it retrieves the following variables:
 *  Hflag -- Hysteresis flag
 *  Ltflag -- less than flag (gating)
 *  currOffset -- offset for the current page (flash)
 *  NSamples -- the number of samples left
 *  Para -- parameter for gating and hysteresis
 *  level -- the value for the gating function
 *  H -- the hysteresis value
 *  storedTime -- this is set using the uart
 *  storedDate -- this is set using the uart
 *  T -- time used for the periodic function
 *  logMask -- variable used to store logging flags, leveling flag, and  sleeping flags.
 *  state -- variable that stores the current state for the lfsr random number generator
 *  startingState -- variable that stores the starting state for the lfsr random number
 *                   generator.
 */
void retrieveData(){
    char str[60];

    // go through each of the 16 words

    uint8_t i =0;

    for(i=0;i<16;++i){
        // for each i store one of the variables
        // we use HWREG to be able to get the register that is used.
        switch(i){
        case 0:
            currOffset = HWREG(0x400FC030 + (i*4));
            break;
        case 1:
            NSamples = HWREG(0x400FC030 + (i*4));
            break;
        case 2:
            Para = HWREG(0x400FC030 + (i*4));
            break;
        case 3:
            level = HWREG(0x400FC030 + (i*4));
            break;
        case 4:
            H = HWREG(0x400FC030 + (i*4));
            break;
        case 5:
            storedTime.hrs =HWREG(0x400FC030 + (i*4));
            break;
        case 6:
            storedTime.min=HWREG(0x400FC030 + (i*4));
            break;
        case 7:
            storedTime.sec=HWREG(0x400FC030 + (i*4));
            break;
        case 8:
            storedDate.day=HWREG(0x400FC030 + (i*4));
            break;
        case 9:
            storedDate.mth=HWREG(0x400FC030 + (i*4));
            break;
        case 10:
            storedDate.yr=HWREG(0x400FC030 + (i*4));
            break;
        case 11:
             T=HWREG(0x400FC030 + (i*4));
            break;
        case 12:
            logMask = 0x3F & HWREG(0x400FC030 + (i*4));
            LTflag = (0x040&HWREG(0x400FC030 + (i*4)))>>6;
            Hflag = (0x080&HWREG(0x400FC030 + (i*4)))>>7;
            Trigflag = (0x100&HWREG(0x400FC030 + (i*4)))>>8;
            break;
        case 13:
            state=HWREG(0x400FC030 + (i*4));
            break;
        case 14:
            startingState=HWREG(0x400FC030 + (i*4));
            break;
        }

    }
    return;
}
/*
 * This function is called at the end of a function to do a
 * hibernation request of the chip is set to sleep.
 */
void startRTC(){

    // if sleep flag is on
    if(logMask&0x20){
        // interrupt for external wakeup
        HIB_IM_R |= HIB_IM_EXTW;
        while (!(HIB_CTL_R & HIB_CTL_WRC));

        // hibernation request
        HIB_CTL_R = HIB_CTL_CLK32EN | HIB_CTL_RTCEN|HIB_CTL_RTCWEN|HIB_CTL_PINWEN|HIB_CTL_VDD3ON|HIB_CTL_HIBREQ;
        while (!(HIB_CTL_R & HIB_CTL_WRC));
        while(1)
        {

        }
    }
    return;
}

/*
 * This function stops the rtc.
 */
void stopRTC(){
    HIB_CTL_R = HIB_CTL_CLK32EN;
    while(!(HIB_CTL_WRC&HIB_CTL_R));
}

/*
 * Function used to handle the hibernation peripheral interrupts
 */
void HibIsr(){

    // read the status
    uint32_t status = HIB_MIS_R;


    if (status & HIB_MIS_RTCALT0==0x01)
    {
        // if its a match interrupt

        // start by retrieving the vital data
        retrieveData();

        // if the sleep flag is on, this means we just woke up.
        // so intialise the temprature functionality
        if(logMask&0x20){
        initTemp();
        }

        // sample
        SampleWrapper();

        // store the vital information
        storeData();

        // keep going if there is more samples to be done,
        // else stop
        if (NSamples>0){
            startMatch();
        }else if(NSamples <=0){
            endMatch();
        }

        // clear the interrupt
        HIB_IC_R |= status;
        while (!(HIB_CTL_R & HIB_CTL_WRC));
    }else if(status&HIB_MIS_EXTW){
        // if it is a external wake up interrupt

        // retrieve the vital data
        retrieveData();

        // if the sleep flag is on, this means we just woke up.
        // so intialise the temprature functionality
        if(logMask&0x20){
        initTemp();
        }

        // if there is a trigger flag ongoing call the trigger
        // handler.
        if (Trigflag ==1){
            MPUIsr();
        }

        // clear the interrupt
        HIB_IC_R |= status;
        while (!(HIB_CTL_R & HIB_CTL_WRC));

        // if we reached this point this means, the user is trying
        // to wake up on a periodic mode. Therefore, wake up.

        // clear sleep flag.
        logMask&=~0x20;

        // store vital data
        storeData();

        // used for the trigger
        if(Trigflag ==1){
            startRTC();
        }

        // clear mpu status.
        readI2c0Register(MPU9250, 0x3A);

    }
}

/*
 * function used to set the sleeping flag and do a hibernation request.
 */
void HibSleep(){
    // set the slepeping flag
    logMask|=0x20;

    // store vital data
    storeData();

    // set the external wake up interrupt
    HIB_IM_R |= HIB_IM_EXTW;
    while (!(HIB_CTL_R & HIB_CTL_WRC));

    // do a hibernation request
    HIB_CTL_R = HIB_CTL_CLK32EN | HIB_CTL_RTCEN|HIB_CTL_RTCWEN|HIB_CTL_PINWEN|HIB_CTL_VDD3ON|HIB_CTL_HIBREQ;
    while (!(HIB_CTL_R & HIB_CTL_WRC));

    // wait for sleep
    while(1)
    {

    }
}
