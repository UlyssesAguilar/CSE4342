#include <stdint.h>
#include "tm4c123gh6pm.h"
#include "Utility.h"
#include <stdio.h>
#include "UART0.h"
#include "MPU.h"
#include <math.h>
#include "Temp.h"
#include "RTC.h"

/*
 * The utility module handles various misc functions
 */

// function to convert a string to uint8
uint8_t asciiToUint8(const char str[])
{
    uint8_t data;
    if (str[0] == '0' && tolower(str[1]) == 'x')
        sscanf(str, "%hhx", &data);
    else
        sscanf(str, "%hhu", &data);
    return data;
}

// function to convert strong to float
float asciiToFloat(const char str[])
{
    float data;
    if (str[0] == '0' && tolower(str[1]) == 'x')
        sscanf(str, "%x", &data);
    else
        sscanf(str, "%f", &data);
    return data;
}


// function to convert string to uint32
uint32_t asciiToUint32(const char str[])
{
    uint32_t data;
    if (str[0] == '0' && tolower(str[1]) == 'x')
        sscanf(str, "%x", &data);
    else
        sscanf(str, "%d", &data);
    return data;
}
// utility function used for delta command, returns absolute value
int32_t abs(int32_t a)
{
    if(a<0)
        return -1*a;
    return a;
}

// function to return the absolute of a float
float absfloat(float a){
    if (a<0.0)
        return -1*a;
    return a;
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

// function to validate time
int validateTime(uint16_t hrs,uint16_t min,uint16_t sec){
    if(hrs>=0 && hrs<24)
        if(min>=0 && min<60)
            if(sec >= 0 && sec<60)
                return 1;
    return 0;
}

// function used to validate dates
int validateDate(uint16_t mth,uint16_t day,uint16_t yr){
    uint16_t daysOfEachMonth[12] = {31,28,31,30,31,30,31,31,30,31,30,31};
    if(mth>0 && mth<=12)
        {
        if(day>0 && day<=daysOfEachMonth[mth-1]&& mth !=2)
            if(yr>=0)
                return 1;

        if(mth == 2 && yr> 0 && (yr%4==0)^(yr%100==0)^(yr%400==0)&& day>0 && day <daysOfEachMonth[mth-1]+1)
            return 1;
        }
    return 0;
}


// function to calculate eucladian distance
float eucDis(int16_t x,int16_t y,int16_t z){
    return sqrt(x*x+y*y+z*z);
}

// function to samples
void Sample(){
    char str[60];

    if(!(logMask&0x0F)){

    // if not logging print to uart
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
    NSamples-=1;
    }else{
        // if logging, log the variables indicated by their flags in the logMask
        int32_t values[3];
        int16_t temps[3];
        uint32_t size=0;
        uint8_t i =0;
        if(logMask & 0x01)
        {
            readAccelData(temps);
            for(i=0;i<3;++i){
                values[i] = temps[i];
            }


            size = 3;
            writeFlash(values,size);
        }
        if(logMask&0x02)
        {
            readGyroData(temps);
            for(i=0;i<3;++i){
                values[i] = temps[i];
            }

            size = 3;
            writeFlash(values,size);

        }
        if(logMask&0x04)
        {
            readMagData(temps);
            for(i=0;i<3;++i){
                values[i] = temps[i];
            }
            size = 3;
            writeFlash(values,size);
        }
        if(logMask&0x08)
        {
            values[0] = getTemp();
            size = 1;
            writeFlash(values,size);
        }
        values[0] =  HIB_RTCC_R;
        size = 1;
        writeFlash(values,size);
        NSamples-=1;
    }
}

// sample wrapping function
// this handles gating, and hysteresis
void SampleWrapper(){
    int16_t values[3];
    if(NSamples>0){
            if (Para == 4){
                    Sample();
            }else{
                // Accel
                if(Para == 0){
                    readAccelData(values);
                    if(absfloat(eucDis(values[0],values[1],values[2])-level)>=H){

                    if(LTflag==0){
                         if(eucDis(values[0],values[1],values[2])>=level&&Hflag ==0){
                             Sample();
                             if(H>0.0){
                                 Hflag=1;
                             }
                         }else if(Hflag ==1&&eucDis(values[0],values[1],values[2])<level){
                             Hflag = 0;
                         }
                    }else if(LTflag==1){
                         if(eucDis(values[0],values[1],values[2])<level&&Hflag ==0){
                             Sample();
                             if(H>0){
                                 Hflag=1;
                             }
                         }else if(Hflag ==1&&eucDis(values[0],values[1],values[2])>level){
                             Hflag = 0;
                         }
                    }
                    }else{
                        if(Hflag==1){
                            Hflag=0;
                        }
                    }
                }else if(Para == 1){
                    // Gyroscope
                    readGyroData(values);
                    if(absfloat(eucDis(values[0],values[1],values[2])-level)>=H){
                    if(LTflag==0){
                         if(eucDis(values[0],values[1],values[2])>=level&&Hflag ==0){
                             Sample();
                             if(H>0.0){
                                 Hflag=1;
                             }
                         }else if(Hflag ==1&&eucDis(values[0],values[1],values[2])<level){
                             Hflag = 0;
                         }
                    }else if(LTflag==1){
                         if(eucDis(values[0],values[1],values[2])<level&&Hflag ==0){
                             Sample();
                             if(H>0){
                                 Hflag=1;
                             }
                         }else if(Hflag ==1&&eucDis(values[0],values[1],values[2])>level){
                             Hflag = 0;
                         }
                    }
                    }else{
                        if(Hflag==1){
                            Hflag=0;
                        }
                    }
                }else if(Para == 2){
                    // Magenatic
                    readMagData(values);
                    if(absfloat(eucDis(values[0],values[1],values[2])-level)>=H){
                    if(LTflag==0){
                         if(eucDis(values[0],values[1],values[2])>=level&&Hflag ==0){
                             Sample();
                             if(H>0.0){
                                 Hflag=1;
                             }
                         }else if(Hflag ==1&&eucDis(values[0],values[1],values[2])<level){
                             Hflag = 0;
                         }
                    }else if(LTflag==1){
                         if(eucDis(values[0],values[1],values[2])<level&&Hflag ==0){
                             Sample();
                             if(H>0){
                                 Hflag=1;
                             }
                         }else if(Hflag ==1&&eucDis(values[0],values[1],values[2])>level){
                             Hflag = 0;
                         }
                    }
                    }else{
                        if(Hflag==1){
                            Hflag=0;
                        }
                    }
                }
                else if(Para == 3){
                    // Temperature
                    uint32_t Temp = getTemp();
                    if(absfloat(Temp-level)>=H){
                    if(LTflag==0){
                         if(Temp>=level&&Hflag==0){
                             Sample();
                             if(H!=0){
                                 Hflag=1;
                             }
                         }else if(Hflag ==1&&Temp<level){
                             Hflag = 0;
                         }
                    }else if(LTflag==1){
                         if(Temp<level&&Hflag==0){
                             Sample();
                             if(H!=0){
                                 Hflag=1;
                             }
                         }else if(Hflag ==1&&Temp>level){
                             Hflag = 0;
                         }
                    }
                }else{
                    if(Hflag==1){
                        Hflag=0;
                    }
                }
                }

            }
        }

}

// trigger interrupt handler
void MPUIsr(){
    // reset the sensor
    turnOnEverything();
    // read int_status  to clear interrupt
    uint32_t status = HIB_MIS_R;
    // retrieve vital data
    retrieveData();
    // stop the trigger
    stopTrigger();


    waitMicrosecond(10000);
    // sample
    SampleWrapper();

    if(NSamples <=0){
        // stop if done
        stopTrigger();
    }else{
        // go again
        startTrigger();
    }
    if(Trigflag ==1&&NSamples<=0&&logMask&0x20){
         // if sleeping, go back to sleep now that we are done
         Trigflag=0;
         stopTrigger();
         storeData();
         startRTC();
         storeData();
         main();
    }
    // store vital data
    storeData();
    // check if need to go to sleep
    startRTC();

   waitMicrosecond(100);
   // clear interrupt status
   uint8_t stat = readI2c0Register(MPU9250, 0x3A);
   GPIO_PORTF_ICR_R |= 0x01;
}

// function to validate gating, and hyst input
// as well as log input
int validateInput(){
    if(Para <0||Para>4){
        return 0;
    }else if(Para>=0 && Para <=3){
        if(level <0){
            return 0;
        }
        if(LTflag>1||LTflag<0){
            return 0;
        }
    }

    if (T<=0){
        return 0;
    }

    if(logMask&0x0F){
        if(state<0x010000){
            return 0;
        }
    }
    return 1;
}

// function to set the gating input
void SetGating(){
    if(strcmp(tokens[1],"accel")==0){
        Para = 0;
    }else if(strcmp(tokens[1],"gyro")==0){
        Para = 1;
    }else if(strcmp(tokens[1],"compass")==0){
        Para = 2;
    }else if(strcmp(tokens[1],"temp")==0){
        Para = 3;
    }else{
        Para = 4;
    }

    if(strcmp(tokens[2],">")==0){
            LTflag = 0;
    }else if(strcmp(tokens[2],"<")==0){
            LTflag= 1;
    }

    level = asciiToUint32(tokens[3]);
}

// functions to set log input
void setLogging(){

    if(strcmp(tokens[1],"accel")==0){
        logMask|=0x01;
    }else if(strcmp(tokens[1],"gyro")==0){
        logMask|=0x02;
    }else if(strcmp(tokens[1],"compass")==0){
        logMask|=0x04;
    }else if(strcmp(tokens[1],"temp")==0){
        logMask|=0x08;
    }else if(strcmp(tokens[1],"remove")==0){
        logMask&=~0x0F;
    }else if(strcmp(tokens[1],"all")==0){
        logMask|=0x0F;
    }
}

// function to erase a page in flash
void eraseFlash(uint32_t add){
    char str[60];
    FLASH_FMA_R = add;
    FLASH_FMC_R = FLASH_FMC_WRKEY | FLASH_FMC_ERASE;
    sprintf(str, "State is: %d \r\n",state);
    putsUart0(str);
    while(FLASH_FMC_R & FLASH_FMC_ERASE);

}

// function to write a word to flash
void writeFlash(int32_t data[],uint32_t size){
    char str[60];

    uint32_t add = (state&0x3FC00)|(currOffset);
    uint32_t k = 0;

    while(size>0){
        FLASH_FMA_R = add;
        FLASH_FMD_R = data[k];
        FLASH_FMC_R = FLASH_FMC_WRKEY | FLASH_FMC_WRITE;
        while(FLASH_FMC_R & FLASH_FMC_WRITE);
        add+=sizeof(int32_t);
        if(add>=(state&0x3FC00)+1024){
            add=nextPage()&0x3FC00;
            eraseFlash(add);
        }
        size-=1;
        k+=1;
    }
    currOffset = add & 0x03FF;
}

// function to get the next page.
// this function takes into account leveling and non leveling
uint32_t nextPage()
{
 if(logMask&0x10){
 do{
  state>>=2;
  uint16_t bit;
  bit  = ((state >> 2) ^(state >> 6) ^(state >> 7) ^ (state >> 8) ^ (state >> 10) ^ (state >> 15) ) & 1;
  state =  (state >> 1) | (bit << 15);
  state<<=2;
 }while(state<0x010000);
 return state;

 }else{
     state+=1024;
     return state;
 }
}
// function used to read flash, used in the data command
void readFlash(int32_t* data,uint32_t size){
    uint32_t add = (state&0x3FC00)|(currOffset);
    uint32_t k = 0;
    while(size>0){
        data[k]=*((int32_t*)(add));
        add+=sizeof(int32_t);
        if(add>=(state&0x3FC00)+1024){
            add=nextPage()&0x3FC00;
        }
        k+=1;
        size-=1;
    }
    currOffset = add & 0x03FF;
}

// function used to print the logged data, used in the data command
void printData(){
    uint32_t currState = state;
    uint32_t offset = currOffset;
    state = startingState;
    currOffset = 0x0000;
    char str[60];
    uint32_t i=0;
    while(state!=currState||currOffset!=offset){
        int32_t values[3];
        sprintf(str, "DATA: %d\r\n", i);
        putsUart0(str);
        i++;
        if(logMask & 0x01)
        {
            readFlash(values,3);
            sprintf(str, "Accel data - > %d   %d   %d\r\n", values[0], values[1], values[2]);
            putsUart0(str);
        }
        if(logMask&0x02)
        {
            readFlash(values,3);
            sprintf(str, "Gyro data - > %d   %d   %d\r\n", values[0], values[1], values[2]);
            putsUart0(str);
        }
        if(logMask&0x04)
        {
            readFlash(values,3);
            sprintf(str, "Compass data - > %d   %d   %d\r\n", values[0], values[1], values[2]);
            putsUart0(str);
        }
        if(logMask&0x08)
        {
            readFlash(values,1);
            sprintf(str, "temprature data - > %d\r\n", (uint32_t) values[0]);
            putsUart0(str);
        }
        readFlash(values,1);
        sprintf(str, "timestamp: - > %d\r\n", (uint32_t) values[0]);
        putsUart0(str);
    }
    state = currState;
    currOffset = offset;
    return;
}

