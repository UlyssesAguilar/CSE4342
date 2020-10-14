#include "Temp.h"
#include <stdint.h>
#include "tm4c123gh6pm.h"
#include "Utility.h"

// initilise the temprature sensore
void initTemp(){
    SYSCTL_RCGCADC_R |= 1;       /* enable clock to ADC0 */
    waitMicrosecond(10000);

    ADC0_CC_R=ADC_CC_CS_SYSPLL;
    waitMicrosecond(10000);

    /* initialize ADC0 */
    ADC0_ACTSS_R &= ~8;          /* disable SS3 during configuration */
    ADC0_EMUX_R = ADC_EMUX_EM3_PROCESSOR;
    ADC0_SSMUX3_R = 0;           /* get input from channel 0 */
    waitMicrosecond(10000);

    ADC0_SSCTL3_R |= 0x0A;       /* take chip temperature, set flag at 1st sample */
    ADC0_ACTSS_R |= 8;           /* enable ADC0 sequencer 3 */

}

// function to get the temprature
int32_t getTemp(){
    int32_t temperature;
    ADC0_PSSI_R |=ADC_PSSI_SS3;
    while(ADC0_ACTSS_R&ADC_ACTSS_BUSY);                    /* wait for conversion complete */
      temperature = 147 - (247 * ADC0_SSFIFO3_R) / 4096;
      return temperature;
}
// function that uses the adc to get a random starting state
uint16_t getRandomStart()
{
    uint32_t start = 0;
    uint16_t temp = 0;
    uint8_t i=0;
    for(i=0;i<16;i++)
    {
        // get the temp value
        ADC0_PSSI_R |=ADC_PSSI_SS3;
        while(ADC0_ACTSS_R&ADC_ACTSS_BUSY);/* wait for conversion complete */
        temp = ADC0_SSFIFO3_R;

        // invert bits at even indexes
        if(i%2==0){
        start |= (temp & 0x01)<<i;
        }else{
        start |= (~temp & 0x01)<<i;
        }
        waitMicrosecond(1000);
    }
    // return the state
    return (uint16_t)(start&0x0000FFFF);
}
