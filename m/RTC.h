#ifndef RTC_H_
#define RTC_H_
#include <stdint.h>
void initRTC();
void resetRTC();
void startMatch();
void HibIsr();
void HibSleep();
void endMatch();
void storeData();
void retrieveData();
void stopRTC();
void startRTC();
extern uint32_t currOffset;
extern uint32_t NSamples;
extern uint32_t Para,LTflag,Hflag,Trigflag;
extern uint32_t level,H;
extern uint8_t sleepflag;
extern struct time storedTime;
extern struct date storedDate;
extern uint32_t T;
extern char* tokens[10];
extern uint32_t logMask;
#define HWREG(x)(*((volatile uint32_t *)(x)))
extern uint32_t state,startingState;
#endif
