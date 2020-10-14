#ifndef UART0_H_
#define UART0_H_
void initUART0();
void putcUart0(char c);
void putsUart0(char* str);
char getcUart0();
char * strtokk(char * str, char *comp);
void getsUart0(char * str, uint8_t maxChars);
#endif
