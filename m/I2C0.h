#include <stdint.h>
#include <stdbool.h>

#ifndef I2C0_H_
#define I2C0_H_
// PortB masks
#define SDA_MASK 8
#define SCL_MASK 4
void initI2c0();
void writeI2c0Register(uint8_t add, uint8_t reg, uint8_t data);
void writeI2c0Registers(uint8_t add, uint8_t reg, uint8_t data[], uint8_t size);
uint8_t readI2c0Register(uint8_t add, uint8_t reg);
bool pollI2c0Address(uint8_t add);
bool isI2c0Error();


#endif
