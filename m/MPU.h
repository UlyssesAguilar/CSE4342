#ifndef MPU_H_
#define MPU_H_


#include <stdint.h>
#define MPU9250 0x68
#define AK8963 0x0C
#define PWR_MGMT_1 0x6B
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define ACCEL_CONFIG2 0x1D
#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define CONFIG 0x1A
#define ACCEL_CONFIG2 0x1D
#define ACCEL_ZOUT_L 0x40
#define GYRO_XOUT_H 0x43
#define GYRO_XOUT_L 0x44
#define GYRO_YOUT_H 0x45
#define GYRO_YOUT_L 0x46
#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48
#define AK8963_ST1 0x02  // data ready status bit 0
#define AK8963_XOUT_L 0x03  // data
#define AK8963_XOUT_H 0x04
#define SMPLRT_DIV 0x19
#define MOT_DETECT_CTRL    0x69

#define AK8963_YOUT_L 0x05
#define AK8963_YOUT_H 0x06
#define AK8963_ZOUT_L 0x07
#define AK8963_ZOUT_H 0x08
#define AK8963_ST2 0x09  // Data overflow bit 3 and data read error status bit 2


void initMPU9250();
void readAccelData(int16_t * destination);
void readGyroData(int16_t * destination);
void readMagData(int16_t * destination);
void startTrigger();
void stopTrigger();


extern uint32_t logMask;
#endif
