#ifndef Simple_MPU6050_h
#define Simple_MPU6050_h

#define interruptPin 2
#include <Wire.h>
#include <I2Cdev.h>
#include "DMP_Image.h"
#include "MPU_WriteMacros.h"
#include "MPU_ReadMacros.h"

#define ENABLE_MPU_OVERFLOW_PROTECTION(...) void yield(void){mpu.OverflowProtection();} // yield is called from within the delay() function 



class Simple_MPU6050 : public I2Cdev {
    static void nothing(void) {};
    static void nothing(int16_t *, int16_t *, int32_t *, uint32_t *) {};
    typedef void (*_ON_FIFO_CB_PTR)(int16_t *gyro, int16_t *accel, int32_t *quat, uint32_t *timestamp); // Create a type to point to a function.
    _ON_FIFO_CB_PTR on_FIFO_cb = nothing;
  public:
    typedef struct SensorList_s {
      int16_t ax ;
      int16_t ay ;
      int16_t az ;
      int16_t gx ;
      int16_t gy ;
      int16_t gz ;
    };

	float mx, my, mz; // variables to hold latest magnetometer data values

    typedef union AccelGyro_u {
      SensorList_s V;
      int16_t intData[sizeof(SensorList_s) / 2];
    };

	uint8_t HIGH_SENS  = 1; // 0 = 14-BIT, 1 = 16-BIT 
    AccelGyro_u S;
    uint8_t buffer[14];
	uint16_t IntBuffer[7];
    uint8_t devAddr;
	uint8_t akm_addr = 0;
	uint8_t akm_WhoAmI;
	uint8_t WhoAmI;
    uint8_t dmp_on;/* 1 if DMP is enabled. */
    uint8_t data[16];
	uint8_t TVal; // TVal For any read
    uint8_t packet_length;
    uint16_t dmp_features;
    uint16_t sensor_timestamp;
    int16_t  gyro[3], accel[3];
	float mag[3];
    int32_t quat[4];
    float mRes;
	float mag_bias[3] = {0, 0, 0}; // max+min /2
	float mag_scale[3] = {0, 0, 0};// max-min /2
	float dest1[3]; // hard iron correction mag biases in G  magBias * mRes * mag_sens_adj
	float dest2[3]; // soft iron correction estimate  = ( "average" magBias(x+y+z) /3) / mag_bias


    int8_t I2CReadCount; //items Read 
    bool I2CWriteStatus; //  True False
	int16_t sax_,say_,saz_,sgx_,sgy_,sgz_;

    //Startup Functins MPU
    Simple_MPU6050(); // Constructor
    Simple_MPU6050 & SetAddress(uint8_t address);
    uint8_t CheckAddress();
    uint8_t TestConnection(int Stop = 1);
    void OverflowProtection(void);
	Simple_MPU6050 & CalibrateMPU(uint8_t Loops = 30);
    Simple_MPU6050 & load_DMP_Image(int16_t ax_, int16_t ay_, int16_t az_, int16_t gx_, int16_t gy_, int16_t gz_);
    Simple_MPU6050 & load_DMP_Image(uint8_t CalibrateMode = 0);
	Simple_MPU6050 & resetOffset();
    Simple_MPU6050 & setOffset(int16_t ax_, int16_t ay_, int16_t az_, int16_t gx_, int16_t gy_, int16_t gz_);
    Simple_MPU6050 & on_FIFO(void (*CB)(int16_t *, int16_t *, int32_t *, uint32_t *));
    Simple_MPU6050 & reset_fifo();
    Simple_MPU6050 & resetFIFO(){reset_fifo();return *this;};
    Simple_MPU6050 & resetDMP(){USER_CTRL_WRITE_DMP_RST();return *this;};
    Simple_MPU6050 & full_reset_fifo(void); //Clears fifo and sensor paths.
    Simple_MPU6050 & DMP_InterruptEnable(uint8_t Data);

    //Startup Functins AKM
	Simple_MPU6050 & AKM_Init();
	Simple_MPU6050 & mpu_set_bypass(unsigned char bypass_on);

    // usage functions
    uint8_t CheckForInterrupt(void);
    int16_t getFIFOCount();
	int8_t GetCurrentFIFOPacket(uint8_t *data, uint8_t length);
    Simple_MPU6050 & dmp_read_fifo(uint8_t CheckInterrupt = 1); // 0 = No interrupt needed to try to get data
    uint8_t dmp_read_fifo(int16_t *gyro, int16_t *accel, int32_t *quat, uint32_t *timestamp);// Basic Send and receive
    Simple_MPU6050 & MPUi2cRead(uint8_t regAddr,  uint8_t length, uint8_t bitNum, uint8_t *data);
    Simple_MPU6050 & MPUi2cRead(uint8_t AltAddress,uint8_t regAddr,  uint8_t length, uint8_t bitNum, uint8_t *data);
    Simple_MPU6050 & MPUi2cReadByte(uint8_t regAddr,  uint8_t *Data);
    Simple_MPU6050 & MPUi2cReadByte(uint8_t AltAddress,uint8_t regAddr, uint8_t *Data);
    Simple_MPU6050 & MPUi2cReadBytes(uint8_t regAddr, uint8_t length, uint8_t *Data);
    Simple_MPU6050 & MPUi2cReadBytes(uint8_t AltAddress,uint8_t regAddr, uint8_t length, uint8_t *Data);
    Simple_MPU6050 & MPUi2cReadInt(uint8_t regAddr, uint16_t *data);
    Simple_MPU6050 & MPUi2cReadInt(uint8_t AltAddress,uint8_t regAddr, uint16_t *data);
	Simple_MPU6050 & MPUi2cReadInts(uint8_t AltAddress,uint8_t regAddr, uint16_t size, uint16_t *Data);
    Simple_MPU6050 & MPUi2cReadInts(uint8_t regAddr, uint16_t size, uint16_t *Data);
	// Wrappered I2Cdev write functions
    Simple_MPU6050 & MPUi2cWrite(uint8_t regAddr, uint8_t length, uint8_t bitNum, uint8_t Val);
    Simple_MPU6050 & MPUi2cWrite(uint8_t AltAddress,uint8_t regAddr, uint8_t length, uint8_t bitNum, uint8_t Val);
    Simple_MPU6050 & MPUi2cWriteByte(uint8_t regAddr,  uint8_t Val);
    Simple_MPU6050 & MPUi2cWriteByte(uint8_t AltAddress,uint8_t regAddr,  uint8_t Val);
    Simple_MPU6050 & MPUi2cWriteBytes(uint8_t regAddr, uint8_t length, uint8_t *Data);
    Simple_MPU6050 & MPUi2cWriteBytes(uint8_t AltAddress,uint8_t regAddr, uint8_t length, uint8_t *Data);
    Simple_MPU6050 & MPUi2cWriteInt(uint8_t regAddr,  uint16_t Val);
    Simple_MPU6050 & MPUi2cWriteInt(uint8_t AltAddress,uint8_t regAddr,  uint16_t Val);
    Simple_MPU6050 & MPUi2cWriteInts(uint8_t regAddr, uint16_t size,  uint16_t *data);
    Simple_MPU6050 & MPUi2cWriteInts(uint8_t AltAddress,uint8_t regAddr, uint16_t size,  uint16_t *data);
	int8_t ReadStatus(){return I2CReadCount>0;};
	bool WriteStatus(){return I2CWriteStatus;};


    // helper math functions

    Simple_MPU6050 & GetQuaternion(Quaternion *q, const int32_t* qI);
    Simple_MPU6050 & GetGravity(VectorFloat *v, Quaternion *q);
    Simple_MPU6050 & MagneticNorth(float*data, VectorInt16 *v, Quaternion*q );

    // firmware management functions
    Simple_MPU6050 & load_firmware(uint16_t  length, const uint8_t *firmware);
    Simple_MPU6050 & read_mem(uint16_t mem_addr, uint16_t length, uint8_t *data);
    Simple_MPU6050 & write_mem(uint16_t mem_addr, uint16_t length, uint8_t *data);

    // Default data gathering functions for program revisions
    void view_Vital_MPU_Registers();
    bool view_DMP_firmware_Instance(uint16_t  length);
	Simple_MPU6050 & PrintActiveOffsets(); // See the results of the Calibration
	
	
	// Calibration Routines
	Simple_MPU6050 & CalibrateGyro(uint8_t Loops = 30); // Fine tune after setting offsets with less Loops.
	Simple_MPU6050 & CalibrateAccel(uint8_t Loops = 30); // Fine tune after setting offsets with less Loops.
	Simple_MPU6050 & PID(uint8_t ReadAddress, float kP, float kI, uint8_t Loops);  // Does the math

	//Compass functions:
	Simple_MPU6050 & I2CScanner();
	uint8_t FindAddress(uint8_t Address,uint8_t Limit);

};



#endif
