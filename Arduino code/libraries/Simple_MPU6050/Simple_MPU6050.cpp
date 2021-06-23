#include <Wire.h>
#include <I2Cdev.h>
#include "DMP_Image.h"
#include "Simple_MPU6050.h"
#include "MPU_ReadMacros.h"
#include "MPU_WriteMacros.h"

#define MPU6050_DEFAULT_ADDRESS     0x68

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has changed
volatile uint8_t _maxPackets;

/**
@brief      Initialization functions
*/
Simple_MPU6050::Simple_MPU6050() {

	SetAddress(MPU6050_DEFAULT_ADDRESS);
	packet_length = 28;
	/*
	packet_length += 6;//DMP_FEATURE_SEND_RAW_ACCEL
	packet_length += 6;//DMP_FEATURE_SEND_RAW_GYRO
	packet_length += 16;//DMP_FEATURE_6X_LP_QUAT
	*/
	_maxPackets = floor(512 / packet_length); // MPU 9250 can only handle 512 bytes of data in the FIFO
}

Simple_MPU6050 &  Simple_MPU6050::SetAddress(uint8_t address) {
	devAddr = address;
	return *this;
}

uint8_t  Simple_MPU6050::CheckAddress() {
	return devAddr;
}

/**
@brief      Set FIFO Callback
*/
Simple_MPU6050 & Simple_MPU6050::on_FIFO(void (*CB)(int16_t *, int16_t *, int32_t *, uint32_t *)) {
	on_FIFO_cb = CB;
	return *this; // return Pointer to this class
}

/**
@brief      Reset funnctions
*/
Simple_MPU6050 & Simple_MPU6050::reset_fifo() {
	USER_CTRL_WRITE_FIFO_RST(); //   Reset FIFO module. Reset is asynchronous. This bit auto clears after one clock cycle.
	return *this;
}
Simple_MPU6050 & Simple_MPU6050::full_reset_fifo(void) { // Official way to reset fifo
	USER_CTRL_WRITE_RESET_FIFO_DMP();
	return *this;
}


/**
@brief      Start and Stop DMP int pin triggering  //   1  Enable, 0 = Disable
*/
Simple_MPU6050 & Simple_MPU6050::DMP_InterruptEnable(uint8_t Data) {
	INT_ENABLE_WRITE_RAW_DMP_INT_EN(Data);
	dmp_on = Data;
	return *this;
};


//***************************************************************************************
//**********************      Overflow Protection functions        **********************
//***************************************************************************************


/**
@brief      manages properly testing interrupt trigger from interrupt pin
*/
uint8_t Simple_MPU6050::CheckForInterrupt(void) {
	uint8_t InteruptTriggered;
	noInterrupts ();
	InteruptTriggered = mpuInterrupt;
	mpuInterrupt = false;
	interrupts ();
	return InteruptTriggered;
}

/**
@brief      During the Dreded delay() using yield() limit the fifo packets to 10 less than max packets
*/
void Simple_MPU6050::OverflowProtection(void) {
	uint32_t Timestamp ;
	Timestamp = millis();
	static uint32_t spamtimer;
	if ((Timestamp - spamtimer) >= 30) {
		spamtimer = Timestamp;
		uint16_t fifo_count;
		int8_t Packets;
		FIFO_COUNTH_READ_FIFO_CNT(&fifo_count);
		Packets = (fifo_count / packet_length) - (_maxPackets - 10); // Leaves room for 2 more readings
		if (Packets <= 0)return;
		uint8_t trash[packet_length + 1] ;
		while (0 < Packets--) {
			FIFO_READ(packet_length, trash);
		}
	}
}


//***************************************************************************************
//**********************              FIFO functions               **********************
//***************************************************************************************
/**
@brief      Reads Newest packet from fifo then on success triggers Callback routine
*/
Simple_MPU6050 & Simple_MPU6050::dmp_read_fifo(uint8_t CheckInterrupt = 1) {
	if (CheckInterrupt && !CheckForInterrupt()) return *this;
	if (!dmp_read_fifo(gyro, accel, quat, sensor_timestamp)) {
		return *this;
	}
	if (on_FIFO_cb) on_FIFO_cb(gyro, accel, quat, sensor_timestamp);
	return *this;
}
 

 int16_t Simple_MPU6050::getFIFOCount(){
	int16_t fifo_count;
	FIFO_COUNTH_READ_FIFO_CNT(&fifo_count);
	return (fifo_count);
 }


/** Get latest byte from FIFO buffer no matter how much time has passed.
 * ===                  GetCurrentFIFOPacket                    ===
 * ================================================================
 * Returns 1) when nothing special was done
 *         0) when no valid data is available
 * ================================================================ */
 int8_t Simple_MPU6050::GetCurrentFIFOPacket(uint8_t *data, uint8_t length) { // overflow proof
     int16_t fifoC;
     // This section of code is for when we allowed more than 1 packet to be acquired
     uint32_t BreakTimer = micros();
     do {
         if ((fifoC = getFIFOCount())  > length) {

             if (fifoC > 200) { // if you waited to get the FIFO buffer to > 200 bytes it will take longer to get the last packet in the FIFO Buffer than it will take to  reset the buffer and wait for the next to arrive
                 USER_CTRL_WRITE_FIFO_RST(); // Fixes any overflow corruption
                 fifoC = 0;
                 while (!(fifoC = getFIFOCount()) && ((micros() - BreakTimer) <= (11000))); // Get Next New Packet
             } else { //We have more than 1 packet but less than 200 bytes of data in the FIFO Buffer
                 uint8_t Trash[BUFFER_LENGTH];
                 while ((fifoC = getFIFOCount()) > length) {  // Test each time just in case the MPU is writing to the FIFO Buffer
                     fifoC = fifoC - length; // Save the last packet
                     uint16_t  RemoveBytes;
                     while (fifoC) { // fifo count will reach zero so this is safe
                         RemoveBytes = min((int)fifoC, BUFFER_LENGTH); // Buffer Length is different than the packet length this will efficiently clear the buffer
//                        getFIFOBytes(Trash, (uint8_t)RemoveBytes);
						 FIFO_READ((uint8_t)RemoveBytes, Trash);
                         fifoC -= RemoveBytes;
                     }
                 }
             }
         }
         if (!fifoC) return 0; // Called too early no data or we timed out after FIFO Reset
         // We have 1 packet
         if ((micros() - BreakTimer) > (11000)) return 0;
     } while (fifoC != length);
	 FIFO_READ((uint8_t)length, data);  //Get 1 packet
//     getFIFOBytes(data, length); //Get 1 packet
     return 1;
}

/**
@brief      Get the Newest packet from the FIFO. FIFO Buffer will be empty awaiting for next packet
*/
uint8_t Simple_MPU6050::dmp_read_fifo(int16_t *gyro, int16_t *accel, int32_t *quat, uint32_t *timestamp) {
	/* Get a packet. */
	uint8_t fifo_data[MAX_PACKET_LENGTH];
	if(GetCurrentFIFOPacket(fifo_data, packet_length)){

	//	FIFO_READ(packet_length, fifo_data);
		timestamp = micros();
	//	fifo_count -= packet_length;
		/* Parse DMP packet. */
		uint8_t ii = 0;
		quat[0] = ((int32_t)fifo_data[0] << 24) | ((int32_t)fifo_data[1] << 16) | ((int32_t)fifo_data[2] << 8) | fifo_data[3];
		quat[1] = ((int32_t)fifo_data[4] << 24) | ((int32_t)fifo_data[5] << 16) | ((int32_t)fifo_data[6] << 8) | fifo_data[7];
		quat[2] = ((int32_t)fifo_data[8] << 24) | ((int32_t)fifo_data[9] << 16) | ((int32_t)fifo_data[10] << 8) | fifo_data[11];
		quat[3] = ((int32_t)fifo_data[12] << 24) | ((int32_t)fifo_data[13] << 16) | ((int32_t)fifo_data[14] << 8) | fifo_data[15];
		ii += 16;
		accel[0] = ((int16_t)fifo_data[ii + 0] << 8) | fifo_data[ii + 1];
		accel[1] = ((int16_t)fifo_data[ii + 2] << 8) | fifo_data[ii + 3];
		accel[2] = ((int16_t)fifo_data[ii + 4] << 8) | fifo_data[ii + 5];
		ii += 6;
		gyro[0] = ((int16_t)fifo_data[ii + 0] << 8) | fifo_data[ii + 1];
		gyro[1] = ((int16_t)fifo_data[ii + 2] << 8) | fifo_data[ii + 3];
		gyro[2] = ((int16_t)fifo_data[ii + 4] << 8) | fifo_data[ii + 5];
		return 1;
	}
	return 0;
}

//***************************************************************************************
//**********************         i2cdev wrapper functions          **********************
//***************************************************************************************

// I did this to simplify managing all the macros found in MPU_ReadMacros.h and MPU_WriteMacros.h


// Wrappered I2Cdev read functions
Simple_MPU6050 & Simple_MPU6050::MPUi2cRead(uint8_t regAddr, uint8_t length, uint8_t bitNum, uint8_t *Data) {
	return MPUi2cRead( devAddr,  regAddr,  length,  bitNum,  Data);
}
Simple_MPU6050 & Simple_MPU6050::MPUi2cRead(uint8_t AltAddress, uint8_t regAddr, uint8_t length, uint8_t bitNum, uint8_t *Data) {
	if(length == 1) I2CReadCount = readBit(AltAddress,  regAddr, bitNum, Data);
	else I2CReadCount = readBits(AltAddress,  regAddr, bitNum, length,  Data);
	return *this;
}
// MPUi2cReadBytes


Simple_MPU6050 & Simple_MPU6050::MPUi2cReadByte(uint8_t regAddr,  uint8_t *Data) {
	I2CReadCount = readBytes(devAddr, regAddr,  1, Data);
	return *this;
}
Simple_MPU6050 & Simple_MPU6050::MPUi2cReadByte(uint8_t AltAddress,uint8_t regAddr,  uint8_t *Data) {
	I2CReadCount = readBytes(AltAddress, regAddr,  1, Data);
	return *this;
}

Simple_MPU6050 & Simple_MPU6050::MPUi2cReadBytes(uint8_t regAddr, uint8_t length, uint8_t *Data) {
	I2CReadCount = readBytes(devAddr, regAddr,  length, Data);
	return *this;
}
Simple_MPU6050 & Simple_MPU6050::MPUi2cReadBytes(uint8_t AltAddress,uint8_t regAddr, uint8_t length, uint8_t *Data) {
	I2CReadCount = readBytes(AltAddress, regAddr,  length, Data);
	return *this;
}

// MPUi2cReadInt or Word
Simple_MPU6050 & Simple_MPU6050::MPUi2cReadInt(uint8_t regAddr, uint16_t *Data) {
	I2CReadCount = readWords(devAddr, regAddr, 1, Data); // reads 1 or more 16 bit integers (Word)
	return *this;
}
Simple_MPU6050 & Simple_MPU6050::MPUi2cReadInt(uint8_t AltAddress,uint8_t regAddr, uint16_t *Data) {
	I2CReadCount = readWords(AltAddress, regAddr, 1, Data); // reads 1 or more 16 bit integers (Word)
	return *this;
}

// MPUi2cReadInts or Words
Simple_MPU6050 & Simple_MPU6050::MPUi2cReadInts(uint8_t regAddr, uint16_t size, uint16_t *Data) {
	I2CReadCount = readWords(devAddr, regAddr, size, Data); // reads 1 or more 16 bit integers (Word)
	return *this;
}
Simple_MPU6050 & Simple_MPU6050::MPUi2cReadInts(uint8_t AltAddress,uint8_t regAddr, uint16_t size, uint16_t *Data) {
	I2CReadCount = readWords(AltAddress, regAddr, size, Data); // reads 1 or more 16 bit integers (Word)
	return *this;
}



// Wrappered I2Cdev write functions
// MPUi2cWrite
Simple_MPU6050 & Simple_MPU6050::MPUi2cWrite(uint8_t regAddr, uint8_t length, uint8_t bitNum, uint8_t Val) {
	return MPUi2cWrite(devAddr, regAddr,  length,  bitNum,  Val);
}
Simple_MPU6050 & Simple_MPU6050::MPUi2cWrite(uint8_t AltAddress,uint8_t regAddr, uint8_t length, uint8_t bitNum, uint8_t Val) {
	if (length == 1) {
		I2CWriteStatus = writeBit(AltAddress, regAddr, bitNum, &Val); // Alters 1 bit by reading the byte making a change and storing the byte (faster than writeBits)
	}
	else if (bitNum != 255) {
		I2CWriteStatus = writeBits(AltAddress, regAddr, bitNum, length, &Val); // Alters several bits by reading the byte making a change and storing the byte
	}
	return *this;
}

// MPUi2cWriteByte
Simple_MPU6050 & Simple_MPU6050::MPUi2cWriteByte(uint8_t regAddr,  uint8_t Val) {
	I2CWriteStatus = writeBytes(devAddr, regAddr,  1, &Val); //Writes 1 or more 8 bit Bytes
	return *this;
}
Simple_MPU6050 & Simple_MPU6050::MPUi2cWriteByte(uint8_t AltAddress,uint8_t regAddr,  uint8_t Val) {
	I2CWriteStatus = writeBytes(AltAddress, regAddr,  1, &Val); //Writes 1 or more 8 bit Bytes
	return *this;
}

// MPUi2cWriteBytes
Simple_MPU6050 & Simple_MPU6050::MPUi2cWriteBytes(uint8_t regAddr, uint8_t length, uint8_t *Data) {
	I2CWriteStatus = writeBytes(devAddr, regAddr,  length, Data); //Writes 1 or more 8 bit Bytes
	return *this;
}
Simple_MPU6050 & Simple_MPU6050::MPUi2cWriteBytes(uint8_t AltAddress,uint8_t regAddr, uint8_t length, uint8_t *Data) {
	I2CWriteStatus = writeBytes(AltAddress, regAddr,  length, Data); //Writes 1 or more 8 bit Bytes
	return *this;
}

// MPUi2cWriteInt
Simple_MPU6050 & Simple_MPU6050::MPUi2cWriteInt(uint8_t regAddr,  uint16_t Val) {
	I2CWriteStatus = writeWords(devAddr, regAddr, 1,  &Val);// Writes 1 or more 16 bit integers (Word)
	return *this;
}
Simple_MPU6050 & Simple_MPU6050::MPUi2cWriteInt(uint8_t AltAddress,uint8_t regAddr,  uint16_t Val) {
	I2CWriteStatus = writeWords(AltAddress, regAddr, 1,  &Val);// Writes 1 or more 16 bit integers (Word)
	return *this;
}

// MPUi2cWriteInts
Simple_MPU6050 & Simple_MPU6050::MPUi2cWriteInts(uint8_t regAddr, uint16_t size, uint16_t *Data) {
	I2CWriteStatus = writeWords(devAddr, regAddr, size / 2,  Data);
	return *this;
}
Simple_MPU6050 & Simple_MPU6050::MPUi2cWriteInts(uint8_t AltAddress,uint8_t regAddr, uint16_t size, uint16_t *Data) {
	I2CWriteStatus = writeWords(AltAddress, regAddr, size / 2,  Data);
	return *this;
}




//***************************************************************************************
//**********************      Firmwaer Read Write Functions        **********************
//***************************************************************************************

/**
@brief      Read and Write to the DMP FIRMWARE memory. using these functions alters the firmware instance
*/

Simple_MPU6050 & Simple_MPU6050::read_mem(uint16_t mem_addr, uint16_t length, uint8_t *Data) {

	BANK_SEL_WRITE(mem_addr);
	DMP_MEM_READ(length, Data);
	return *this;
	} Simple_MPU6050 & Simple_MPU6050::write_mem(uint16_t  mem_addr, uint16_t  length, uint8_t *Data) {
	BANK_SEL_WRITE(mem_addr);
	DMP_MEM_WRITE(length, Data);
	return *this;
}


//***************************************************************************************
//**********************              Setup Functions              **********************
//***************************************************************************************
/**
@brief      ***EVERYTHING!*** needed to get DMP up and running!
*/


//#define PWR_MGMT_1_WRITE_DEVICE_RESET(...) MPUi2cWrite(0x6B, 1, 7, 1);delay(100);MPUi2cWrite(0x6A, 4, 3, 0b1111);delay(100);  //   1  Reset the internal registers and restores the default settings. Write a 1 to set the reset, the bit will auto clear.
Simple_MPU6050 & Simple_MPU6050::load_DMP_Image(uint8_t CalibrateMode) {
	uint8_t val;
	TestConnection(1);

	PWR_MGMT_1_WRITE_DEVICE_RESET();			//PWR_MGMT_1:(0x6B Bit7 true) reset with 100ms delay and full SIGNAL_PATH_RESET:(0x6A Bits 3,2,1,0 True) with another 100ms delay
	MPUi2cWriteByte(0x6B, 0x00);
	MPUi2cWriteByte(0x6C, 0x00);
	MPUi2cWriteByte(0x1A, 0x03);
	MPUi2cWriteByte(0x1B, 0x18);
	MPUi2cWriteByte(0x1C, 0x00);
	MPUi2cWriteByte(0x23, 0x00);
	MPUi2cWriteByte(0x38, 0x00);
	MPUi2cWriteByte(0x6A, 0x04);
	MPUi2cWriteByte(0x19, 0x04);
	if(!CalibrateMode){
		load_firmware(DMP_CODE_SIZE, dmp_memory);	// Loads the DMP image into the MPU6050 Memory
		MPUi2cWriteInt(0x70,  0x0400);				// DMP Program Start Address
	}
	resetOffset();	// Load Calibration offset values into MPU
	if(CalibrateMode)return;
	PrintActiveOffsets();
	AKM_Init();
	MPUi2cWriteByte(0x6A, 0xC0);				// 1100 1100 USER_CTRL: Enable FIFO and Reset FIFO
	MPUi2cWriteByte(0x38, 0x02);				// 0000 0010 INT_ENABLE: RAW_DMP_INT_EN on

	dmp_on = 1;
#ifdef interruptPin
	attachInterrupt(digitalPinToInterrupt(interruptPin), [] {mpuInterrupt = true;}, RISING); //NOTE: "[]{mpuInterrupt = true;}" Is a complete funciton without a name. It is handed to the callback of attachInterrupts Google: "Lambda anonymous functions"
#endif
	//These are the features the above code initialized for you by default (ToDo Allow removal of one or more Features)
	dmp_features = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_RAW_GYRO |  DMP_FEATURE_SEND_CAL_GYRO; // These are Fixed into the DMP_Image and Can't be change easily at this time.
	return *this;
}

Simple_MPU6050 & Simple_MPU6050::CalibrateMPU(uint8_t Loops) {
	load_DMP_Image(true);
	CalibrateAccel(Loops);
	CalibrateGyro(Loops);
	if(!WhoAmI) WHO_AM_I_READ_WHOAMI(&WhoAmI);
	if(WhoAmI < 0x38){
		XA_OFFSET_H_READ_XA_OFFS(&sax_);
		YA_OFFSET_H_READ_YA_OFFS(&say_);
		ZA_OFFSET_H_READ_ZA_OFFS(&saz_);
		}else {
		XA_OFFSET_H_READ_0x77_XA_OFFS(&sax_);
		YA_OFFSET_H_READ_0x77_YA_OFFS(&say_);
		ZA_OFFSET_H_READ_0x77_ZA_OFFS(&saz_);
	}
	XG_OFFSET_H_READ_X_OFFS_USR(&sgx_);
	YG_OFFSET_H_READ_Y_OFFS_USR(&sgy_);
	ZG_OFFSET_H_READ_Z_OFFS_USR(&sgz_);
	return *this;
}

/**
@brief      Loads the DMP firmware.
*/
Simple_MPU6050 & Simple_MPU6050::load_firmware(uint16_t  length, const uint8_t *firmware) {
	static bool loaded = false;
	uint16_t  ii;
	uint16_t  this_write;

	uint16_t bankNum = 0;
	/* Must divide evenly into st.hw->bank_size to avoid bank crossings. */
	#define LOAD_CHUNK  (16)
	uint8_t cur[LOAD_CHUNK], tmp[2];
	uint8_t firmware_chunk[LOAD_CHUNK];
	if (loaded)return *this; /* DMP should only be loaded once. */
	if (!firmware) *this;

	for (ii = 0; ii < length; ii += this_write) {
		this_write = min(LOAD_CHUNK, length - ii);
		int16_t x;
		uint8_t *pFirmware = (uint8_t *)&firmware[ii];
		for ( x = 0; x < this_write; x++ ) firmware_chunk[x] = pgm_read_byte_near(pFirmware + x);
		write_mem(ii, this_write, firmware_chunk);
		#ifdef DEBUG
		read_mem(ii, this_write, cur);
		if ((ii % (16 * 16)) == 0) {
			bankNum++;
		}
		#endif
	}
	loaded = true;
	return *this;
}

/**
@brief      Test to be sure we have communication to the MPU
returns 1 on success
stops or returns 0 on fail
*/
uint8_t Simple_MPU6050::TestConnection(int Stop = 1) {
	byte x;
	Wire.beginTransmission(CheckAddress());
	if(Wire.endTransmission() != 0){
		if(Stop == 2){
			return 2;
		}
	}
	WHO_AM_I_READ_WHOAMI(&WhoAmI);
	uint16_t Device = (WhoAmI < 0x38 )? 6050:6500;
	switch(Device){
		case 6500:
		if(Stop<0){
		}
		break;
		case 6050:
		if(Stop<0){
		}
		break;
		default:
		if (Stop > 0) {
			while (1) {}
		}
		return 1;
	}

	return 0;
}


//***************************************************************************************
//********************** Gather Configuration from working MPU6050 **********************
//***************************************************************************************

// usage after configuration of the MPU6050 to your liking Get these registers to simplify MPU6050 startup
void Simple_MPU6050::view_Vital_MPU_Registers() {
	uint8_t val;
	// Reset code for your convenience:
	readBytes(0x68, 0x1C, 1, &val);  // ACCEL_CONFIG:
	readBytes(0x68, 0x37, 1, &val);  // INT_PIN_CFG:
	readBytes(0x68, 0x6B, 1, &val); // PWR_MGMT_1:
	readBytes(0x68, 0x19, 1, &val);  // SMPLRT_DIV:
	readBytes(0x68, 0x1A, 1, &val);   // CONFIG:
	readBytes(0x68, 0x19, 1, &val); 	 // GYRO_CONFIG:
}

void view_MPU_Startup_Registers() {
	uint8_t val;
	// Reset code for your convenience:

}

#define A_OFFSET_H_READ_A_OFFS(Data)    MPUi2cReadInts(0x06, 3, Data)  //   X accelerometer offset cancellation
#define XG_OFFSET_H_READ_OFFS_USR(Data) MPUi2cReadInts(0x13, 3, Data)  //   Remove DC bias from the gyro sensor Step 0.0305 dps



Simple_MPU6050 & Simple_MPU6050::PrintActiveOffsets( ) {
	int16_t Data[3];
	if(!WhoAmI) WHO_AM_I_READ_WHOAMI(&WhoAmI);
	if(WhoAmI < 0x38)	A_OFFSET_H_READ_A_OFFS(Data);
	else {
		XA_OFFSET_H_READ_0x77_XA_OFFS(Data);
		YA_OFFSET_H_READ_0x77_YA_OFFS(Data+1);
		ZA_OFFSET_H_READ_0x77_ZA_OFFS(Data+2);
	}

	XG_OFFSET_H_READ_OFFS_USR(Data);

	return *this;
}

// I used the following function to retrieve a working configured DMP firmware instance for use in this program
// copy and modify this function to work elseware
/**
@brief      View the DMP firmware.
*/
bool Simple_MPU6050::view_DMP_firmware_Instance(uint16_t  length) {
	uint16_t  ii;
	uint16_t  this_read;
	uint16_t bankNum = 0;
	#define LOAD_CHUNK  (16)
	uint8_t cur[LOAD_CHUNK];
		for (ii = 0; ii < length; ii += this_read) {
			this_read = min(LOAD_CHUNK, length - ii);
			writeWords(devAddr, 0x6D, 1,  ii);
			readBytes(devAddr, 0x6F,  this_read, cur);
			if ((ii % (16 * 16)) == 0) {
				bankNum++;
			}
		}
	return true;
}

//***************************************************************************************
//**********************           Calibration Routines            **********************
//***************************************************************************************
/**
@brief      Fully calibrate Gyro from ZERO in about 6-7 Loops 600-700 readings
*/
Simple_MPU6050 & Simple_MPU6050::CalibrateGyro(uint8_t Loops ) {
	double kP = 0.3;
	double kI = 90;
	float x;
	x = (100 - map(Loops, 1, 5, 20, 0)) * .01;
	kP *= x;
	kI *= x;
	PID( 0x43,  kP, kI,  Loops);

	return *this;
}

/**
@brief      Fully calibrate Accel from ZERO in about 6-7 Loops 600-700 readings
*/

Simple_MPU6050 & Simple_MPU6050::CalibrateAccel(uint8_t Loops ) {
	float kP = 0.3;
	float kI = 90;
	float x;
	x = (100 - map(Loops, 1, 5, 20, 0)) * .01;
	kP *= x;
	kI *= x;
	PID( 0x3B, kP, kI,  Loops);
	return *this;
}

Simple_MPU6050 & Simple_MPU6050::PID(uint8_t ReadAddress, float kP,float kI, uint8_t Loops) {
	uint8_t SaveAddress = (ReadAddress == 0x3B)?((WhoAmI < 0x38 )? 0x06:0x77):0x13;

	int16_t  Data;
	float Reading;
	int16_t BitZero[3];
	uint8_t shift =(SaveAddress == 0x77)?3:2;
	float Error, PTerm, ITerm[3];
	int16_t eSample;
	uint32_t eSum ;
	for (int i = 0; i < 3; i++) {
		I2Cdev::readWords(devAddr, SaveAddress + (i * shift), 1, (uint16_t *)&Data); // reads 1 or more 16 bit integers (Word)
		Reading = Data;
		if(SaveAddress != 0x13){
			BitZero[i] = Data & 1;										 // Capture Bit Zero to properly handle Accelerometer calibration
			ITerm[i] = ((float)Reading) * 8;
			} else {
			ITerm[i] = Reading * 4;
		}
	}
	for (int L = 0; L < Loops; L++) {
		eSample = 0;
		for (int c = 0; c < 100; c++) {// 100 PI Calculations
			eSum = 0;
			for (int i = 0; i < 3; i++) {
				I2Cdev::readWords(devAddr, ReadAddress + (i * 2), 1, (uint16_t *)&Data); // reads 1 or more 16 bit integers (Word)
				Reading = Data;
				if ((ReadAddress == 0x3B)&&(i == 2)) Reading -= 16384;	//remove Gravity
				Error = -Reading;
				eSum += abs(Reading);
				PTerm = kP * Error;
				ITerm[i] += (Error * 0.001) * kI;				// Integral term 1000 Calculations a second = 0.001
				if(SaveAddress != 0x13){
					Data = round((PTerm + ITerm[i] ) / 8);		//Compute PID Output
					Data = ((Data)&0xFFFE) |BitZero[i];			// Insert Bit0 Saved at beginning
				} else Data = round((PTerm + ITerm[i] ) / 4);	//Compute PID Output
				I2Cdev::writeWords(devAddr, SaveAddress + (i * shift), 1, (uint16_t *)&Data);
			}
			if((c == 99) && eSum > 1000){						// Error is still to great to continue
				c = 0;
			}
			if((eSum * ((ReadAddress == 0x3B)?.05: 1)) < 5) eSample++;	// Successfully found offsets prepare to  advance
			if((eSum < 100) && (c > 10) && (eSample >= 10)) break;		// Advance to next Loop
			delay(1);
		}

		kP *= .75;
		kI *= .75;
		for (int i = 0; i < 3; i++){
			if(SaveAddress != 0x13) {
				Data = round((ITerm[i] ) / 8);		//Compute PID Output
				Data = ((Data)&0xFFFE) |BitZero[i];	// Insert Bit0 Saved at beginning
			} else Data = round((ITerm[i]) / 4);
			I2Cdev::writeWords(devAddr, SaveAddress + (i * shift), 1, (uint16_t *)&Data);
		}
	}
	SIGNAL_PATH_FULL_RESET_WRITE_RESET();
	return *this;
}

Simple_MPU6050 & Simple_MPU6050::resetOffset() {
	setOffset( sax_,  say_,  saz_,  sgx_,  sgy_,  sgz_);
	return *this; // return Pointer to this class
}

Simple_MPU6050 & Simple_MPU6050::setOffset(int16_t ax_, int16_t ay_, int16_t az_, int16_t gx_, int16_t gy_, int16_t gz_) {
	sax_ = ax_;
	say_ = ay_;
	saz_ = az_;
	sgx_ = gx_;
	sgy_ = gy_;
	sgz_ = gz_;
	
	if(!WhoAmI) WHO_AM_I_READ_WHOAMI(&WhoAmI);
	if(WhoAmI < 0x38){
		XA_OFFSET_H_WRITE_XA_OFFS(ax_);
		YA_OFFSET_H_WRITE_YA_OFFS(ay_);
		ZA_OFFSET_H_WRITE_ZA_OFFS(az_);
		} else {
		XA_OFFSET_H_WRITE_0x77_XA_OFFS(ax_);
		YA_OFFSET_H_WRITE_0x77_YA_OFFS(ay_);
		ZA_OFFSET_H_WRITE_0x77_ZA_OFFS(az_);
	}

	XG_OFFSET_H_WRITE_X_OFFS_USR(gx_);
	YG_OFFSET_H_WRITE_Y_OFFS_USR(gy_);
	ZG_OFFSET_H_WRITE_Z_OFFS_USR(gz_);
	return *this;
}
//***************************************************************************************
//**********************          Helper Math Functions            **********************
//***************************************************************************************

Simple_MPU6050 &  Simple_MPU6050::GetQuaternion(Quaternion *q, const int32_t* qI) {
	// TODO: accommodate different arrangements of sent data (ONLY default supported now)
	q -> w = (float)(qI[0] >> 16) / 16384.0f;
	q -> x = (float)(qI[1] >> 16) / 16384.0f;
	q -> y = (float)(qI[2] >> 16) / 16384.0f;
	q -> z = (float)(qI[3] >> 16) / 16384.0f;
	return *this;
}

Simple_MPU6050 & Simple_MPU6050::GetGravity(VectorFloat *v, Quaternion *q) {
	v -> x = 2 * (q -> x * q -> z - q -> w * q -> y);
	v -> y = 2 * (q -> w * q -> x + q -> y * q -> z);
	v -> z = q -> w * q -> w - q -> x * q -> x - q -> y * q -> y + q -> z * q -> z;
	return *this;
}

Simple_MPU6050 & Simple_MPU6050::MagneticNorth(float*data, VectorInt16 *v, Quaternion*q ) {
	float ax = v->x, ay = v->y, az = v->z;
	float q1 = q->w, q2 = q->x, q3 = q->y, q4 = q->z;   // short name local variable for readability
	float mx = mag[0], my = mag[1], mz = mag[2];
	float hx, hy, bx, bz,vx,vy,vz,wx,wy,wz,ex,ey,ez;
	float q1q1 = q1 * q1;
	float q1q2 = q1 * q2;
	float q1q3 = q1 * q3;
	float q1q4 = q1 * q4;
	float q2q2 = q2 * q2;
	float q2q3 = q2 * q3;
	float q2q4 = q2 * q4;
	float q3q3 = q3 * q3;
	float q3q4 = q3 * q4;
	float q4q4 = q4 * q4;


	// Reference direction of Earth's magnetic field
	hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
	hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
	bx = sqrt((hx * hx) + (hy * hy));
	bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);

	// Estimated direction of gravity and magnetic field
	vx = 2.0f * (q2q4 - q1q3);
	vy = 2.0f * (q1q2 + q3q4);
	vz = q1q1 - q2q2 - q3q3 + q4q4;
	wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
	wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
	wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);

	// Error is cross product between estimated direction and measured direction of gravity
	ex = (ay * vz - az * vy) + (my * wz - mz * wy);
	ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
	ez = (ax * vy - ay * vx) + (mx * wy - my * wx);

	data[0] = wx;
	data[1] = wy;
	data[2] = wz;
	data[3] = ex;
	data[4] = ey;
	data[5] = ez;
	return *this;
}



// //***************************************************************************************
// //**********************      Helper Magnetometer Functtions       **********************
// //***************************************************************************************
Simple_MPU6050 & Simple_MPU6050::I2CScanner(){
	for (int x = 1;x < 128;x++){
		if(!(x = FindAddress(x,128))) break;
	}
	return *this;
}




uint8_t Simple_MPU6050::FindAddress(uint8_t Address,uint8_t Limit){
	do {
		Wire.beginTransmission(Address);
		if (Wire.endTransmission() == 0)
		return Address;
	} while (Limit != Address++);// using rollover ate 255 to allow for any number on Limit
	return 0;
}

Simple_MPU6050 & Simple_MPU6050::AKM_Init(){
	
	INT_PIN_CFG_WRITE_BYPASS_EN(1);
	akm_addr = FindAddress(0x0C,0x0F);
	AKM_WHOAMI_READ(akm_addr,&akm_WhoAmI);
	//viewMagRegisters();
	if(!ReadStatus()){
		INT_PIN_CFG_WRITE_BYPASS_EN(0);
		akm_addr = 0;
		return *this;
	}
	
	AKM_SOFT_RESET(akm_addr);
	delay(100);
	if(!HIGH_SENS) mRes = 10.*4912./8190.; // Proper scale to return milliGauss MFS_14BITS
	else mRes = 10.*4912./32760.0; // Proper scale to return milliGauss MFS_16BITS
	uint8_t AKMData[3];



	// Directly access the Magnetometer:
	AKM_CNTL_WRITE_POWER_DOWN(akm_addr,0);
	delay(10);
	AKM_CNTL_WRITE_FUSE_ROM_ACCESS(akm_addr,0);
	delay(10);
	AKM_ASAXYZ_READ_SENS_ADJ_XYZ(akm_addr,AKMData);
	

	AKM_CNTL_WRITE_POWER_DOWN(akm_addr,0);

	delay(10);
	uint8_t DirectAccessToMag = 0;
	if(!DirectAccessToMag){

		int8_t Num;
		//I2Cdev::writeByte(0x0C,0x0A ,  (HIGH_SENS & 1) << 4 | 0x01 );// 16bit single measurement mode
		AKM_CNTL_WRITE_SINGLE_MEAS_MODE(akm_addr,HIGH_SENS);
		INT_PIN_CFG_WRITE_BYPASS_EN(0);

		// Configure MPU I2C Secondary Bus as a Master Bus at 400khz
		//writeByte(MPU9250_ADDRESS, I2C_MST_CTRL		  0x24, 0x1D	0B 0001 1101);       // I2C configuration STOP after each transaction, master I2C bus at 400 KHz
		I2C_MST_CTRL_WRITE_MULT_MST_EN(1);
		I2C_MST_CTRL_WRITE_I2C_MST_P_NSR(1);
		I2C_MST_CTRL_WRITE_I2C_MST_CLK_400();
		I2C_MST_CTRL_READ_ALL(&Num);
		//writeByte(MPU9250_ADDRESS, I2C_MST_DELAY_CTRL 0x67, 0x81	0B 1000 0001) ; // Use blocking data retreival and enable delay for mag sample rate mismatch
		I2C_MST_DELAY_CTRL_WRITE_DELAY_ES_SHADOW(1);
		I2C_MST_DELAY_CTRL_WRITE_I2C_SLV1_DLY_EN(1);
		I2C_MST_DELAY_CTRL_WRITE_I2C_SLV0_DLY_EN(1);
		I2C_MST_DELAY_CTRL_READ_ALL(&Num);
		
		//writeByte(MPU9250_ADDRESS, I2C_SLV4_CTRL	  0x34, 0x01	0B 000 0001);      // Delay mag data retrieval to once every other accel/gyro data sample
		I2C_SLV4_CTRL_WRITE_I2C_MST_DLY(1); // (1+I2C_MST_DLY) // Delay mag data retrieval to once every other accel/gyro data sample
		I2C_SLV4_CTRL_READ_ALL(&Num);
		
		// Slave 0 Retrieves the data
		I2C_SLV0_ADDR_WRITE_I2C_SLV0_RNW(1);			//Slave 0 reads from AKM data registers.
		I2C_SLV0_ADDR_WRITE_I2C_ID_0(akm_addr);			//compass address

		I2C_SLV0_ADDR_READ_ALL(&Num);
		

		I2C_SLV0_REG_WRITE_I2C_SLV0_REG(AKM_XOUT_L);	//0x02 Compass reads start at this register.

		I2C_SLV0_REG_READ_I2C_SLV0_REG(&Num);
		

		I2C_SLV0_CTRL_WRITE_I2C_SLV0_EN(1);				// Enable slave 0,
		I2C_SLV0_CTRL_WRITE_I2C_SLV0_LENG(7);			// 8-byte reads.

		I2C_SLV0_CTRL_READ_ALL(&Num);
		

		I2C_MST_DELAY_CTRL_WRITE_I2C_SLV0_DLY_EN(1); //Trigger slave 0 and slave 1 actions at each sample.

		I2C_MST_DELAY_CTRL_READ_ALL(&Num);
		

		// Slave 1 Asks for more data to be retrieved
		I2C_SLV1_ADDR_WRITE_I2C_SLV1_RNW(0);			//Slave 1 reads from AKM data registers.
		I2C_SLV1_ADDR_WRITE_I2C_ID_1(akm_addr);			//compass address

		I2C_SLV1_ADDR_READ_ALL(&Num);
		

		I2C_SLV1_REG_WRITE_I2C_SLV1_REG(AKM_REG_CNTL);	//0x0A AKM measurement mode register

		I2C_SLV1_REG_READ_I2C_SLV1_REG(&Num);
		

		I2C_SLV1_CTRL_WRITE_I2C_SLV1_EN(1);				//Enable slave 1
		I2C_SLV1_CTRL_WRITE_I2C_SLV1_LENG(1);			//1-byte writes.

		I2C_SLV1_CTRL_READ_ALL(&Num);
		

		I2C_SLV1_DO_WRITE_I2C_SLV1_DO(AKM_SINGLE_MEASUREMENT|(HIGH_SENS<<4)); //Set slave 1 data.

		I2C_SLV1_DO_READ_I2C_SLV1_DO(&Num);
		

		I2C_MST_DELAY_CTRL_WRITE_I2C_SLV1_DLY_EN(1);

		I2C_MST_DELAY_CTRL_READ_ALL(&Num);
		
		//	AKM_SOFT_RESET(akm_addr);
		delay(100);
	}
	delay(10);
	return *this;
}


// Simple_MPU6050 & Simple_MPU6050::mpu_set_bypass(unsigned char bypass_on){
// 	if (bypass_on) {
// 		USER_CTRL_WRITE_I2C_MST_EN(0);
// 		delay(3);
// 		INT_PIN_CFG_WRITE_ACTL(0);				//7
// 		INT_PIN_CFG_WRITE_OPEN(0);				//6
// 		INT_PIN_CFG_WRITE_FSYNC_INT_MODE_EN(0);	//2
// 		INT_PIN_CFG_WRITE_BYPASS_EN(1);			//1
// 		} else {
// 		/* Enable I2C master mode if compass is being used. */
// 		USER_CTRL_WRITE_I2C_MST_EN(akm_addr>0);
// 		delay(3);
// 		INT_PIN_CFG_WRITE_ACTL(0);				//7
// 		INT_PIN_CFG_WRITE_OPEN(0);				//6
// 		INT_PIN_CFG_WRITE_FSYNC_INT_MODE_EN(0);	//2
// 		INT_PIN_CFG_WRITE_BYPASS_EN(0);			//1
// 	}
// 	return *this;
// }
