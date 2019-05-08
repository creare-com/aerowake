/**
 * Invensense MPU-9250 library using the SPI interface
 *
 * Copyright (c) 2015 Brian Chen
 * Modified heavily in 2017 by John Walthour for Creare LLC (jdw@creare.com)
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <spiSensors/MPU9250.h>

/*                                     INITIALIZATION
 * usage: call this function at startup, giving the sample rate divider (raging from 0 to 255) and
 * low pass filter value; suitable values are:
 * BITS_DLPF_CFG_256HZ_NOLPF2
 * BITS_DLPF_CFG_188HZ
 * BITS_DLPF_CFG_98HZ
 * BITS_DLPF_CFG_42HZ
 * BITS_DLPF_CFG_20HZ
 * BITS_DLPF_CFG_10HZ 
 * BITS_DLPF_CFG_5HZ 
 * BITS_DLPF_CFG_2100HZ_NOLPF
 * returns 1 if an error occurred
 */

#define MPU_InitRegNum 17

bool MPU9250::init(Logger * lgr, std::string devicePath, 
		bool calib_gyro, bool calib_acc){
	SpiDevice::init(lgr, devicePath);
	bool succeeded = true;
	
	

	// double temp[3];
	// if(calib_gyro && calib_acc){
		// calibrate(g_bias, a_bias);
	// }
	// else if(calib_gyro){
		// calibrate(g_bias, temp);
	// }
	// else if(calib_acc){
		// calibrate(temp, a_bias);
	// }
	memset(a_bias, 0, sizeof(a_bias));
	memset(g_bias, 0, sizeof(g_bias));
	
	uint8_t i = 0;
	uint8_t MPU_Init_Data[MPU_InitRegNum][2] = {
		{BIT_H_RESET, MPUREG_PWR_MGMT_1},        // Reset Device
		{0x01, MPUREG_PWR_MGMT_1},               // Clock Source
		{0x00, MPUREG_PWR_MGMT_2},               // Enable Acc & Gyro
		{my_low_pass_filter, MPUREG_CONFIG},     // Use DLPF set Gyroscope bandwidth 184Hz, temperature bandwidth 188Hz
		{BITS_FS_250DPS, MPUREG_GYRO_CONFIG},    // +-250dps
		{BITS_FS_2G, MPUREG_ACCEL_CONFIG},       // +-2G
		{my_low_pass_filter_acc, MPUREG_ACCEL_CONFIG_2}, // Set Acc Data Rates, Enable Acc LPF , Bandwidth 184Hz
		{0x12, MPUREG_INT_PIN_CFG},      //
		//{0x40, MPUREG_I2C_MST_CTRL},   // I2C Speed 348 kHz
		//{0x20, MPUREG_USER_CTRL},      // Enable AUX
		{0x30, MPUREG_USER_CTRL},        // I2C Master mode and set I2C_IF_DIS to disable slave mode I2C bus
		{0x0D, MPUREG_I2C_MST_CTRL},     // I2C configuration multi-master  IIC 400KHz
		
		{AK8963_I2C_ADDR, MPUREG_I2C_SLV0_ADDR},  // Set the I2C slave addres of AK8963 and set for write.
		//{0x09, MPUREG_I2C_SLV4_CTRL},
		//{0x81, MPUREG_I2C_MST_DELAY_CTRL}, // Enable I2C delay

		{AK8963_CNTL2, MPUREG_I2C_SLV0_REG}, // I2C slave 0 register address from where to begin data transfer
		{0x01, MPUREG_I2C_SLV0_DO},   // Reset AK8963
		{0x81, MPUREG_I2C_SLV0_CTRL}, // Enable I2C and set 1 byte

		{AK8963_CNTL1, MPUREG_I2C_SLV0_REG}, // I2C slave 0 register address from where to begin data transfer
#ifdef AK8963FASTMODE
		{0x16, MPUREG_I2C_SLV0_DO},   // Register value to 100Hz continuous measurement in 16bit
#else
		{0x12, MPUREG_I2C_SLV0_DO},   // Register value to 8Hz continuous measurement in 16bit
#endif
		{0x81, MPUREG_I2C_SLV0_CTRL}  //Enable I2C and set 1 byte
		
	};

	for(i = 0; i < MPU_InitRegNum; i++) {
		succeeded = succeeded && writeByteRegister(MPU_Init_Data[i][1], MPU_Init_Data[i][0]);
		delayMicroseconds(1000);  // I2C must slow down the write speed, otherwise it won't work
	}

	if(!succeeded) {
		logger->logError("Failed to configure MPU9250.");
	}
	set_acc_scale(BITS_FS_2G);
	set_gyro_scale(BITS_FS_250DPS);
	
//	calib_mag();  // If experiencing problems here, just comment it out. Should still be somewhat functional.
	return 0;
}

/*                                ACCELEROMETER SCALE
 * usage: call this function at startup, after initialization, to set the right range for the
 * accelerometers. Suitable ranges are:
 * BITS_FS_2G
 * BITS_FS_4G
 * BITS_FS_8G
 * BITS_FS_16G
 * returns the range set (2,4,8 or 16)
 */

uint8_t MPU9250::set_acc_scale(uint8_t scale){
	uint8_t temp_scale;
	writeByteRegister(MPUREG_ACCEL_CONFIG, scale);
	
	switch (scale){
		case BITS_FS_2G:
			acc_divider=16384;
		break;
		case BITS_FS_4G:
			acc_divider=8192;
		break;
		case BITS_FS_8G:
			acc_divider=4096;
		break;
		case BITS_FS_16G:
			acc_divider=2048;
		break;   
	}
	readByteRegister(MPUREG_ACCEL_CONFIG, &temp_scale);
	
	switch (temp_scale){
		case BITS_FS_2G:
			temp_scale=2;
		break;
		case BITS_FS_4G:
			temp_scale=4;
		break;
		case BITS_FS_8G:
			temp_scale=8;
		break;
		case BITS_FS_16G:
			temp_scale=16;
		break;   
	}
	return temp_scale;
}



/*                                 GYROSCOPE SCALE
 * usage: call this function at startup, after initialization, to set the right range for the
 * gyroscopes. Suitable ranges are:
 * BITS_FS_250DPS
 * BITS_FS_500DPS
 * BITS_FS_1000DPS
 * BITS_FS_2000DPS
 * returns the range set (250,500,1000 or 2000)
 */

unsigned int MPU9250::set_gyro_scale(uint8_t scale){
	uint8_t temp_scale;
	unsigned int set_scale;
	writeByteRegister(MPUREG_GYRO_CONFIG, scale);

	switch (scale){
		case BITS_FS_250DPS:   gyro_divider = 131;  break;
		case BITS_FS_500DPS:   gyro_divider = 65.5; break;
		case BITS_FS_1000DPS:  gyro_divider = 32.8; break;
		case BITS_FS_2000DPS:  gyro_divider = 16.4; break;   
	}

	readByteRegister(MPUREG_GYRO_CONFIG, &temp_scale);

	switch (temp_scale){
		case BITS_FS_250DPS:   set_scale = 250;     break;
		case BITS_FS_500DPS:   set_scale = 500;     break;
		case BITS_FS_1000DPS:  set_scale = 1000;    break;
		case BITS_FS_2000DPS:  set_scale = 2000;    break;   
	}
	return set_scale;
}



/*                                 WHO AM I?
 * usage: call this function to know if SPI is working correctly. It checks the I2C address of the
 * mpu9250 which should be 0x71
 */

unsigned int MPU9250::whoami(){
	uint8_t response;
	readByteRegister(MPUREG_WHOAMI, &response);
	return response;
}



/*                                 READ ACCELEROMETER
 * usage: call this function to read accelerometer data. Axis represents selected axis:
 * 0 -> X axis
 * 1 -> Y axis
 * 2 -> Z axis
 */

void MPU9250::read_acc()
{
	uint8_t response[6];
	int16_t bit_data;
	double data;
	int i;
	if(readDataStartingAtReg<6>(MPUREG_ACCEL_XOUT_H, response)) {
		for(i = 0; i < 3; i++) {
			bit_data = ((int16_t)response[i*2]<<8)|response[i*2+1];
			data = (double)bit_data;
			accel_data[i] = data/acc_divider - a_bias[i];
		}
	} else {
		logger->logError("Failed to read accelerometer.");
		memset(&accel_data, 0, sizeof(accel_data));
	}
	
}

/*                                 READ GYROSCOPE
 * usage: call this function to read gyroscope data. Axis represents selected axis:
 * 0 -> X axis
 * 1 -> Y axis
 * 2 -> Z axis
 */

void MPU9250::read_gyro()
{
	uint8_t response[6];
	int16_t bit_data;
	double data;
	int i;
	if(readDataStartingAtReg<6>(MPUREG_GYRO_XOUT_H, response)) {
		for(i = 0; i < 3; i++) {
			bit_data = ((int16_t)response[i*2]<<8) | response[i*2+1];
			data = (double)bit_data;
			gyro_data[i] = data/gyro_divider - g_bias[i];
		}
	} else {
		logger->logError("Failed to read gyroscope.");
		memset(&gyro_data, 0, sizeof(gyro_data));
	}
	
}


/*                                 READ temperature
 * usage: call this function to read temperature data. 
 * returns the value in °C
 */

void MPU9250::read_temp(){
	uint8_t response[2];
	int16_t bit_data;
	double data;
	if(readDataStartingAtReg<2>(MPUREG_TEMP_OUT_H, response)) {

		bit_data = ((int16_t)response[0]<<8)|response[1];
		data = (double)bit_data;
		temperature = (data/340)+36.53;
	} else {
		logger->logError("Failed to read temperature.");
		memset(&temperature, 0, sizeof(temperature));
	}
}

/*                                 READ ACCELEROMETER CALIBRATION
 * usage: call this function to read accelerometer data. Axis represents selected axis:
 * 0 -> X axis
 * 1 -> Y axis
 * 2 -> Z axis
 * returns Factory Trim value
 */

void MPU9250::calib_acc()
{
	uint8_t response[4];
	uint8_t temp_scale;
	//READ CURRENT ACC SCALE
	readByteRegister(MPUREG_ACCEL_CONFIG, &temp_scale);
	set_acc_scale(BITS_FS_8G);
	//ENABLE SELF TEST need modify
	//temp_scale=writeByteRegister(MPUREG_ACCEL_CONFIG, 0x80>>axis);

	readDataStartingAtReg<4>(MPUREG_SELF_TEST_X, response);
	calib_data[0] = ((response[0]&0b11100000)>>3) | ((response[3]&0b00110000)>>4);
	calib_data[1] = ((response[1]&0b11100000)>>3) | ((response[3]&0b00001100)>>2);
	calib_data[2] = ((response[2]&0b11100000)>>3) | ((response[3]&0b00000011));

	set_acc_scale(temp_scale);
}

	
uint8_t MPU9250::AK8963_whoami(){
	uint8_t response;
	readMagRegister(AK8963_WIA, &response);
	return response;
}

bool MPU9250::triggerMagXfer(bool read_mag, uint8_t reg, uint8_t len) {
	bool succeeded = true;
	succeeded = succeeded && writeByteRegister(MPUREG_I2C_SLV0_ADDR,
		AK8963_I2C_ADDR|(read_mag? READ_FLAG : 0)); //Set the I2C slave addres of AK8963 and set for read.
	succeeded = succeeded && writeByteRegister(MPUREG_I2C_SLV0_REG, reg); //I2C slave 0 register address from where to begin data transfer
	succeeded = succeeded && writeByteRegister(MPUREG_I2C_SLV0_CTRL, 0x80 | len); //transfer len bytes to/from the magnetometer
	return succeeded;
}


bool MPU9250::readMagRegister(uint8_t reg, uint8_t * dest) {
	bool succeeded = triggerMagXfer(true, reg, 1);
	delayMicroseconds(DELAY_FOR_I2C_REG_ACCESS_US);
	succeeded = succeeded && readByteRegister(MPUREG_EXT_SENS_DATA_00, dest);    //Read I2C 

	return succeeded;
}

template <int LEN>
bool MPU9250::readMagDataStartingAtReg(uint8_t reg, uint8_t * dest) {
	const uint8_t actual_len = LEN & 0x0F;
	bool succeeded = triggerMagXfer(true, reg, actual_len);

	delayMicroseconds(DELAY_FOR_I2C_REG_ACCESS_US);
	succeeded = succeeded && readDataStartingAtReg<actual_len>(MPUREG_EXT_SENS_DATA_00, dest);    //Read I2C 

	return succeeded;

}

bool MPU9250::writeMagRegister(uint8_t reg, uint8_t value) {
	bool succeeded = true;
	succeeded = succeeded && writeByteRegister(MPUREG_I2C_SLV0_DO, value); 
	succeeded = succeeded && triggerMagXfer(true, reg, 1);

	delayMicroseconds(DELAY_FOR_I2C_REG_ACCESS_US);
	return succeeded;
}

void MPU9250::calib_mag(){
	uint8_t response[3];
	double data;
	int i;
	// Choose either 14-bit or 16-bit magnetometer resolution
	//uint8_t MFS_14BITS = 0; // 0.6 mG per LSB
	uint8_t MFS_16BITS =1; // 0.15 mG per LSB
	// 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read
	uint8_t M_8HZ = 0x02; // 8 Hz update
	//uint8_t M_100HZ = 0x06; // 100 Hz continuous magnetometer

	writeMagRegister(AK8963_CNTL1, 0x00);                               // set AK8963 to Power Down
	delayMicroseconds(50000);                                                  // long wait between AK8963 mode changes
	writeMagRegister(AK8963_CNTL1, 0x0F);                               // set AK8963 to FUSE ROM access
	delayMicroseconds(50000);                                                  // long wait between AK8963 mode changes

	/* get the magnetometer sensitivity scale factor */
	readMagDataStartingAtReg<3>(AK8963_ASAX, response);

	stringstream ss;
	ss << "Magnetometer_ASA[] = {";
	for(i = 0; i < 3; i++) {
		data=response[i];
		Magnetometer_ASA[i] = ((data-128)/256+1)*Magnetometer_Sensitivity_Scale_Factor;
		ss << Magnetometer_ASA[i] << ", ";
	}
	ss << "}";
	logger->logDebug(ss.str());
	writeMagRegister(AK8963_CNTL1, 0x00); // set AK8963 to Power Down
	delayMicroseconds(50000);
	// Configure the magnetometer for continuous read and highest resolution.
	// Set bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL
	// register, and enable continuous mode data acquisition (bits [3:0]),
	// 0010 for 8 Hz and 0110 for 100 Hz sample rates.   
	writeMagRegister(AK8963_CNTL1, MFS_16BITS << 4 | M_8HZ);            // Set magnetometer data resolution and sample ODR
	delayMicroseconds(50000);
}

void MPU9250::read_mag(){
	uint8_t response[7];
	double data;
	int i;

	readMagDataStartingAtReg<7>(AK8963_HXL, response);
	// must start your read from AK8963A register 0x03 and read seven bytes so that upon read of ST2 register 0x09 the AK8963A will unlatch the data registers for the next measurement.

	stringstream ss;
	ss << "mag_data_raw[] = {";
	for(i = 0; i < 3; i++) {
		mag_data_raw[i] = ((int16_t)response[i*2+1]<<8)|response[i*2];
		data = (double)mag_data_raw[i];
		mag_data[i] = data*Magnetometer_ASA[i];
		ss << mag_data_raw[i] << ", ";
	}
	ss << "}";
	logger->logDebug(ss.str());
}

uint8_t MPU9250::get_CNTL1(){
	uint8_t response;
	readMagRegister(AK8963_CNTL1, &response);
	return response;
}

void MPU9250::read_all(){
	uint8_t response[21];
	int16_t bit_data;
	double data;
	int i;
	bool succeeded = true;

	// Send I2C command at first
	succeeded = triggerMagXfer(true, AK8963_HXL, 7);
	// must start your read from AK8963A register 0x03 and read seven bytes so that upon read of ST2 register 0x09 the AK8963A will unlatch the data registers for the next measurement.
	if(!succeeded) {
		logger->logError("Failed to set up I2C read.");
	}
	delayMicroseconds(DELAY_FOR_I2C_REG_ACCESS_US);

	succeeded = readDataStartingAtReg<21>(MPUREG_ACCEL_XOUT_H, (uint8_t*)response);
	if(!succeeded) {
		logger->logError("Failed to read inertial data.");
	}
	// Get accelerometer value
	// stringstream ss;
	// ss << "accel_data[] = {";
	for(i = 0; i < 3; i++) {
		bit_data = ((int16_t)response[i*2]<<8) | response[i*2+1];
		data = (double)bit_data;
		accel_data[i] = data/acc_divider - a_bias[i];
		// ss << accel_data[i] << ", ";
	}
	// ss << "}\n";
	// Get temperature
	bit_data = ((int16_t)response[i*2]<<8) | response[i*2+1];
	data = (double)bit_data;
	temperature = ((data-21)/333.87)+21;
	// Get gyroscope value
	// ss << "gyro_data[] = {";
	for(i=4; i < 7; i++) {
		bit_data = ((int16_t)response[i*2]<<8) | response[i*2+1];
		data = (double)bit_data;
		gyro_data[i-4] = data/gyro_divider - g_bias[i-4];
		// ss << gyro_data[i-4] << ", ";
	}
	// ss << "}\n";
	// Get Magnetometer value
	// ss << "mag_data[] = {";
	for(i=7; i < 10; i++) {
		mag_data_raw[i-7] = ((int16_t)response[i*2+1]<<8) | response[i*2];
		data = (double)mag_data_raw[i-7];
		mag_data[i-7] = data * Magnetometer_ASA[i-7];
		// ss << mag_data[i-7] << ", ";
	}
	// ss << "}";
	// logger->logDebug(ss.str());
	
}

/*

This method assumes the sensor has a specific orientation relative to gravity,
and is stationary.

void MPU9250::calibrate(double *dest1, double *dest2){  
	uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
	uint16_t ii, packet_count, fifo_count;
	int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
  
	// reset device
	writeByteRegister(MPUREG_PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
	delay(100);
   
	// get stable time source; Auto select clock source to be PLL gyroscope reference if ready 
	// else use the internal oscillator, bits 2:0 = 001
	writeByteRegister(MPUREG_PWR_MGMT_1, 0x01);  
	writeByteRegister(MPUREG_PWR_MGMT_2, 0x00);
	delay(200);                                    

	// Configure device for bias calculation
	writeByteRegister(MPUREG_INT_ENABLE, 0x00);   // Disable all interrupts
	writeByteRegister(MPUREG_FIFO_EN, 0x00);      // Disable FIFO
	writeByteRegister(MPUREG_PWR_MGMT_1, 0x00);   // Turn on internal clock source
	writeByteRegister(MPUREG_I2C_MST_CTRL, 0x00); // Disable I2C master
	writeByteRegister(MPUREG_USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
	writeByteRegister(MPUREG_USER_CTRL, 0x0C);    // Reset FIFO and DMP
	delay(15);
  
	// Configure MPU6050 gyro and accelerometer for bias calculation
	writeByteRegister(MPUREG_CONFIG, 0x01);      // Set low-pass filter to 188 Hz
	writeByteRegister(MPUREG_SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
	writeByteRegister(MPUREG_GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
	writeByteRegister(MPUREG_ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity
	
	uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
	uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g
	
	// Configure FIFO to capture accelerometer and gyro data for bias calculation
	writeByteRegister(MPUREG_USER_CTRL, 0x40);   // Enable FIFO  
	writeByteRegister(MPUREG_FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
	delay(40); // accumulate 40 samples in 40 milliseconds = 480 bytes

	// At end of sample accumulation, turn off FIFO sensor read
	writeByteRegister(MPUREG_FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
	readDataStartingAtReg<2>(MPUREG_FIFO_COUNTH, data); // read FIFO sample count
	fifo_count = ((uint16_t)data[0] << 8) | data[1];
	packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging
	
	for (ii = 0; ii < packet_count; ii++) {
		int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
		readDataStartingAtReg<12>(MPUREG_FIFO_R_W, data); // read data for averaging
		accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
		accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
		accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;    
		gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
		gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
		gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;
		
		accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
		accel_bias[1] += (int32_t) accel_temp[1];
		accel_bias[2] += (int32_t) accel_temp[2];
		gyro_bias[0]  += (int32_t) gyro_temp[0];
		gyro_bias[1]  += (int32_t) gyro_temp[1];
		gyro_bias[2]  += (int32_t) gyro_temp[2];
			
	}
	accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
	accel_bias[1] /= (int32_t) packet_count;
	accel_bias[2] /= (int32_t) packet_count;
	gyro_bias[0]  /= (int32_t) packet_count;
	gyro_bias[1]  /= (int32_t) packet_count;
	gyro_bias[2]  /= (int32_t) packet_count;
	
	if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
	else {accel_bias[2] += (int32_t) accelsensitivity;}
   
	// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
	data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
	data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
	data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
	data[3] = (-gyro_bias[1]/4)       & 0xFF;
	data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
	data[5] = (-gyro_bias[2]/4)       & 0xFF;
  
	// Push gyro biases to hardware registers
	writeByteRegister(MPUREG_XG_OFFS_USRH, data[0]);
	writeByteRegister(MPUREG_XG_OFFS_USRL, data[1]);
	writeByteRegister(MPUREG_YG_OFFS_USRH, data[2]);
	writeByteRegister(MPUREG_YG_OFFS_USRL, data[3]);
	writeByteRegister(MPUREG_ZG_OFFS_USRH, data[4]);
	writeByteRegister(MPUREG_ZG_OFFS_USRL, data[5]);
  
	// Output scaled gyro biases for display in the main program
	dest1[0] = (double) gyro_bias[0]/(double) gyrosensitivity;  
	dest1[1] = (double) gyro_bias[1]/(double) gyrosensitivity;
	dest1[2] = (double) gyro_bias[2]/(double) gyrosensitivity;

	// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
	// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
	// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
	// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
	// the accelerometer biases calculated above must be divided by 8.

	int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
	readDataStartingAtReg<2>(MPUREG_XA_OFFSET_H, data); // Read factory accelerometer trim values
	accel_bias_reg[0] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
	readDataStartingAtReg<2>(MPUREG_YA_OFFSET_H, data);
	accel_bias_reg[1] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
	readDataStartingAtReg<2>(MPUREG_ZA_OFFSET_H, data);
	accel_bias_reg[2] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
	
	uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
	uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis
	
	for(ii = 0; ii < 3; ii++) {
	  if((accel_bias_reg[ii] & mask)) { mask_bit[ii] = 0x01; }// If temperature compensation bit is set, record that fact in mask_bit
	}
	
	// Construct total accelerometer bias, including calculated average accelerometer bias from above
	accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
	accel_bias_reg[1] -= (accel_bias[1]/8);
	accel_bias_reg[2] -= (accel_bias[2]/8);
  
	data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
	data[1] = (accel_bias_reg[0])      & 0xFF;
	data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
	data[3] = (accel_bias_reg[1])      & 0xFF;
	data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
	data[5] = (accel_bias_reg[2])      & 0xFF;
	data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers
 
// Apparently this is not working for the acceleration biases in the MPU-9250
// Are we handling the temperature correction bit properly?
// Push accelerometer biases to hardware registers
	writeByteRegister(MPUREG_XA_OFFSET_H, data[0]);
	writeByteRegister(MPUREG_XA_OFFSET_L, data[1]);
	writeByteRegister(MPUREG_YA_OFFSET_H, data[2]);
	writeByteRegister(MPUREG_YA_OFFSET_L, data[3]);
	writeByteRegister(MPUREG_ZA_OFFSET_H, data[4]);
	writeByteRegister(MPUREG_ZA_OFFSET_L, data[5]);

// Output scaled accelerometer biases for display in the main program
	dest2[0] = (double)accel_bias[0]/(double)accelsensitivity; 
	dest2[1] = (double)accel_bias[1]/(double)accelsensitivity;
	dest2[2] = (double)accel_bias[2]/(double)accelsensitivity;
}
*/