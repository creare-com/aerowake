/*
	DlhrPressureSensor.hpp
	
	Class for the DLHR line of sensors from AllSensors.
	
	2019-06-25	JDW	Created.
*/

#ifndef __DLHRPRESSURESENSOR_HPP__
#define __DLHRPRESSURESENSOR_HPP__
#include <SpiDev.hpp>

using namespace std;


/**
 * Class for the DLHR line of sensors from AllSensors.
 * We use templated values for the sensor parameters so that each sensor PN is
 * distinct at compile time, without writing a trivial class for each one.
 * 
 * @tparam OSdigx10 10x Digital offset
 *   We can't do floating-point template values, but the DLHR line only has increments of 0.1 for this.
 */
template <unsigned int OSdigx10, unsigned int FSSinH2O>
class DlhrPressureSensor {
	
public:
	/**
	 * Constructor.
	 * Note that subsequent operations assume that the port is open and functioning.
	 * Do not destruct port before destructing this DlhrPressureSensor.
	 * 
	 * @param port an open SPI port
	 */
	DlhrPressureSensor(SpiDev& port) : 
		port(port)
	{
		
	}
	virtual ~DlhrPressureSensor() {
		
	}

	enum MeasurementType {
		Single    = 0xAA,
		Average2  = 0xAC,
		Average4  = 0xAD,
		Average8  = 0xAE,
		Average16 = 0xAF,
	};
	
	struct Reading {
		double temperatureC;
		double pressureInH20;
	};
	
	/**
	 * Tell the sensor to begin a measurement.
	 * 
	 * @param type the type of measurement (see datasheet)
	 */
	void commandReading(MeasurementType type = MeasurementType::Single) {
		if(port.isOpen()) {
			char outbuf[COMMAND_LEN_B];
			memset(outbuf, 0, sizeof(outbuf));
			outbuf[0] = type;
			port.write(outbuf, COMMAND_LEN_B);
		}
	}
	
	/**
	 * Retrieve the reading from the sensor.
	 * This should only be called >= 4ms after calling commandReading()
	 * @returns pressure and temperature returned by the sensor
	 */
	Reading retrieveReading() {
		Reading reading = {0,0};
			char inbuf[READING_LEN_B];
			char outbuf[READING_LEN_B];
			memset(inbuf, 0, sizeof(inbuf));
			memset(outbuf, 0, sizeof(outbuf));
			outbuf[0] = READ_STATUS_COMMAND; // required
			port.transfer(inbuf, outbuf, READING_LEN_B);
			
			// Parse the response
			unsigned int pOutDig = (inbuf[1] << 16) | (inbuf[2] << 8) | (inbuf[3] << 0);
			unsigned int tOutDig = (inbuf[4] << 16) | (inbuf[5] << 8) | (inbuf[6] << 0);
			reading.pressureInH20 = computePressureFromAdcWord(tOutDig);
			reading.temperatureC = computeTemperatureFromAdcWord(tOutDig);
		}
		return reading;
	}
	
private:
	const char READ_STATUS_COMMAND = 0xF0;
	const char COMMAND_LEN_B = 3;
	const char READING_LEN_B = 7;
	const char STATUS_BUSY = 0x20;
	const unsigned int adcBitWidth = 24;
	SpiDev & port; // will not be destructed at destruction of DlhrPressureSensor
	
	
	/**
	 * Convert from the sensor's pressure output word to pressure, in inches of water
	 * @param pOutDig the raw ADC value from the sensor
	 * @returns the pressure sensed, in inches of water
	 */
	virtual double computePressureFromAdcWord(unsigned int pOutDig) {
		// This equation is from the DLHR series datasheet, DS-0350_Rev_B
		return 1.25 * ((pOutDig - (OSdigx10 / 10.0)) / (1 << adcBitWidth)) * FSSinH2O;
	}
	
	/**
	 * Convert from the sensor's temperature output word to temperature, in degrees C
	 * @param tOutDig the raw ADC value from the sensor
	 * @returns the temperature sensed, in degrees Celsius
	 */
	virtual double computeTemperatureFromAdcWord(unsigned int tOutDig) {
		// This equation is from the DLHR series datasheet, DS-0350_Rev_B
		return ((tOutDig * 125) / (1 << adcBitWidth)) - 40;
	}
};

//                           OSdigx10     , FSSinH2O
typedef DlhrPressureSensor<(5 * (1 << 24)),    1> DLHR_F50D;
typedef DlhrPressureSensor<(5 * (1 << 24)), 2*01> DLHR_L01D;
typedef DlhrPressureSensor<(5 * (1 << 24)), 2*02> DLHR_L02D;
typedef DlhrPressureSensor<(5 * (1 << 24)), 2*05> DLHR_L05D;
typedef DlhrPressureSensor<(5 * (1 << 24)), 2*10> DLHR_L10D;
typedef DlhrPressureSensor<(5 * (1 << 24)), 2*20> DLHR_L20D;
typedef DlhrPressureSensor<(5 * (1 << 24)), 2*30> DLHR_L30D;
typedef DlhrPressureSensor<(5 * (1 << 24)), 2*60> DLHR_L60D;
typedef DlhrPressureSensor<(1 * (1 << 24)), 1*01> DLHR_L01G;
typedef DlhrPressureSensor<(1 * (1 << 24)), 1*02> DLHR_L02G;
typedef DlhrPressureSensor<(1 * (1 << 24)), 1*05> DLHR_L05G;
typedef DlhrPressureSensor<(1 * (1 << 24)), 1*10> DLHR_L10G;
typedef DlhrPressureSensor<(1 * (1 << 24)), 1*20> DLHR_L20G;
typedef DlhrPressureSensor<(1 * (1 << 24)), 1*30> DLHR_L30G;
typedef DlhrPressureSensor<(1 * (1 << 24)), 1*60> DLHR_L60G;


#endif // __DLHRPRESSURESENSOR_HPP__
