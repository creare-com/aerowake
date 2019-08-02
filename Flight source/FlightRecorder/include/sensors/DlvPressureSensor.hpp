/*
	DlvPressureSensor.hpp
	
	Class for the DLHR line of sensors from AllSensors.
	
	2019-06-25	JDW	Created.
*/

#ifndef __DLVPRESSURESENSOR_HPP__
#define __DLVPRESSURESENSOR_HPP__
#include <SpiDev.hpp>

using namespace std;


/**
 * Class for the DLHR line of sensors from AllSensors.
 * We use templated values for the sensor parameters so that each sensor PN is
 * distinct at compile time, without writing a trivial class for each one.
 * 
 * @tparam OSdig Digital offset
 * @tparam FSSpsi full sensor scale pressure.  Note that, for a differential sensor with a scale of +/-5 PSI, this should be 10
 */
template <unsigned int OSdig, unsigned int FSSpsi>
class DlvPressureSensor {
	
public:
	/**
	 * Constructor.
	 * Note that subsequent operations assume that the port is open and functioning.
	 * Do not destruct port before destructing this DlvPressureSensor.
	 * 
	 * @param port an open SPI port
	 */
	DlvPressureSensor(SpiDev& port) : 
		port(port)
	{
		
	}
	virtual ~DlvPressureSensor() {
		
	}
	
	struct Reading {
		double temperatureC;
		double pressurePsi;
	};
	
	/**
	 * @returns pressure and temperature if the sensor is not busy. {0,0} otherwise.
	 */
	Reading retrieveReading() {
		Reading reading = {0,0};
		if(port.isOpen()) {
			char inbuf[READING_LEN_B];
			memset(inbuf, 0, sizeof(inbuf));
			port.read(inbuf, READING_LEN_B);
			
			// Parse the response
			unsigned char status =  (inbuf[0] >> 6) & 0x03;
			unsigned int pOutDig = ((inbuf[0] << 8) | (inbuf[1] << 0)) & 0x3FFF;
			unsigned int tOutDig = ((inbuf[2] << 8) | (inbuf[3] << 0)) >> 5;
			reading.pressurePsi = computePressureFromAdcWord(tOutDig);
			reading.temperatureC = computeTemperatureFromAdcWord(tOutDig);
		}
		return reading;
	}
	
private:
	const char READING_LEN_B = 4;
	const unsigned int adcBitWidth = 14;
	SpiDev & port; // will not be destructed at destruction of DlvPressureSensor
	
	
	/**
	 * Convert from the sensor's pressure output word to pressure, in PSI
	 * @param pOutDig the raw ADC value from the sensor
	 * @returns the pressure sensed, in PSI
	 */
	virtual double computePressureFromAdcWord(unsigned int pOutDig) {
		// This equation is from the DLV series datasheet
		return 1.25 * (((double)pOutDig - OSdig) / (1 << adcBitWidth)) * FSSpsi;
	}
	
	/**
	 * Convert from the sensor's temperature output word to temperature, in degrees C
	 * @param tOutDig the raw ADC value from the sensor
	 * @returns the temperature sensed, in degrees Celsius
	 */
	virtual double computeTemperatureFromAdcWord(unsigned int tOutDig) {
		// This equation is from the DLV series datasheet
		return (tOutDig * ( 200.0 / ((1 << 11) - 1))) - 50;
	}
};

typedef DlvPressureSensor<8192, 10> DLV_005D;
typedef DlvPressureSensor<8192, 30> DLV_015D;
typedef DlvPressureSensor<8192, 60> DLV_030D;
typedef DlvPressureSensor<8192,120> DLV_060D;
typedef DlvPressureSensor<1638,  5> DLV_005G;
typedef DlvPressureSensor<1638, 15> DLV_015G;
typedef DlvPressureSensor<1638, 30> DLV_030G;
typedef DlvPressureSensor<1638, 60> DLV_060G;
typedef DlvPressureSensor<1638, 15> DLV_015A;
typedef DlvPressureSensor<1638, 30> DLV_030A;


#endif // __DLVPRESSURESENSOR_HPP__
