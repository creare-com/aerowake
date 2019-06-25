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
 * @tparam FSSinH2O 10x the Full sensor span, in inches of water.  
 *   We can't do floating-point template values, but the DLHR line only has increments of 0.1 for this.
 */
template <unsigned int OSdigx10, unsigned int FSSinH2Ox10>
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
	
private:
	const unsigned int adcBitWidth = 24;
	SpiDev & port; // will not be destructed at destruction of DlhrPressureSensor
	
	/**
	 * Convert from the sensor's pressure output word to pressure, in inches of water
	 * @param pOutDig the raw ADC value from the sensor
	 * @returns the pressure sensed, in inches of water
	 */
	virtual double computePressureFromAdcWord(unsigned int pOutDig) {
		// This equation is from the DLHR series datasheet, DS-0350_Rev_B
		return 1.25 * ((pOutDig - (OSdigx10 / 10.0)) / (1 << adcBitWidth)) * (FSSinH2Ox10 / 10.0);
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

typedef DlhrPressureSensor<(5 * (1 << 24)), (4 * (1 << 24))> DlhrPressureSensorDiff;
typedef DlhrPressureSensor<(1 * (1 << 24)), (8 * (1 << 24))> DlhrPressureSensorGage;

typedef DlhrPressureSensorDiff DLHR_F50D;
typedef DlhrPressureSensorDiff DLHR_L01D;
typedef DlhrPressureSensorDiff DLHR_L02D;
typedef DlhrPressureSensorDiff DLHR_L05D;
typedef DlhrPressureSensorDiff DLHR_L10D;
typedef DlhrPressureSensorDiff DLHR_L20D;
typedef DlhrPressureSensorDiff DLHR_L30D;
typedef DlhrPressureSensorDiff DLHR_L60D;
typedef DlhrPressureSensorGage DLHR_L01G;
typedef DlhrPressureSensorGage DLHR_L02G;
typedef DlhrPressureSensorGage DLHR_L05G;
typedef DlhrPressureSensorGage DLHR_L10G;
typedef DlhrPressureSensorGage DLHR_L20G;
typedef DlhrPressureSensorGage DLHR_L30G;
typedef DlhrPressureSensorGage DLHR_L60G;


#endif // __DLHRPRESSURESENSOR_HPP__
