/*
	DlhrPressureSensor.hpp
	
	Class for the DLHR line of sensors from AllSensors.
	
	2019-06-25	JDW	Created.
*/

#ifndef __DLHRPRESSURESENSOR_HPP__
#define __DLHRPRESSURESENSOR_HPP__
#include <SpiDev.hpp>

using namespace std;

typedef DLHR_L01D_E1NJ_I_NAV8 DlhrPressureSensor<24, (1 << 24) >> 1, 0.8 * (1 << 24)>

/**
 * Class for the DLHR line of sensors from AllSensors.
 * 
 * @tparam adcBitWidth Number of bits in the ADC words
 * @tparam OSdig Digital offset
 * @tparam FSSinH2O Full sensor scale, in inches of water
 */
template <unsigned int adcBitWidth,
		unsigned int OSdig,
		double FSSinH2O>
class DlhrPressureSensor {
	
public:
	/**
	 * Constructor.
	 * Note that subsequent operations assume that the port is open and functioning.
	 * Do not destruct port before destructing this DlhrPressureSensor.
	 * 
	 * @param port an open SPI port
	 * @param adcBitWidth Number of bits in the ADC words
	 * @param OSdig Digital offset
	 * @param FSSinH2O Full sensor scale, in inches of water
	 */
	DlhrPressureSensor(SpiDev& port) : 
		port(port)
	{
		
	}
	virtual ~DlhrPressureSensor();
	
private:
	SpiDev & port; // will not be destructed at destruction of DlhrPressureSensor
	
	/**
	 * Convert from the sensor's pressure output word to pressure, in inches of water
	 * @param pOutDig the raw ADC value from the sensor
	 * @returns the pressure sensed, in inches of water
	 */
	virtual double computePressureFromAdcWord(unsigned int pOutDig) {
		// This equation is from the DLHR series datasheet, DS-0350_Rev_B
		return 1.25 * ((pOutDig - OSdig) / (1 << adcBitWidth)) * FSSinH2O;
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
}

#endif // __DLHRPRESSURESENSOR_HPP__
