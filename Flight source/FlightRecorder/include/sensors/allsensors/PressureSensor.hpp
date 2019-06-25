/*
	PressureSensor.hpp
	
	Common ancestor for pressure sensors manufactured by Allsensors 
	
	2019-06-25	JDW	Created.
*/

#ifndef __PRESSURESENSOR_HPP__
#define __PRESSURESENSOR_HPP__
#include <SpiDev.hpp>

using namespace std;

class PressureSensor {
	
public:
	/**
	 * Constructor.
	 * Note that subsequent operations assume that the port is open and functioning.
	 * Do not destruct port before destructing this PressureSensor.
	 * 
	 * @param port an open SPI port
	 * @param adcBitWidth Number of bits in the ADC words
	 * @param OSdig Digital offset
	 * @param FSSinH2O Full sensor scale, in inches of water
	 */
	PressureSensor(SpiDev& port, 
		const unsigned int adcBitWidth,
		const unsigned int OSdig,
		const double FSSinH2O) : 
		port(port),
		adcBitWidth(adcBitWidth),
		OSdig(OSdig),
		FSSinH2O(FSSinH2O)
	{
		
	}
	virtual ~PressureSensor();
	
private:
	SpiDev & port; // will not be destructed at destruction of PressureSensor
	
	// Sensor-specific parameters
	const unsigned int adcBitWidth; // Number of bits in the ADC words
	const unsigned int OSdig; // Digital offset
	const double FSSinH2O; // Full sensor scale, in inches of water
	
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

#endif // __SPIDEV_HPP__
