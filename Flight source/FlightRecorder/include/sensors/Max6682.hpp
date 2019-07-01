/*
	Max6682.hpp
	
	Class for the MAX6682 thermistor interface from Maxim Integrated Products.
	
	2019-07-01	JDW	Created.
*/

#ifndef __MAX6682_HPP__
#define __MAX6682_HPP__
#include <SpiDev.hpp>

using namespace std;

class Max6682 {
	
public:
	/**
	 * Constructor.
	 * Note that subsequent operations assume that the port is open and functioning.
	 * Do not destruct port before destructing this Max6682.
	 * 
	 * @param port an open SPI port
	 */
	Max6682(SpiDev& port) : 
		port(port)
	{
		
	}
	virtual ~Max6682() {
		
	}
	
	
	/**
	 * @returns ADC output, in ADC ticks.  (Called Dout in the datasheet)
	 */
	int getAdcValue() {
		int reading = 0;
		if(port.isOpen()) {
			int16_t inbuf;
			memset(&inbuf, 0, sizeof(inbuf));
			port.read((char *)(&inbuf), sizeof(inbuf));
			
			// It's a signed int value 11 bits long, aligned such that its sign
			// bit is the MSb.  So rather than right-shift it, which is compiler-dependent,
			// we divide, which is always an arithmetic shift.
			reading = (inbuf / (1<<5));
		}
		return reading;
	}
	
private:
	SpiDev & port; // will not be destructed at destruction of Max6682
};

#endif // __MAX6682_HPP__
