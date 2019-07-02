/*
	Adg725.hpp
	
	Class to control the ADG725 analog multiplexer from Analog Devices
	
	2019-06-25	JDW	Created.
*/

#ifndef __ADG725_HPP__
#define __ADG725_HPP__
#include <SpiDev.hpp>

using namespace std;


class Adg725 {
	
public:
	/**
	 * Constructor.
	 * Note that subsequent operations assume that the port is open and functioning.
	 * Do not destruct port before destructing this Adg725.
	 * 
	 * @param port an open SPI port
	 */
	Adg725(SpiDev& port) : 
		port(port)
	{
		
	}
	virtual ~Adg725() {
		
	}
	
	
	enum Bank {
		A, B, BOTH, NEITHER
	};
	
	/**
	 * Apply a configuration to the mutliplexer.
	 *
	 * @param muxNum number to select, 1-16 inclusive.  For example, muxNum = 4 means pin S4A will be connected to pin DA, and S4B will be connected to pin DB.
	 * @param bankToChange which bank to set.  For example, if bank = A, the settings on bank B will remain unchanged.
	 * @param enable set to false to set "all switches off".  This parameter overrides the other parameters.
	 */
	void setMux(unsigned char muxNum, Bank bankToChange = Bank::BOTH, bool enable = true) {
		if(port.isOpen()) {
			char command = 0;
			if(enable) {
				if(bankToChange == Bank::NEITHER || bankToChange == Bank::B) {
					command |= COMMAND_CSA; // Setting the CSA# bit retains previous value for bank A
				}
				if(bankToChange == Bank::NEITHER || bankToChange == Bank::A) {
					command |= COMMAND_CSB; // Setting the CSB# bit retains previous value for bank B
				}
				command |= (COMMAND_A & muxNum);
			} else {
				command = COMMAND_EN; // Setting the EN# bit disables the device's switch connections
			}
			port.write(&command, 1);
		}
	}
	
private:
	// Bits in the command byte
	const unsigned char COMMAND_EN  = 0x80;
	const unsigned char COMMAND_CSA = 0x40;
	const unsigned char COMMAND_CSB = 0x20;
	const unsigned char COMMAND_A   = 0x0F;

	SpiDev & port; // will not be destructed at destruction of this Adg725
};

#endif // __ADG725_HPP__
