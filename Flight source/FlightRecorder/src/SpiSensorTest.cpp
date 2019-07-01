/*
	SpiSensorTest.cpp
	
	Class for testing SPI sensors without vision or camera libs
	
	2019-07-1	JDW	Created.
*/

#include <string>

#include <sensors/DlhrPressureSensor.hpp>
#include <sensors/DlvPressureSensor.hpp>
#include <Adg725.hpp>
#include <SpiDev.hpp>
#include <CsvLogger.hpp>


using namespace std;

int main() {
	SpiDev port;
	DLHR_L01D probeSensor(port);
	Adg725 mux(port);
	DLV_030A absSensor(port);
}

