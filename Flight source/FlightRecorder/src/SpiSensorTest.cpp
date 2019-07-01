/*
	SpiSensorTest.cpp
	
	Class for testing SPI sensors without vision or camera libs
	
	2019-07-1	JDW	Created.
*/

#include <string>
#include <thread>
#include <chrono>

#include <sensors/DlhrPressureSensor.hpp>
#include <sensors/DlvPressureSensor.hpp>
#include <Adg725.hpp>
#include <SpiDev.hpp>
#include <CsvLogger.hpp>


using namespace std;

// Mux number for each sensor.
// Note that these are 1-indexed on the board, and 0-indexed in software.
enum MuxAssignment_e {
	PROBE_CH_1  =  0,
	PROBE_CH_2  =  1,
	PROBE_CH_3  =  2,
	PROBE_CH_4  =  3,
	PROBE_CH_5  =  4,
	PROBE_CH_6  =  5,
	PROBE_CH_7  =  6,
	PROBE_CH_8  =  7,
	PROBE_CH_9  =  8,
	PROBE_CH_10 =  9,
	PROBE_CH_11 = 10,
	PROBE_CH_12 = 11,
	ABS_PRESS   = 12,
	TEMPERATURE = 13
};

int main() {
	// Multiplexer dictates to which device the sensor port is routed
	SpiDev muxPort;
	Adg725 mux(muxPort);

	// Sensor port is routed by the mux
	SpiDev sensorPort;
	DLHR_L01D probeSensor(sensorPort);
	DLV_030A absSensor(sensorPort);
	const unsigned int NUM_PROBE_CH = 12;
	// mux port on which each probe channel sensor resides
	unsigned char probChMuxNum[NUM_PROBE_CH] = {
		PROBE_CH_1 ,
		PROBE_CH_2 ,
		PROBE_CH_3 ,
		PROBE_CH_4 ,
		PROBE_CH_5 ,
		PROBE_CH_6 ,
		PROBE_CH_7 ,
		PROBE_CH_8 ,
		PROBE_CH_9 ,
		PROBE_CH_10,
		PROBE_CH_11,
		PROBE_CH_12
	};
	
	// const unsigned int MUX_PORT_HZ = 30000000;
	// const unsigned int SENSOR_PORT_HZ = 5000000;
	const unsigned int MUX_PORT_HZ = 30000;
	const unsigned int SENSOR_PORT_HZ = 5000;
	muxPort.openPort("/dev/spidev0.0", MUX_PORT_HZ);
	sensorPort.openPort("/dev/spidev0.1", SENSOR_PORT_HZ);
	
	const unsigned int MAX_WAIT_MILLIS = 500;
	DLHR_L01D::Reading reading[NUM_PROBE_CH];
	for(unsigned int ch; ch < NUM_PROBE_CH; ch++) {
		mux.setMux(probChMuxNum[ch]);
		probeSensor.commandReading();
		for (unsigned int waitCt = 0; waitCt < MAX_WAIT_MILLIS; ++waitCt) {
			if(!probeSensor.isBusy()) {
				break;
			} else {
				this_thread::sleep_for(chrono::milliseconds(1));
			}
		}
		if(!probeSensor.isBusy()) {
			reading[ch] = probeSensor.retrieveReading();
		} else {
			printf("Timeout waiting for probe channel %d (1-indexed).\n", ch+1);
		}
	}
	
	printf("Probe channel readings: \n");
	printf("Pressure (inH2O): ");
	for(unsigned int ch; ch < NUM_PROBE_CH; ch++) {
		printf("%3.8f ", reading[ch].pressureInH20);
	}
	printf("\nTemperature(C): ");
	for(unsigned int ch; ch < NUM_PROBE_CH; ch++) {
		printf("%8.3f ", reading[ch].temperatureC);
	}
	printf("\n");
}

