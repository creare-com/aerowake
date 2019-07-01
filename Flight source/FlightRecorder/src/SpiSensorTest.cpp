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
#include <sensors/Max6682.hpp>
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
	SpiDev probeSensorPort;
	DLHR_L01D probeSensor(probeSensorPort);
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
	// const unsigned int PROBE_SENSOR_PORT_HZ = 5000000;
	// const unsigned int ABS_SENSOR_PORT_HZ = 800000;
	const unsigned int MUX_PORT_HZ = 5000;
	const unsigned int PROBE_SENSOR_PORT_HZ = 5000;
	const unsigned int ABS_SENSOR_PORT_HZ = 5000;
	muxPort.openPort("/dev/spidev0.0", MUX_PORT_HZ);
	probeSensorPort.openPort("/dev/spidev0.1", PROBE_SENSOR_PORT_HZ);
	// Set up a version of this port that uses the same port, just a different clock rate
	SpiDev absSensorPort;
	absSensorPort.openPort(probeSensorPort.getFd(), ABS_SENSOR_PORT_HZ);
	DLV_030A absSensor(absSensorPort);
	// This device happens to use the same max clock rate as the probe sensors
	Max6682 thermistor(probeSensorPort);
	
	printf("Port speeds:\n");
	printf("    Probe sensors: %dHz\n", probeSensorPort.getClockRate());
	printf("    Abs. Press sensor: %dHz\n", absSensorPort.getClockRate());
	printf("    Multiplexer: %dHz\n", muxPort.getClockRate());
	
	// Take readings from the probe sensors
	const unsigned int MAX_WAIT_MILLIS = 500;
	DLHR_L01D::Reading probeReading[NUM_PROBE_CH];
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
			probeReading[ch] = probeSensor.retrieveReading();
		} else {
			printf("Timeout waiting for probe channel %d (1-indexed).\n", ch+1);
		}
	}
	
	printf("Probe channel readings: \n");
	printf("Pressure (inH2O): ");
	for(unsigned int ch; ch < NUM_PROBE_CH; ch++) {
		printf("%3.8f ", probeReading[ch].pressureInH20);
	}
	printf("\nTemperature(C): ");
	for(unsigned int ch; ch < NUM_PROBE_CH; ch++) {
		printf("%8.3f ", probeReading[ch].temperatureC);
	}
	printf("\n");

	// Take readings from the other sensors
	DLV_030A::Reading absReading = absSensor.retrieveReading();
	printf("Absolute pressure sensor reading: Pressure (PSI): %3.8f Temperature (C): %8.3f\n", absReading.pressurePsi, absReading.temperatureC);
	
	int Dout = thermistor.getAdcValue();
	printf("Thermistor ADC (Dout) value: %d\n", Dout);
}

