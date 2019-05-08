/*

A test application for parsing data from the Quanergy M8.

This is only for development.  Compile with:
  g++ -std=c++11  -I../include quanergyTestMain.cpp -o qtest

2017-06-22  JDW  Created

*/
#include <chrono>
#include <stdio.h>
#include <string>
#include <iostream>
#include <fstream>
#include <sys/time.h>
#include <unistd.h>
#include <string.h>
#include "quanergyMsgs.h"

using namespace std;
using namespace chrono;

// Entry point
int main(int argc, char ** argv)
{
	char buffer[M8PacketHeader::EXPECTED_SIZE_B];
	memset(buffer, 0, sizeof(buffer));
	
	while(cin.read(buffer, sizeof(buffer))) {
		// Parse
		M8Packet * pkt = (M8Packet *)(buffer);
		
		// Print parsed values
		printf("header.packetSignature = %08X = %d\n"      , pkt->header.getPacketSignature(), pkt->header.getPacketSignature());
		printf("header.messageSizeB    = %08X = %d\n"      , pkt->header.getMessageSizeB   (), pkt->header.getMessageSizeB   ());
		printf("header.timestampS      = %08X = %d\n"      , pkt->header.getTimestampS     (), pkt->header.getTimestampS     ());
		printf("header.timestampNs     = %08X = %d\n"      , pkt->header.getTimestampNs    (), pkt->header.getTimestampNs    ());
		printf("header.apiVerMajor     =       %02X = %d\n", pkt->header.getApiVerMajor    (), pkt->header.getApiVerMajor    ());
		printf("header.apiVerMinor     =       %02X = %d\n", pkt->header.getApiVerMinor    (), pkt->header.getApiVerMinor    ());
		printf("header.apiVerPatch     =       %02X = %d\n", pkt->header.getApiVerPatch    (), pkt->header.getApiVerPatch    ());
		printf("header.packetType      =       %02X = %d\n", pkt->header.getPacketType     (), pkt->header.getPacketType     ());
		printf("data.timestampS        = %08X = %d\n"      , pkt->data.getTimestampS (), pkt->data.getTimestampS ());
		printf("data.timestampNs       = %08X = %d\n"      , pkt->data.getTimestampNs(), pkt->data.getTimestampNs());
		printf("data.apiVer            =     %04X = %d\n"  , pkt->data.getApiVer     (), pkt->data.getApiVer     ());
		printf("data.status            =     %04X = %d\n"  , pkt->data.getStatus     (), pkt->data.getStatus     ());
		
		uint16_t oldpos = 0;
		for(int j = 0; j < 50; j++) {
			if(pkt->data.firingData[j].getPosition() != oldpos) {
				oldpos = pkt->data.firingData[j].getPosition();
				printf("firing[%02d].position =     %04X = %d\n", j , pkt->data.firingData[j].getPosition(), pkt->data.firingData[j].getPosition());
			}
			for(int i = 0; i < 8; i++) {
				if(pkt->data.firingData[j].getReturnIntensity(i) > 0 ||
					pkt->data.firingData[j].getReturnDistance(i) > 0) {
					printf("firing[%02d].returnsDistances  [%d] = %08X = %d\n"    ,j, i, pkt->data.firingData[j].getReturnDistance (i), pkt->data.firingData[j].getReturnDistance (i));
					printf("firing[%02d].returnsIntensities[%d] =     %04X = %d\n",j, i, pkt->data.firingData[j].getReturnIntensity(i), pkt->data.firingData[j].getReturnIntensity(i));
					printf("firing[%02d].returnsStatus     [%d] =     %04X = %d\n",j, i, pkt->data.firingData[j].getReturnStatus   (i), pkt->data.firingData[j].getReturnStatus   (i));
				}
			}
		}
		
		
		memset(buffer, 0, sizeof(buffer));
	}
}