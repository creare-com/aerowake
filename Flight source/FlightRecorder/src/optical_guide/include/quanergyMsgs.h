/*

Classes to parse data from the Quanergy M8 LIDAE


*/

#ifndef __QUANERGY_MSGS_H__
#define __QUANERGY_MSGS_H__

#include <netinet/in.h>

const int M8_NUM_BEAMS = 8;
const int M8_NUM_VERT_LINES_PER_PKT = 50;

// From the sensor's datasheet.  Positive is away from the base.
// Coincidentally, in the UAV's references frame, positive is also away from
// the base of the unit, even though the unit is mounted upside down.
const double M8_VERTICAL_ANGLES_RAD[M8_NUM_BEAMS] = {
	-0.318505,
	-0.2692,
	-0.218009,
	-0.165195,
	-0.111003,
	-0.0557982,
	 0.0,
	 0.0557982
};

// NOTE: if you add virtual functions to this class, a hidden member void *__vptr will be added
// to the beginning of each class.  This is fine but you'll need to account for it when casting
// incoming buffers.
#pragma pack(push, 1)
class M8FiringData {
private:
	uint16_t position;
	uint16_t reserved;
	uint32_t returnsDistances  [3*M8_NUM_BEAMS]; // In 10micrometer ticks.  Row-major.  Final 2 returns always 0.
	uint8_t  returnsIntensities[3*M8_NUM_BEAMS]; // Row-major.  Final 2 returns always 0.
	uint8_t  returnsStatus     [  M8_NUM_BEAMS]; // Always 0
public:
	uint16_t getPosition       ()         const { return ntohs(position           ); }
	uint16_t getReserved       ()         const { return ntohs(reserved           ); }
	uint32_t getReturnDistance (size_t i) const { return ntohl(returnsDistances[i]); }
	uint8_t  getReturnIntensity(size_t i) const { return     returnsIntensities[i] ; }
	uint8_t  getReturnStatus   (size_t i) const { return     returnsStatus     [i] ; }

};

class M8PacketData {
public:
	M8FiringData firingData[M8_NUM_VERT_LINES_PER_PKT];
private:
	uint32_t timestampS;
	uint32_t timestampNs;
	uint16_t apiVer;
	uint16_t status;
public:
	uint32_t getTimestampS () const { return ntohl(timestampS );}
	uint32_t getTimestampNs() const { return ntohl(timestampNs);}
	uint16_t getApiVer     () const { return ntohs(apiVer     );}
	uint16_t getStatus     () const { return ntohs(status     );}
};

class M8PacketHeader {
public:
	static const uint32_t EXPECTED_SIZE_B = 6632;
	static const uint32_t EXPECTED_SIGNATURE = 0x75bd7e97;
private:
	uint32_t packetSignature;
	uint32_t messageSizeB;
	uint32_t timestampS;
	uint32_t timestampNs;
	uint8_t  apiVerMajor;
	uint8_t  apiVerMinor;
	uint8_t  apiVerPatch;
	uint8_t  packetType;
public:
	bool isSignatureValid() const { return getPacketSignature() == M8PacketHeader::EXPECTED_SIGNATURE; }
	bool isSizeValid()      const { return getMessageSizeB() == M8PacketHeader::EXPECTED_SIZE_B; }
	bool isMessageValid()   const { return isSignatureValid() && isSizeValid(); }
	
	uint32_t getPacketSignature() const { return ntohl(packetSignature);}
	uint32_t getMessageSizeB   () const { return ntohl(messageSizeB   );}
	uint32_t getTimestampS     () const { return ntohl(timestampS     );}
	uint32_t getTimestampNs    () const { return ntohl(timestampNs    );}
	uint8_t  getApiVerMajor    () const { return       apiVerMajor    ;}
	uint8_t  getApiVerMinor    () const { return       apiVerMinor    ;}
	uint8_t  getApiVerPatch    () const { return       apiVerPatch    ;}
	uint8_t  getPacketType     () const { return       packetType     ;}
	
};

class M8Packet {
public:
public:
	M8PacketHeader header;
	M8PacketData   data;
};
#pragma pack(pop)

#endif // __QUANERGY_MSGS_H__