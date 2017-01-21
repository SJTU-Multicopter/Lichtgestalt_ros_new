#ifndef LICHTBEDIENUNG_H
#define LICHTBEDIENUNG_H
#include <stdio.h>
#include <stdint.h>
#include <vector>

class Lichtradio;
class Lichtgestalt;
typedef std::vector<Lichtgestalt*> LichtgestaltList;
typedef std::vector<Lichtradio*> LichtradioList;

class Lichtradio
{
public:
	Lichtradio(uint32_t addr_l, const char device_name[30]);
	~Lichtradio();
	int SerialOpen(const int baud);
	uint32_t AddDestination(Lichtgestalt * lichtgestalt);
	//returns index of the destination
	void sendPacket(uint32_t destIndex, uint8_t * data, uint8_t len);
	void readPacket(uint32_t *destIndex, uint8_t * data, uint8_t len);
	uint32_t listSize(void);
private:
	uint32_t _addr_l;
	char _device_name[30];
	int _fd;
	LichtgestaltList _LichtList;
};



#endif