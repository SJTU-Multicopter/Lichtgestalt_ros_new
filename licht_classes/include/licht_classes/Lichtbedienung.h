#ifndef LICHTBEDIENUNG_H
#define LICHTBEDIENUNG_H

#include <stdint.h>
#include <vector>

class Lichtradio;
class Lichtgestalt;
typedef std::vector<Lichtgestalt*> LichtgestaltList;
typedef std::vector<Lichtradio*> LichtradioList;

class Lichtradio
{
public:
	Lichtradio(uint32_t addr_l);
	~Lichtradio();
	int SerialOpen(const char *device, const int baud);
	uint32_t AddDestination(Lichtgestalt * lichtgestalt);
	//returns index of the destination
	void sendPacket(uint32_t destIndex, uint8_t * data, uint8_t len);
	uint32_t listSize(void);
private:
	uint32_t _addr_l;
//	char *device;
	int _fd;
	LichtgestaltList _LichtList;
};



#endif