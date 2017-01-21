#include "Lichtbedienung.h"
#include "Lichtgestalt.h"
Lichtgestalt::Lichtgestalt(uint32_t addr_l, uint32_t index_in_radio, Lichtradio* radio)
{
	_radio = radio;
	_index_in_radio = index_in_radio;
	_addr_l = addr_l;
}
Lichtgestalt::~Lichtgestalt()
{

}
void Lichtgestalt::sendAck(void)
{

}
void Lichtgestalt::sendAll(float q0,float q1,float q2,float q3,float thrust,float ax,float ay,float az)
{
	
}
void Lichtgestalt::acquireYaw(void)
{
	
}
uint32_t Lichtgestalt::getAddr(void)
{
	return _addr_l;
}