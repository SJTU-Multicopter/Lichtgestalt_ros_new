#include "Lichtbedienung.h"
#include "Lichtgestalt.h"
#include "XbeeAPI.h"
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
	uint8_t content_len = encode_cmd_acc(_sendBuf, q0,q1,q2,q3,thrust,ax,ay,az);
	api_tx_encode(_sendBuf, 0x00A21300, _addr_l);
	api_pack_encode(_sendBuf, content_len + 14);
	_radio->sendPacket(_sendBuf, content_len + 14 + 4);
}
void Lichtgestalt::sendTune(float pr_P,float pr_p,float pr_i,float pr_d,float y_P,float y_p,float y_i,float y_d)
{
	uint8_t content_len = encode_pid(_sendBuf, pr_P,pr_p,pr_i,pr_d,y_P,y_p,y_i,y_d);
	api_tx_encode(_sendBuf, 0x00A21300, _addr_l);
	api_pack_encode(_sendBuf, content_len + 14);
	_radio->sendPacket(_sendBuf, content_len + 14 + 4);
}

/*added by Wade*/
void Lichtgestalt::sendPosSp(float x,float y,float z,float vx,float vy,float vz,float ax,float ay,float az,float yaw, short em)
{

	uint8_t content_len = encode_pos_sp(_sendBuf, x,y,z,vx,vy,vz,ax,ay,az,yaw,em);
	api_tx_encode(_sendBuf, 0x00A21300, _addr_l);
	api_pack_encode(_sendBuf, content_len + 14);
	_radio->sendPacket(_sendBuf, content_len + 14 + 4);
}

void Lichtgestalt::acquireYaw(void)
{
	
}
uint32_t Lichtgestalt::getAddr(void)
{
	return _addr_l;
}