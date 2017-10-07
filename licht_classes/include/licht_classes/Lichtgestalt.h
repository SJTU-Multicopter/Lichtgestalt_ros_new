#ifndef LICHTGESTALT_H
#define LICHTGESTALT_H
#include <stdint.h>
#include <vector>

class Lichtradio;
class Lichtgestalt;

class Lichtgestalt
{
public:
	Lichtgestalt(uint32_t addr_l, uint32_t index_in_radio, Lichtradio* radio);
	~Lichtgestalt();
	void sendPosSp(float x,float y,float z,float vx,float vy,float vz,float ax,float ay,float az,float yaw,short em);//added by Wade
	void sendAck(void);
	void sendAll(float q0,float q1,float q2,float q3,float thrust,float ax,float ay,float az);
	void sendTune(float pr_P,float pr_p,float pr_i,float pr_d,float y_P,float y_p,float y_i,float y_d);
	void acquireYaw(void);
	uint32_t getAddr(void);
//private:
	Lichtradio* _radio;
	uint32_t _index_in_radio;
	uint32_t _addr_l;
	uint8_t _status;
	uint8_t _sendBuf[256];
};

#endif
