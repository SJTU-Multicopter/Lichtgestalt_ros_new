#include "Lichtbedienung.h"
#include "Lichtgestalt.h"
#include "XbeeAPI.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdarg.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>

#include "ros/ros.h"
#include "/home/wade/catkin_ws/devel/include/licht_controls/Lichtyaw.h"//receive yaw
Lichtradio::Lichtradio(uint32_t addr_l, const char device_name[30])
{
	_addr_l = addr_l;
	memcpy(_device_name, device_name,30);
}
Lichtradio::~Lichtradio()
{


}
int Lichtradio::SerialOpen(const int baud)
{
	char *device = _device_name;
	struct termios options ;
	speed_t myBaud ;
	int     status, fd ;
	switch (baud)
	{
		case     50:	myBaud =     B50 ; break ;
		case     75:	myBaud =     B75 ; break ;
		case    110:	myBaud =    B110 ; break ;
		case    134:	myBaud =    B134 ; break ;
		case    150:	myBaud =    B150 ; break ;
		case    200:	myBaud =    B200 ; break ;
		case    300:	myBaud =    B300 ; break ;
		case    600:	myBaud =    B600 ; break ;
		case   1200:	myBaud =   B1200 ; break ;
		case   1800:	myBaud =   B1800 ; break ;
		case   2400:	myBaud =   B2400 ; break ;
		case   4800:	myBaud =   B4800 ; break ;
		case   9600:	myBaud =   B9600 ; break ;
		case  19200:	myBaud =  B19200 ; break ;
		case  38400:	myBaud =  B38400 ; break ;
		case  57600:	myBaud =  B57600 ; break ;
		case 115200:	myBaud = B115200 ; break ;
		case 230400:	myBaud = B230400 ; break ;
		default:
			return -2 ;
	}
	printf("now open\n");
	if ((fd = open (device, O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK)) == -1)
		return -1 ;
	printf("now fcntl\n");
	fcntl (fd, F_SETFL, O_RDWR) ;
	// Get and modify current options:
	printf("now tcgetattr\n");
	tcgetattr (fd, &options) ;
	printf("now cfmakeraw\n");
    cfmakeraw   (&options) ;
    printf("now cfsetispeed\n");
    cfsetispeed (&options, myBaud) ;
    printf("now cfsetospeed\n");
    cfsetospeed (&options, myBaud) ;

    options.c_cflag |= (CLOCAL | CREAD) ;
    options.c_cflag &= ~PARENB ;
    options.c_cflag &= ~CSTOPB ;
    options.c_cflag &= ~CSIZE ;
    options.c_cflag |= CS8 ;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG) ;
    options.c_oflag &= ~OPOST ;

    options.c_cc [VMIN]  =   0 ;
    options.c_cc [VTIME] = 100 ;	// Ten seconds (100 deciseconds)
	printf("now tcsetattr\n");
	tcsetattr(fd, TCSANOW,&options);
	//  tcsetattr (fd, TCSANOW | TCSAFLUSH, &options) ;
	printf("now ioctl\n");
	ioctl (fd, TIOCMGET, &status);

	status |= TIOCM_DTR ;
	status |= TIOCM_RTS ;
	printf("now fcntl again\n");
	ioctl (fd, TIOCMSET, &status);
	printf("now finished\n");
	usleep (10000) ;	// 10mS
	_fd = fd;
	return fd ;
}
uint32_t Lichtradio::AddDestination(Lichtgestalt * lichtgestalt)
{
	uint32_t index = _LichtList.size();
	_LichtList.push_back(lichtgestalt);

	//returns index of the destination
	return index;
}
uint32_t Lichtradio::listSize(void)
{
	return _LichtList.size();
}
void Lichtradio::sendPacket(uint8_t * data, uint8_t len)
{
	write(_fd, data, len);
//	printf("package ");
//	fflush(stdout);
}
void Lichtradio::readBuf(std::vector<licht_controls::Lichtyaw> &v_yaw)
{
	int bufCnt;
	ioctl (_fd, FIONREAD, &bufCnt);
	read(_fd, _rcvBuf, bufCnt);
	int start_index = 0;
	for(int i=0;;i++){
		int pack_head;
		int pack_len;
		bool ret = buffer_cut(_rcvBuf, bufCnt, start_index, &pack_head, &pack_len);
		if (ret == false || start_index >= bufCnt)
			break;
		else{
			start_index += pack_len;
			//now process the package from _rcvBuf+pack_head to _rcvBuf+pack_head+pack_len
			unsigned int from_addr_l;
			float yaw;
			uint32_t lichtIndex;
			uint8_t api_id = api_pack_decode(_rcvBuf + pack_head, pack_len);
			switch(api_id){
				case API_ID_TX_REQ:{}//not for receiving
				break;
				case API_ID_TX_STATUS:{
					api_tx_status_decode(_rcvBuf + pack_head, pack_len);
				}
				break;
				case API_ID_RX_PACK:{
					uint8_t descriptor = api_rx_decode(_rcvBuf + pack_head, pack_len, &from_addr_l);
					lichtIndex = findIndex(from_addr_l);
					if(lichtIndex == -1){
						ROS_INFO("received from unregistered %x",from_addr_l);
						continue;
					}
					switch(descriptor){
						case DSCR_YAW:{
							decode_yaw(_rcvBuf + pack_head, pack_len, &yaw);
							v_yaw[lichtIndex].yaw = yaw;
						}
						break;
						case DSCR_ATT:{}
						break;
						case DSCR_GEN:{}
						break;
						case DSCR_PID:{
							ROS_INFO("received param from #%d",lichtIndex);
							decode_pid(_rcvBuf + pack_head, pack_len);

						}
						break;
						default:
						break;
					}//switch descriptor
				}//case rx_pack of api_id
				break;
				default:{}
				break;
			}//switch api_id
		}
	}
}

uint32_t Lichtradio::findIndex(uint32_t addr_l)
{
	for(int i=0;i<_LichtList.size();i++){
		if(_LichtList[i]->getAddr() == addr_l){

			return i;
		}
	}
	return -1;
}