#ifndef XBEE_API_H
#define XBEE_API_H

/*unsigned char data2send[17]=
	{0x10,//API_ID
	0x22,//FRAME_ID
	0x00,0x13,0xA2,0x00,0x41,0x4E,0x6D,0x61,//destination address
	0xFF,0xFE,//unknown 16-bit address
	0x00,0x00,//radius and options default 0
	0x41,0x42,0x38};//payload
*/
#define API_START_DELIMITER 0x7E

#define API_ID_TX_REQ 0x10
#define API_ID_TX_STATUS 0x8B
#define API_ID_RX_PACK 0x90



#define API_ID_AT_CMD 0x08
#define API_ID_AT_CMD_QUEUE 0x09
#define API_ID_EXPLICIT_ADD_CMD_FRAME 0x11
#define API_ID_REMOTE_CMD_REQ 0x17
#define API_ID_CREATE_SRC_ROUTE 0x21
#define API_ID_AT_CMD_RES 0x88
#define API_ID_MODEM_STATUS 0x8A
#define API_ID_EXPLICIT_RX_INDICATOR 0x91
#define API_ID_IO_DATA_SMPL_RX_INDICATOR 0x92
#define API_ID_SENSOR_READ_INDICATOR 0x94
#define API_ID_NODE_ID_INDICATOR 0x95
#define API_ID_REMOTE_CMD_RES 0x97
#define API_ID_EXTENDED_MODEM_STATUS 0x98
#define API_ID_OVER_THE_AIR 0xA0
#define API_ID_ROUTE_RECORD_INDICATOR 0xA1
#define API_ID_MANY_TO_ONE_ROUTE_REQ_INDICATOR 0xA3

#define TX_STATUS_SUCCESS 0x00
#define TX_STATUS_MAC_ACK_FAIL 0x01
#define TX_STATUS_CCA_FAIL 0x02
#define TX_STATUS_INVALID_DEST_ENDPOINT 0x15
#define TX_STATUS_NETWORK_ACK_FAIL 0x21
#define TX_STATUS_NOT_JOINED_TO_NETWORK 0x22
#define TX_STATUS_SELF_ADDRESSED 0x23
#define TX_STATUS_ADD_NOT_FOUND 0x24
#define TX_STATUS_ROUTE_NOT_FOUND 0x25

#define TX_DISCOVERY_STATUS_NO_DISC 0x00
#define TX_DISCOVERY_STATUS_ADD_DISC 0x01
#define TX_DISCOVERY_STATUS_ROUTE_DISC 0x02
#define TX_DISCOVERY_STATUS_ADD_AND_ROUTE 0x03
#define TX_DISCOVERY_STATUS_EXTENDED_TIMEOUT_DISC 0x40

#define RX_STATUS_PACK_ACK 0x01
#define RX_STATUS_PACK_BROADCAST 0x02
#define RX_STATUS_PACK_ENCRYPT 0x20 
//see datasheet P174
#define DSCR_CMD_ACC 0x01
#define DSCR_CAL 0x02
#define DSCR_CFG 0x03
#define DSCR_POSCTL 0x04

#define DSCR_SENS_RAW 0x11
#define DSCR_SENS 0x12
#define DSCR_ATT 0x13
#define DSCR_GEN 0x14
#define DSCR_YAW 0x15
#define DSCR_PID 0x16
#define DSCR_POSYAW 0x17

unsigned char api_pack_decode(unsigned char * data, unsigned int pack_len);
void api_tx_status_decode(unsigned char * data, unsigned int pack_len);
unsigned char  api_rx_decode(unsigned char * data, unsigned int pack_len, unsigned int *from_addr_l);
void decode_yaw(unsigned char * data, unsigned int pack_len, float * yaw);
void decode_pid(unsigned char * data, unsigned int pack_len);
void decode_general_18(unsigned char * data, unsigned int pack_len, short * data2receive);
void decode_pos_yaw(unsigned char * data, unsigned int pack_len, float * yaw, float* x,float* y, float* z);


/*
void decode_cmd_acc(unsigned char * data, unsigned int pack_len, command_t* cmd, vec3f_t* mot_acc);
void decode_calibrate(unsigned char * data, unsigned int pack_len, calib_t* cal);
*/

void api_pack_encode(unsigned char * data, unsigned char content_len);
void api_tx_encode(unsigned char * data, unsigned int dest_addr_h, unsigned int dest_addr_l);
unsigned char encode_cmd_acc(unsigned char * data, float q0,float q1,float q2,float q3,float thrust,float ax,float ay,float az);
unsigned char encode_pos_sp(unsigned char * data, float x,float y,float z,float vx,float vy,float vz,float ax,float ay,float az,float yaw,short em);
unsigned char encode_pid(unsigned char * data, float pr_P,float pr_p,float pr_i,float pr_d,float y_P,float y_p,float y_i,float y_d);
/*
unsigned char encode_sens_raw(unsigned char * data, const vec3i16_t* acc, const vec3i16_t* gyr,const vec3i16_t* mag);
unsigned char encode_att(unsigned char * data, const stateAtt_t* att);
unsigned char encode_general_18(unsigned char * data, const void * data2send);
unsigned char encode_sens(unsigned char * data, const marg_t * marg);
*/
/*
given the length of the whole area to search, from 0 on
given the start index
return the pointer to start-deliminators 0x3E of the first package after start_index
return the lengths of the first package after start_index
return true if any package has been found
method: identify 0x3E as well as addr_h
*/
bool buffer_cut(unsigned char * buf, int search_len, int start_index, int * pack_head, int * pack_len);
#define ATT_Q 14
#define ATT_F 16384.0f
#define YAW_Q 12
#define YAW_F 4096.0f
#define THR_Q 0
#define THR_F 1.0f
#define ACC_Q 0
#define ACC_F 1.0f
#define ACC_SENS (8192.0f/9.81f)
#endif
