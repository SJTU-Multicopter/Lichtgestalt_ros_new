#include "XbeeAPI.h"
#include "string.h"
#include <stdio.h>
unsigned char api_pack_decode(unsigned char * data, unsigned int pack_len)
{
	unsigned char start_delimiter;
	unsigned char api_len;
	unsigned char zero = 0;
	unsigned char checksum = 0;
	unsigned char checksum_comming;
	unsigned char i = 0;

	unsigned char frame_type;
	memcpy(&start_delimiter, data, 1);
	memcpy(&zero, data + 1, 1);
	memcpy(&api_len, data + 2, 1);
	memcpy(&checksum_comming, data + pack_len - 1, 1);
	if(start_delimiter != API_START_DELIMITER)
		return 1;//none XBee API
	for(i=0;i<api_len;i++){
		checksum += *((unsigned char *)(data)+i+3);
	}
	checksum = 0xFF-checksum;
	if(checksum != checksum_comming)
		return 2;//checksum failed
	else
		memcpy(&frame_type, data + 3, 1);
	return frame_type;
}
void api_tx_status_decode(unsigned char * data, unsigned int pack_len)
{
	unsigned char frame_id, dest_addr_high, dest_addr_low, retry_cnt;
	unsigned char delivery_status, discovery_status;
	memcpy(&frame_id, data + 4, 1);
	memcpy(&dest_addr_high, data + 5, 1);
	memcpy(&dest_addr_low, data + 6, 1);
	memcpy(&retry_cnt, data + 7, 1);
	memcpy(&delivery_status, data + 8, 1);
	memcpy(&discovery_status, data + 9, 1);
}
unsigned char  api_rx_decode(unsigned char * data, unsigned int pack_len, unsigned int *from_addr_l)
{
	unsigned int src_address_high, src_address_low;
	unsigned char add0,add1,add2,add3;
	unsigned short src_network_address;
	unsigned char rcv_options;
	unsigned char descriptor;
	memcpy(&src_address_high, data + 4, 4);
	memcpy(&src_address_low, data + 8, 4);
//	memcpy(&add0, data + 8, 1);
//	memcpy(&add1, data + 9, 1);
//	memcpy(&add2, data + 10, 1);
//	memcpy(&add3, data + 11, 1);
	memcpy(&src_network_address, data + 12, 2);
	memcpy(&rcv_options, data + 14, 1);
	memcpy(&descriptor, data + 15, 1);
//	src_address_low = add3<<24|add2<<16|add1<<8|add0;
	*from_addr_l = src_address_low;
	return descriptor;
}
void decode_yaw(unsigned char * data, unsigned int pack_len, float * yaw)
{
	short yaw_sh;
	unsigned int timestamp;
	memcpy(&timestamp, data + 16, 4);
	memcpy(&yaw_sh, data + 20, 2);
	*yaw = (float)yaw_sh / YAW_F;
}
void decode_pos_yaw(unsigned char * data, unsigned int pack_len, float * yaw, float* x,float* y, float* z)
{
	short yaw_sh;
	int pos_i[3];
	unsigned int timestamp;
	memcpy(&timestamp, data + 16, 4);
	memcpy(&pos_i, data + 20, 12);
	memcpy(&yaw_sh, data + 32, 2);
	*yaw = (float)yaw_sh / YAW_F;
	*x = (float)pos_i[0] / 1000.0f;
	*y = (float)pos_i[1] / 1000.0f;
	*z = (float)pos_i[2] / 1000.0f;
}
void decode_pid(unsigned char * data, unsigned int pack_len)
{
//	[3E,0,api_len,90,	0-3
//	add_H,H,H,H,	4-7
//	add_L,L,L,L,	8-11
//	src_net,src_net,rcv_opt,descriptor 0x01	12-15
//	t,t,t,t,		16-19
//	P,P,P,P		20-23
//	p,p,p,p		24-27
//	i,i,i,i		28-31
//	d,d,d,d		32-35
//	P,P,P,P		36-39
//	p,p,p,p		40-43
//	i,i,i,i		44-47
//	d,d,d,d		48-51
//	checksum		52	
	float pr_P,pr_p,pr_i,pr_d;
	float y_P,y_p,y_i,y_d;
	unsigned int timestamp;
	memcpy(&timestamp, data + 16, 4);
	memcpy(&pr_P, data + 20, 4);
	memcpy(&pr_p, data + 24, 4);
	memcpy(&pr_i, data + 28, 4);
	memcpy(&pr_d, data + 32, 4);
	memcpy(&y_P, data + 36, 4);
	memcpy(&y_p, data + 40, 4);
	memcpy(&y_i, data + 44, 4);
	memcpy(&y_d, data + 48, 4);
	printf("Pitch and Roll:\n");
	printf("\tP: %f\n",pr_P);
	printf("\tPrate: %f\n",pr_p);
	printf("\tIrate: %f\n",pr_i);
	printf("\tDrate: %f\n",pr_d);
	printf("Yaw:\n");
	printf("\tP: %f\n",y_P);
	printf("\tPrate: %f\n",y_p);
	printf("\tIrate: %f\n",y_i);
	printf("\tDrate: %f\n",y_d);
	printf("check ok:\n");
}
void decode_general_18(unsigned char * data, unsigned int pack_len, short * data2receive)
{
//	short data2receive[18];
	unsigned int timestamp;
	memcpy(&timestamp, data + 16, 4);
	memcpy(data2receive, data+20, 36);
}
/*
void decode_cmd_acc(unsigned char * data, unsigned int pack_len, command_t* cmd, vec3f_t* mot_acc)
{
//	[3E,0,api_len,90,	0-3
//	add_H,H,H,H,	4-7
//	add_L,L,L,L,	8-11
//	src_net,src_net,rcv_opt,descriptor 0x01	12-15
//	t,t,t,t,		16-19
//	q0,q0,q1,q1		20-23
//	q2,q2,q3,q3		24-27
//	thr,thr,ax,ax	28-31
//	ay,ay,az,az		32-35
//	checksum		36	
	short q[4];
	short thrust;
	short acc[3];
	int i;
	unsigned int timestamp;
	memcpy(&timestamp, data + 16, 4);
	memcpy(q, data + 20, 8);
	memcpy(&thrust, data + 28, 2);
	memcpy(acc, data + 30, 6);
	cmd->Q.q0 = (float)q[0] / ATT_F;
	cmd->Q.q1 = (float)q[1] / ATT_F;
	cmd->Q.q2 = (float)q[2] / ATT_F;
	cmd->Q.q3 = (float)q[3] / ATT_F;
	cmd->thrust = (float)thrust / THR_F;
	for(i=0;i<3;i++)
		mot_acc->v[i] = (float)acc[i] / ACC_F;
}

void decode_calibrate(unsigned char * data, unsigned int pack_len, calib_t* cal)
{
//	[3E,0,api_len,90,	0-3
//	add_H,H,H,H,	4-7
//	add_L,L,L,L,	8-11
//	src_net,src_net,rcv_opt,descriptor 0x02	12-15
//	t,t,t,t,		16-19
//	ax,ax,ay,ay		20-23
//	az,az,gx,gx		24-27
//	gy,gy,gz,gz		28-31
//	mx,mx,my,my		32-35
//	mz,mz,checksum	36-38	
	int i;
	short acc[3], gyr[3],mag[3];
	unsigned int timestamp;
	memcpy(&timestamp, data + 16, 4);
	memcpy(acc, data + 20, 6);
	memcpy(gyr, data + 26, 6);
	memcpy(mag, data + 32, 6);
	for(i=0;i<3;i++){
		cal->acc_bias.v[i] = acc[i];
		cal->gyr_bias.v[i] = gyr[i];
		cal->mag_bias.v[i] = mag[i];
	}
}
*/
void api_pack_encode(unsigned char * data, unsigned char frame_len)
{
	unsigned char start_delimiter = API_START_DELIMITER;
	unsigned char zero = 0;
	unsigned char checksum = 0;
	unsigned char i = 0;
	for(i=0;i<frame_len;i++){
		checksum += *((unsigned char *)(data)+i+3);
	}
	checksum = 0xFF-checksum;
	memcpy(data,&start_delimiter,1);
	memcpy(data+1,&zero,1);
	memcpy(data+2,&frame_len,1);
	memcpy(data+3+frame_len,&checksum,1);
}
void api_tx_encode(unsigned char * data, unsigned int dest_addr_h, unsigned int dest_addr_l)
{
	unsigned char api_id = API_ID_TX_REQ;
	unsigned char frame_id = 0x22;
	unsigned char zero = 0;
	unsigned char unknown_16_addr_h = 0xFF;
	unsigned char unknown_16_addr_l = 0xFE;
	memcpy(data+3,&api_id,1);
	memcpy(data+4,&frame_id,1);
	memcpy(data+5,&dest_addr_h,4);
	memcpy(data+9,&dest_addr_l,4);
	memcpy(data+13,&unknown_16_addr_h,1);
	memcpy(data+14,&unknown_16_addr_l,1);
	memcpy(data+15,&zero,1);
	memcpy(data+16,&zero,1);
}
unsigned char encode_cmd_acc(unsigned char * data, float q0,float q1,float q2,float q3,float thrust,float ax,float ay,float az)
{
	unsigned char descriptor = DSCR_CMD_ACC;
	int i;
	short q[4], thrust_sh,a[3];
	unsigned int timestamp = 0;
	q[0] = q0 * ATT_F;
	q[1] = q1 * ATT_F;
	q[2] = q2 * ATT_F;
	q[3] = q3 * ATT_F;
	thrust_sh = thrust * THR_F;
	a[0] = ax * ACC_F * ACC_SENS;
	a[1] = ay * ACC_F * ACC_SENS;
	a[2] = az * ACC_F * ACC_SENS;
	memcpy(data + 17, &descriptor, 1);
	memcpy(data + 18, &timestamp, 4);
	memcpy(data + 22, q, 8);
	memcpy(data + 30, &thrust_sh, 2);
	memcpy(data + 32, a, 6);
	return 21;//length from desc to last data
}
unsigned char encode_pos_sp(unsigned char * data, float x,float y,float z,float vx,float vy,float vz,float ax,float ay,float az,float yaw,short em)
{
	unsigned char descriptor = DSCR_POSCTL;
	short vel[3], acc[3], yw;//, e;
	int pos[3];
	unsigned int timestamp = 0;
	pos[0] = x * 1000;
	pos[1] = y * 1000;
	pos[2] = z * 1000;
	vel[0] = vx * 1000;
	vel[1] = vy * 1000;
	vel[2] = vz * 1000;
	acc[0] = 0 * 1000;
	acc[1] = 0 * 1000;
	acc[2] = 0 * 1000;
	yw = yaw * YAW_F;
	memcpy(data + 17, &descriptor, 1);
	memcpy(data + 18, &timestamp, 4);
	memcpy(data + 22, pos, 12);
	memcpy(data + 34, vel, 6);
	memcpy(data + 40, acc, 6);
	memcpy(data + 46, &yw, 2);
	memcpy(data + 48, &em, 2);
	return 33;//length from desc to last data
}
unsigned char encode_pid(unsigned char * data, float pr_P,float pr_p,float pr_i,float pr_d,float y_P,float y_p,float y_i,float y_d)
{
	unsigned char descriptor = DSCR_CFG;
	unsigned int timestamp = 0;

	memcpy(data + 17, &descriptor, 1);
	memcpy(data + 18, &timestamp, 4);
	memcpy(data + 22, &pr_P, 4);
	memcpy(data + 26, &pr_p, 4);
	memcpy(data + 30, &pr_i, 4);
	memcpy(data + 34, &pr_d, 4);
	memcpy(data + 38, &y_P, 4);
	memcpy(data + 42, &y_p, 4);
	memcpy(data + 46, &y_i, 4);
	memcpy(data + 50, &y_d, 4);
	return 37;//length from desc to last data
}
bool buffer_cut(unsigned char * buf, int search_len, int start_index, int * pack_head, int * pack_len)
{
	for(int i = start_index; i < search_len; i++){
		if(buf[i] == API_START_DELIMITER){
			//this will skip API_ID_TX_STATUS because i+4 is 
			//occupied by frameID for API_ID_TX_STATUS
			if(buf[i+4] == 0x00 && buf[i+5] == 0x13 && buf[i+6] == 0xA2 && buf[i+7] == 0x00){
				*pack_head = i;
				int len = buf[i+2];
				*pack_len = len + 4;
				return true;
			}
		}
	}
	return false;
}
/*
unsigned char encode_sens_raw(unsigned char * data, const vec3i16_t* acc, const vec3i16_t* gyr,const vec3i16_t* mag)
{
	unsigned char descriptor = DSCR_SENS_RAW;
	int i;
	short a[3], g[3],m[3];
	unsigned int timestamp = xTaskGetTickCount();
	for(i=0;i<3;i++){
		a[i] = acc->v[i];
		g[i] = gyr->v[i];
		m[i] = mag->v[i];
	}
	memcpy(data + 17, &descriptor, 1);
	memcpy(data + 18, &timestamp, 4);
	memcpy(data + 22, a, 6);
	memcpy(data + 28, g, 6);
	memcpy(data + 34, m, 6);
	return 23;//length from desc to last data
}
unsigned char encode_sens(unsigned char * data, const marg_t * marg)
{
	unsigned char descriptor = DSCR_SENS;
	int i;
	short a[3], g[3],m[3];
	unsigned int timestamp = xTaskGetTickCount();
	for(i=0;i<3;i++){
		a[i] = marg->acc.v[i];
		g[i] = marg->gyr.v[i];
		m[i] = marg->mag.v[i];
	}
	memcpy(data + 17, &descriptor, 1);
	memcpy(data + 18, &timestamp, 4);
	memcpy(data + 22, a, 6);
	memcpy(data + 28, g, 6);
	memcpy(data + 34, m, 6);
	return 23;//length from desc to last data
}
unsigned char encode_att(unsigned char * data, const stateAtt_t* att)
{
	unsigned char descriptor = DSCR_ATT;
	short q[4];
	int i;
	unsigned int timestamp = xTaskGetTickCount();
	for(i=0;i<4;i++){
		q[i] = att->Q.q[i]*ATT_F;
	}
	memcpy(data + 17, &descriptor, 1);
	memcpy(data + 18, &timestamp, 4);
	memcpy(data + 22, q, 8);
	return 13;//length from desc to last data
}

unsigned char encode_general_18(unsigned char * data, const void * data2send)
{
	unsigned char descriptor = DSCR_GEN;
	unsigned int timestamp = 0;//xTaskGetTickCount();
	memcpy(data + 17, &descriptor, 1);
	memcpy(data + 18, &timestamp, 4);
	memcpy(data + 22, data2send, 18);
	return 23;//length from desc to last data
}
*/