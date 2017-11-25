#include "ros/ros.h"
#include "commons.h"
#include <stdio.h> //sprintf
#include <iostream>
#include <vector>
#include <sensor_msgs/Joy.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include </home/ubuntu/catkin_ws/devel/include/licht_controls/Lichtsetpoints.h>
#include </home/ubuntu/catkin_ws/devel/include/licht_controls/Lichtoutput.h>
#include </home/ubuntu/catkin_ws/devel/include/licht_controls/Lichtsetpointsraw.h>
#include </home/ubuntu/catkin_ws/devel/include/licht_controls/Lichtstate.h>
#include </home/ubuntu/catkin_ws/devel/include/licht_controls/Lichtcommands.h>
//#include <licht_coordination/rc.h>
//#include <licht_controls/Lichtyaw.h>
#include  "/home/ubuntu/catkin_ws/devel/include/licht_controls/Lichtyaw.h"
#define MAX_ATT_MANUEL 0.698f//11437 40deg,0.698rad
#define MAX_YAW_RATE_MANEUL 0.698f
#define YAWRATE_DEADZONE 0.17f
#define MAX_XY_RATE_MANEUL 1.5f
#define XY_RATE_DEADZONE 0.2f
#define MAX_Z_RATE_MANEUL 1.0f
#define Z_RATE_DEADZONE 0.2f
int g_vehicle_num=2;
int g_joy_num=2;
/*added by Wade*/
typedef enum place_e {
	PLACE_INDOOR = 0,
	PLACE_OUTDOOR
} flightplace_t;
typedef enum mode_e {
	MODE_RAW = 0,
	MODE_POS,
	MODE_TRJ
} flightmode_t;
typedef enum status_e {
	STATUS_LANDED = 0,
	STATUS_FLYING,
	STATUS_TAKINGOFF,
	STATUS_LANDING,
	STATUS_EMERGENCY
} flightstatus_t;
typedef struct joy_s
{
	bool curr_buttons[14];
	bool changed_buttons[14];
	float axes[4];
	float aux_axes[4];
	int curr_arrow[2];
	bool changed_arrow[2];
}joy_t;
typedef struct Setpoint_s
{
	licht_controls::Lichtsetpoints pos;
	licht_controls::Lichtsetpointsraw raw;
}setpoints_t;
class Commander
{
private:
	std::vector<ros::Publisher> vm_rawstptpub, vm_posstptpub;//, vm_rc_pub;
	ros::Publisher m_cmdpub;
	std::vector<ros::Subscriber> vm_joysub, vm_statesub, vm_yawsub;
	std::vector<ros::Subscriber> vm_state_from_linker_sub;
	flightmode_t m_mode;
	flightplace_t m_place;//added by Wade
	flightstatus_t m_status;
	std::vector<joy_t> vm_joy;
//	std::vector<joy_t> vm_rc;
	std::vector<setpoints_t> vm_sp;
	std::vector<licht_controls::Lichtstate> vm_state;
	std::vector<licht_controls::Lichtyaw> vm_yaw;// for initializing or resetting
	std::vector<licht_controls::Lichtstate> vm_state_from_linker;
	licht_controls::Lichtcommands m_cmd;
public:
	Commander(ros::NodeHandle& nh);
	~Commander();
	void run(double freq);
	void iteration(const ros::TimerEvent& e);
	void posspReset(int index);
	void yawspReset(int index);
	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy, int joy_index);
	void stateCallback(const licht_controls::Lichtstate::ConstPtr& msg, int vehicle_index);
	void yawCallback(const licht_controls::Lichtyaw::ConstPtr& msg, int vehicle_index);
	void state_from_linkerCallback(const licht_controls::Lichtstate::ConstPtr& msg, int vehicle_index);
};
Commander::Commander(ros::NodeHandle& nh)
:vm_rawstptpub(g_vehicle_num)
,vm_posstptpub(g_vehicle_num)
//,vm_rc_pub(g_joy_num)
,vm_joysub(g_joy_num)
,vm_statesub(g_vehicle_num)
,vm_yawsub(g_vehicle_num)
,vm_state_from_linker_sub(g_vehicle_num)
,vm_joy(g_joy_num)
,vm_sp(g_vehicle_num)
,vm_state(g_vehicle_num)
,vm_yaw(g_vehicle_num)
,vm_state_from_linker(g_vehicle_num)
{
	m_status = STATUS_LANDED;
	m_cmd.cut = 0;
	m_cmd.l_flight_state = STATUS_LANDED;
	char msg_name[50];
	m_cmdpub = nh.advertise<licht_controls::Lichtcommands>("commands", 1);
	for(int i=0;i<g_vehicle_num;i++){
		sprintf(msg_name,"/vehicle%d/raw_ctrl_sp",i);
		vm_rawstptpub[i] = nh.advertise<licht_controls::Lichtsetpointsraw>(msg_name, 1);
		sprintf(msg_name,"/vehicle%d/pos_ctrl_sp",i);
		vm_posstptpub[i] = nh.advertise<licht_controls::Lichtsetpoints>(msg_name, 1);
		sprintf(msg_name,"/vehicle%d/state_est",i);
		vm_statesub[i] = nh.subscribe<licht_controls::Lichtstate>(msg_name,5,boost::bind(&Commander::stateCallback, this, _1, i));
		sprintf(msg_name,"/vehicle%d/yaw",i);
		vm_yawsub[i] = nh.subscribe<licht_controls::Lichtyaw>(msg_name,5,boost::bind(&Commander::yawCallback, this, _1, i));
		sprintf(msg_name,"/vehicle%d/state_from_linker",i);
		vm_state_from_linker_sub[i] = nh.subscribe<licht_controls::Lichtstate>(msg_name,5,boost::bind(&Commander::state_from_linkerCallback, this, _1, i));
	}
	for(int i=0;i<g_joy_num;i++){
		sprintf(msg_name,"/joygroup%d/joy",i);
		vm_joysub[i] = nh.subscribe<sensor_msgs::Joy>(msg_name,5,boost::bind(&Commander::joyCallback, this, _1, i));
	}
}
Commander::~Commander()
{

}
void Commander::run(double freq)
{
	ros::NodeHandle node;
	int mode;
	node.getParam("/flight_mode", mode);
	switch(mode){
		case 0: m_mode = MODE_RAW;break;
		case 1: m_mode = MODE_POS;break;
		case 2: m_mode = MODE_TRJ;break;
		default:break;
	}
	/*added by Wade*/
	int place;
	node.getParam("/flight_place", place);
	switch(place){
		case 0: m_place = PLACE_INDOOR;break;
		case 1: m_place = PLACE_OUTDOOR;break;
		default:break;
	}
	ros::Timer timer = node.createTimer(ros::Duration(1.0/freq), &Commander::iteration, this);
	ros::spin();
}
void Commander::iteration(const ros::TimerEvent& e)
{
	static float time_elapse = 0;
	float dt = e.current_real.toSec() - e.last_real.toSec();
	time_elapse += dt;

	if (m_place)
	{
		if (vm_joy[0].changed_buttons[3] == true)
		{
			vm_joy[0].changed_buttons[3] = false;
			if(m_status == STATUS_FLYING || m_status == STATUS_TAKINGOFF ||m_status == STATUS_LANDING)
				m_status = STATUS_EMERGENCY;
			else if(m_status == STATUS_EMERGENCY)//disable emergyency
				m_status = STATUS_LANDED;
		}
		if(vm_joy[0].changed_arrow[1] == true && vm_joy[0].curr_arrow[1] == 1){//take off
			vm_joy[0].changed_arrow[1] = false;
			if(m_status == STATUS_LANDED){
				for(int i=0;i<g_vehicle_num;i++){
					posspReset(i);
					yawspReset(i);
					vm_sp[i].pos.pos_sp.z += 1.0f;
				}
				m_status = STATUS_TAKINGOFF;
			}
		}
		else if(vm_joy[0].changed_arrow[1] == true && vm_joy[0].curr_arrow[1] == -1){//land
			vm_joy[0].changed_arrow[1] = false;
			if(m_status == STATUS_FLYING || m_status == STATUS_TAKINGOFF)
				m_status = STATUS_LANDING;
		}

		switch(m_status){
			case STATUS_EMERGENCY:{
				for(int i=0;i<g_joy_num && i<g_vehicle_num;i++){
					/*emergyency procedure*/
					if (vm_sp[i].pos.commands == 2)
						vm_sp[i].pos.commands = 1;
					vm_posstptpub[i].publish(vm_sp[i].pos);
				}
			}
			break;
			case STATUS_LANDED:{
				for(int i=0;i<g_joy_num && i<g_vehicle_num;i++){
					// posspReset(i);//added by Wade
					// yawspReset(i);//added by Wade
					vm_sp[i].pos.commands = 1;//1:stop 2:move
					vm_posstptpub[i].publish(vm_sp[i].pos);
				}
				//all motors off
			}
			break;
			case STATUS_FLYING:{
				for(int i=0;i<g_joy_num && i<g_vehicle_num;i++){
					float pos_moverate[3];
					pos_moverate[0] = dead_zone_f(vm_joy[i].axes[1] * MAX_XY_RATE_MANEUL,XY_RATE_DEADZONE);
					pos_moverate[1] = -dead_zone_f(vm_joy[i].axes[0] * MAX_XY_RATE_MANEUL,XY_RATE_DEADZONE);
					pos_moverate[2] = -dead_zone_f(vm_joy[i].axes[2] * MAX_Z_RATE_MANEUL,Z_RATE_DEADZONE);
					vm_sp[i].pos.pos_sp.x += pos_moverate[0]*dt;
					vm_sp[i].pos.pos_sp.y += pos_moverate[1]*dt;
					vm_sp[i].pos.pos_sp.z += pos_moverate[2]*dt;
					vm_sp[i].pos.vel_ff.x = pos_moverate[0];
					vm_sp[i].pos.vel_ff.y = pos_moverate[1];
					vm_sp[i].pos.vel_ff.z = pos_moverate[2];
					float yaw_moverate = -dead_zone_f(vm_joy[i].axes[3] * MAX_YAW_RATE_MANEUL, YAWRATE_DEADZONE);//rate
					vm_sp[i].pos.yaw_sp += yaw_moverate *dt;
					vm_sp[i].pos.commands = 2;
					vm_posstptpub[i].publish(vm_sp[i].pos);
				}
			}
			break;
			case STATUS_TAKINGOFF:{
				for(int i=0;i<g_joy_num && i<g_vehicle_num;i++){

					vm_sp[i].pos.pos_sp.x += 0;
					vm_sp[i].pos.pos_sp.y += 0;
					vm_sp[i].pos.pos_sp.z -= 0.5*dt;
					vm_sp[i].pos.vel_ff.x = 0;
					vm_sp[i].pos.vel_ff.y = 0;
					vm_sp[i].pos.vel_ff.z = -0.5;
					vm_sp[i].pos.commands = 2;
					vm_posstptpub[i].publish(vm_sp[i].pos);
					if (vm_sp[i].pos.pos_sp.z <= -3.0)
						m_status = STATUS_FLYING;
				}
				//TODO judge if it is time to get into Automatic
			}
			break;
			case STATUS_LANDING:{
				for(int i=0;i<g_joy_num && i<g_vehicle_num;i++){

					vm_sp[i].pos.pos_sp.x += 0;
					vm_sp[i].pos.pos_sp.y += 0;
					vm_sp[i].pos.pos_sp.z += 0.2*dt;
					vm_sp[i].pos.vel_ff.x = 0;
					vm_sp[i].pos.vel_ff.y = 0;
					vm_sp[i].pos.vel_ff.z = 0.2;
					vm_sp[i].pos.commands = 2;
					vm_sp[i].pos.commands = 2;
					vm_posstptpub[i].publish(vm_sp[i].pos);
					if (vm_sp[i].pos.pos_sp.z >= 0.0)//how to judge whether it's landed or not
						m_status = STATUS_LANDED;
				}
				//TODO judge if it is time to get into Idle
			}
			break;
			default:
			break;
		}




		
	}
	else {
		//cut off
		if(vm_joy[0].curr_buttons[4] == 1 && vm_joy[0].curr_buttons[5] == 1){
			m_cmd.cut = 1;
		}
		else{
			switch(m_mode){
				case MODE_RAW:{
				//	static float yaw_sp;
					for(int i=0;i<g_joy_num && i<g_vehicle_num;i++){
						float throttle = vm_joy[i].axes[2];//0-1
						if(throttle<0){
							throttle=0;
						}
						if(throttle < 0.1){
							yawspReset(i);
							throttle=0;
						}
						vm_sp[i].raw.raw_att_sp.x = -vm_joy[i].axes[0] * MAX_ATT_MANUEL;//+-1
						vm_sp[i].raw.raw_att_sp.y = vm_joy[i].axes[1] * MAX_ATT_MANUEL;
						float yaw_moverate = dead_zone_f(vm_joy[i].axes[3] * MAX_YAW_RATE_MANEUL, YAWRATE_DEADZONE);//rate
						vm_sp[i].raw.raw_att_sp.z += yaw_moverate *dt;
										
						vm_sp[i].raw.thrust = VEHICLE_MASS * GRAVITY * throttle * 2.0f;
						vm_rawstptpub[i].publish(vm_sp[i].raw);
					}
				}
				break;
				case MODE_POS:{
					if(vm_joy[0].changed_arrow[1] == true && vm_joy[0].curr_arrow[1] == 1){//take off
						vm_joy[0].changed_arrow[1] = false;
						if(m_status == STATUS_LANDED){
							for(int i=0;i<g_vehicle_num;i++){
								posspReset(i);
								yawspReset(i);
							}
							m_status = STATUS_TAKINGOFF;
						}
					}
					else if(vm_joy[0].changed_arrow[1] == true && vm_joy[0].curr_arrow[1] == -1){//land
						vm_joy[0].changed_arrow[1] = false;
						if(m_status == STATUS_FLYING || m_status == STATUS_TAKINGOFF)
							m_status = STATUS_LANDING;
					}

					switch(m_status){
						case STATUS_LANDED:{
							for(int i=0;i<g_joy_num && i<g_vehicle_num;i++){
							}
							//all motors off
						}
						break;
						case STATUS_FLYING:{
							for(int i=0;i<g_joy_num && i<g_vehicle_num;i++){
								float pos_moverate[3];
								pos_moverate[0] = dead_zone_f(vm_joy[i].axes[0] * MAX_XY_RATE_MANEUL,XY_RATE_DEADZONE);
								pos_moverate[1] = dead_zone_f(vm_joy[i].axes[1] * MAX_XY_RATE_MANEUL,XY_RATE_DEADZONE);
								pos_moverate[2] = dead_zone_f(vm_joy[i].axes[3] * MAX_Z_RATE_MANEUL,Z_RATE_DEADZONE);
								vm_sp[i].pos.pos_sp.x += pos_moverate[0]*dt;
								vm_sp[i].pos.pos_sp.y += pos_moverate[1]*dt;
								vm_sp[i].pos.pos_sp.z += pos_moverate[2]*dt;
								vm_sp[i].pos.vel_ff.x = pos_moverate[0];
								vm_sp[i].pos.vel_ff.y = pos_moverate[1];
								vm_sp[i].pos.vel_ff.z = pos_moverate[2];
								float yaw_moverate = dead_zone_f(vm_joy[i].axes[3] * MAX_YAW_RATE_MANEUL, YAWRATE_DEADZONE);//rate
								vm_sp[i].pos.yaw_sp += yaw_moverate *dt;
								vm_sp[i].pos.commands = 1;//added by Wade
								vm_posstptpub[i].publish(vm_sp[i].pos);
							}
						}
						break;
						case STATUS_TAKINGOFF:{
							for(int i=0;i<g_joy_num && i<g_vehicle_num;i++){
							}
							//TODO judge if it is time to get into Automatic
						}
						break;
						case STATUS_LANDING:{
							for(int i=0;i<g_joy_num && i<g_vehicle_num;i++){
							}
							//TODO judge if it is time to get into Idle
						}
						break;
						default:
						break;
					}//end switch state
				}//end case posctrl mode
				break;
				case MODE_TRJ:{

				}
				break;
				default:
				break;
			}//end switch mode
		}//end of cut off case
	}

	
	m_cmd.flight_state = m_status;
	m_cmdpub.publish(m_cmd);
	m_cmd.l_flight_state = m_status;
}
void Commander::posspReset(int index)
{
	vm_sp[index].pos.pos_sp.x = vm_state_from_linker[index].pos_est.x;
	vm_sp[index].pos.pos_sp.y = vm_state_from_linker[index].pos_est.y;
	vm_sp[index].pos.pos_sp.z = vm_state_from_linker[index].pos_est.z;
	vm_sp[index].pos.vel_ff.x = 0;
	vm_sp[index].pos.vel_ff.y = 0;
	vm_sp[index].pos.vel_ff.z = 0;
	vm_sp[index].pos.acc_ff.x = 0;
	vm_sp[index].pos.acc_ff.y = 0;
	vm_sp[index].pos.acc_ff.z = 0;
}
void Commander::yawspReset(int index)
{
	vm_sp[index].pos.yaw_sp = vm_yaw[index].yaw;
	vm_sp[index].raw.raw_att_sp.z = vm_yaw[index].yaw;
}
void Commander::joyCallback(const sensor_msgs::Joy::ConstPtr& joy, int joy_index)
{
	//0 PosCtrl, 1 AttCtrl
	#define MAX_JOYS 5
	static bool l_buttons[MAX_JOYS][14];//at most 5 joysticks
	static int l_arrow[MAX_JOYS][2];
	for(int i=0;i<14;i++){
		vm_joy[joy_index].curr_buttons[i] = joy->buttons[i];
		if(vm_joy[joy_index].curr_buttons[i] != l_buttons[joy_index][i])
			vm_joy[joy_index].changed_buttons[i] = true;
		else
			;//changed_buttons cleared in iteration
	}
	for(int i=0;i<14;i++){
		l_buttons[joy_index][i] = vm_joy[joy_index].curr_buttons[i];
	}
	vm_joy[joy_index].axes[0] = joy->axes[2];//roll
	vm_joy[joy_index].axes[1] = joy->axes[5];//pitch
	vm_joy[joy_index].axes[2] = joy->axes[1];//yaw
	vm_joy[joy_index].axes[3] = joy->axes[0];//thr
	vm_joy[joy_index].aux_axes[0] = joy->axes[3];//left zeiger
	vm_joy[joy_index].aux_axes[1] = joy->axes[4];//right zeiger
	vm_joy[joy_index].aux_axes[2] = joy->axes[6];//left right button
	vm_joy[joy_index].aux_axes[3] = joy->axes[7];//up down button
	if(joy->axes[6]>0.5)//left and right button is one axes
		vm_joy[joy_index].curr_arrow[0] = 1;
	else if(joy->axes[6]<-0.5)
		vm_joy[joy_index].curr_arrow[0] = -1;
	else
		vm_joy[joy_index].curr_arrow[0] = 0;
	if(joy->axes[7]>0.5)//up and down button is one axes
		vm_joy[joy_index].curr_arrow[1] = 1;
	else if(joy->axes[7]<-0.5)
		vm_joy[joy_index].curr_arrow[1] = -1;
	else
		vm_joy[joy_index].curr_arrow[1] = 0;
	for(int i=0;i<2;i++){
		if(vm_joy[joy_index].curr_arrow[i] != l_arrow[joy_index][i])
			vm_joy[joy_index].changed_arrow[i] = true;
	}
	for(int i=0;i<2;i++){
		l_arrow[joy_index][i] = vm_joy[joy_index].curr_arrow[i];
	}
}
void Commander::stateCallback(const licht_controls::Lichtstate::ConstPtr& msg, int vehicle_index)
{
	vm_state[vehicle_index].pos_est.x = msg->pos_est.x;
	vm_state[vehicle_index].pos_est.y = msg->pos_est.y;
	vm_state[vehicle_index].pos_est.z = msg->pos_est.z;
	vm_state[vehicle_index].vel_est.x = msg->vel_est.x;
	vm_state[vehicle_index].vel_est.y = msg->vel_est.y;
	vm_state[vehicle_index].vel_est.z = msg->vel_est.z;
//	vm_state[vehicle_index].yaw_est = msg->yaw_est;
}
void Commander::state_from_linkerCallback(const licht_controls::Lichtstate::ConstPtr& msg, int vehicle_index)
{
	vm_state_from_linker[vehicle_index].pos_est.x = msg->pos_est.x;
	vm_state_from_linker[vehicle_index].pos_est.y = msg->pos_est.y;
	vm_state_from_linker[vehicle_index].pos_est.z = msg->pos_est.z;
	vm_state_from_linker[vehicle_index].vel_est.x = msg->vel_est.x;
	vm_state_from_linker[vehicle_index].vel_est.y = msg->vel_est.y;
	vm_state_from_linker[vehicle_index].vel_est.z = msg->vel_est.z;
//	vm_state[vehicle_index].yaw_est = msg->yaw_est;
}
void Commander::yawCallback(const licht_controls::Lichtyaw::ConstPtr& msg, int vehicle_index)
{
	vm_yaw[vehicle_index].yaw = msg->yaw;
}
int main(int argc, char **argv)
{
	ros::init(argc, argv, "commander");
	ros::NodeHandle n("~");
	n.getParam("/vehicle_num", g_vehicle_num);
	n.getParam("/joy_num", g_joy_num);
	Commander commander(n);
	commander.run(50);
	return 0;


}
