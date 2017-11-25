#include "ros/ros.h"
#include "commons.h"
#include <stdio.h> //sprintf
#include <iostream>
#include <vector>
#include <sensor_msgs/Joy.h>
#include <licht_controls/Lichtoutput.h>//send att setpoint out
#include <licht_controls/Lichtstate.h>//send acc out
#include <licht_controls/Lichtcommands.h>
#include <licht_controls/Lichtyaw.h>//receive yaw
#include <licht_controls/Lichtsetpoints.h>
#include <licht_controls/data18.h>
#include <std_msgs/Int32.h>

#include <licht_classes/Lichtgestalt.h>
#include <licht_classes/Lichtbedienung.h>
#include <boost/program_options.hpp>
int g_vehicle_num = 2;
int g_radio_num = 2;
int g_joy_num=2;
/*added by Wade*/
typedef enum place_e {
	PLACE_INDOOR = 0,
	PLACE_OUTDOOR
} flightplace_t;
double get(const ros::NodeHandle& n, const std::string& name) 
{
	double value;
	n.getParam(name, value);
	return value;
}
typedef struct joy_s
{
	bool curr_buttons[14];
	bool changed_buttons[14];
	float axes[4];
	float aux_axes[4];
	int curr_arrow[2];
	bool changed_arrow[2];
}joy_t;
typedef struct rc_s
{
	float channels[10];
	short short_ch[10];
}rc_t;
class Linker
{
private:
	std::vector<ros::Publisher> vm_yawpub;
	std::vector<ros::Publisher> vm_data18pub;
	std::vector<ros::Subscriber> vm_statesub, vm_outputsub;
	std::vector<ros::Subscriber> vm_joysub;
	std::vector<ros::Publisher> vm_state_from_linker_pub;
	std::vector<joy_t> vm_joy;
	std::vector<rc_t> vm_rc;
	std::vector<ros::Subscriber> vm_outdoorsub;//added by Wade
	ros::Subscriber tunesub;
	std::vector<licht_controls::Lichtstate> vm_state;
	std::vector<licht_controls::Lichtstate> vm_state_from_linker;
	std::vector<licht_controls::Lichtoutput> vm_output;
	std::vector<licht_controls::Lichtyaw> vm_yaw;
	std::vector<licht_controls::data18> vm_data18;
	std::vector<Lichtgestalt> vm_vehicle;
	std::vector<Lichtradio> vm_radio;
	std::vector<licht_controls::Lichtsetpoints> vm_outdoor;//added by Wade
	int PID_rewrite;
	flightplace_t m_place;//added by Wade
public:
	Linker(ros::NodeHandle& nh);
	~Linker();
	void add_radio_vehicle(void);
//	void add_vehicle(void);
	void connect_radio(void);
	void connect_vehicle(void);
	void run(double freq);
	void iteration(const ros::TimerEvent& e);
	void outputCallback(const licht_controls::Lichtoutput::ConstPtr& msg, int vehicle_index);
	void stateCallback(const licht_controls::Lichtstate::ConstPtr&msg, int vehicle_index);
	void tuneCallback(const std_msgs::Int32::ConstPtr&msg);
	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy, int joy_index);
	void outdoorCallback(const licht_controls::Lichtsetpoints::ConstPtr& msg, int vehicle_index);//added by Wade
};
Linker::Linker(ros::NodeHandle& nh)
:vm_statesub(g_vehicle_num)
,vm_joysub(g_joy_num)
,vm_joy(g_joy_num)
,vm_rc(g_joy_num)
,vm_outputsub(g_vehicle_num)
,vm_yawpub(g_vehicle_num)
,vm_data18pub(g_vehicle_num)
,vm_state_from_linker_pub(g_vehicle_num)
,vm_state(g_vehicle_num)
,vm_output(g_vehicle_num)
,vm_yaw(g_vehicle_num)
,vm_state_from_linker(g_vehicle_num)
,vm_data18(g_vehicle_num)
,vm_outdoorsub(g_vehicle_num)//added by Wade
,vm_outdoor(g_vehicle_num)//added by Wade
//,vm_radio(g_radio_num)
//,vm_vehicle(g_vehicle_num)
{
	
	char msg_name[50];
	PID_rewrite = -1;
	for(int i=0;i<g_vehicle_num;i++){
		sprintf(msg_name,"/vehicle%d/state_est",i);
		vm_statesub[i] = nh.subscribe<licht_controls::Lichtstate>(msg_name,5,boost::bind(&Linker::stateCallback, this, _1, i));
		
		sprintf(msg_name,"/vehicle%d/output",i);
		vm_outputsub[i] = nh.subscribe<licht_controls::Lichtoutput>(msg_name,5,boost::bind(&Linker::outputCallback, this, _1, i));
		
		/*added by Wade*/
		sprintf(msg_name,"/vehicle%d/pos_ctrl_sp",i);
		vm_outdoorsub[i] = nh.subscribe<licht_controls::Lichtsetpoints>(msg_name,5,boost::bind(&Linker::outdoorCallback, this, _1, i));

		sprintf(msg_name,"/vehicle%d/yaw",i);
		vm_yawpub[i] = nh.advertise<licht_controls::Lichtyaw>(msg_name, 1);

		sprintf(msg_name,"/vehicle%d/data18",i);
		vm_data18pub[i] = nh.advertise<licht_controls::data18>(msg_name, 1);

		sprintf(msg_name,"/vehicle%d/state_from_linker",i);
		vm_state_from_linker_pub[i] = nh.advertise<licht_controls::Lichtstate>(msg_name, 1);
	}
	for(int i=0;i<g_joy_num;i++){
		sprintf(msg_name,"/joygroup%d/joy",i);
		vm_joysub[i] = nh.subscribe<sensor_msgs::Joy>(msg_name,5,boost::bind(&Linker::joyCallback, this, _1, i));
		vm_rc[i].channels[4] = -1;
		vm_rc[i].channels[2] = -1;
	}
//	ros::NodeHandle n;
	tunesub = nh.subscribe("/tune_index",1, &Linker::tuneCallback, this);
	for(int i=0;i<g_vehicle_num;i++){
		vm_state[i].acc_est.x = 0;
		vm_state[i].acc_est.y = 0;
		vm_state[i].acc_est.z = 0;
	}
}
Linker::~Linker()
{

}
void Linker::add_radio_vehicle(void)
{
	ros::NodeHandle n;
	char msg_name[50];
	for(int i=0;i<g_radio_num;i++){
		std::string addr_l_str, device_str;
		uint32_t addr_l;
		sprintf(msg_name,"/Lichtlinker/radio%d_addr_l",i);
		n.getParam(msg_name, addr_l_str);
		const char* addr_l_char = addr_l_str.c_str();
		sscanf(addr_l_char, "%x", &addr_l);

		sprintf(msg_name,"/radio_dev%d",i);
		n.getParam(msg_name, device_str);
		const char* device_char = device_str.c_str();
		Lichtradio radio(addr_l, device_char);
		vm_radio.push_back(radio);
//		vm_radio[i] = new Lichtradio(addr_l, device_char);
	}
	
	
	for(int i=0;i<g_vehicle_num;i++){
		int index_in_radio = 0;
		std::string addr_l_str;
		uint32_t addr_l;
		sprintf(msg_name,"/vehicle%d/vehicle_addr_l",i);
		n.getParam(msg_name, addr_l_str);
		const char* addr_l_char = addr_l_str.c_str();
		sscanf(addr_l_char, "%x", &addr_l);
		int radio_index;
		sprintf(msg_name,"/vehicle%d/radio_index",i);
		n.getParam(msg_name, radio_index);
		sprintf(msg_name,"/vehicle%d/index_in_radio",i);
		n.getParam(msg_name, index_in_radio);

		Lichtgestalt gestalt(addr_l, index_in_radio, &vm_radio[radio_index]);
//		vm_vehicle[i] = new Lichtgestalt(addr_l, index_in_radio, &vm_radio[radio_index]);
		vm_vehicle.push_back(gestalt);
		
	///	vm_radio[radio_index].AddDestination(&(vm_vehicle[i]));
		
		ROS_INFO("Lichtgestalt address %x added\n",vm_vehicle[i]._addr_l);
		ROS_INFO("Lichtgestalt index in radio %x\n",vm_vehicle[i]._index_in_radio);
		ROS_INFO("Lichtgestalt radio index %x\n",radio_index);
	}
	//push_back changes the address of all contents of the vector, so only after
	//all Lichtgestalten are pushed back, can the pointer be set 
	for(int j=0;j<g_radio_num;j++){
		for(int i=0;i<g_vehicle_num;i++){
			if(vm_vehicle[i]._radio == &vm_radio[j])
			vm_radio[j]._LichtList.push_back(&(vm_vehicle[i]));
		}
	}
//	ROS_INFO("Lichtradio address %x added\n",vm_radio[0]._LichtList[0]->getAddr());
//	ROS_INFO("Lichtradio address %x added\n",vm_radio[0]._LichtList[1]->getAddr());
//	ROS_INFO("Lichtradio index %x added\n",vm_radio[0]._LichtList[0]->_index_in_radio);
//	ROS_INFO("Lichtradio index %x added\n",vm_radio[0]._LichtList[1]->_index_in_radio);

}
void Linker::connect_radio(void)
{
	for(int i=0;i<g_radio_num;i++){
		vm_radio[i].SerialOpen(57600);
	}
}
void Linker::connect_vehicle(void)
{
	
}
void Linker::run(double freq)
{
	ros::NodeHandle node;
	/*added by Wade*/
	int place;
	node.getParam("/flight_place", place);
	switch(place){
		case 0: m_place = PLACE_INDOOR;break;
		case 1: m_place = PLACE_OUTDOOR;break;
		default:break;
	}
	ros::Timer timer = node.createTimer(ros::Duration(1.0/freq), &Linker::iteration, this);
	ros::spin();
}
void Linker::iteration(const ros::TimerEvent& e)
{
	static float time_elapse = 0;
	static int i = 0;
	float dt = e.current_real.toSec() - e.last_real.toSec();
	time_elapse += dt;
	if(PID_rewrite>=0){
		ros::NodeHandle n("~");
		for(int i=0;i<g_vehicle_num;i++){
			vm_vehicle[i].sendTune(
				get(n, "PIDs/PR/P"),
				get(n, "PIDs/PR/Prate"),
				get(n, "PIDs/PR/Irate"),
				get(n, "PIDs/PR/Drate"),
				get(n, "PIDs/Yaw/P"),
				get(n, "PIDs/Yaw/Prate"),
				get(n, "PIDs/Yaw/Irate"),
				get(n, "PIDs/Yaw/Drate"));
		}
		PID_rewrite = -1;
	}
	else{
//		if (m_place)
		// {
		// 	/*added by Wade*/
		// 	vm_vehicle[i].sendPosSp(
		// 		vm_outdoor[i].pos_sp.x,
		// 		vm_outdoor[i].pos_sp.y,
		// 		vm_outdoor[i].pos_sp.z,
		// 		vm_outdoor[i].vel_ff.x,
		// 		vm_outdoor[i].vel_ff.y,
		// 		vm_outdoor[i].vel_ff.z,
		// 		0.0,0.0,0.0,
		// 		vm_outdoor[i].yaw_sp,vm_outdoor[i].commands);
		// }
		// else {
			
		// 	vm_vehicle[i].sendAll(
		// 	vm_output[i].q_sp[0],
		// 	vm_output[i].q_sp[1],
		// 	vm_output[i].q_sp[2],
		// 	vm_output[i].q_sp[3],
		// 	vm_output[i].thrust*1000,
		// 	vm_state[i].acc_est.x,
		// 	vm_state[i].acc_est.y,
		// 	vm_state[i].acc_est.z);
		// }
		vm_vehicle[i].sendRC(vm_rc[i].short_ch);
		i++;
		if(i==g_vehicle_num)
			i=0;
	}

	//	usleep(1000000 / (2 * LINK_FREQ * g_vehicle_num));//send separately
//	}
//	std::vector<licht_controls::Lichtyaw*> yawList = &;
//	std::vector<float> yawList(g_vehicle_num);
	for(int i=0;i<g_radio_num;i++){
		vm_radio[i].readBuf(vm_yaw, vm_state_from_linker, vm_data18);
	}
	for(int i=0;i<g_vehicle_num;i++){
		vm_yawpub[i].publish(vm_yaw[i]);
		vm_data18pub[i].publish(vm_data18[i]);
		vm_state_from_linker_pub[i].publish(vm_state_from_linker[i]);
	}
}
void Linker::outputCallback(const licht_controls::Lichtoutput::ConstPtr& msg, int vehicle_index)
{
	vm_output[vehicle_index].q_sp[0] = msg->q_sp[0];
	vm_output[vehicle_index].q_sp[1] = msg->q_sp[1];
	vm_output[vehicle_index].q_sp[2] = msg->q_sp[2];
	vm_output[vehicle_index].q_sp[3] = msg->q_sp[3];
	vm_output[vehicle_index].thrust = msg->thrust;
}
void Linker::joyCallback(const sensor_msgs::Joy::ConstPtr& joy, int joy_index)
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

	for(int i=0;i<4;i++){
		vm_rc[joy_index].channels[i] = constrain_f(vm_joy[joy_index].axes[i]*1.41f, -1.0f, 1.0f);
		//joystick has round stick space
		//while RC has squared stick space
	}
	
	if(vm_joy[joy_index].changed_arrow[1] == true){
		vm_rc[joy_index].channels[4] += vm_joy[joy_index].curr_arrow[1];
		vm_rc[joy_index].channels[4] = constrain_f(vm_rc[joy_index].channels[4], -1.0f, 1.0f);
	}
	if(vm_joy[joy_index].changed_arrow[0] == true){
		vm_rc[joy_index].channels[5] -= vm_joy[joy_index].curr_arrow[0];
		vm_rc[joy_index].channels[5] = constrain_f(vm_rc[joy_index].channels[5], -1.0f, 1.0f);
	}
	if(joy->buttons[4]){
		vm_rc[joy_index].channels[0] = 1.0f;
		vm_rc[joy_index].channels[1] = -1.0f;
		vm_rc[joy_index].channels[2] = -1.0f;
		vm_rc[joy_index].channels[3] = -1.0f;
	}
	else if(joy->buttons[5]){
		vm_rc[joy_index].channels[0] = -1.0f;
		vm_rc[joy_index].channels[1] = -1.0f;
		vm_rc[joy_index].channels[2] = -1.0f;
		vm_rc[joy_index].channels[3] = 1.0f;
	}
	for(int i=0;i<10;i++){
		vm_rc[joy_index].short_ch[i] = vm_rc[joy_index].channels[i] * 1000;
	}
	printf("channels:\n%d\n%d\n%d\n%d\n%d\n%d\n", vm_rc[joy_index].short_ch[0], vm_rc[joy_index].short_ch[1], 
		vm_rc[joy_index].short_ch[2], vm_rc[joy_index].short_ch[3], vm_rc[joy_index].short_ch[4], vm_rc[joy_index].short_ch[5]);
}
void Linker::stateCallback(const licht_controls::Lichtstate::ConstPtr&msg, int vehicle_index)
{
	vm_state[vehicle_index].acc_est.x = msg->acc_est.x;
	vm_state[vehicle_index].acc_est.y = msg->acc_est.y;
	vm_state[vehicle_index].acc_est.z = msg->acc_est.z;
}
void Linker::tuneCallback(const std_msgs::Int32::ConstPtr&msg)
{
	PID_rewrite = msg->data;
	ROS_INFO("tuning");
}

/*added by Wade*/
void Linker::outdoorCallback(const licht_controls::Lichtsetpoints::ConstPtr& msg, int vehicle_index)
{
	vm_outdoor[vehicle_index].pos_sp.x = msg->pos_sp.x;
	vm_outdoor[vehicle_index].pos_sp.y = msg->pos_sp.y;
	vm_outdoor[vehicle_index].pos_sp.z = msg->pos_sp.z;
	vm_outdoor[vehicle_index].vel_ff.x = msg->vel_ff.x;
	vm_outdoor[vehicle_index].vel_ff.y = msg->vel_ff.y;
	vm_outdoor[vehicle_index].vel_ff.z = msg->vel_ff.z;
	vm_outdoor[vehicle_index].yaw_sp = msg->yaw_sp;
	vm_outdoor[vehicle_index].commands = msg->commands;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "linker");
	ros::NodeHandle n("~");
	n.getParam("/joy_num", g_joy_num);
	n.getParam("/vehicle_num", g_vehicle_num);
	n.getParam("/radio_num", g_radio_num);
	Linker linker(n);
	linker.add_radio_vehicle();
	linker.connect_radio();
	linker.connect_vehicle();
	linker.run(LINK_FREQ);
	return 0;
}