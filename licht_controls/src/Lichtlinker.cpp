#include "ros/ros.h"
#include "commons.h"
#include <stdio.h> //sprintf
#include <iostream>
#include <vector>

#include <licht_controls/Lichtoutput.h>//send att setpoint out
#include <licht_controls/Lichtstate.h>//send acc out
#include <licht_controls/Lichtcommands.h>
#include <licht_controls/Lichtyaw.h>//receive yaw

#include <licht_classes/Lichtgestalt.h>
#include <licht_classes/Lichtbedienung.h>
#include <boost/program_options.hpp>
int g_vehicle_num = 2;
int g_radio_num = 2;

class Linker
{
private:
	std::vector<ros::Publisher> vm_yawpub;
	std::vector<ros::Subscriber> vm_statesub, vm_outputsub;
	std::vector<licht_controls::Lichtstate> vm_state;
	std::vector<licht_controls::Lichtoutput> vm_output;
	std::vector<licht_controls::Lichtyaw> vm_yaw;
	std::vector<Lichtgestalt> vm_vehicle;
	std::vector<Lichtradio> vm_radio;
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
};
Linker::Linker(ros::NodeHandle& nh)
:vm_statesub(g_vehicle_num)
,vm_outputsub(g_vehicle_num)
,vm_yawpub(g_vehicle_num)
,vm_state(g_vehicle_num)
,vm_output(g_vehicle_num)
,vm_yaw(g_vehicle_num)
//,vm_radio(g_radio_num)
//,vm_vehicle(g_vehicle_num)
{
	char msg_name[50];
	for(int i=0;i<g_vehicle_num;i++){
		sprintf(msg_name,"/vehicle%d/state_est",i);
		vm_statesub[i] = nh.subscribe<licht_controls::Lichtstate>(msg_name,5,boost::bind(&Linker::stateCallback, this, _1, i));
		
		sprintf(msg_name,"/vehicle%d/output",i);
		vm_outputsub[i] = nh.subscribe<licht_controls::Lichtoutput>(msg_name,5,boost::bind(&Linker::outputCallback, this, _1, i));
		
		sprintf(msg_name,"/vehicle%d/yaw",i);
		vm_yawpub[i] = nh.advertise<licht_controls::Lichtyaw>(msg_name, 1);
	}
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
	ROS_INFO("Lichtradio address %x added\n",vm_radio[0]._LichtList[0]->getAddr());
	ROS_INFO("Lichtradio address %x added\n",vm_radio[0]._LichtList[1]->getAddr());
	ROS_INFO("Lichtradio index %x added\n",vm_radio[0]._LichtList[0]->_index_in_radio);
	ROS_INFO("Lichtradio index %x added\n",vm_radio[0]._LichtList[1]->_index_in_radio);

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
	ros::Timer timer = node.createTimer(ros::Duration(1.0/freq), &Linker::iteration, this);
	ros::spin();
}
void Linker::iteration(const ros::TimerEvent& e)
{
	static float time_elapse = 0;
	float dt = e.current_real.toSec() - e.last_real.toSec();
	time_elapse += dt;
	for(int i=0;i<g_vehicle_num;i++){
		vm_vehicle[i].sendAll(
			vm_output[i].q_sp[0],
			vm_output[i].q_sp[1],
			vm_output[i].q_sp[2],
			vm_output[i].q_sp[3],
			vm_output[i].thrust,
			vm_state[i].acc_est.x,
			vm_state[i].acc_est.y,
			vm_state[i].acc_est.z);
		usleep(1000000 / (2 * LINK_FREQ * g_vehicle_num));//send separately
	}
//	std::vector<licht_controls::Lichtyaw*> yawList = &;
//	std::vector<float> yawList(g_vehicle_num);
	for(int i=0;i<g_radio_num;i++){
		vm_radio[i].readBuf(vm_yaw);
	}
	for(int i=0;i<g_vehicle_num;i++)
		vm_yawpub[i].publish(vm_yaw[i]);
}
void Linker::outputCallback(const licht_controls::Lichtoutput::ConstPtr& msg, int vehicle_index)
{
	vm_output[vehicle_index].q_sp[0] = msg->q_sp[0];
	vm_output[vehicle_index].q_sp[1] = msg->q_sp[1];
	vm_output[vehicle_index].q_sp[2] = msg->q_sp[2];
	vm_output[vehicle_index].q_sp[3] = msg->q_sp[3];
	vm_output[vehicle_index].thrust = msg->thrust;
}
void Linker::stateCallback(const licht_controls::Lichtstate::ConstPtr&msg, int vehicle_index)
{
	vm_state[vehicle_index].acc_est.x = msg->acc_est.x;
	vm_state[vehicle_index].acc_est.y = msg->acc_est.y;
	vm_state[vehicle_index].acc_est.z = msg->acc_est.z;
}
int main(int argc, char **argv)
{
	ros::init(argc, argv, "linker");
	ros::NodeHandle n("~");
	n.getParam("/vehicle_num", g_vehicle_num);
	n.getParam("/radio_num", g_radio_num);
	Linker linker(n);
	linker.add_radio_vehicle();
	linker.connect_radio();
	linker.connect_vehicle();
	linker.run(LINK_FREQ);
	return 0;
}