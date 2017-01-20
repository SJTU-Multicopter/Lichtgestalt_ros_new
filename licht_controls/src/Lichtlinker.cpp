#include "ros/ros.h"
#include <stdio.h> //sprintf
#include <iostream>
#include <vector>

#include <licht_controls/Lichtoutput.h>
#include <licht_controls/Lichtstate.h>
#include <licht_controls/Lichtcommands.h>
#include <licht_controls/Lichtyaw.h>

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
//	std::vector<geometry_msgs::Vector3> m_lpos_v;
//	std::vector<ros::Time> m_lpos_time_v;
//	std::vector<std::string> m_defaultUri_v, m_uri_v;
	std::vector<Lichtgestalt> vm_vehicle;
	std::vector<Lichtradio> vm_radio;
public:
	Linker(ros::NodeHandle& nh);
	~Linker();
	uint32_t Connect(void);
	void add_vehicle(void);
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
}
Linker::~Linker()
{}
uint32_t Linker::Connect(void)
{

}
void Linker::add_vehicle(void)
{

}
void Linker::run(double freq)
{

}
void Linker::iteration(const ros::TimerEvent& e)
{

}
void Linker::outputCallback(const licht_controls::Lichtoutput::ConstPtr& msg, int vehicle_index)
{

}
void Linker::stateCallback(const licht_controls::Lichtstate::ConstPtr&msg, int vehicle_index)
{
	
}
int main(int argc, char **argv)
{
	ros::init(argc, argv, "linker");
	ros::NodeHandle n("~");
	n.getParam("/vehicle_num", g_vehicle_num);
	n.getParam("/radio_num", g_radio_num);
	Linker linker(n);
	uint32_t ret = linker.Connect();
	linker.add_vehicle();
	linker.run(50);
	return 0;


}