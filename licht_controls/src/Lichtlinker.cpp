#include "ros/ros.h"
#include <stdio.h> //sprintf
#include <iostream>
#include <vector>

#include <licht_controls/Lichtoutput.h>
#include <licht_controls/Lichtstate.h>

#include <licht_classes/Lichtgestalt.h>
#include <licht_classes/Lichtbedienung.h>
#include <boost/program_options.hpp>
int g_vehicle_num = 2;
int g_radio_num = 1;
class Linker
{
private:
//	std::vector<ros::Publisher> m_estpub_v;
	std::vector<ros::Subscriber> vm_estsub, vm_outputsub;
	std::vector<licht_controls::Lichtstate> vm_state;
	std::vector<licht_controls::Lichtoutput> vm_output;
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
{

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