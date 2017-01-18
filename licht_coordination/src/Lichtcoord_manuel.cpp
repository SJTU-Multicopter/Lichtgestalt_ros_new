#include "ros/ros.h"
#include <stdio.h> //sprintf
#include <iostream>
#include <vector>
#include <sensor_msgs/Joy.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <licht_controls/Lichtsetpoints.h>
#include <licht_controls/Lichtoutput.h>
#include <licht_controls/Lichtsetpointsraw.h>
#include <licht_controls/Lichtstate.h>
#include <licht_controls/Lichtcommands.h>
int g_vehicle_num=2;
int g_joy_num=2;
typedef enum mode_e {
	MODE_RAW = 0,
	MODE_POS,
	MODE_TRJ
} flightmode_t;
typedef enum status_e {
	STATUS_LANDED = 0,
	STATUS_FLYING,
	STATUS_TAKINGOFF,
	STATUS_LANDING
} flightstatus_t;
typedef struct joy_s
{
	bool curr_buttons[14];
	bool changed_buttons[14];
	float axes[4];
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
	std::vector<ros::Publisher> vm_rawstptpub, vm_posstptpub;
	ros::Publisher m_cmdpub;
	std::vector<ros::Subscriber> vm_joysub, vm_statesub;
	flightmode_t m_mode;
	flightstatus_t m_status;
	std::vector<joy_t> vm_joy;
	std::vector<setpoints_t> vm_sp;
	std::vector<licht_controls::Lichtstate> vm_state;
	licht_controls::Lichtcommands m_cmd;
public:
	Commander(ros::NodeHandle& nh);
	~Commander();
	void run(double freq);
	void iteration(const ros::TimerEvent& e);
	void posspReset(int index);
	void yawspReset(int index);
	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy, int joy_index);
	void stateCallback(const licht_controls::Lichtstate::ConstPtr& msg);

};
Commander::Commander(ros::NodeHandle& nh){}
Commander::~Commander(){}
void Commander::run(double freq){}
void Commander::iteration(const ros::TimerEvent& e){}
void Commander::posspReset(int index){}
void Commander::yawspReset(int index){}
void Commander::joyCallback(const sensor_msgs::Joy::ConstPtr& joy, int joy_index){}
void Commander::stateCallback(const licht_controls::Lichtstate::ConstPtr& msg){}
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
