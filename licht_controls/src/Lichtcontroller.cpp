#include "ros/ros.h"
#include <stdio.h> //sprintf
#include <iostream>
#include <vector>

#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <licht_controls/Lichtsetpoints.h>
#include <licht_controls/Lichtoutput.h>
#include <licht_controls/Lichtsetpointsraw.h>
#include <licht_controls/Lichtstate.h>
#include <licht_controls/Lichtcommands.h>

#include <math.h>
#include "Eigen/Eigen/Eigen"
#include "Eigen/Eigen/Geometry"
using namespace Eigen;
//int g_vehicle_num=2;
//int g_joy_num=2;
double get(
    const ros::NodeHandle& n,
    const std::string& name) {
    double value;
    n.getParam(name, value);
    return value;
}
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
typedef struct Setpoint_s
{
	licht_controls::Lichtsetpoints pos;
	licht_controls::Lichtsetpointsraw raw;
}setpoints_t;
class PID
{
private:
	float m_kp;
	float m_kd;
	float m_ki;
	float m_kpp;
	float m_ff;
	float m_minOutput;
	float m_maxOutput;
	float m_integratorMin;
	float m_integratorMax;
	float m_integral;
	float m_previousError;
	ros::Time m_previousTime;
public:
	PID(
		float kp,
		float kd,
		float ki,
		float kpp,
		float ff,
		float minOutput,
		float maxOutput,
		float integratorMin,
		float integratorMax)
		: m_kp(kp)
		, m_kd(kd)
		, m_ki(ki)
		, m_kpp(kpp)
		, m_ff(ff)
		, m_minOutput(minOutput)
		, m_maxOutput(maxOutput)
		, m_integratorMin(integratorMin)
		, m_integratorMax(integratorMax)
		, m_integral(0)
		, m_previousError(0)
		, m_previousTime(ros::Time::now())
	{
	}

	void reset()
	{
		m_integral = 0;
		m_previousError = 0;
		m_previousTime = ros::Time::now();
	}

	void setIntegral(float integral)
	{
		m_integral = integral;
	}

	float ki() const
	{
		return m_ki;
	}
	float ff() const
	{
		return m_ff;
	}
	float pid_update(float est, float setpt)
	{
		ros::Time time = ros::Time::now();
		float dt = time.toSec() - m_previousTime.toSec();
		float error = setpt - est;
		m_integral += error * dt;
		m_integral = std::max(std::min(m_integral, m_integratorMax), m_integratorMin);
		float p = m_kp * error;
		float d = 0;
		if (dt > 0){
			d = m_kd * (error - m_previousError) / dt;
		}
		float i = m_ki * m_integral;
		float output = p + d + i;
		m_previousError = error;
		m_previousTime = time;
		return std::max(std::min(output, m_maxOutput), m_minOutput);
	}
	float pp_update(float est, float setpt)
	{
		float error = setpt - est;
		float output = m_kpp * error;
		return output;
	}
};

class Controller
{
	private:
	int m_launchGroupIndex;

	PID m_pidX;
	PID m_pidY;
	PID m_pidZ;
	flightmode_t m_mode;
	flightstatus_t m_status;
	ros::Publisher m_outputpub;
	ros::Subscriber m_statesub;
	ros::Subscriber m_rawstptsub, m_posstptsub;
	ros::Subscriber m_cmdsub;
	
	setpoints_t m_sp;
	Vector3f v_vel_sp, v_acc_sp;
	licht_controls::Lichtstate m_state;
	licht_controls::Lichtcommands m_cmd;
	licht_controls::Lichtoutput m_output;
public:
	Controller(const ros::NodeHandle& n);
	~Controller();
	void run(double freq);
	void iteration(const ros::TimerEvent& e);
	void stateCallback(const licht_controls::Lichtstate::ConstPtr& msg);
	void stptrawCallback(const licht_controls::Lichtstate::ConstPtr& msg);
	void stptposCallback(const licht_controls::Lichtstate::ConstPtr& msg);
	void cmdCallback(const licht_controls::Lichtstate::ConstPtr& msg);
};
Controller::Controller(const ros::NodeHandle& n)
	:m_pidX(
			get(n, "PIDs/X/kp"),
			get(n, "PIDs/X/kd"),
			get(n, "PIDs/X/ki"),
			get(n, "PIDs/X/kpp"),
			get(n, "PIDs/X/ff"),
			get(n, "PIDs/X/minOutput"),
			get(n, "PIDs/X/maxOutput"),
			get(n, "PIDs/X/integratorMin"),
			get(n, "PIDs/X/integratorMax"))
	,m_pidY(
			get(n, "PIDs/Y/kp"),
			get(n, "PIDs/Y/kd"),
			get(n, "PIDs/Y/ki"),
			get(n, "PIDs/Y/kpp"),
			get(n, "PIDs/Y/ff"),
			get(n, "PIDs/Y/minOutput"),
			get(n, "PIDs/Y/maxOutput"),
			get(n, "PIDs/Y/integratorMin"),
			get(n, "PIDs/Y/integratorMax"))
	,m_pidZ(
			get(n, "PIDs/Z/kp"),
			get(n, "PIDs/Z/kd"),
			get(n, "PIDs/Z/ki"),
			get(n, "PIDs/Z/kpp"),
			get(n, "PIDs/Z/ff"),
			get(n, "PIDs/Z/minOutput"),
			get(n, "PIDs/Z/maxOutput"),
			get(n, "PIDs/Z/integratorMin"),
			get(n, "PIDs/Z/integratorMax"))
{

}
Controller::~Controller(){}
void Controller::run(double freq)
{

}
void Controller::iteration(const ros::TimerEvent& e)
{

}
void Controller::stateCallback(const licht_controls::Lichtstate::ConstPtr& msg)
{

}
void Controller::stptrawCallback(const licht_controls::Lichtstate::ConstPtr& msg)
{

}
void Controller::stptposCallback(const licht_controls::Lichtstate::ConstPtr& msg)
{

}
void Controller::cmdCallback(const licht_controls::Lichtstate::ConstPtr& msg)
{

}
int main(int argc, char **argv)
{
	char indexc = *argv[1];
	int index = indexc;
	char node_name[50];
	sprintf(node_name, "controller%d", index);
	ros::init(argc, argv, node_name);
	ros::NodeHandle n("~");
//	n.getParam("/vehicle_num", g_vehicle_num);
//	n.getParam("/joy_num", g_joy_num);
	Controller controller(n);
	controller.run(50);
	return 0;


}