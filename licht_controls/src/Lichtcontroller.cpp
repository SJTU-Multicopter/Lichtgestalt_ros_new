#include "ros/ros.h"
#include <stdio.h> //sprintf
#include <iostream>
#include <vector>
#include "commons.h"
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <licht_controls/Lichtsetpoints.h>
#include <licht_controls/Lichtoutput.h>
#include <licht_controls/Lichtsetpointsraw.h>
#include <licht_controls/Lichtstate.h>
#include <licht_controls/Lichtcommands.h>
#include <licht_controls/Lichtyaw.h>
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
typedef struct quaternion_s {
//  uint32_t timestamp;
	union {
		struct {
			float q0;
			float q1;
			float q2;
			float q3;
		};
		struct {
			float x;
			float y;
			float z;
			float w;
		};
		float q[4];
	};
} quaternion_t;

typedef struct rotation_s {
//  uint32_t timestamp;  // Timestamp when the data was computed
	float R[3][3];
} rotation_t;
void rotation2quaternion(const rotation_t* R, quaternion_t* Q)
{
	float q[4];
	float tr = R->R[0][0] + R->R[1][1] + R->R[2][2];
	if (tr > 0.0f) {
		float s = sqrtf(tr + 1.0f);
		q[0] = s * 0.5f;
		s = 0.5f / s;
		q[1] = (R->R[2][1] - R->R[1][2]) * s;
		q[2] = (R->R[0][2] - R->R[2][0]) * s;
		q[3] = (R->R[1][0] - R->R[0][1]) * s;
	} 
	else {
		/* Find maximum diagonal element in dcm
		* store index in dcm_i */
		int dcm_i = 0;
		for (int i = 1; i < 3; i++) {
			if (R->R[i][i] > R->R[dcm_i][dcm_i]) {
				dcm_i = i;
			}
		}
		int dcm_j = (dcm_i + 1) % 3;
		int dcm_k = (dcm_i + 2) % 3;
		float s = sqrtf((R->R[dcm_i][dcm_i] - R->R[dcm_j][dcm_j] - R->R[dcm_k][dcm_k]) + 1.0f);
		q[dcm_i + 1] = s * 0.5f;
		s = 0.5f / s;
		q[dcm_j + 1] = (R->R[dcm_i][dcm_j] + R->R[dcm_j][dcm_i]) * s;
		q[dcm_k + 1] = (R->R[dcm_k][dcm_i] + R->R[dcm_i][dcm_k]) * s;
		q[0] = (R->R[dcm_k][dcm_j] - R->R[dcm_j][dcm_k]) * s;
	}
	for(int i = 0; i < 4; i++)
		Q->q[i] = q[i];
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
	ros::Subscriber m_yawsub;
	ros::Subscriber m_rawstptsub, m_posstptsub;
	ros::Subscriber m_cmdsub;
	
	setpoints_t m_sp;
	Vector3f v_vel_sp, v_acc_sp;
	licht_controls::Lichtstate m_state;
	licht_controls::Lichtcommands m_cmd;
	licht_controls::Lichtoutput m_output;
	licht_controls::Lichtyaw m_yaw;
public:
	Controller(const ros::NodeHandle& n);
	~Controller();
	void run(double freq);
	void iteration(const ros::TimerEvent& e);
	void stateCallback(const licht_controls::Lichtstate::ConstPtr& msg);
	void stptrawCallback(const licht_controls::Lichtsetpointsraw::ConstPtr& msg);
	void stptposCallback(const licht_controls::Lichtsetpoints::ConstPtr& msg);
	void cmdCallback(const licht_controls::Lichtcommands::ConstPtr& msg);
	void yawCallback(const licht_controls::Lichtyaw::ConstPtr& msg);
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
	ros::NodeHandle nh("~");//~ means private param
	nh.getParam("ctrl_node_num", m_launchGroupIndex);


	char msg_name[50];

	sprintf(msg_name,"/vehicle%d/output", m_launchGroupIndex);
	m_outputpub = nh.advertise<licht_controls::Lichtoutput>(msg_name, 1);

	sprintf(msg_name,"/vehicle%d/state_est",m_launchGroupIndex);
	m_statesub = nh.subscribe<licht_controls::Lichtstate>(msg_name,5,&Controller::stateCallback, this);
//	m_statesub = nh.subscribe(msg_name,5,&Controller::stateCallback, this);
	sprintf(msg_name,"/vehicle%d/yaw",m_launchGroupIndex);
	m_yawsub = nh.subscribe<licht_controls::Lichtyaw>(msg_name,5,&Controller::yawCallback, this);
//	m_yawsub = nh.subscribe(msg_name,5,&Controller::yawCallback, this);
	sprintf(msg_name,"/vehicle%d/raw_ctrl_sp",m_launchGroupIndex);
	m_rawstptsub = nh.subscribe<licht_controls::Lichtsetpointsraw>(msg_name,5,&Controller::stptrawCallback, this);
//	m_rawstptsub = nh.subscribe(msg_name,5,&Controller::stptrawCallback, this);
	sprintf(msg_name,"/vehicle%d/pos_ctrl_sp",m_launchGroupIndex);
	m_posstptsub = nh.subscribe<licht_controls::Lichtsetpoints>(msg_name,5,&Controller::stptposCallback, this);
//	m_posstptsub = nh.subscribe(msg_name,5,&Controller::stptposCallback, this);

	m_cmdsub = nh.subscribe<licht_controls::Lichtcommands>("commands",5,&Controller::cmdCallback, this);
//	m_cmdsub = nh.subscribe("commands",5,&Controller::cmdCallback, this);
}
Controller::~Controller(){}
void Controller::run(double freq)
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
	ros::Timer timer = node.createTimer(ros::Duration(1.0/freq), &Controller::iteration, this);
	ros::spin();
}
void Controller::iteration(const ros::TimerEvent& e)
{
	static float time_elapse = 0;
	float dt = e.current_real.toSec() - e.last_real.toSec();
	time_elapse += dt;
	if(m_cmd.cut){
		m_output.thrust = 0;
	//	m_output.time_stamp = e.current_real.toSec();
		m_outputpub.publish(m_output);
	}
	else{
		switch(m_mode){
			case MODE_RAW:{
				float hr = m_sp.raw.raw_att_sp.x * 0.5f;
				float hp = m_sp.raw.raw_att_sp.y * 0.5f;
				float hy = m_sp.raw.raw_att_sp.z * 0.5f;
				float q0,q1,q2,q3;
				float shr = sin(hr);
				float chr = cos(hr);
				float shp = sin(hp);
				float chp = cos(hp);
				float shy = sin(hy);
				float chy = cos(hy);
				q0 = chr*chp*chy + shr*shp*shy;  
				q1 = shr*chp*chy - chr*shp*shy;    
				q2 = chr*shp*chy + shr*chp*shy; 
				q3 = chr*chp*shy - shr*shp*chy;
				m_output.q_sp[0] = q0;
				m_output.q_sp[1] = q1;
				m_output.q_sp[2] = q2;
				m_output.q_sp[3] = q3;
				m_output.thrust = m_sp.raw.thrust;
				m_outputpub.publish(m_output);
			}
			break;
			case MODE_POS:{
				v_vel_sp(0) = m_pidX.pp_update(m_state.pos_est.x, m_sp.pos.pos_sp.x);
				v_vel_sp(1) = m_pidY.pp_update(m_state.pos_est.y, m_sp.pos.pos_sp.y);
				v_vel_sp(2) = m_pidZ.pp_update(m_state.pos_est.z, m_sp.pos.pos_sp.z);
				v_vel_sp(0) += m_sp.pos.vel_ff.x * m_pidX.ff();
				v_vel_sp(1) += m_sp.pos.vel_ff.y * m_pidY.ff();
				v_vel_sp(2) += m_sp.pos.vel_ff.z * m_pidZ.ff();
				v_acc_sp(0) = m_pidX.pid_update(m_state.vel_est.x, v_vel_sp(0));
				v_acc_sp(1) = m_pidY.pid_update(m_state.vel_est.y, v_vel_sp(1));
				v_acc_sp(2) = m_pidZ.pid_update(m_state.vel_est.z, v_vel_sp(2));
				v_acc_sp(2) += GRAVITY;
				float thrust_force = v_acc_sp.norm() * (float)VEHICLE_MASS;
				Vector3f body_z_sp = v_acc_sp / v_acc_sp.norm();
				Vector3f y_c;
				y_c(0) = -sin(m_sp.pos.yaw_sp);
				y_c(1) = cos(m_sp.pos.yaw_sp);
				y_c(2) = 0;
				Vector3f body_x_sp = y_c.cross(body_z_sp);
				body_x_sp = body_x_sp / body_x_sp.norm();
				Vector3f body_y_sp = body_z_sp.cross(body_x_sp);
				rotation_t R_sp;
				quaternion_t Q_sp;
				for (int i = 0; i < 3; i++) {
					R_sp.R[i][0] = body_x_sp(i);
					R_sp.R[i][1] = body_y_sp(i);
					R_sp.R[i][2] = body_z_sp(i);
				}
				rotation2quaternion(&R_sp, &Q_sp);
				m_output.q_sp[0] = Q_sp.q0;
				m_output.q_sp[1] = Q_sp.q1;
				m_output.q_sp[2] = Q_sp.q2;
				m_output.q_sp[3] = Q_sp.q3;
				m_output.thrust = thrust_force;				
				m_outputpub.publish(m_output);
			}
			break;
			default:
			break;
		}
	}
}
void Controller::stateCallback(const licht_controls::Lichtstate::ConstPtr& msg)
{
	m_state.pos_est.x = msg->pos_est.x;
	m_state.pos_est.y = msg->pos_est.y;
	m_state.pos_est.z = msg->pos_est.z;
	m_state.vel_est.x = msg->vel_est.x;
	m_state.vel_est.y = msg->vel_est.y;
	m_state.vel_est.z = msg->vel_est.z;
}

void Controller::stptrawCallback(const licht_controls::Lichtsetpointsraw::ConstPtr& msg)
{
	m_sp.raw.raw_att_sp.x = msg->raw_att_sp.x;
	m_sp.raw.raw_att_sp.y = msg->raw_att_sp.y;
	m_sp.raw.raw_att_sp.z = msg->raw_att_sp.z;
	m_sp.raw.thrust = msg->thrust;
}
void Controller::stptposCallback(const licht_controls::Lichtsetpoints::ConstPtr& msg)
{
	m_sp.pos.pos_sp.x = msg->pos_sp.x;
	m_sp.pos.pos_sp.y = msg->pos_sp.y;
	m_sp.pos.pos_sp.z = msg->pos_sp.z;
	m_sp.pos.vel_ff.x = msg->vel_ff.x;
	m_sp.pos.vel_ff.y = msg->vel_ff.y;
	m_sp.pos.vel_ff.z = msg->vel_ff.z;
	m_sp.pos.yaw_sp = msg->yaw_sp;
}
void Controller::cmdCallback(const licht_controls::Lichtcommands::ConstPtr& msg)
{
	m_cmd.flight_state = msg->flight_state;
	m_cmd.l_flight_state = msg->l_flight_state;
	m_cmd.cut = msg->cut;
}
void Controller::yawCallback(const licht_controls::Lichtyaw::ConstPtr& msg)
{
	m_yaw.yaw = msg->yaw;
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
	controller.run(LINK_FREQ);
	return 0;


}