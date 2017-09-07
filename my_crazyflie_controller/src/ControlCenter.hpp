/*
 * ThreeAgentCenter.hpp
 *
 *  Created on: 2017年8月25日
 *      Author: zbf
 */

#ifndef CRAZYFLIE_ECUST_MY_CRAZYFLIE_CONTROLLER_SRC_CONTROLCENTER_HPP_
#define CRAZYFLIE_ECUST_MY_CRAZYFLIE_CONTROLLER_SRC_CONTROLCENTER_HPP_
#include "my_crazyflie_controller/ThreeAgentMsg.h"
#include "my_crazyflie_controller/AgentU.h"
#include <Eigen/Dense>
#include "common.hpp"
#include "sensor_msgs/Joy.h"
// 这次写的是真正的中心节点
// 中心节点需要订阅到各种信息。
// 然后计算出u
// 计算出之后，再吧相应的控制权交给各个飞机

//订阅各个飞机太麻烦了
//所以等每个飞机把相应的信息整理好之后再给中心节点吧
//所以就自定义了个消息



using namespace Eigen;

//一起起飞点这个，否则注释掉就好
#define FLY_TOGETHER
//这个类的作用只是用来计算u的
//给出控制指令交给别的类来做

/**
 * 中心节点来发送目标点的消息吧
 * 不再扩展第三个中心节点了,吗？
 * 为了可扩展性还是再写第三个节点吧
 * 另外，考虑兼容，也在这个函数里面留下后门吧
 */
class ControlCenter{

public:
	ControlCenter(int topologies[USE_AGENT][USE_AGENT], float frequency,
			int frame_id[USE_AGENT]) :
			state(zbf::Idle), srvCalled(0) {
		ros::NodeHandle nh;
		int i;
		log = new LogUtils();
		std::string str_u;
		std::string str_msg;
		this->topologies = TopoMatrix::Zero();
		zbf::eigen_topologies(this->topologies,topologies);
		this->frequency = frequency;
		u = PosMatrix::Zero();
		pos = PosMatrix::Zero();
		posGoal = PosMatrix::Zero();
		vel = PosMatrix::Zero();
		velGoal = PosMatrix::Zero();
		init_frame_id(frame_id);
		show_frame_ids();
		for(i=0;i<USE_AGENT;i++){
			//这个类需要订阅这样的消息：
			//pos, vel的消息是每个飞机提供的
			posSub[i] = nh.subscribe(zbf::agent_str_pos(i), 10,
					&ControlCenter::receive_pos, this);
			velSub[i] = nh.subscribe(zbf::agent_str_vel(i), 10,
					&ControlCenter::receive_vel, this);
			posGoalSub[i] = nh.subscribe(zbf::center_str_pos_goal(i), 10,
					&ControlCenter::receive_pos_goal, this);
			velGoalSub[i] = nh.subscribe(zbf::center_str_vel_goal(i), 10,
					&ControlCenter::receive_vel_goal, this);
			//这个类提供这样的服务：
			uSrv[i] = nh.advertiseService(zbf::str_u(i),
					&ControlCenter::send, this);
			ROS_INFO("wait for agent%d emergency service", i);
			if (ros::service::waitForService(zbf::emergency(i),
					ros::Duration(5.0))) {
				emergencySrv[i] = nh.serviceClient<
						std_srvs::Empty>(
					zbf::emergency(i));
				ROS_INFO("agent%d emergency success!", i);
			}

			ROS_INFO("wait for agent%d land service", i);
			if (ros::service::waitForService(zbf::emergency(i),
					ros::Duration(5.0))) {
				landSrv[i] = nh.serviceClient<std_srvs::Empty>(
					zbf::land(i));
				ROS_INFO("agent%d land success!", i);
			}

			ROS_INFO("wait for agent%d takeoff service", i);
			if (ros::service::waitForService(zbf::emergency(i),
					ros::Duration(5.0))) {
				takeoffSrv[i] =
						nh.serviceClient<std_srvs::Empty>(
					zbf::takeoff(i));
				ROS_INFO("agent%d takeoff success!", i);
			}	

			ROS_INFO("wait for agent%d to_automatic service", i);
			if (ros::service::waitForService(zbf::to_automatic(i),
					ros::Duration(5.0))) {
				toAutomaticSrv[i] = nh.serviceClient<std_srvs::Empty>(
						zbf::takeoff(i));
				ROS_INFO("agent%d to_automatic success!", i);
			}
		}
		joySub = nh.subscribe("/joy", 1, &ControlCenter::joyCallback, this);
		kp = Matrix3f::Identity();
		kv = Matrix3f::Identity();
	}

	void run() {
		ros::NodeHandle nh("~");
		ros::Timer timer = nh.createTimer(ros::Duration(1.0 / frequency),
				&ControlCenter::calc_u,
				this);
		ros::Timer
		timer2 = nh.createTimer(ros::Duration(10.0),
				&ControlCenter::timer_callback, this);
		int i;
		std_srvs::Empty srv;
		while (ros::ok()) {
			switch (state) {
			case zbf::Emergency:
				for (i = 0; i < USE_AGENT; i++) {
					if (emergencySrv[i].call(srv)) {
						ROS_INFO("agent%d emergency called success!", i);
						state = zbf::Idle;
					} else {
						ROS_INFO("agent%d emergency called fail!", i);
					}
				}
				break;
			case zbf::Agent0_takeoff:
#ifdef FLY_TOGETHER
				for (i = 0; i < USE_AGENT; i++) {
					ROS_INFO("agent%d will takeoff", i);
					if (takeoffSrv[i].call(srv)) {
						ROS_INFO("agent%d takeoff", i);
					}
				}
				srvCalled = zbf::Agent2_takeoff;
				state = zbf::Idle;
#else
				if (takeoffSrv[0].call(srv)) {
					ROS_INFO("agent0 takeoff");
				}
				srvCalled = zbf::Agent0_takeoff;
				state = zbf::Idle;
#endif

				break;
			case zbf::Agent1_takeoff:
				if (takeoffSrv[1].call(srv)) {
					ROS_INFO("agent1 takeoff");
				}
				srvCalled = zbf::Agent1_takeoff;
				state = zbf::Idle;
				break;
			case zbf::Agent2_takeoff:
				if (takeoffSrv[2].call(srv)) {
					ROS_INFO("agent2 takeoff");
				}
				srvCalled = zbf::Agent2_takeoff;
				state = zbf::Idle;
				break;
			case zbf::Joy_control:
				break;
			case zbf::Idle:
				break;
			case zbf::Land:
				for (i = 0; i < USE_AGENT; i++) {
					if (landSrv[i].call(srv)) {
						ROS_INFO("agent%d land called success!", i);
					} else {
						ROS_INFO("agent%d land called fail!", i);
					}
				}
				break;
			case zbf::To_Automatic:
				for (i = 0; i < USE_AGENT; i++) {
					if (toAutomaticSrv[i].call(srv)) {
						ROS_INFO("agent%d to_automatic called success!", i);
					} else {
						ROS_INFO("agent%d to_automatic called fail!", i);
					}
				}
				break;
			}
			ros::spinOnce();
		}
	}
private:

	void record() {
		int i, j;
		for (i = 0; i < USE_AGENT; i++) {
			for (j = 0; j < USE_AGENT; j++) {
				log->log_in((float) pos(i, j));
				log->log_pause();
			}
		}

		for (i = 0; i < USE_AGENT; i++) {
			for (j = 0; j < USE_AGENT; j++) {
				log->log_in((float) posGoal(i, j));
				log->log_pause();
			}
		}

		for (i = 0; i < USE_AGENT; i++) {
			for (j = 0; j < USE_AGENT; j++) {
				log->log_in((float) u(i, j));
				log->log_pause();
			}
		}

		log->log_end();
	}
	void timer_callback(const ros::TimerEvent & e) {
		ROS_INFO("timer callback called.");
		if (srvCalled == zbf::Agent0_takeoff) {
			state = zbf::Agent1_takeoff;
		} else if (srvCalled == zbf::Agent1_takeoff) {
			state = zbf::Agent2_takeoff;
		} else if (srvCalled == zbf::Agent2_takeoff) {
			state = zbf::Joy_control;
		}
	}
	void joyCallback(const sensor_msgs::Joy::ConstPtr & msg) {
		int i;
		//假定有10个按钮吧
		for (i = 0; i < 10; i++) {
			if (_buttons[i] != -1 && msg->buttons[i] != _buttons[i]
					&& msg->buttons[i] == 1) {
				switch (i) {
				case 0:
					if (msg->buttons[i] == 1 && state != zbf::Emergency) {
						state = zbf::Land;
					}
					//降落
					break;
				case 1:
					if (msg->buttons[i] == 1 && state != zbf::Emergency) {
						state = zbf::Emergency;
					}
					//急停
					break;
				case 2:
					//起飞
					ROS_INFO("Try to take off agent0 ");
					if (msg->buttons[i] == 1 && state != zbf::Emergency) {
						state = zbf::Agent0_takeoff;
						ROS_INFO("state changed, take off agent0 ");
					}
					break;
				case 3:
					//变换队形吧
					formation = (zbf::Formation) ((formation + 1)
							% (int) zbf::formation_length);
					if (formation == zbf::formation_circle) {
						ROS_INFO("formation is circle now");
					} else if (formation == zbf::formation_line) {
						ROS_INFO("formation is line now");
					} else if (formation == zbf::formation_triangle) {
						ROS_INFO("formation is triangle now");
					}
					break;
				case 4:
					//稳定后，开始新的控制方法吧
//					if (msg->buttons[i] == 1 && state != zbf::Emergency) {
//						state = zbf::Land;
//					}
					state = zbf::Joy_control;
					break;
				case 5:
					break;
				}
			}
		}
		for (i = 0; i < 10; i++) {
			_buttons[i] = msg->buttons[i];
		}
	}
	void init_frame_id(int frame_id[USE_AGENT]) {
		int i;
		for (i = 0; i < USE_AGENT; i++) {
			frame_ids[i] = frame_id[i];
		}
	}
	void show_frame_ids() {
		int i;
		std::cout << "[";
		for (i = 0; i < USE_AGENT; i++) {
			std::cout << frame_ids[i] << ",";
		}
		std::cout << "]" << std::endl;
	}
	void calc_u(const ros::TimerEvent & e) {
		float dt=e.current_real.toSec()-e.last_real.toSec();
		UVector sumP;
		UVector sumV;
		UVector sum;
        int id;
        int i;
		int j;
		id = 0;
		//外层的循环是计算每个飞机的u
		//这里面的控制其实是为了保持队形
		//但是，由于拓扑结构的影响，leader节点的输入计算出来是0
		//所以，对于leader节点，需要有内部输入，让leader到达指定的目标位置才可以。
		for (id = 0; id < USE_AGENT; id++) {
			sumP = Vector3f::Zero();
			sumV = Vector3f::Zero();
			sum = Vector3f::Zero();
			//内层的循环是u利用的条件
			i = 0;
			for (i = 0; i < USE_AGENT; i++) {
				sumP += topologies(id, i)
						* (
					pos.col(id)-pos.col(i)
					-posGoal.col(id)+posGoal.col(i));
				sumV += topologies(id, i)
						* (
					vel.col(id)-vel.col(i)
					-velGoal.col(id)+velGoal.col(i));
        	}

			for (j = 0; j < USE_AGENT; j++) {
				log->log_in((float) sumP(i));
				log->log_pause();
			}
			sum = -(kp * sumP) - (kv * sumV);
			u.col(id) = sum;
        }
		record();

	}
	int get_frame_id(const std::string str) {
		int len = str.length();
		int i;
		int vicon_id = str.c_str()[len - 1] - '0';
		for (i = 0; i < USE_AGENT; i++) {
			if (vicon_id == frame_ids[i]) {
				return i;
			}
		}
		std::cout << str << "  fail vicon_id:" << vicon_id << std::endl;
		return -1;
	}
	void receive_pos(const geometry_msgs::PoseStamped::ConstPtr& msg) {
		int id = get_frame_id(msg->header.frame_id);
		pos(0, id) = msg->pose.position.x;
		pos(1, id) = msg->pose.position.y;
		pos(2, id) = msg->pose.position.z;
	}
	void receive_vel(const geometry_msgs::PoseStamped::ConstPtr& msg) {
		int id = get_frame_id(msg->header.frame_id);
		vel(0, id) = msg->pose.position.x;
		vel(1, id) = msg->pose.position.y;
		vel(2, id) = msg->pose.position.z;
	}
	void receive_pos_goal(
			const my_crazyflie_controller::IDPose::ConstPtr& msg) {
		int id = msg->id;
		posGoal(0, id) = msg->pos.pose.position.x;
		posGoal(1, id) = msg->pos.pose.position.y;
		posGoal(2, id) = msg->pos.pose.position.z;
	}
	void receive_vel_goal(
			const my_crazyflie_controller::IDPose::ConstPtr& msg) {
		int id = msg->id;
		velGoal(0, id) = msg->pos.pose.position.x;
		velGoal(1, id) = msg->pos.pose.position.y;
		velGoal(2, id) = msg->pos.pose.position.z;
	}
	bool send(
			my_crazyflie_controller::AgentU::Request & req,
			my_crazyflie_controller::AgentU::Response & res){
		res.id=req.id;
		res.u[0] = u(0, req.id);
		res.u[1] = u(1, req.id);
		res.u[2] = u(2, req.id);
		return true;
	}
private:
	LogUtils* log;
	int srvCalled;
	zbf::State state;
	zbf::Formation formation;
	int _buttons[10] = { -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 };
	ros::Subscriber joySub;
	int frame_ids[USE_AGENT];
	Matrix3f kp;
	Matrix3f kv;
	float frequency;
	//订阅的各种消息：
	ros::Subscriber posSub[USE_AGENT];
	ros::Subscriber velSub[USE_AGENT];
	ros::Subscriber posGoalSub[USE_AGENT];
	ros::Subscriber velGoalSub[USE_AGENT];
	//消息
	my_crazyflie_controller::ThreeAgentMsg msg[USE_AGENT];
	//订阅的服务
	ros::ServiceServer uSrv[USE_AGENT];
	ros::ServiceClient takeoffSrv[USE_AGENT];
	ros::ServiceClient toAutomaticSrv[USE_AGENT];
	ros::ServiceClient landSrv[USE_AGENT];
	ros::ServiceClient emergencySrv[USE_AGENT];

	my_crazyflie_controller::AgentU agentU[USE_AGENT];
	PosMatrix u;

	TopoMatrix topologies;

	PosMatrix pos;PosMatrix vel;PosMatrix posGoal;PosMatrix velGoal;
};


#endif /* CRAZYFLIE_ECUST_MY_CRAZYFLIE_CONTROLLER_SRC_CONTROLCENTER_HPP_ */
