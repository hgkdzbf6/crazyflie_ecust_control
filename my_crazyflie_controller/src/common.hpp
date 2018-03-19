/*
 * common.hpp
 *
 *  Created on: 2017年8月26日
 *      Author: zbf
 */

#ifndef CRAZYFLIE_ECUST_MY_CRAZYFLIE_CONTROLLER_SRC_COMMON_HPP_
#define CRAZYFLIE_ECUST_MY_CRAZYFLIE_CONTROLLER_SRC_COMMON_HPP_

#include <ros/ros.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Joy.h>
#include <Eigen/Dense>
#include "iostream"
#include "string"
#include "string.h"
#include "pid.hpp"
#include "my_crazyflie_controller/AgentU.h"
#include "my_crazyflie_controller/IDPose.h"
#include "LogUtils.hpp"
#include "CsvHelper.hpp"
using namespace Eigen;
/**
 * 总结：
 * 主题，服务的名称：
 * 每个飞机的命名空间：/agentX
 * 中心节点的命名空间：/center
 * 位置的主题，由飞机发出吧：/agentX/agentX_pos
 * 速度的主题，由飞机发出吧：/agentX/agentX_vel
 * 位置目标点的消息，由中心节点发出：/center/agentX_pos
 * 速度目标点的消息，由中心节点发出：/center/agentX_vel
 * 输出的主题，由中心节点发出：/center/agentX_u
 * 考虑到后面还要在rivz中显示，所以还是要把
 * 打包的消息展开吧，不利用自己的写的消息
 * 自己写的u还是要用的呢
 * 就是这样瞄～
 */
#ifndef USE_AGENT
#define USE_AGENT 1
#endif

#if USE_AGENT==1
//使用到的矩阵，分为这么几种情况
//1，就是u，u对应三个方向的
#define TopoMatrix MyMatrix11f
//2，就是向量吧
#define UVector MyVector1f
//3，pos，vel，posGoal，velGoal用的
#define PosMatrix MyMatrix31f
#elif USE_AGENT==2
//使用到的矩阵，分为这么几种情况
//1，就是u，u对应三个方向的
#define TopoMatrix MyMatrix22f
//2，就是向量吧
#define UVector MyVector2f
//3，pos，vel，posGoal，velGoal用的
#define PosMatrix MyMatrix32f
#elif USE_AGENT==3
//使用到的矩阵，分为这么几种情况
//1，就是u，u对应三个方向的
#define TopoMatrix MyMatrix33f
//2，就是向量吧
#define UVector MyVector3f
//3，pos，vel，posGoal，velGoal用的
#define PosMatrix MyMatrix33f
#elif USE_AGENT==4
//使用到的矩阵，分为这么几种情况
//1，就是u，u对应三个方向的
#define TopoMatrix MyMatrix44f
//2，就是向量吧
#define UVector MyVector4f
//3，pos，vel，posGoal，velGoal用的
#define PosMatrix MyMatrix34f

#elif USE_AGENT==5
//使用到的矩阵，分为这么几种情况
//1，就是u，u对应三个方向的
#define TopoMatrix MyMatrix55f
//2，就是向量吧
#define UVector MyVector5f
//3，pos，vel，posGoal，velGoal用的
#define PosMatrix MyMatrix35f

#elif USE_AGENT==6
//使用到的矩阵，分为这么几种情况
//1，就是u，u对应三个方向的
#define TopoMatrix MyMatrix66f
//2，就是向量吧
#define UVector MyVector6f
//3，pos，vel，posGoal，velGoal用的
#define PosMatrix MyMatrix36f
#elif USE_AGENT==7
//使用到的矩阵，分为这么几种情况
//1，就是u，u对应三个方向的
#define TopoMatrix MyMatrix77f
//2，就是向量吧
#define UVector MyVector7f
//3，pos，vel，posGoal，velGoal用的
#define PosMatrix MyMatrix37f
#endif

typedef Matrix<float, USE_AGENT, USE_AGENT> TopoMatrix;
typedef Matrix<float, USE_AGENT, 1> UVector;
typedef Matrix<float, 3, USE_AGENT> PosMatrix;

#define MY_PI 3.141592653589793238462643383279502884
#define MY_SQRT_1_DIV_3 0.577350269189626
#define MY_SQRT_2 1.414213562373095

namespace zbf {

int frame_ids[USE_AGENT] = { 0 };
int frame_ids2[USE_AGENT] = { 0 };

int constrain(int value, int max, int min) {
	int ret = value;
	if (ret > max)
		ret = max;
	if (ret < min)
		ret = min;
	return ret;
}

float constrainf(float value, float max, float min) {
	float ret = value;
	if (ret > max)
		ret = max;
	if (ret < min)
		ret = min;
	return ret;
}
double constraind(double value, double max, double min) {
	double ret = value;
	if (ret > max)
		ret = max;
	if (ret < min)
		ret = min;
	return ret;
}
void eigen_topologies(TopoMatrix& top, const int data[USE_AGENT][USE_AGENT]) {
	int i, j;
	for (i = 0; i < USE_AGENT; i++) {
		for (j = 0; j < USE_AGENT; j++) {
			top(i, j) = data[i][j];
		}
	}
}
void init_frame_id(int frame_id[USE_AGENT]) {
	int i;
	for (i = 0; i < USE_AGENT; i++) {
		frame_ids[i] = frame_id[i];
	}
}
void init_frame_id2(int frame_id[USE_AGENT]) {
	int i;
	for (i = 0; i < USE_AGENT; i++) {
		frame_ids2[i] = frame_id[i];
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
void show_frame_ids2() {
	int i;
	std::cout << "[";
	for (i = 0; i < USE_AGENT; i++) {
		std::cout << frame_ids2[i] << ",";
	}
	std::cout << "]" << std::endl;
}
const std::string str(const int id) {
	return "/agent" + std::to_string(id);
}
const std::string str_u(const int id) {
	return "/agent" + std::to_string(id) + "_u";
}
const std::string str_pos(const int id) {
	return "/agent" + std::to_string(id) + "_pos";
}
const std::string str_vel(const int id) {
	return "/agent" + std::to_string(id) + "_vel";
}
const std::string str_acc(const int id) {
	return "/agent" + std::to_string(id) + "_acc";
}
const std::string str_pos_goal(const int id) {
	return "/agent" + std::to_string(id) + "_pos_goal";
}
const std::string str_vel_goal(const int id) {
	return "/agent" + std::to_string(id) + "_vel_goal";
}
const std::string str_acc_goal(const int id) {
	return "/agent" + std::to_string(id) + "_acc_goal";
}
const std::string center_str_pos_goal(const int id) {
	return "/GoalCenter/" + str_pos_goal(id);
}
const std::string center_str_vel_goal(const int id) {
	return "/GoalCenter/" + str_vel_goal(id);
}
const std::string center_str_acc_goal(const int id) {
	return "/GoalCenter/" + str_acc_goal(id);
}
const std::string center_str_vel_goal_no_id(const int id) {
	return "/GoalCenter/" + str_vel_goal(id) + "_no_id";
}
const std::string center_str_acc_goal_no_id(const int id) {
	return "/GoalCenter/" + str_acc_goal(id) + "_no_id";
}
const std::string center_str_pos_goal_no_id(const int id) {
	return "/GoalCenter/" + str_pos_goal(id) + "_no_id";
}
const std::string agent_str_pos(const int id) {
	return "/agent" + std::to_string(id) + "/" + str_pos(id);
}
const std::string agent_str_vel(const int id) {
	return "/agent" + std::to_string(id) + "/" + str_vel(id);
}
const std::string agent_str_acc(const int id) {
	return "/agent" + std::to_string(id) + "/" + str_acc(id);
}
const int get_id(const std::string str) {
	return str.c_str()[0] - '0';
}
const std::string takeoff(int id) {
	return "/" + str(id) + "/takeoff";
}
const std::string to_automatic(int id) {
	return "/" + str(id) + "/to_automatic";
}

const std::string cmd_vel(int id) {
	return "/" + str(id) + "/cmd_vel";
}
const std::string land(int id) {
	return "/" + str(id) + "/land";
}
const std::string emergency(int id) {
	return "/" + str(id) + "/emergency";
}
void test(const std::string str) {
	int id = get_id(str);
	if (id < 0 || id > 3) {
		std::cout << str << std::endl;
	}
}
const std::string get_frame(const int id) {
	return "/vicon/C" + std::to_string(frame_ids[id]) + "/C"
			+ std::to_string(frame_ids[id]);
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
	std::cout << "fail vicon_id:" << vicon_id << std::endl;
	return -1;
}
int get_frame_id2(const std::string str) {
	int len = str.length();
	int i;
	int vicon_id = str.c_str()[len - 1] - '0';
	for (i = 0; i < USE_AGENT; i++) {
		if (vicon_id == frame_ids[i]) {
			return i;
		}
	}
	std::cout << "fail vicon_id:" << vicon_id << std::endl;
	return -1;
}
typedef enum _state {
	Idle = 0,
	Agent0_takeoff,
	Agent1_takeoff,
	Agent2_takeoff,
	Joy_control,
	Auto_formation,
	Emergency,
	Land,
	To_Automatic,
} State;

typedef enum _formation {
	formation_line, formation_circle, formation_triangle, formation_length
} Formation;
}

struct V3 {
	double x;
	double y;
	double z;
	V3(double x, double y, double z) {
		this->x = x;
		this->y = y;
		this->z = z;
	}
	const V3 operator +(const V3 & v) {
		V3 res;
		res.x = this->x + v.x;
		res.y = this->y + v.y;
		res.z = this->z + v.z;
		return res;
	}
	const V3 operator -(const V3 & v) {
		V3 res;
		res.x = this->x - v.x;
		res.y = this->y - v.y;
		res.z = this->z - v.z;
		return res;
	}
	const V3 operator /(double v) {
		V3 res;
		res.x = this->x / v;
		res.y = this->y / v;
		res.z = this->z / v;
		return res;
	}
	const V3 operator *(double v) {
		V3 res;
		res.x = this->x * v;
		res.y = this->y * v;
		res.z = this->z * v;
		return res;
	}
	const V3 & operator +=(const V3 & v) {
		this->x += v.x;
		this->y += v.y;
		this->z += v.z;
		return *this;
	}
	const V3 & operator =(const V3 & v) {
		this->x = v.x;
		this->y = v.y;
		this->z = v.z;
		return *this;
	}
	V3() :
			x(0), y(0), z(0) {
	}
};

#define DATA_LENGTH 6

struct MyFilter {

	V3 data[DATA_LENGTH];
	V3 output;
	V3 & update(const V3 & input) {
//		static const int weight[DATA_LENGTH] = { 1 };
		static const int weight_len = DATA_LENGTH;
		int i;
		output = V3(0, 0, 0);
		for (i = 0; i < DATA_LENGTH - 1; i++) {
			data[i].x = data[i + 1].x;
			data[i].y = data[i + 1].y;
			data[i].z = data[i + 1].z;
		}
		data[DATA_LENGTH - 1] = input;
		for (i = 0; i < DATA_LENGTH; i++) {
//			output.x += (data[i].x * weight[i]);
//			output.y += (data[i].y * weight[i]);
//			output.z += (data[i].z * weight[i]);
			output.x += (data[i].x);
			output.y += (data[i].y);
			output.z += (data[i].z);
		}
		output.x = output.x / weight_len;
		output.y = output.y / weight_len;
		output.z = output.z / weight_len;
		return output;
	}
};











#endif /* CRAZYFLIE_ECUST_MY_CRAZYFLIE_CONTROLLER_SRC_COMMON_HPP_ */
