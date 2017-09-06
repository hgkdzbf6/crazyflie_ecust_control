/*
 * SingleAgent.cpp
 *
 *  Created on: 2017年8月26日
 *      Author: zbf
 */




#include "SingleAgent.hpp"

int main(int argc, char **argv) {
	ros::init(argc, argv, "SingleAgent");
	ros::NodeHandle nh("~");
	std::string worldFrame;
	std::string frame;
	std::string name;
	double frequency = 50.0;
	int num_id;
	int frame_id;
	bool internal_u;
	nh.getParam("world", worldFrame);
	nh.getParam("frame", frame);
	nh.getParam("name", name);
	nh.getParam("id", num_id);
	nh.getParam("frame_id", frame_id);
	nh.getParam("internal_u", internal_u);
	SingleAgent a(num_id, frame_id, internal_u, worldFrame, frame, name, nh);
	a.run(frequency);
}
