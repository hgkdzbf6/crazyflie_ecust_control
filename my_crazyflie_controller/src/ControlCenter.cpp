/*
 * ThreeAgentCenter.cpp
 *
 *  Created on: 2017年8月27日
 *      Author: zbf
 */




#include "ControlCenter.hpp"

int main(int argc, char ** argv) {

	ros::init(argc, argv, "ControlCenter");
	ros::NodeHandle n("~");
	//这是拓扑结构
	int topologies[3][3] = { { 0, 0, 0 }, { -1, 1, 0 }, { 0, -1, 1 } };
	int frame_id_1;
	int frame_id_2;
	int frame_id_3;
	n.getParam("frame_id_1", frame_id_1);
	n.getParam("frame_id_2", frame_id_2);
	n.getParam("frame_id_3", frame_id_3);
	int frame_id[USE_AGENT] = { frame_id_1, frame_id_2, frame_id_3 };
	ControlCenter a(topologies, 50.0, frame_id);
	
	a.run();
}
