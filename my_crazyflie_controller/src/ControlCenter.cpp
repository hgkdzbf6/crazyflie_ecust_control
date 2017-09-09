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
#if USE_AGENT==1
	int topologies[USE_AGENT][USE_AGENT]= {0};
#elif USE_AGENT==2
	int topologies[USE_AGENT][USE_AGENT]= {0,0,-1,1};
#elif USE_AGENT==3
	int topologies[USE_AGENT][USE_AGENT] = { { 0, 0, 0 }, { -1, 1, 0 }, { 0, -1,
			1 } };
#elif USE_AGENT==4
	int topologies[USE_AGENT][USE_AGENT] = { {0, 0, 0, 0}, {-1, 1, 0, 0}, {
			0, -1, 1, 0}, {0, 0, -1, 1}};
#elif USE_AGENT==5
	int topologies[USE_AGENT][USE_AGENT] = { {0, 0, 0, 0, 0},
		{	-1, 1, 0, 0, 0}, {0, -1, 1, 0, 0}, {0, 0, -1, 1, 0}, {0, 0,
			0, -1, 1}};
#elif USE_AGENT==6
	int topologies[USE_AGENT][USE_AGENT] = { {0, 0, 0, 0, 0, 0}, {-1, 1, 0,
			0, 0, 0}, {0, -1, 1, 0, 0, 0}, {0, 0, -1, 1, 0, 0}, {0, 0, 0,
			-1, 1, 0}, {0, 0, 0, 0, -1, 1}};
#elif USE_AGENT==7
	int topologies[USE_AGENT][USE_AGENT] = { { 0, 0, 0, 0, 0, 0, 0 }, { -1, 1,
			0, 0, 0, 0, 0 }, { 0, -1, 1, 0, 0, 0, 0 }, { 0, 0, -1, 1, 0, 0, 0 },
			{ 0, 0, 0, -1, 1, 0, 0 }, { 0, 0, 0, 0, -1, 1, 0 }, { 0, 0, 0, 0, 0,
					-1, 1 } };
#endif
	int frame_ids[USE_AGENT];

#if USE_AGENT>=1
	n.getParam("frame_id_1", frame_ids[0]);
#endif
#if USE_AGENT>=2
	n.getParam("frame_id_2", frame_ids[1]);
#endif
#if USE_AGENT>=3
	n.getParam("frame_id_3", frame_ids[2]);
#endif
#if USE_AGENT>=4
	n.getParam("frame_id_4", frame_ids[3]);
#endif
#if USE_AGENT>=5
	n.getParam("frame_id_5", frame_ids[4]);
#endif
#if USE_AGENT>=6
	n.getParam("frame_id_6", frame_ids[5]);
#endif
#if USE_AGENT>=7
	n.getParam("frame_id_7", frame_ids[6]);
#endif
	ControlCenter a(topologies, 50.0, frame_ids);
	
	a.run();
}
