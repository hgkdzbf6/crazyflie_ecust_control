#include "GoalCenter.hpp"

//使用了多少飞机

int main(int argc, char **argv){

    // 首先初始化节点
    ros::init(argc, argv, "center");
    ros::NodeHandle n("~");
#if USE_AGENT==1
	int topologies[USE_AGENT][USE_AGENT]= {0};
#elif USE_AGENT==2
	int topologies[USE_AGENT][USE_AGENT]= {0,0,-1,1};
#elif USE_AGENT==3
	int topologies[USE_AGENT][USE_AGENT] = { { 0, 0, 0 }, { -1, 1, 0 }, { 0, -1,
			1 } };
#elif USE_AGENT==4
	int topologies[USE_AGENT][USE_AGENT] = { { 0, 0, 0, 0 }, { -1, 1, 0, 0 }, {
			0, -1, 1, 0 }, { 0, 0, -1, 1 } };
#elif USE_AGENT==5
	int topologies[USE_AGENT][USE_AGENT] = { { 0, 0, 0, 0, 0 },
			{ -1, 1, 0, 0, 0 }, { 0, -1, 1, 0, 0 }, { 0, 0, -1, 1, 0 }, { 0, 0,
					0, -1, 1 } };
#elif USE_AGENT==6
	int topologies[USE_AGENT][USE_AGENT] = { { 0, 0, 0, 0, 0, 0 }, { -1, 1, 0,
			0, 0, 0 }, { 0, -1, 1, 0, 0, 0 }, { 0, 0, -1, 1, 0, 0 }, { 0, 0, 0,
			-1, 1, 0 }, { 0, 0, 0, 0, -1, 1 } };
#elif USE_AGENT==7
	int topologies[USE_AGENT][USE_AGENT] = { { 0, 0, 0, 0, 0, 0, 0 }, { -1, 1,
			0, 0, 0, 0, 0 }, { 0, -1, 1, 0, 0, 0, 0 }, { 0, 0, -1, 1, 0, 0, 0 },
			{ 0, 0, 0, -1, 1, 0, 0 }, { 0, 0, 0, 0, -1, 1, 0 }, { 0, 0, 0, 0, 0,
					-1, 1 } };
#endif

    // 就先写三架飞机吧。
    //得到名字之后，初始化结构体
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
	GoalCenter center(topologies, frame_ids);

	center.run();
}
