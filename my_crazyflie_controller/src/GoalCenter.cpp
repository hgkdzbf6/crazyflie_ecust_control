#include "GoalCenter.hpp"

//使用了多少飞机

int main(int argc, char **argv){

    // 首先初始化节点
    ros::init(argc, argv, "center");
    ros::NodeHandle n("~");
	int topologies[3][3] = { { 0, 0, 0 }, { -1, 1, 0 }, { 0, -1, 1 } };
    // 就先写三架飞机吧。
    //得到名字之后，初始化结构体
	int frame_id_1;
	int frame_id_2;
	int frame_id_3;
	n.getParam("frame_id_1", frame_id_1);
	n.getParam("frame_id_2", frame_id_2);
	n.getParam("frame_id_3", frame_id_3);
	GoalCenter center(topologies, frame_id_1, frame_id_2, frame_id_3);

	center.run();
}
