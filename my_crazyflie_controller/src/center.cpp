#include "center.hpp"

//使用了多少飞机


int main(int argc, char **argv){

    // 首先初始化节点
    ros::init(argc, argv, "center");
    ros::NodeHandle n("~");
    // 就先写三架飞机吧。
    std::string agentName[3];
    std::string agentFrame[3];
    int i;
    for(i=0;i<USE_NUM;i++){
        //agents.posSub[i]=n.subscribe(agentName[si],1,);
        //得到每个智能体的名字吧
        n.getParam(zbf::get_name(i),agentName[i]);
    }    
    for(i=0;i<USE_NUM;i++){
        //agents.posSub[i]=n.subscribe(agentName[si],1,);
        //得到每个智能体的名字吧
        n.getParam(zbf::get_frame(i),agentName[i]);
    }
    //得到名字之后，初始化结构体
    //这是拓扑结构
    int topologies[9]={0,0,0,-1,2,-1,0,-1,1};
    Center center(3,topologies,agentName,agentFrame,n);

}