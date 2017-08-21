#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>

#include <string.h>
#include "pid.hpp"

#define USE_NUM 3

#define FIRST_ID 0
#define SECOND_ID 1
#define THIRD_ID 2


#define MAX_AGENT_NUM 6
namespace zbf
{
    // 一些小工具函数吧
    const std::string&  generate_frame(int id);
    int get_id_by_frame(const std::string& frame);
    const std::string&  get_name(int id);
    const std::string&  get_frame(int id);
    const std::string&  get_goal(const std::string& prefix);
    const std::string&  get_vel_goal(const std::string& prefix);
    const std::string&  get_vel(const std::string& prefix);

    // 根据vicon的id生成帧
    const std::string&  generate_frame(int id){
        std::string header=std::string("/vicon/C");
        std::string tail=std::string("/C");
        std::string num=std::to_string(id);
        std::string res=header.append(num);
        res=res.append(tail);
        return res.append(num);
    }
    // 根据帧生成id
    int get_id_by_frame(const std::string& frame){
        int i;
        for(i=0;i<MAX_AGENT_NUM;i++){
            if(frame.compare(get_frame(i))==0)return i;
        }
    }
    // 得到节点名称
    const std::string&  get_name(int id){
        std::string str;
        if(id==FIRST_ID)str= std::string("/leaderNode");
        else if(id==SECOND_ID)str= std::string("/follower1Node");
        else if(id==THIRD_ID)str= std::string("/follower2Node");
        else str= "";
        return str.assign(std::string(""));
    }
    // 根据拓扑图的编号得到帧
    const std::string&  get_frame(int id){
        std::string emptyStr=std::string("");
        if(id==FIRST_ID)return generate_frame(1);
        else if(id==SECOND_ID)return generate_frame(2);
        else if(id==THIRD_ID)return generate_frame(3);
        else return emptyStr.assign(std::string(""));
    }
    // 生成位置目标的主题吧
    const std::string&  get_goal(const std::string& prefix){
        std::string p;
        p.assign(prefix);
        return p.append(std::string("/goal"));
    }
    // 生成速度目标的主题吧
    const std::string&  get_vel_goal(const std::string& prefix){
        std::string p;
        p.assign(prefix);
        return p.append(std::string("/velGoal"));
    }
    // 生成速度的主题吧
    const std::string&  get_vel(const std::string& prefix){
        std::string p;
        p.assign(prefix);
        return p.append(std::string("/vel"));
    }    
    // 生成速度的主题吧
    const std::string&  get_cmd(const std::string& prefix){
        std::string p;
        p.assign(prefix);
        return p.append(std::string("/cmd_vel"));
    }
    // 生成速度的主题吧
    const std::string&  get_takeoff(const std::string& prefix){
        std::string p;
        p.assign(prefix);
        return p.append(std::string("/takeoff"));
    }
    // 生成速度的主题吧
    const std::string&  get_land(const std::string& prefix){
        std::string p;
        p.assign(prefix);
        return p.append(std::string("/land"));
    }

}
typedef enum __AIRCRAFT_STATE{
        Idle = 0,
        Automatic = 1,
        TakingOff = 2,
        Landing = 3,
}State;

typedef enum __AGENT_NUM{
    THREE_AGENT=3,
    FOUR_AGENT=4,
    FIVE_AGENT=5,
}AGENT_NUM;

typedef struct __Three_agent{
    int topologies[3][3];
    //用来订阅飞机的位置
    ros::Subscriber posSub[3];
    //订阅飞机的目标位置
    ros::Subscriber posGoalSub[3];
    //订阅飞机的速度
    ros::Subscriber velSub[3];
    //订阅飞机的目标速度
    ros::Subscriber velGoalSub[3];

    //广播控制指令
    ros::Publisher cmdPub[3];

    geometry_msgs::PoseStamped pos[3];
    geometry_msgs::PoseStamped posGoal[3];
    geometry_msgs::PoseStamped vel[3];
    geometry_msgs::PoseStamped velGoal[3];

    ros::ServiceServer takeoffSrv[3];
    ros::ServiceServer landSrv[3];

    tf::TransformListener listener[3];
    State state[3];
    
    __Three_agent()
    {
        // 在下面将要会初始化的变量：
        // posSub posGoalSub velGoalSub velSub
        // 没有初始化的变量：
        // listener

        // int i;
        // for(i=0;i<3;i++){
        //     listener[i]=
        // }
        memset(&topologies,0,3*3*sizeof(int));
        memset(&state,0,3*sizeof(State));
    }
}ThreeAgent;

typedef struct __Four_agent{
    int topologies[4][4];
    //用来订阅飞机的位置
    ros::Subscriber agentPosSub[4];
    //订阅飞机的目标位置
    ros::Subscriber agentPosGoalSub[4];
    //订阅飞机的速度
    ros::Subscriber agentVelSub[4];
    //订阅飞机的目标速度
    ros::Subscriber agentVelGoalSub[4];

    //广播控制指令
    ros::Publisher agentCmdPub[4];
    
}FourAgent;

typedef struct __Five_agent{
    int topologies[5][5];
    //用来订阅飞机的位置
    ros::Subscriber agentPosSub[5];
    //订阅飞机的目标位置
    ros::Subscriber agentPosGoalSub[5];
    //订阅飞机的速度
    ros::Subscriber agentVelSub[5];
    //订阅飞机的目标速度
    ros::Subscriber agentVelGoalSub[5];

    //广播控制指令
    ros::Publisher agentCmdPub[5];
}FiveAgent;

class Center{
    
private:
    ThreeAgent m_agents;
    float kp;
    float kv;
    // void get_pos(const geometry_msgs::PoseStamped::ConstPtr& msg, const int& id){
    //     m_agents.pos[id]=*msg;
    // }      
    void get_pos(const geometry_msgs::PoseStamped::ConstPtr& msg){
        int id=zbf::get_id_by_frame(msg->header.frame_id);
        m_agents.pos[id]=*msg;
    }
    void get_pos_goal(const geometry_msgs::PoseStamped::ConstPtr& msg){
        int id=zbf::get_id_by_frame(msg->header.frame_id);
        m_agents.posGoal[id]=*msg;
    }
    void get_vel_goal(const geometry_msgs::PoseStamped::ConstPtr& msg){
        int id=zbf::get_id_by_frame(msg->header.frame_id);
        m_agents.velGoal[id]=*msg;
    }
    void get_vel(const geometry_msgs::PoseStamped::ConstPtr& msg){
        int id=zbf::get_id_by_frame(msg->header.frame_id);
        m_agents.vel[id]=*msg;
    }
public:
    // static get_id(const std::string frame_id){
    //     if(frame_id.compare(""))
    // }
    Center(
        const int count,
        const int topologies[],
        const std::string agentName[],
        const std::string agentFrame[],
        const ros::NodeHandle& n 
    ):m_agents(),
    kp(1),
    kv(1)
    {
        int i;
        // 这个是外部控制器，需要知道所有飞机的位置，速度，目标位置，目标速度信息。
        // 所以要订阅所有飞机的位置
        memcpy(m_agents.topologies,topologies,3*3*sizeof(int));
        ros::NodeHandle nh;
        for(i=0;i<count;i++){
            // tf listener的信息
            m_agents.listener[i].waitForTransform("/world",agentFrame[i],ros::Time(0),ros::Duration(10.0));
            //vicon的信息
            //没有必要记录发送者的信息，因为消息内部已经有相应的说明了
            m_agents.posSub[i]=nh.subscribe(agentFrame[i],1,&Center::get_pos,this);
            // m_agents.posSub[i]=nh.subscribe(agentFrame[i],1,&get_pos,this);
            //goal的信息
            m_agents.posGoalSub[i]=nh.subscribe(zbf::get_goal(agentFrame[i]),1,&Center::get_pos_goal,this);
            //m_agents.posGoalSub[i]=n.subscribe(agentName+"/goal",1,boost::bind(get_pos_goal,std::placeholders::_1,i));
            //goal_vel的信息
            m_agents.velGoalSub[i]=nh.subscribe(zbf::get_vel_goal(agentFrame[i]),1,&Center::get_pos,this);
            //m_agents.velGoalSub[i]=n.subscribe(agentName+"/goal_vel",1,boost::bind(get_vel_goal,std::placeholders::_1,i));
            //vel的信息还是飞机上面得到吧，只不过计算节点上不一样。
            m_agents.velSub[i]=nh.subscribe(zbf::get_vel(agentFrame[i]),1,&Center::get_pos,this);
            //m_agents.velSub[i]=n.subscribe(agentName+"/vel",boost::bind(get_vel,std::placeholders::_1,i));
            //广播的消息
            m_agents.cmdPub[i]=nh.advertise<geometry_msgs::Twist>(zbf::get_cmd(agentFrame[i]),1);
            // 起飞的服务
            m_agents.takeoffSrv[i]=nh.advertiseService(zbf::get_takeoff(agentFrame[i]),&Center::takeoff,this);
            // 降落的服务
            m_agents.landSrv[i]=nh.advertiseService(zbf::get_land(agentFrame[i]),&Center::land,this);

        }
        //以上就获取到了所有所需的信息，吗？
        // 不，还有拓扑信息。
        // 速度

        // 目标位置

        // 目标速度信息
    }
    bool takeoff(
        std_srvs::Empty::Request& req,
        std_srvs::Empty::Response& res){

        ROS_INFO("My takeoff requested!");
    }    
    bool land(
        std_srvs::Empty::Request& req,
        std_srvs::Empty::Response& res){

        ROS_INFO("My land requested!");

    }
    void run(double frequency)
    {
        ros::NodeHandle node;
        // 创建个定时器定时执行某目标
        // 用定时器的好处：可以精确地控制时间吧
        ros::Timer timer = node.createTimer(ros::Duration(1.0/frequency), &Center::iteration, this);
        ros::spin();
    }
    //计算每一个飞机的输入吧，使用最简单的公式
    void calc_u(){

    }
    void iteration(const ros::TimerEvent& e)
    {
        // 当然还是双保险，两次时间做差分吧
        float dt = e.current_real.toSec() - e.last_real.toSec();

    }
};

