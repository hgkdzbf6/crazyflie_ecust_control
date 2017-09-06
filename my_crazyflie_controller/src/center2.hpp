/*
 * center2.hpp
 *
 *  Created on: 2017年8月23日
 *      Author: zbf
 */

#ifndef CRAZYFLIE_ECUST_MY_CRAZYFLIE_CONTROLLER_SRC_CENTER2_HPP_
#define CRAZYFLIE_ECUST_MY_CRAZYFLIE_CONTROLLER_SRC_CENTER2_HPP_

//推到重来的原因是因为每架飞机的控制，定时器什么的都是独立的，添加参数很困难
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <Eigen/Dense>
#include <iostream>
#include <string.h>
#include "pid.hpp"


using Eigen::MatrixXf;
using Eigen::Matrix3f;
using Eigen::Vector3f;
#define USE_NUM 3
#define WORLD_FRAME "/world"
#define FIRST_ID 0
#define SECOND_ID 1
#define THIRD_ID 2


#define MAX_AGENT_NUM 6
namespace zbf2
{
	// 我的类型转换：第一个参数是要改变的对象
    // 第二个以及以后是要改变这个对象需要的必要的信息。
    // 一些小工具函数吧
    const std::string&  generate_frame(int id);
    int get_id_by_frame(const std::string& frame);
    const std::string&  get_name(int id);
    const std::string&  get_frame(int id);
    const std::string&  get_goal(const std::string& prefix);
    const std::string&  get_vel_goal(const std::string& prefix);
    const std::string&  get_vel(const std::string& prefix);

    void eigen_topologies(Eigen::Matrix3f& top,const int data[3][3]){
    	int i,j;
    	for(i=0;i<3;i++){
    		for(j=0;j<3;j++){
    			top(i,j)=data[i][j];
    		}
    	}
    }
    /**
     * ps2st：PoseStamped 转换为 StampedTransform
     * Pose和Transform的区别：pose是在一个坐标系下某个点的坐标
     * Transform是两个坐标系之间的转换吧。
     * 不过一般情况下Transform的第一个坐标系都是/world
     * 所以还是能够转换的。
     * 或者返回一个bool值，如果transform的第一个坐标系是/world返回true，‘
     * 否则返回false
     */
    bool ps2st(geometry_msgs::PoseStamped& pose,const tf::StampedTransform& transform){
    	pose.header.frame_id=transform.child_frame_id_;
    	if(transform.frame_id_.compare(WORLD_FRAME)==0){
    		return true;
    	}
    	return false;
    }
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



typedef enum __AGENT_NUM{
    THREE_AGENT=3,
    FOUR_AGENT=4,
    FIVE_AGENT=5,
}AGENT_NUM;



typedef struct __Three_agent{
	std::string agentName[3];
	std::string agentFrame[3];

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

class Center2 {

private:
	float dt;
    ThreeAgent m_agents;
    Matrix3f topologies;
    Matrix3f u;
    Matrix3f pos;
    Matrix3f posGoal;
    Matrix3f vel;
    Matrix3f velGoal;
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
	dt(0.0f)
    {
        int i;
        // 这个是外部控制器，需要知道所有飞机的位置，速度，目标位置，目标速度信息。
        // 所以要订阅所有飞机的位置
        memcpy(m_agents.topologies,topologies,3*3*sizeof(int));
        ros::NodeHandle nh;
        for(i=0;i<count;i++){
            // tf listener的信息
            m_agents.listener[i].waitForTransform(WORLD_FRAME,agentFrame[i],ros::Time(0),ros::Duration(10.0));
            //vicon的信息
            m_agents.agentFrame[i].assign(agentFrame[i]);
            m_agents.agentName[i].assign(agentName[i]);
            //没有必要记录发送者的信息，因为消息内部已经有相应的说明了
            //m_agents.posSub[i]=nh.subscribe(agentFrame[i],1,&Center::get_pos,this);
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
    /**
     * 计算速度吧
     * 两难：直接改Matrix3f的矩阵还是改PoseStamped的消息？
     * 还是改消息吧，工作量大一点
     */
    void calc_v(float dt){
    	static Matrix3f pre_pos=Matrix3f::Zero();
    	int i;
    	for(i=0;i<USE_NUM;i++){
    		m_agents.vel[i].header=m_agents.pos[i].header;
    		if(pre_pos==Matrix3f::Zero()){
    			m_agents.vel[i].pose.position.x=0;
    			m_agents.vel[i].pose.position.y=0;
    			m_agents.vel[i].pose.position.z=0;
    		}else{
				m_agents.vel[i].pose.position.x=
						(m_agents.pos[i].pose.position.x-
						pre_pos(0,i))/dt;
				m_agents.vel[i].pose.position.y=
						(m_agents.pos[i].pose.position.y-
						pre_pos(1,i))/dt;
				m_agents.vel[i].pose.position.z=
						(m_agents.pos[i].pose.position.z-
						pre_pos(2,i))/dt;
    		}
    		//角速度的话计算就放在后面吧，先不计算
    	}
    	pre_pos=pos;
    }
    /**
     *   计算每一个飞机的输入吧，使用最简单的公式
     *   强行使用eigen库，这样后面处理也方便一点的。
     *   计算的话，这一个函数把所有的u都计算出来，
     *   放在静态变量里面需要的时候再拿出来用。
     *   因为每架飞机前面的计算都是相似的，减少
     *   计算量
     *   速度的计算还是放在飞机节点上面计算吧？
     *   无所谓了，先做粗一个再说吧
     */
    void calc_u(){
        Matrix3f kp;
        Matrix3f kv;
        int i;
        // 引用方便计算
        // 首先获取agent的位置，位置原来是tf，要变成eigen里面的tf吧。
        // 因为vicon的消息也是transform类型的消息吧。
        // 找别人的transform转pose太麻烦了，还是自己写吧


        //然后是飞行器的位置，应该用的是tf::transfrom消息，不是pose。
        //我再写个pose转transform吧。
        //获取所有飞行器的位置和速度
        //这一步是给m_agents.pos赋值吧
        tf::StampedTransform transform;
        geometry_msgs::PoseStamped targetWorld;

        for(i=0;i<USE_NUM;i++){
        	m_agents.listener[i].lookupTransform(WORLD_FRAME
        		,m_agents.agentFrame[i],ros::Time(0),transform);
        	//下面的这个变量，就是用来放goal pose的，但只是更改了header
        	targetWorld.header.stamp=transform.stamp_;
        	targetWorld.header.frame_id=m_agents.agentFrame[i];
        	targetWorld.pose=m_agents.posGoal[i];
            m_agents.listener.transformPose(m_agents.agentFrame[i]
    			,targetWorld,m_agents.pos[i]);
        }
        //下一步，给m_agents.vel和矩阵赋值
        this->calc_v(this->dt);
        //下面开始正式操作，也就是eigen的格式转换吧
        //首先是拓扑结构
        zbf::eigen_topologies(topologies,m_agents.topologies);
        //然后是四个坐标吧


        for(i=0;i<USE_NUM;i++){
        	pos(i,0)=m_agents.pos[i].pose.position.x;
        	pos(i,1)=m_agents.pos[i].pose.position.y;
        	pos(i,2)=m_agents.pos[i].pose.position.z;

        	posGoal(i,0)=m_agents.posGoal[i].pose.position.x;
        	posGoal(i,1)=m_agents.posGoal[i].pose.position.y;
        	posGoal(i,2)=m_agents.posGoal[i].pose.position.z;

        	vel(i,0)=m_agents.vel[i].pose.position.x;
        	vel(i,1)=m_agents.vel[i].pose.position.y;
        	vel(i,2)=m_agents.vel[i].pose.position.z;

        	velGoal(i,0)=m_agents.velGoal[i].pose.position.x;
        	velGoal(i,1)=m_agents.velGoal[i].pose.position.y;
        	velGoal(i,2)=m_agents.velGoal[i].pose.position.z;
        }
        //kp,kv初始化
        kp<<1,0,0,
			0,1,0,
			0,0,1;
        kv<<1,0,0,
			0,1,0,
			0,0,1;
        Vector3f sumP;
        Vector3f sumV;
        int id;
        for(i=0;i<USE_NUM;i++){
            sumP=Vector3f::Zero();
            sumV=Vector3f::Zero();
        	for(id=0;id<USE_NUM;id++){
				sumP+=topologies.row(i)*(
					pos.col(id)-pos.col(i)
					-posGoal.col(id)+posGoal.col(i));
				sumV+=topologies.row(i)*(
					vel.col(id)-vel.col(i)
					-velGoal.col(id)+velGoal.col(i));
        	}
            u.col(id)=-kp*sumP-kv*sumV;
        }
    }
    void get_u(float & res,const int id,const int i){
    	res=u(i,id);
    }
    void iteration(const ros::TimerEvent& e)
    {
    	int i;
        // 当然还是双保险，两次时间做差分吧
        this->dt = e.current_real.toSec() - e.last_real.toSec();
        this->calc_u();
        for(i=0;i<USE_NUM;i++){
        	switch(m_agents.state[i]){
        	case TakingOff:
        		tf::StampedTransform transform;
        		m_agents.listener[i].lookupTransform(WORLD_FRAME,
        			m_agents.agentFrame[i],
					ros::Time(0),transform);
        		if(transform.getOrigin().z()>)
        		break;
        	case Landing:
        		break;
        	case Automatic:
        		break;
        	case Idle:
        		break;
        	}
        }
    }
};



#endif /* CRAZYFLIE_ECUST_MY_CRAZYFLIE_CONTROLLER_SRC_CENTER2_HPP_ */
