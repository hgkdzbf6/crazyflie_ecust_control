/*
 * SingleAgent.hpp
 *
 *  Created on: 2017年8月23日
 *      Author: zbf
 */

#ifndef CRAZYFLIE_ECUST_MY_CRAZYFLIE_CONTROLLER_SRC_SINGLEAGENT_HPP_
#define CRAZYFLIE_ECUST_MY_CRAZYFLIE_CONTROLLER_SRC_SINGLEAGENT_HPP_

#include "common.hpp"

using namespace Eigen;

typedef enum __AIRCRAFT_STATE{
        Idle = 0,
        Automatic = 1,
        TakingOff = 2,
        Landing = 3,
		BeforeStable = 4,
}State;

class SingleAgent{
public:

	SingleAgent(
			const int id,
			const int frame_id,
			const bool internal_u_flag,
			const std::string& worldFrame,
			const std::string& frame,
			const std::string& name,
			//从参数服务器上获取数据，才需要n，不然不需要
			const ros::NodeHandle& n)
	:
			agent_id(id), internal_u_flag(internal_u_flag),
			frame_id(frame_id),
				m_worldFrame(
					worldFrame), m_agentFrame(frame), listener(), state(Idle), posGoalSub(), velGoalSub(), takeoffSrv(), landSrv(), posGoalMsg(), velGoalMsg(), pos(), vel(), pre_pos(), m_agentName(
					name), thrust(0), startZ(0), m_pidYaw(-200, -20, 0, -200,
					200, 0, 0, "yaw"), m_pidX(33, 33, 0.8, -10, 10, -0.5, 0.5,
					"x"), m_pidY(-33, -33, -0.8, -10, 10, -0.5, 0.5, "y"), m_pidZ(
					6200, 7500, 1500, 10000, 60000, -2000, 2000, "z") {
		//每个不同的飞机有不同的命名空间
		ros::NodeHandle nh("~");
		ros::NodeHandle n2;
		listener.waitForTransform(m_worldFrame,m_agentFrame,
				ros::Time(0),ros::Duration(10.0));
		//广播消息
		posPub = n2.advertise<geometry_msgs::PoseStamped>(
				zbf::agent_str_pos(id), 1);
		velPub = n2.advertise<geometry_msgs::PoseStamped>(
				zbf::agent_str_vel(id), 1);
		//从中心节点订阅目标位置主题
		posGoalSub = n2.subscribe(zbf::center_str_pos_goal(id), 1,
				&SingleAgent::posGoalChanged,
				this);
		//订阅速度主题
		velGoalSub = n2.subscribe(zbf::center_str_vel_goal(id), 1,
				&SingleAgent::velGoalChanged,
				this);
		//广播起飞服务
		takeoffSrv = nh.advertiseService(zbf::takeoff(agent_id),
				&SingleAgent::takeoff, this);
		//广播降落服务
		landSrv = nh.advertiseService(zbf::land(agent_id), &SingleAgent::land,
				this);
		toAutomaticSrv = nh.advertiseService(zbf::to_automatic(agent_id),
				&SingleAgent::takeoff, this);
		//从中心节点订阅u服务
		cmdPub = nh.advertise<geometry_msgs::Twist>(zbf::cmd_vel(id), 1);
		uClient = nh.serviceClient<my_crazyflie_controller::AgentU>(
				zbf::str_u(id));
		log = new LogUtils(id);
	}

	void run(double frequency){
		ros::NodeHandle node;
		ros::Timer timer=node.createTimer(
				ros::Duration(1.0/frequency),
				&SingleAgent::iteration,
				this);
		ros::Timer timer2 = node.createTimer(ros::Duration(1.0 / frequency),
				&SingleAgent::broadcast_pos_and_vel, this);
		ros::spin();

	}
private:
	bool internal_u_flag;
	int agent_id;
	int frame_id;
	std::string m_worldFrame;
	std::string m_agentName;
	std::string m_agentFrame;

	//拓扑是智能体集合的信息，所以不写
	//int topologies;

	tf::TransformListener listener;

	ros::Subscriber posGoalSub;
	ros::Subscriber velGoalSub;

	ros::Publisher posPub;
	ros::Publisher velPub;

	Vector3f pos;
	Vector3f vel;

	Vector3f u;
	geometry_msgs::PoseStamped posMsg;
	geometry_msgs::PoseStamped velMsg;
	geometry_msgs::PoseStamped posGoalMsg;
	geometry_msgs::PoseStamped velGoalMsg;

	ros::ServiceServer takeoffSrv;
	ros::ServiceServer landSrv;
	ros::ServiceServer toAutomaticSrv;

	float startZ;
	float thrust;

	ros::ServiceClient uClient;
	my_crazyflie_controller::AgentU req;

	ros::Publisher cmdPub;

	State state;
	Vector3f pre_pos;
	PID m_pidX;
	PID m_pidY;
	PID m_pidZ;
    PID m_pidYaw;
	LogUtils * log;
	void pidReset() {
		m_pidX.reset();
		m_pidZ.reset();
		m_pidZ.reset();
		m_pidYaw.reset();
	}
	void broadcast_pos_and_vel(const ros::TimerEvent & e) {
		float dt = e.current_real.toSec() - e.last_real.toSec();
		pub_pos();
		get_pos();
		get_vel(dt);
		pub_vel();
	}
	void pub_pos() {
		tf::StampedTransform transform;
		geometry_msgs::PoseStamped targetWorld;
		ros::Time now = ros::Time::now();
		this->listener.waitForTransform(m_worldFrame, m_agentFrame, now,
				ros::Duration(5));
		this->listener.lookupTransform(m_worldFrame, m_agentFrame, now,
				transform);
//		targetWorld.header.stamp = transform.stamp_;
//		targetWorld.header.frame_id = m_worldFrame;
//		targetWorld.pose.orientation.w = 1;
//		//世界坐标系的goal转化为自身坐标系的pos
//		//不用goal，直接goal就是0,0,0，转化后就是pos的坐标
//		listener.transformPose(m_agentFrame, targetWorld, posMsg);
//		std::cout << "{" << transform.getOrigin().getX() << ","
//				<< transform.getOrigin().getY() << ","
//				<< transform.getOrigin().getZ()
//				<< "}" << std::endl;
		posMsg.header.stamp = transform.stamp_;
		posMsg.header.frame_id = m_agentFrame;
		posMsg.pose.orientation.w = 1;
		posMsg.pose.position.x = transform.getOrigin().getX();
		posMsg.pose.position.y = transform.getOrigin().getY();
		posMsg.pose.position.z = transform.getOrigin().getZ();
		posPub.publish(posMsg);
	}
	void get_pos() {
		pos(0) = posMsg.pose.position.x;
		pos(1) = posMsg.pose.position.y;
		pos(2) = posMsg.pose.position.z;
	}
	void pub_vel() {
		velMsg.header.frame_id = posMsg.header.frame_id;
		velMsg.header.seq = posMsg.header.seq;
		velMsg.header.stamp = posMsg.header.stamp;
		velMsg.pose.position.x = vel(0);
		velMsg.pose.position.y = vel(1);
		velMsg.pose.position.z = vel(2);
		velMsg.pose.orientation.w = 1;
		velPub.publish(velMsg);
	}
    void getTransform(
        const std::string& sourceFrame,
        const std::string& targetFrame,
        tf::StampedTransform& result)
    {
        listener.lookupTransform(sourceFrame,
        		targetFrame, ros::Time(0), result);
    }
	void posGoalChanged(const my_crazyflie_controller::IDPose::ConstPtr& msg) {
		posGoalMsg = (msg->pos);
	}
	void velGoalChanged(const my_crazyflie_controller::IDPose::ConstPtr& msg) {
		velGoalMsg = (msg->pos);
	}
	bool takeoff(
	        std_srvs::Empty::Request& req,
	        std_srvs::Empty::Response& res)
	{
        ROS_INFO("Takeoff requested!");
        state=TakingOff;
        tf::StampedTransform transform;
        listener.lookupTransform(m_worldFrame, m_agentFrame, ros::Time(0), transform);
        startZ = transform.getOrigin().z();
        return true;
	}
	bool land(
	        std_srvs::Empty::Request& req,
	        std_srvs::Empty::Response& res)
	{
		ROS_INFO("Landing requested!");
		state = Landing;

		return true;
	}
	bool toAutomatic(std_srvs::Empty::Request& req,
			std_srvs::Empty::Response& res) {
		ROS_INFO("toAutomatic requested!");
		state = Automatic;
		return true;
	}
	//直接获取速度，放在矩阵里面吧
	void get_vel(float dt){
		if (pre_pos == Vector3f::Zero()) {
			vel = Vector3f::Zero();
		}else{
			vel=(pos-pre_pos)/dt;
		}
		vel = Vector3f::Zero();
	}
	//中心节点计算u，这里get_u只是从中心节点获取那个消息。
	//所以要写获取u的服务？还是主题？服务吧还是
	void unpack_u() {
		u(0) = req.response.u[0];
		u(1) = req.response.u[1];
		u(2) = req.response.u[2];
	}


    void iteration(const ros::TimerEvent& e){
		tf::StampedTransform transform;
		geometry_msgs::Twist msg;
		geometry_msgs::Twist empty_msg;
		tfScalar roll, pitch, yaw;
		geometry_msgs::PoseStamped targetWorld;
		geometry_msgs::PoseStamped targetDrone;
		float dt = e.current_real.toSec() - e.last_real.toSec();
    	switch(state){
    	case TakingOff:
    		//ros的习惯是把最后的输出放在最后，而我放在前面= 0
    		listener.lookupTransform(m_worldFrame
    				,m_agentFrame,
    				ros::Time(0),transform);
            if (transform.getOrigin().z() > startZ + 0.05
            		|| thrust > 50000)
            {
				pidReset();
				m_pidZ.setIntegral(thrust / m_pidZ.ki());
				state = Automatic;
                thrust = 0;
            }
            else
            {
            	thrust += 10000 * dt;
                //一开始飘的原因是因为只是控制了thrust，没有
                //控制位置
                msg.linear.z = thrust;
				cmdPub.publish(msg);
            }
    		break;
    	case Landing:
    	{
			posGoalMsg.pose.position.z = startZ + 0.05;
            listener.lookupTransform(m_worldFrame
            		, m_agentFrame, ros::Time(0), transform);
            if (transform.getOrigin().z() <= startZ + 0.05) {
                state = Idle;
				cmdPub.publish(empty_msg);
            }
    	}
			//注意，这里没有break,也就是说x和y方向还是受控的
    	//问题：landing有一个transform，
		//automatic也有一个transform，这两个不会冲突吗？
    	case Automatic:
			req.request.id = this->agent_id;
			if (uClient.call(req)) {
				unpack_u();
				try {
					ros::Time now = ros::Time::now();
					this->listener.waitForTransform(m_worldFrame, m_agentFrame,
							now, ros::Duration(5));
					this->listener.lookupTransform(m_worldFrame, m_agentFrame,
							now, transform);
					targetWorld.header.stamp = transform.stamp_;
					targetWorld.header.frame_id = m_worldFrame;
					targetWorld.pose = posGoalMsg.pose;
					if (state == Landing) {
						targetWorld.pose.position.z = startZ + 0.05;
					}
					listener.transformPose(m_agentFrame, targetWorld,
							targetDrone);
					//现在的想法是，位置控制用那个算法，角度控制的话还是
					//pid吧
					tf::Matrix3x3(
							tf::Quaternion(targetDrone.pose.orientation.x,
									targetDrone.pose.orientation.y,
									targetDrone.pose.orientation.z,
									targetDrone.pose.orientation.w)).getRPY(
							roll, pitch, yaw);

					if (fabs(pitch) > 1.2 || fabs(roll) > 1.2) {
						throw new tf::TransformException("fan che le!");
					}
					//现在又遇到一个问题：u的计算肯定是中心节点来计算，
					//但是中心节点要怎么发给呢相应的飞机呢？
					//有两种方式：
					//第一种，把中心节点引用给其他的飞机
					//第二种，中心节点实时计算，然后吧相应的计算结果
					//给其他的飞机吧
					//第一种会引入耦合，和ros设计初衷相违背
					//所以用第二种吧～

					/**
					 * 第二个问题:这个u应该在什么地方被获取到呢？
					 * 定时器里面吧，恩没错，不然也没别的方式了
					 * timer调用了iteration方法，50Hz的频率
					 * 保证u和位置的同步性吧
					 */
					/**
					 * 第三个问题：如何设计协议？
					 * 我的想法是：中心节点就一直广播vector3信息吧
					 * 然后终端飞机就接受就行了吧
					 */

					if (internal_u_flag) {
//						msg.linear.x = m_pidX.update(0,
//								targetDrone.pose.position.x);
//						msg.linear.y = m_pidY.update(0.0,
//								targetDrone.pose.position.y);
//						msg.linear.z = m_pidZ.update(0.0,
//								targetDrone.pose.position.z);
					} else {
//						msg.linear.x = (float) zbf::constrainf(
//								-(float) u(0) * 5.0f, 10.0f, -10.0f);
//						msg.linear.y = (float) zbf::constrainf(
//								(float) u(1) * 5.0f, 10.0f, -10.0f);
//						msg.linear.z = (float) zbf::constrainf(
//								(float) (-u(2) * 6000 + 48000), 60000, 40000);
					}
					msg.linear.x = m_pidX.update(0,
							targetDrone.pose.position.x);
					msg.linear.x += (float) zbf::constrainf(-(float) 0.5 * u(0),
							1.0f,
							-1.0f);
					msg.linear.y = m_pidY.update(0.0,
							targetDrone.pose.position.y);
					msg.linear.y += (float) zbf::constrainf(-(float) 0.5 * u(1),
							1.0f, -1.0f);
					msg.linear.z = m_pidZ.update(0.0,
							targetDrone.pose.position.z);
					msg.linear.z += (float) zbf::constrainf(
							(float) (-u(2) * 1000), 1000, -1000);
//					std::cout << "targetWorld" << agent_id << ":  ["
//							<< targetWorld.pose.position.x << ","
//							<< targetWorld.pose.position.y << ","
//							<< targetWorld.pose.position.z << "]" << std::endl;
//
//					std::cout << "targetDrone" << agent_id << ":  ["
//							<< targetDrone.pose.position.x << ","
//							<< targetDrone.pose.position.y << ","
//							<< targetDrone.pose.position.z << "]" << std::endl;
//
//					std::cout << "agent" << agent_id << " output:  ["
//							<< msg.linear.x << "," << msg.linear.y << ","
//							<< msg.linear.z << "]" << std::endl;
					if (agent_id == 0) {
						log->log_in((float) targetWorld.pose.position.x);
						log->log_pause();
						log->log_in((float) targetWorld.pose.position.y);
						log->log_pause();
						log->log_in((float) targetWorld.pose.position.z);
						log->log_pause();
						log->log_in((float) targetDrone.pose.position.x);
						log->log_pause();
						log->log_in((float) targetDrone.pose.position.y);
						log->log_pause();
						log->log_in((float) targetDrone.pose.position.z);
						log->log_pause();
					} else {
						log->log_in((float) targetDrone.pose.position.x);
						log->log_pause();
						log->log_in((float) targetDrone.pose.position.y);
						log->log_pause();
						log->log_in((float) targetDrone.pose.position.z);
						log->log_pause();
						log->log_in((float) u(0));
						log->log_pause();
						log->log_in((float) u(1));
						log->log_pause();
						log->log_in((float) u(2));
						log->log_pause();
					}
					log->log_in((float) msg.linear.x);
					log->log_pause();
					log->log_in((float) msg.linear.y);
					log->log_pause();
					log->log_in((float) msg.linear.z);
					log->log_end();
					msg.angular.z = m_pidYaw.update(0.0f, yaw);
					cmdPub.publish(msg);
				} catch (tf::TransformException & ex) {
					ROS_INFO("drop out! %s", ex.what());
					msg.linear.x = 0;
					msg.linear.y = 0;
					msg.linear.z = 0;
					msg.angular.z = 0;
					cmdPub.publish(msg);
					state = Idle;
				}
			}
			break;
		case BeforeStable: {
			tf::StampedTransform transform;
			geometry_msgs::Twist msg;
			geometry_msgs::PoseStamped targetWorld;
			geometry_msgs::PoseStamped targetDrone;
			//输出位置的差，输出为transform
			// m_listener.lookupTransform(m_worldFrame, m_frame, ros::Time(0), transform);
			try {
				ros::Time now = ros::Time::now();
				listener.waitForTransform(m_worldFrame, m_agentFrame, now,
						ros::Duration(0.5));
				listener.lookupTransform(m_worldFrame, m_agentFrame, now,
						transform);

				targetWorld.header.stamp = transform.stamp_;
				targetWorld.header.frame_id = m_worldFrame;
				targetWorld.pose = posGoalMsg.pose;

				//坐标转换
				listener.transformPose(m_agentFrame, targetWorld, targetDrone);

				tfScalar roll, pitch, yaw;
				tf::Matrix3x3(
						tf::Quaternion(targetDrone.pose.orientation.x,
								targetDrone.pose.orientation.y,
								targetDrone.pose.orientation.z,
								targetDrone.pose.orientation.w)).getRPY(roll,
						pitch, yaw);

				if (fabs(pitch) > 1.2 || fabs(roll) > 1.2) {
					throw new tf::TransformException("fan che le!");
				}
				//目标是targetDrone，自身因为已经进行过了坐标转换，所以是0
				msg.linear.x = m_pidX.update(0, targetDrone.pose.position.x);
				msg.linear.y = m_pidY.update(0.0, targetDrone.pose.position.y);
				msg.linear.z = m_pidZ.update(0.0, targetDrone.pose.position.z);
				msg.angular.z = m_pidYaw.update(0.0, yaw);
				//ROS_INFO("%f,%f,%f,%f",msg.linear.x,msg.linear.y,msg.angular.z,msg.linear.z);
			} catch (tf::TransformException ex) {
				ROS_INFO("drop out! %s", ex.what());
				msg.linear.x = 0;
				msg.linear.y = 0;
				msg.linear.z = 0;
				msg.angular.z = 0;
				state = Idle;
			}
			cmdPub.publish(msg);
		}
			break;
    	case Idle:
			cmdPub.publish(empty_msg);
    		break;
    	}
    }
};


#endif /* CRAZYFLIE_ECUST_MY_CRAZYFLIE_CONTROLLER_SRC_SINGLEAGENT_HPP_ */