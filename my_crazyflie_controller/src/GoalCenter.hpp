/**
 * 这个中心节点的任务：提供目标点的位置吧
 */
#include "common.hpp"
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

/**
 * 这个中心节点的任务：提供目标点的位置吧
 *
 */

//#define JOY_CONTROL
#define STICK_CONTROL


using namespace Eigen;
class GoalCenter {
public:
	GoalCenter(int topologies[USE_AGENT][USE_AGENT], int* frame_ids) :
			topologies_switch(false), internal_formation(false), frequency(
					0.02), lineNum(0)
	{
		ros::NodeHandle nh;
		int i;
		zbf::eigen_topologies(this->topologies, topologies);

		for (i = 0; i < USE_AGENT; i++) {
			this->frame_ids[i] = frame_ids[i];
			//这个类需要订阅这样的消息：
			posSub[i] = nh.subscribe(zbf::agent_str_pos(i), 10,
					&GoalCenter::sub_pos,
					this);
			velSub[i] = nh.subscribe(zbf::agent_str_vel(i), 10,
					&GoalCenter::sub_vel,
					this);
			accSub[i] = nh.subscribe(zbf::agent_str_acc(i), 10,
					&GoalCenter::sub_acc, this);
			//这个类广播这样的消息：
			posGoalPub[i] = nh.advertise<my_crazyflie_controller::IDPose>(
					zbf::center_str_pos_goal(i), 10);
			velGoalPub[i] = nh.advertise<my_crazyflie_controller::IDPose>(
					zbf::center_str_vel_goal(i), 10);
			accGoalPub[i] = nh.advertise<my_crazyflie_controller::IDPose>(
					zbf::center_str_acc_goal(i), 10);
			posGoalPubNoID[i] = nh.advertise<geometry_msgs::PoseStamped>(
					zbf::center_str_pos_goal_no_id(i), 10);
			velGoalPubNoID[i] = nh.advertise<geometry_msgs::PoseStamped>(
					zbf::center_str_vel_goal_no_id(i), 10);
			accGoalPubNoID[i] = nh.advertise<geometry_msgs::PoseStamped>(
					zbf::center_str_acc_goal_no_id(i), 10);
		}
#ifdef JOY_CONTROL
		joyGoalSub = nh.subscribe("/joy_goal", 10,
				&GoalCenter::joy_goal_callback,
				this);
#endif
#ifdef STICK_CONTROL
		stickGoalSub = nh.subscribe("/stick_goal", 10,
				&GoalCenter::stick_goal_callback, this);
#endif
		joySub = nh.subscribe("/joy", 2, &GoalCenter::joy_callback, this);

		helper.read("/home/zbf/dance1.csv", dataFromFile);
		lineNum = helper.CountLines("/home/zbf/dance1.csv");
	}
	void run() {
		ros::NodeHandle nh("~");
		ros::Timer timer = nh.createTimer(ros::Duration(frequency),
				&GoalCenter::pub_msg,
				this);
		ros::spin();
	}
private:
	/**
	 * @params i 就是运行到了第几个点吧
	 * @params theta 以0,0和1,0 的夹角为0度，然后旋转吧，单位为角度吧
	 * dt 和circle-time的时间单位应该是一样的才对
	 */
	V3 generate_circle(double i, double theta,
			double radius,
			double circle_time, double dt, double x, double y, double z) {
		static V3 v3;
		v3.x = x
				+ radius
						* sin(
								MY_PI / 180
										* (theta + dt * i / circle_time * 360.0));
		v3.y = y
				+ radius
						* cos(
								MY_PI / 180
										* (theta + dt * i / circle_time * 360.0));
		v3.z = z;
		return v3;
	}
	V3 generate_circle(double i, double theta, double radius,
			double circle_time, double dt) {
		static V3 v3;
		ROS_FATAL("need fix");
		v3.x = joy_goal.pose.position.x + radius * sin(
		MY_PI / 180 * (theta + dt * i / circle_time * 360.0));
		v3.y = joy_goal.pose.position.y + radius * cos(
		MY_PI / 180 * (theta + dt * i / circle_time * 360.0));
		v3.z = joy_goal.pose.position.z;
		return v3;
	}
	V3 generate_circle(double i, double theta, double radius,
			double circle_time, double dt, const V3& v) {
		static V3 v3;
		v3.x = v.x + radius * sin(
		MY_PI / 180 * (theta + dt * i / circle_time * 360.0));
		v3.y = v.y + radius * cos(
		MY_PI / 180 * (theta + dt * i / circle_time * 360.0));
		v3.z = v.z;
		return v3;
	}
	V3 generate_rotate(double radius, const int id, int i, double dt,
			double period) {
		V3 v;
		v.x = 0.6
				* cos(
						2 * MY_PI
								* ((float) id / USE_AGENT
										+ (float) i * dt / period));
		v.y = 0.6
				* sin(
						2 * MY_PI
								* ((float) id / USE_AGENT
										+ (float) i * dt / period));
		v.z = 0.8;
		return v;
	}

	V3 generate_regular_shape(double radius, const int id, const V3& center) {
		V3 v;
		v.x = center.x + radius * cos(2 * MY_PI * (id / (double) USE_AGENT));

		v.y = center.y + radius * sin(2 * MY_PI * (id / (double) USE_AGENT));
		v.z = center.z;
		return v;
	}
	V3 generate_regular_shape2(double radius, const int id, const V3& center) {
		V3 v;
		v.x = center.x
				+ radius * cos(2 * MY_PI * (id * 2 / (double) USE_AGENT));
		v.y = center.y
				+ radius * sin(2 * MY_PI * (id * 2 / (double) USE_AGENT));
		if (id < USE_AGENT / 2)
			v.z = center.z;
		else
			v.z = center.z - 0.4;
		return v;
	}
	//和generate_ragular_shape组合使用，效果更佳
	V3 generate_triangle_center(int i, double dt, double period,
			double radius) {
		V3 v; //1,0,0.8
		double num = period / dt;
		if (i > 0 && i < num / 8) {
			v.x = radius * 1;
			v.y = radius * 8 * i * dt / period;
		} else if (i > num / 8 && i < num * 3 / 8) {
			v.x = radius * (num / 4 - i) * 8 * dt / period;
			v.y = radius * 1;
		} else if (i > num * 3 / 8 && i < num * 5 / 8) {
			v.x = -1 * radius;
			v.y = radius * (num / 2 - i) * 8 * dt / period;
		} else if (i > num * 5 / 8 && i < num * 7 / 8) {
			v.x = radius * (i - num * 3 / 4) * 8 * dt / period;
			v.y = -1 * radius;
		} else if (i > num * 7 / 8 && i < period) {
			v.x = radius * 1;
			v.y = radius * (i - num) * 8 * dt / period;
		} else {
			v.x = radius * 1;
			v.y = 0;
		}
		v.z = 0.8;
		return v;
	}
	//控制方法应该多弄几个函数出来吧。
	//首先是用手柄控制leader
	//其次是画圆

	void hover(geometry_msgs::PoseStamped & msg, const int id) {
		//		msg.pose.position.x = 0.6 * cos(2 * MY_PI * (float) id / USE_AGENT);
		//		msg.pose.position.y = 0.6 * sin(2 * MY_PI * (float) id / USE_AGENT);
		//		msg.pose.position.z = 0.05;

		//		msg.pose.position.x = -1.25 + id * 0.5;
		//		msg.pose.position.y = 0;
		//		msg.pose.position.z = 0.4;

		msg.pose.position.x = v[id].x;
		msg.pose.position.y = v[id].y;
		msg.pose.position.z = v[id].z;
	}
	void hover_vel(geometry_msgs::PoseStamped & msg, const int id) {
		//		msg.pose.position.x = 0.6 * cos(2 * MY_PI * (float) id / USE_AGENT);
		//		msg.pose.position.y = 0.6 * sin(2 * MY_PI * (float) id / USE_AGENT);
		//		msg.pose.position.z = 0.05;

		//		msg.pose.position.x = -1.25 + id * 0.5;
		//		msg.pose.position.y = 0;
		//		msg.pose.position.z = 0.4;

		msg.pose.position.x = vv[id].x;
		msg.pose.position.y = vv[id].y;
		msg.pose.position.z = vv[id].z;
	}
	void rotate(geometry_msgs::PoseStamped & msg, const int id, const int i,
			const double dt, const double period) {
		msg.pose.position.x =
				0.6
						* cos(
								2 * MY_PI
										* ((float) id / USE_AGENT
										+ (float) i * dt / period));
		msg.pose.position.y =
				0.6
						* sin(
								2 * MY_PI
										* ((float) id / USE_AGENT
										+ (float) i * dt / period));
		msg.pose.position.z = 0.8;
	}

	void joy_control(geometry_msgs::PoseStamped & msg, const int id) {
		msg.pose.position.x = joy_goal.pose.position.x;
		msg.pose.position.y = joy_goal.pose.position.y;
		msg.pose.position.z = joy_goal.pose.position.z;
	}

	V3 generate_sin(double i, double prase, double amplitude, double period,
			double dt, double x, double y, double z) {
		static V3 v3;
		double t;
		t = i * dt / period;
		v3.x = x - amplitude * cos(prase + 2 * MY_PI * t);
		v3.y = y + amplitude * sin(prase + 2 * MY_PI * t);
		v3.z = z;
		return v3;
	}

	V3 generate_sin_vel(double i, double prase, double amplitude, double period,
			double dt) {
		static V3 v3;
		double t;
		t = i * dt / period;
		v3.x = amplitude * 2 * MY_PI * dt / period
						* sin(prase + 2 * MY_PI * t);
		v3.y = amplitude * 2 * MY_PI * dt / period
						* cos(prase + 2 * MY_PI * t);
		v3.z = 0;
		return v3;
	}

	void linear_sin_control(geometry_msgs::PoseStamped & msg, const int id) {
		msg.pose.position.x = v_base[id].x;
		msg.pose.position.y = v_base[id].y;
		msg.pose.position.z = v_base[id].z;
	}

	double frequency;
	int frame_ids[USE_AGENT];
	TopoMatrix topologies;
	bool topologies_switch;
	bool internal_formation;
	ros::Publisher posGoalPub[USE_AGENT];
	ros::Publisher velGoalPub[USE_AGENT];
	ros::Publisher accGoalPub[USE_AGENT];
	ros::Publisher posGoalPubNoID[USE_AGENT];
	ros::Publisher velGoalPubNoID[USE_AGENT];
	ros::Publisher accGoalPubNoID[USE_AGENT];
	ros::Subscriber posSub[USE_AGENT];
	ros::Subscriber velSub[USE_AGENT];
	ros::Subscriber accSub[USE_AGENT];

	CSVHelper helper;
	std::vector<std::vector<double>> dataFromFile;
	int lineNum;
#ifdef JOY_CONTROL
	ros::Subscriber joyGoalSub;
#endif
#ifdef STICK_CONTROL
	ros::Subscriber stickGoalSub;
#endif
	ros::Subscriber joySub;

	MyFilter filters[USE_AGENT];
	MyFilter velFilters[USE_AGENT];
	geometry_msgs::PoseStamped pos[USE_AGENT];
	geometry_msgs::PoseStamped vel[USE_AGENT];
	geometry_msgs::PoseStamped acc[USE_AGENT];
	V3 v[USE_AGENT];
	V3 pre_v[USE_AGENT];
	V3 vv[USE_AGENT];
	V3 v_base[USE_AGENT];
	geometry_msgs::PoseStamped joy_goal;
	geometry_msgs::PoseStamped stick_goal;
	sensor_msgs::Joy joy;
	/*
	 * 问题：中间节点还需要什么信息？
	 * 不需要反馈吗？
	 * 如果这么写的话，反馈是没有的。
	 * 如果订阅位置和速度的话，那就是有反馈的。
	 */
	void sub_pos(const geometry_msgs::PoseStamped::ConstPtr & msg) {
		int id = get_frame_id(msg->header.frame_id);
		pos[id] = *msg;
	}
	void sub_vel(const geometry_msgs::PoseStamped::ConstPtr & msg) {
		int id = get_frame_id(msg->header.frame_id);
		vel[id] = *msg;
	}
	void sub_acc(const geometry_msgs::PoseStamped::ConstPtr & msg) {
		int id = get_frame_id(msg->header.frame_id);
		acc[id] = *msg;
	}
	void joy_callback(const sensor_msgs::Joy::ConstPtr & msg) {
		joy = *msg;
		static int last_joy_value_5 = 0;
		static int last_joy_value_4 = 0;
		if (joy.buttons[5] == 1 && last_joy_value_5 == 0) {
			//改变拓扑结构
			ROS_INFO("topologies changed!");
			if (topologies_switch) {
				//topologies << 0, 0, 0, -1, 1, 0, 0, -1, 1;
			} else {
				//topologies << 0, 0, 0, 0, 1, -1, -1, 0, 1;
			}
			topologies_switch = !topologies_switch;
		} else if (joy.buttons[4] == 1 && last_joy_value_4 == 0) {
			internal_formation = true;
			ROS_INFO("internal formation launched");
		}
		last_joy_value_5 = joy.buttons[5];
	}

	void joy_goal_callback(const geometry_msgs::PoseStamped::ConstPtr & msg) {
		joy_goal = *msg;
	}
	void stick_goal_callback(const geometry_msgs::PoseStamped::ConstPtr & msg) {
		stick_goal = *msg;
	}
	V3 msg2V3(const geometry_msgs::PoseStamped& msg, V3 offset) {
		V3 v;
		v.x = msg.pose.position.x + offset.x;
		v.y = msg.pose.position.y + offset.y;
		v.z = msg.pose.position.z + offset.z;
		return v;
	}
	V3 msg2V3(const geometry_msgs::PoseStamped& msg) {
		V3 v;
		v.x = msg.pose.position.x;
		v.y = msg.pose.position.y;
		v.z = msg.pose.position.z;
		return v;
	}
	void pub_msg(const ros::TimerEvent & e) {
		int i, j;
		double dt;
		static int times = 0;
		V3 vCenter;
		dt = e.current_real.toSec() - e.last_real.toSec();
		for (i = 0; i < USE_AGENT; i++) {
			filters[i].update(msg2V3(pos[i]));
			velFilters[i].update(msg2V3(vel[i]));
//			ROS_INFO("filter%d value: %lf", i, filters[i].output.z);
		}
		//唐老师的想法，用这个
//		vCenter = generate_triangle_center(times, dt, 30, 0.8);
//		for (i = 0; i < USE_AGENT; i++) {
//			if (times < 1500) {
//				v[i] = generate_regular_shape(0.5, i, vCenter);
//			} else {
//				v[i] = generate_regular_shape2(0.5, i, vCenter);
//			}
//		}
		//唐老师的想法，第二个圈，分两层
//		vv[0].x = 0;
//		vv[0].y = 0;
//		vv[0].z = 0;

		if (internal_formation) {
			//v[0] = generate_sin(times, 0, 0.6, 10, 0.02, 0, 0, 0.8);
			if (!topologies_switch) {
#if USE_AGENT>0
				v[0] = generate_sin(times, 0, 0.6, 17, 0.02, -0.3, 0, 0.8);
#endif
#if USE_AGENT>1
				v[1] = filters[0].output + V3(0, -1, 0) * 0.6;
#endif
#if USE_AGENT>2
				v[2] = filters[0].output + V3(0, -2, 0) * 0.6;
#endif
#if USE_AGENT>3
				v[3] = filters[0].output + V3(1, 0, 0) * 0.6;
#endif
#if USE_AGENT>4
				v[4] = filters[0].output + V3(1, -1, 0) * 0.6;
#endif
#if USE_AGENT>5
				v[5] = filters[0].output + V3(1, -2, 0) * 0.6;
#endif
			} else {
#if USE_AGENT>0
				v[0] = generate_sin(times, 0, 0.6, 17, 0.02, -0.3, 0, 0.8);
#endif
#if USE_AGENT>1
				v[1] = filters[2].output + V3(-MY_SQRT_1_DIV_3, 1, 0) * 0.6;
#endif
#if USE_AGENT>2
				v[2] = filters[0].output + V3(0, -2, 0) * 0.6;
#endif
#if USE_AGENT>3
				v[3] = filters[0].output + V3(1, 0, 0) * 0.6;
#endif
#if USE_AGENT>4
				v[4] = filters[5].output
						+ V3(MY_SQRT_1_DIV_3, 1, 0) * 0.6;
#endif
#if USE_AGENT>5
				v[5] = filters[0].output + V3(1, -2, 0) * 0.6;
#endif

			}
//			for (i = 0; i < USE_AGENT; i++) {
//				v[i] = generate_rotate(0.6, i, times, 0.02, 20.0);
//			}

			//开始按照前一个时刻的位置来飞吧。

//从文件中读入，用这个
//			for (i = 0; i < USE_AGENT; i++) {
//				v[i].x = dataFromFile[times / 4][i * 3];
//				v[i].y = dataFromFile[times / 4][i * 3 + 1];
//				v[i].z = dataFromFile[times / 4][i * 3 + 2];
//			}
			
//从stick_goal读入，用这个
//			v[0].x = stick_goal.pose.position.x;
//			v[0].y = stick_goal.pose.position.y;
//			v[0].z = stick_goal.pose.position.z;


			//第一种情况：绝对位置
//			v_base[0] = generate_sin(times, 0, 1, 10, frequency, 0, 0, 0.8);
//			v_base[1] = generate_sin(times, 0, 1, 10, frequency, 0.5, 0.5, 0.8);
//			v_base[2] = generate_sin(times, 0, 1, 10, frequency, 1, 0, 0.8);
			//第二种情况：相对位置
//			v_base[0] = generate_sin(times, 0, 1, 10, frequency, -1, 0, 0.8);
//			if (topologies_switch) {
////				v_base[1] = msg2V3(pos[0], V3(0.5, 0.5, 0));
////				v_base[2] = msg2V3(pos[1], V3(0.5, -0.5, 0));
//				v_base[1].x = filters[0].output.x + 0.5;
//				v_base[1].y = filters[0].output.y + 0.5;
//				v_base[1].z = filters[0].output.z;
//				v_base[2].x = filters[1].output.x + 0.5;
//				v_base[2].y = filters[1].output.y - 0.5;
//				v_base[2].z = filters[1].output.z;
//			} else {
////				v_base[2] = msg2V3(pos[0], V3(1, 0, 0));
////				v_base[1] = msg2V3(pos[2], V3(-0.5, 0.5, 0));
////				v_base[2] = filters[0].output + V3(1, 0, 0);
////				v_base[1] = filters[2].output + V3(-0.5, 0.5, 0);
//				v_base[2].x = filters[0].output.x + 1;
//				v_base[2].y = filters[0].output.y;
//				v_base[2].z = filters[0].output.z;
//				v_base[1].x = filters[2].output.x - 0.5;
//				v_base[1].y = filters[2].output.y + 0.5;
//				v_base[1].z = filters[2].output.z;
//			}
			times++;
		} else {
#if USE_AGENT==1
			v[0].x = 0;
			v[0].y = 0;
			v[0].z = 0.8;
#endif
#if USE_AGENT==3
			for (i = 0; i < 1; i++) {
				for (j = 0; j < 3; j++) {
					v[i * 3 + j].x = i * 0.6 - 1 - 0.3;
					v[i * 3 + j].y = -j * 0.6;
					v[i * 3 + j].z = 0.8;
				}
			}
#endif
#if USE_AGENT==6
			for (i = 0; i < 2; i++) {
				for (j = 0; j < 3; j++) {
					v[i * 3 + j].x = i * 0.6 - 1 - 0.3;
					v[i * 3 + j].y = -j * 0.6;
					v[i * 3 + j].z = 0.8;
				}
			}
#endif
		}
		for (i = 0; i < USE_AGENT; i++) {
			vv[i] = (v[i] - pre_v[i]);
			vv[i] = vv[i] / dt;
			pre_v[i] = v[i];
			//ROS_INFO("%f,%f,%f", vv[i].x, vv[i].y, vv[i].z);
		}
		for (i = 0; i < USE_AGENT; i++) {
			pub_pos_goal(i, dt);
			pub_vel_goal(i, dt);
			pub_acc_goal(i, dt);
		}
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

	const std::string get_frame(const int id) {
		return "/vicon/C" + std::to_string(frame_ids[id]) + "/C"
				+ std::to_string(frame_ids[id]);
	}

	void pub_pos_goal(int id, double dt) {
		//需要进行坐标转换
		int i;
		//算法goes here
		static my_crazyflie_controller::IDPose id_msg;
		static geometry_msgs::PoseStamped msg;
		msg.header.frame_id = "/world";
//		msg.header.frame_id = get_frame(id);
		msg.header.seq += 1;
		msg.header.stamp = ros::Time(0);

		//如果一行全是0，那就是leader了
		//暂时先不控制角度，只控制位置吧

		if (!internal_formation) {
			//先飞到指定高度
			hover(msg, id);
		} else {
//			if (id == 0) {
//				//如果是leader的话
//				msg.pose.position.x = joy_goal.pose.position.x;
//				msg.pose.position.y = joy_goal.pose.position.y;
//				msg.pose.position.z = joy_goal.pose.position.z;
//			} else {
//				//如果不是leader的话
//				//u2
//				msg.pose.position.x = 0;
//				msg.pose.position.y = 0;
//				msg.pose.position.z = 0;
//				for (i = 0; i < USE_AGENT; i++) {
//					if (id == i)
//						continue;
//					msg.pose.position.x += -topologies(id, i)
//							* (pos[i].pose.position.x + 0.5 * (id - i));
//					msg.pose.position.y += -topologies(id, i)
//							* (pos[i].pose.position.y);
//					msg.pose.position.z += -topologies(id, i)
//							* (pos[i].pose.position.z);
//				}
//
//				//u1+u2
//				msg.pose.position.x = joy_goal.pose.position.x + id * 0.5;
//				msg.pose.position.y = joy_goal.pose.position.y;
//				msg.pose.position.z = joy_goal.pose.position.z;
//			}
			msg.pose.position.x = v[id].x;
			msg.pose.position.y = v[id].y;
			msg.pose.position.z = v[id].z;
		}

		//不控制角度，只控制位置
		msg.pose.orientation.w = 1;
		msg.pose.orientation.x = 0;
		msg.pose.orientation.y = 0;
		msg.pose.orientation.z = 0;
		posGoalPubNoID[id].publish(msg);
		//算法goes here
		id_msg.id = id;
		id_msg.pos = msg;
		posGoalPub[id].publish(id_msg);
	}
	void pub_vel_goal(int id, double dt) {
		//算法goes here
		static my_crazyflie_controller::IDPose id_msg;
		static geometry_msgs::PoseStamped msg;
		msg.header.frame_id = "/world";
//		msg.header.frame_id = get_frame(id);
		msg.header.seq += 1;
		msg.header.stamp = ros::Time(0);
		msg.pose.position.x = vv[id].x;
		msg.pose.position.y = vv[id].y;
		msg.pose.position.z = vv[id].z;
//		msg.pose.position.x = 0;
//		msg.pose.position.y = 0;
//		msg.pose.position.z = 0;
		//ROS_INFO("%f,%f,%f", vv[id].x, vv[id].y, vv[id].z);
		msg.pose.orientation.w = 1;
		msg.pose.orientation.x = 0;
		msg.pose.orientation.y = 0;
		msg.pose.orientation.z = 0;
		velGoalPubNoID[id].publish(msg);
		//算法goes here
		id_msg.id = id;
		id_msg.pos = msg;
		velGoalPub[id].publish(id_msg);
	}
	void pub_acc_goal(int id, double dt) {
		//算法goes here
		static my_crazyflie_controller::IDPose id_msg;
		static geometry_msgs::PoseStamped msg;
		msg.header.frame_id = "/world";
//		msg.header.frame_id = get_frame(id);
		msg.header.seq += 1;
		msg.header.stamp = ros::Time::now();
		msg.pose.position.x = 0;
		msg.pose.position.y = 0;
		msg.pose.position.z = 0;
		msg.pose.orientation.w = 1;
		msg.pose.orientation.x = 0;
		msg.pose.orientation.y = 0;
		msg.pose.orientation.z = 0;
		accGoalPubNoID[id].publish(msg);
		//算法goes here
		id_msg.id = id;
		id_msg.pos = msg;
		accGoalPub[id].publish(id_msg);
	}
};
