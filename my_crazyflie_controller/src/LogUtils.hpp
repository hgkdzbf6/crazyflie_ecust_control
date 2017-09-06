/*
 * LogUtils.hpp
 *
 *  Created on: 2017年8月30日
 *      Author: zbf
 */

#ifndef CRAZYFLIE_ECUST_MY_CRAZYFLIE_CONTROLLER_SRC_LOGUTILS_HPP_
#define CRAZYFLIE_ECUST_MY_CRAZYFLIE_CONTROLLER_SRC_LOGUTILS_HPP_

/*
 * LogUtils.h
 *
 *  Created on: 2017年6月19日
 *      Author: zbf
 */

#ifndef LOGUTILS_H_
#define LOGUTILS_H_
#include <iostream>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <time.h>
#include <string>
#include <string.h>
#include <unistd.h>
//#include <cstdint>

typedef float single;
#define DATA_NUM 6
class LogUtils {
public:
	LogUtils(int i = 0) {
		// TODO Auto-generated constructor stub
		createName(i);
		_file = new std::ofstream(this->fileName.c_str());
		//_file_read=new std::ifstream("./setting.ini");
		_file->is_open();
	}
	void log_in(int num) {
		*_file << num;
	}
	void log_in(long num) {
		*_file << num;
	}
	void log_in(std::string num) {
		*_file << num;
	}
	void log_in(single num) {
		*_file << num;
	}
	void log_pause() {
		*_file << ",";
	}
	void log_end() {
		*_file << std::endl;
		_file->flush();
	}
	void log(short int* nums) {
		int i;
		log_in(nums[0]);
		for (i = 1; i < DATA_NUM; i++) {
			log_pause();
			log_in(nums[i]);
		}
		log_end();
	}
	void log(long* nums) {
		int i;
		log_in(nums[0]);
		for (i = 1; i < DATA_NUM; i++) {
			log_pause();
			log_in(nums[i]);
		}
		log_end();
	}
	void log(single* nums) {
		int i;
		log_in(nums[0]);
		for (i = 1; i < DATA_NUM; i++) {
			log_pause();
			log_in(nums[i]);
		}
		log_end();
	}
	bool readIn();
	~LogUtils() {
		// TODO Auto-generated destructor stub
		_file->close();
		_file_read->close();
		delete _file;
		delete _file_read;
		_file = NULL;
		_file_read = NULL;
	}
private:
	std::string fileName;
	std::ostringstream timeStream;
	std::string timeStr;
	std::string createName() {
		return createName(0);
	}
	std::string createName(int i) {
		time_t tt = time(NULL);
		tm* t = localtime(&tt);
		timeStream << 1900 + t->tm_year << std::setfill('0') << std::setw(2)
				<< 1 + t->tm_mon << std::setfill('0') << std::setw(2)
				<< t->tm_mday << std::setfill('0') << std::setw(2) << t->tm_hour
				<< std::setfill('0') << std::setw(2) << t->tm_min
				<< std::setfill('0') << std::setw(2) << t->tm_sec;
		timeStr = timeStream.str();
		fileName = "/home/zbf/data" + std::to_string(i) + "_" + timeStr
				+ ".csv";
		return fileName;
	}
	std::string createHeader();
	int data[DATA_NUM];
	std::ofstream* _file;
	std::ifstream* _file_read;
};

#endif /* LOGUTILS_H_ */




#endif /* CRAZYFLIE_ECUST_MY_CRAZYFLIE_CONTROLLER_SRC_LOGUTILS_HPP_ */
