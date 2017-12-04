/*
 * CsvHelper.hpp
 *
 *  Created on: 2017年9月7日
 *      Author: zbf
 */

#ifndef CRAZYFLIE_ECUST_MY_CRAZYFLIE_CONTROLLER_SRC_CSVHELPER_HPP_
#define CRAZYFLIE_ECUST_MY_CRAZYFLIE_CONTROLLER_SRC_CSVHELPER_HPP_


#include <iostream>
#include <fstream>
using namespace std;
//想要读入csv文件吧
class CSVHelper {
public:
	int CountLines(std::string filename) //获取文件的行数
			{
		int n = 0;
		string temp;
		if (!file.is_open()) {
			file.open(filename, ios::in); //ios::in 表示以只读的方式读取文件
		}
		if (file.fail()) //文件打开失败:返回0
		{
			return 0;
		} else //文件存在,返回文件行数
		{
			while (getline(file, temp)) {
				n++;
			}
			return n;
		}
	}
	//作用是把字符串都分门别类放好
	int read(std::string filename, vector<vector<double> > &vlStr) {
		if (!file.is_open()) {
			file.open(filename, std::ios::in);
		}
		if (!(file))
			return (-errno);
		string _strIn("");
		while (getline(file, _strIn)) {
			/// 每行的源字符串
			char* _pcSrc = (char*) _strIn.c_str();
			/// 存储一行‘,'分隔解析后的各个元素
			vector<double> _ltStr;
			/// Parse values in this line
			while (*_pcSrc != '\0') {
				/// string to hold this value
				string _strElem("");
				/// 针对每个字符分析
				if (*_pcSrc == '"') {
					/// Bump past opening quote
					_pcSrc++;
					/// Parse quoted value
					while (*_pcSrc != '\0') {
						/// Test for quote character
						if (*_pcSrc == '"') {
							/// Found one quote
								_pcSrc++;
							// If pair of quotes, keep one
							// Else interpret as end of value
							if (*_pcSrc != '"') {
								_pcSrc++;
								break;
							}
							}
						/// Add this character to value
						_strElem.push_back(*_pcSrc++);
						}
				} else {
					// Parse unquoted value
					while (*_pcSrc != '\0' && *_pcSrc != ',')
						_strElem.push_back(*_pcSrc++);
					// Advance to next character (if not already end of string)
					if (*_pcSrc != '\0')
						_pcSrc++;
				}
				/// Add this string to container
				_ltStr.push_back(stod(_strElem));
			}
			/// 分析后的一行文件内容所得的元素列表添加到容器中
			vlStr.push_back(_ltStr);
			/// 归零，防止下次分析旧的数据。
			_strIn.assign("");
		}

		return 0;
	}
	void close() {
		file.close();
	}
private:
	std::ifstream file;
};



#endif /* CRAZYFLIE_ECUST_MY_CRAZYFLIE_CONTROLLER_SRC_CSVHELPER_HPP_ */
