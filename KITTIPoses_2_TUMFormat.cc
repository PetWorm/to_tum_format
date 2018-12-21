/**
* This file is for transforming kitti ground truth into TUM Data Format.
* Editted by Xiaochen Qiu (Beihang University)
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <iomanip>

#include<opencv2/core/core.hpp>

#include "Converter.h"

using namespace std;
using namespace ORB_SLAM2;

int main(int argc, char **argv)
{
	if(argc != 4)
	{
		cerr << endl << "Usage: ./KITTIPoses_2_TUMFormat path_to_poses path_to_times path_to_result" << endl;
		return 1;
	}
	
	// group truth文件
	ifstream if_poses(argv[1]);
	if(!if_poses)
	{
		cerr << endl << "Can't find POSES_FILE in this location: " << argv[1] << endl;
		return 1;
	}
	// 时间序列文件
	ifstream if_times(argv[2]);
	if(!if_times)
	{
		cerr << endl << "Can't find TIMES_FILE in this location: " << argv[2] << endl;
		return 1;
	}
	// 转换结果文件
	ofstream of_result(argv[3]);
	of_result << fixed;
	
	// 读取数据
	string poseLine;
	string timeLine;
	int ierrorCounter_poses = 0;
	int ierrorCounter_times = 0;
	while(!if_poses.eof())
	{
		if(!if_times.eof())
		{
			// 读取本行的时间
			getline(if_times,timeLine,'\n');
			stringstream timetream(timeLine);
			float ftime;
			vector<float> vtimeline;
			while(timetream >> ftime)
				vtimeline.push_back(ftime);
			if(vtimeline.size()==1)		// 时间
				of_result << setprecision(6) << ftime << " ";
			else
				ierrorCounter_poses++;
			
			// 转换本行表达的位姿
			getline(if_poses,poseLine,'\n');
			stringstream posetream(poseLine);
			float fpose;
			vector<float> vposeline;
			while(posetream >> fpose)		// 分解每行中的元素（以空格为分隔符）
				vposeline.push_back(fpose);
			if(vposeline.size()==12)	// 位姿
			{
				cv::Mat R = ( cv::Mat_<float>(3,3) << vposeline[0], vposeline[1], vposeline[2], 
							  vposeline[4], vposeline[5], vposeline[6], 
							  vposeline[8], vposeline[9], vposeline[10]
							);																			// 存储第i个相机系到第0个相机系的旋转矩阵
				vector<float> q = Converter::toQuaternion(R);
				
				cv::Mat t = ( cv::Mat_<float>(3,1) << vposeline[3], vposeline[7], vposeline[11] );		// 存储第i个相机原点位置在第0个相机系下的坐标
				
				of_result << setprecision(7) << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2) << " " 
							<< q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
			}
			else
				ierrorCounter_times++;
			
			// 缺项对比
			if(ierrorCounter_poses!=ierrorCounter_times)
			{
				cerr << "One of the FILE have an empty line while the other FILE does not." << endl;
				if_poses.close();
				if_times.close();
				of_result.close();
				return 1;
			}
		}
		else				// 位姿文件中还有数据但时间序列文件已经读完
		{
			cerr << endl << "POSES_FILE and TIMES_FILE have unmatched line number: TIMES_FILE has less line number!" << endl;
			if_poses.close();
			if_times.close();
			of_result.close();
			return 1;
		}
	}
	if(!if_times.eof())		// 位姿文件已经读完但时间序列文件中还有数据
	{
		cerr << endl << "POSES_FILE and TIMES_FILE have unmatched line number: POSES_FILE has less line number!" << endl;
		if_poses.close();
		if_times.close();
		of_result.close();
		return 1;
	}
	else
	{
		if_poses.close();
		if_times.close();
		of_result.close();
		cout << "Transformation completed!" << endl;
	}
	
	return 0;
}