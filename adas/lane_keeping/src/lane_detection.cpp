#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32.h>

#include <iostream>
#include <fstream>
#include <stdio.h>
#include <unistd.h>
#include <vector>
#include <chrono>
#include <tuple>
#include <math.h>
#include "subfunctions.h"

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/cudawarping.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudaarithm.hpp>

using namespace std;
using namespace cv;

class lane_deviation{
private:
	int row;
	double desire_speed; // initialize desired speed
	float radar_reading; // initialize radar reading
	chrono::high_resolution_clock::time_point past_time; //initialize timer for PID derivative term
	double dx,predx,dtheta; // PID terms
	double Kp,Kd,Kp_theta,PD,prePD,increment; // PID coefficients
	Mat lane_hls;
	
	ros::NodeHandle n;
	ros::Publisher pub;
	ros::Subscriber sub1;
	ros::Subscriber sub2;


public:
	ofstream data;

	lane_deviation(){
		ROS_INFO("OpenCV version: %d.%d",CV_MAJOR_VERSION,CV_MINOR_VERSION);
		
		// Open files
		data.open("/home/nvidia/lane_keeping_data.txt");
	
		// Define desired Speed
		n.param("/lane_detection/desire_speed", desire_speed, 0.0);
		n.param("/lane_detection/Kp", Kp, 0.0);
		n.param("/lane_detection/Kd", Kd, 0.0);
		n.param("/lane_detection/Kp_theta", Kp_theta, 0.0);
		n.param("/lane_detection/increment", increment, 0.0);
		ROS_INFO("Desire Speed is: %.1f m/s", desire_speed);
		ROS_INFO("Kp is: %.4f, Kp_theta is: %.4f", Kp, Kp_theta);
			
		// Look ahead row
		row = -570*(desire_speed - 1);
		if (row > 399){
			row = 399;
		}
		ROS_INFO("Row index: %d", row);
		
		// Define initial time for PID controller derivative term
		past_time = chrono::high_resolution_clock::now();
		
		// Define publisher and subscriber
		pub = n.advertise<sensor_msgs::Joy>("/vesc/joy",1);
		sub1 = n.subscribe("/front_cam/lane", 5, &lane_deviation::visionCB,this);
		sub2 = n.subscribe("/front_radar", 1, &lane_deviation::radarCB,this);
	}
 	
	double sum(Mat pts){
		double ave;
		for(int i = 0; i < pts.rows; i++){
			ave += pts.at<int>(i,0);
		}
		ave = ave/pts.rows;
		if (ave >= 0 && ave <=799){
			return ave;
		}
		else{
			return 0.0;
		}
	}
	

	void radarCB(const std_msgs::Float32::ConstPtr& msg){
		radar_reading = msg->data;
	}


	void visionCB(const sensor_msgs::Image::ConstPtr& msg)
	{
		if(radar_reading >= 20)
		{
			/****************************************************************************************************/
			cv_bridge::CvImagePtr cv_ptr;
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
			lane_hls = cv_ptr->image;
			/****************************************************************************************************/
			
			/****************************************************************************************************/
			// Find center Points
			vector<double> center_x;
			vector<double> center_y;
			for(int i = 0; i < 20; i++){
				Mat ind;
				//findNonZero(lane_hls_erosion.row((i+1)*20-1), ind);
				findNonZero(lane_hls.row((i+1)*20-1), ind);
				double result = sum(ind);
				if (result > 0){
					center_x.push_back(result);
					center_y.push_back((i+1)*20-1);
				}
			}
	
			// Curve fitting of second order polynomial
			vector<double> a = polynomialfit(center_y, center_x, 2);
			/****************************************************************************************************/
			
			/****************************************************************************************************/
			// In case curve fitting will fail
			if (!isnan(a[0]) && !isnan(a[1]) & !isnan(a[2])){
				// Calculate center deviation
				dx = 400 - (a[2]*row*row + a[1]*row + a[0] - 135);
				dtheta = atan(2*a[2]*row + a[1]);
				//cout << a[2] << "," << a[1] << "," << a[0] << endl;
	
				// PD control
				sensor_msgs::Joy joy_command;
				joy_command.header.stamp = ros::Time::now();
				chrono::high_resolution_clock::time_point current_time = chrono::high_resolution_clock::now();
				chrono::duration<double> dt = current_time - past_time;
				//PD = Kp*dx + Kd*(dx-predx)/dt.count() + Kp_theta*dtheta + Kd_theta*(dtheta-predtheta)/dt.count();
				
				if (desire_speed < 0.3){
					PD = Kp_theta*dtheta + atan((Kp*dx + Kd*(dx - predx)/dt.count())/0.3);
				}
				else{
					PD = Kp_theta*dtheta + atan((Kp*dx + Kd*(dx - predx)/dt.count())/desire_speed);
				}
				if (abs(PD-prePD) >= increment){
					if (PD > prePD){
						PD = prePD + increment;
					}
					else if (PD <= prePD){
						PD = prePD - increment;
					}
				}
				data << dx << " " << dtheta << " " << PD << endl;
				//cout << dtheta << "," << dx << "," << PD << endl;
				prePD = PD;
				predx = dx;
				past_time = current_time;
				
				vector<float> joy_axes(6,0.0);
				joy_axes[0] = PD;
				joy_axes[2] = desire_speed; //speed control
				joy_command.axes = joy_axes;
				vector<int> joy_buttons(25,0);
				joy_command.buttons = joy_buttons;
				pub.publish(joy_command);
			}
			/****************************************************************************************************/
			
			// Show frame
			/*Mat test(frame_corp_gpu);
			imshow("Test", test);
			waitKey(30);*/
			//imwrite("test.jpg", lane_hls);
		}
	}
};


int main(int argc, char **argv)
{
	// ROS
	ros::init(argc, argv, "lane_deviation");
	lane_deviation ld;
	ros::spin();
	ld.data.close();
	return 0;
}
