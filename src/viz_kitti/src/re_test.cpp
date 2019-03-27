#include <iostream>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <fstream>
#include <string.h>
#include <boost/algorithm/string.hpp>
#include <sensor_msgs/image_encodings.h>
#include <sstream>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <map>
#include <Eigen/Dense>

using namespace std;



int main(){

    Eigen::Matrix<float,2,2> da;
    Eigen::VectorXf v(2);
    v <<  1.0,2.0;
    da << 1.0,2.0,3.0,4.0;
    // v = da.colwise().sum();
    // cout<<v<<endl;


}


// #include "stdafx.h"  
// #include <iostream>  
// #include <string.h>  
// #include <opencv2/opencv.hpp>  
// using namespace cv;  
// using namespace std;  
// int main()  
// {  
  
//     Mat src = imread("/home/curry/study/researches/kitti_tracking/catkin_ws_v2/data/test.png");//图片必须添加到工程目录下  
  
//     // // 这里的图必须是BGR的 注意 二值化的图无法显示      
//     // cvtColor(src, src, CV_GRAY2BGR);
  
//     // //在原图画一个圆圈点  
//     // cv::Point point;//特征点，用以画在图像中    
//     // point.x = 20;//特征点在图像中横坐标    
//     // point.y = 50;//特征点在图像中纵坐标    
//     // cv::circle(src, point, 4, cv::Scalar(0, 0, 255));//在图像中画出特征点，2是圆的半径   
  
//     //在原图画一条直线  
//     cv::Point start = cv::Point(10, 100); //直线起点  
//     cv::Point end = cv::Point(50, 200);   //直线终点  
//     cv::Point st1 = cv::Point(50, 200); //直线起点  
//     cv::Point end1 = cv::Point(100, 50);   //直线终点  
//     cv::line(src, start, end, cv::Scalar(0, 0, 255));  
//     cv::line(src, st1, end1, cv::Scalar(0, 0, 255));  
  
//     //在原图某个位置添加文字标记  
//     //char str[4];  
//     //int num = 100;  
//     //_itoa_s(num, str, 10);//数字需要转为字符串来显示  
//     string str = "Love100";  
//     putText(src, str, end, cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(0, 255,0 ), 2);  
//     imshow("src", src);  
//     waitKey(0);  
// }  