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



struct CalParam{
    // Projection matrix from rect camera coord to image2 coord (3*4)
    Eigen::Matrix<float,3,4> P;
    // Rigid transform from Velodyne coord to reference camera coord (3*4)
    Eigen::Matrix<float,3,4> V2C;
    // Rigid transform from reference camera coord to Velodyne coord (3*4)
    Eigen::Matrix<float,3,4> C2V;
    // Rotation from reference camera coord to rect camera coord (3*3)
    Eigen::Matrix<float,3,3> R0;
    // Camera intrinsics and extrinsics
    float c_u, c_v;
    float f_u, f_v;
    float b_x, b_y;
};


Eigen::Matrix<float,3,4> inverse_rigid_trans(Eigen::Matrix<float,3,4> & tr){

    Eigen::Matrix<float,3,4> inv_tr; inv_tr.setZero();
    inv_tr.block<3,3>(0,0) = tr.block<3,3>(0,0).transpose();
    inv_tr.block<3,1>(0,3) = -tr.block<3,3>(0,0).transpose() * tr.block<3,1>(0,3);
    return inv_tr;
}


bool cal_file_parser(const string cal_filepath, CalParam & calparam ){
    fstream infile(cal_filepath);
    if(!infile.is_open()){
        printf("ERROR: Calibration file is invalid.");
        return false;
    }
    string line; string s_val; float f_val;
    while(getline(infile,line)){
        stringstream ss;
        ss << line;
        ss >> s_val;
        if(s_val == "P2:"){
            for(int i=0; i<calparam.P.size(); i++){
                int row = calparam.P.rows(), col = calparam.P.cols();
                ss >> calparam.P(i/col, i%col);  
            }
            // cout<<"P2: "<<endl;
            // cout<<calparam.P<<endl<<endl;
        }else if( s_val == "R0_rect:"){
            for(int i=0; i<calparam.R0.size(); i++){
                int row = calparam.R0.rows(), col = calparam.R0.cols();
                ss >> calparam.R0(i/col, i%col);  
            }
            // cout<<"R0_rect: "<<endl;
            // cout<<calparam.R0<<endl<<endl;
        }else if( s_val == "Tr_velo_to_cam:"){
            for(int i=0; i<calparam.V2C.size(); i++){
                int row = calparam.V2C.rows(), col = calparam.V2C.cols();
                ss >> calparam.V2C(i/col, i%col);  
            }
            calparam.C2V = inverse_rigid_trans(calparam.V2C);
            // cout<<"C2V: "<<endl;
            // cout << calparam.C2V <<endl;
            // cout<<"V2C: "<<endl;
            // cout<<calparam.V2C<<endl<<endl;
        }
    }
}

class Calibration{

public:
    Calibration(string cal_filepath){
        cal_file_parser(cal_filepath,cal_param);
    }

    Eigen::MatrixXf cart2hom(Eigen::MatrixXf& pts);
    Eigen::MatrixXf project_rect_to_ref(Eigen::MatrixXf& pts);
    Eigen::MatrixXf project_ref_to_velo(Eigen::MatrixXf& pts);
    Eigen::MatrixXf project_rect_to_velo(Eigen::MatrixXf& pts);
    Eigen::MatrixXf project_rect_to_image(Eigen::MatrixXf& pts);
    CalParam cal_param;
};


//Pending 1 to each point, nx3 points in Carteasian to Homogeneous nx4
Eigen::MatrixXf Calibration::cart2hom(Eigen::MatrixXf & pts){
    int row = pts.rows();
    Eigen::MatrixXf pts_h(row,4);
    pts_h.block(0,0,row,3) = pts;
    pts_h.block(0,3,row,1).setOnes();
    return pts_h;
}

Eigen::MatrixXf Calibration::project_rect_to_ref(Eigen::MatrixXf& pts){
    int row = pts.rows(), col = pts.cols();
    Eigen::MatrixXf pts_3d_rect(row, col);
    pts_3d_rect = (cal_param.R0.inverse() * pts.transpose()).transpose();
    return pts_3d_rect;
}


Eigen::MatrixXf Calibration::project_ref_to_velo(Eigen::MatrixXf& pts){
    int row = pts.rows(), col = pts.cols();
    Eigen::MatrixXf pts_3d_velo(row, col);
    pts_3d_velo = cart2hom(pts) * cal_param.C2V.transpose();
    return pts_3d_velo;
}


Eigen::MatrixXf Calibration::project_rect_to_velo(Eigen::MatrixXf& pts){
    int row = pts.rows(), col = pts.cols();
    Eigen::MatrixXf pts_3d_velo(row, col);
    pts_3d_velo = project_rect_to_ref(pts);
    pts_3d_velo = project_ref_to_velo(pts_3d_velo);
    return pts_3d_velo;
}

// project cam 3d points to image plane (corresponds to refered camera) (2-th camera to 2-image plane)
Eigen::MatrixXf Calibration::project_rect_to_image(Eigen::MatrixXf& pts_rect_cam){
    int num_pts = pts_rect_cam.cols();
    Eigen::MatrixXf pts_h = cart2hom(pts_rect_cam);
    Eigen::MatrixXf pts_re = (cal_param.P * pts_h.transpose()).transpose();
    int lcol = pts_re.cols();
    for(int i=0; i<pts_re.rows(); i++){
        pts_re(i,0) = pts_re(i,0)/pts_re(i,lcol-1);
        pts_re(i,1) = pts_re(i,1)/pts_re(i,lcol-1);
    }
    return pts_re;
}




