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
// #include <opencv2/opencv.hpp>  
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/ImageMarker.h>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <map>
#include <Eigen/Dense>
#include <t.h>
#include <cal.hpp>

using namespace std;

#define FRAME_PER_SEQ 120
#define SEQ_NUM 2
#define OBJ_TYPE_NUM 9
#define DATA_ITEM_LEN 17
#define RATE 8

typedef pcl::PointXYZ PointT;
typedef pcl::PointXYZI PointI;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<PointI> PointCloudI;


struct BoxColor{
    double r;
    double g;
    double b;
    void assign(double x,double y, double z){
        r = x; g = y; b = z;
    }
};

std::map<string,int>  object_map;
string * object_types;
BoxColor * box_color ;

struct LabelFrame{
    int frame_id;
    vector<string> fra_data;
};

struct LabelSeq{
    string seq_id;
    vector<LabelFrame> seq_data;
};


struct PosPose{
    PointT pos;
    PointT lwh;
    double yaw;
    int track_id;
    string type;
};





void read_bin(PointCloudT &points, string& files_dir, string& file_name){
    points.clear();
    string infile = files_dir+file_name+".bin";
    fstream input(infile.c_str(), ios::in | ios::binary);
    if(!input.good()){
        cerr << "Could not read file: " << infile << endl;
        exit(EXIT_FAILURE);
    }
    input.seekg(0, ios::beg);
    float inten;
    for (int i=0; input.good() && !input.eof(); i++) {
        PointT point;
        input.read((char *) &point.x, 3*sizeof(float));
        input.read((char *) &inten, sizeof(float));
        points.push_back(point);
    }
    input.close();
}


void read_image(cv_bridge::CvImagePtr cvPtr,string& files_dir, string& file_name){
    string infile = files_dir+file_name+".png";
    cvPtr->image = cv::imread(infile);
    cvPtr->encoding = sensor_msgs::image_encodings::BGR8;
}



void load_seq_label_files(string files_dir, int seq_num, LabelSeq & seq){
    string tmp;
    stringstream ss;
    ss << std::right << std::setfill('0') << std::setw(4) <<seq_num<<endl;
    ss>>tmp; 
    seq.seq_id = tmp;
    fstream infile(files_dir+"label_2/"+tmp+".txt");
    int frame_id=0;  int pre_frame_id=0;
    string line;
    LabelFrame fl;
    while(getline(infile,line)){
        stringstream ss1;
        ss1 << line;   ss1 >> frame_id;
        if(frame_id!=pre_frame_id){
            seq.seq_data.push_back(fl);
            pre_frame_id = frame_id;
            fl.frame_id = frame_id;
            fl.fra_data.clear();  
        }
        fl.fra_data.push_back(line);        
    }
}


//0 0 Van 0 0 -1.793451 296.744956 161.752147 455.226042 292.372804 2.000000 1.823255 4.433886 
//-4.552284 1.858523 13.410495 -2.115488
bool frame_data_str_parser(string & data, PosPose & pos_pose){
    stringstream ss;
    ss << data;
    int count = 0;
    int val; string val_str;
    while(count < DATA_ITEM_LEN){

        if(count == 1){
            ss >> val;
            if(val == -1) return false;
            else{
                pos_pose.track_id = val;
                count++;
                continue;
            }
        }
        if(count == 2){
            ss >> val_str;
            pos_pose.type = val_str;
            count++; continue;
        }
        if(count == 10){
            double x,y,z,h,w,l,yaw;  ss>>h>>w>>l>>x>>y>>z>>yaw;
            // pos_pose.pos.x = z;  pos_pose.pos.y = -x;  pos_pose.pos.z = -y + h/2.0;
            pos_pose.pos.x = x;  pos_pose.pos.y = y;  pos_pose.pos.z = z;
            pos_pose.lwh.x = l;  pos_pose.lwh.y = w; pos_pose.lwh.z = h;
            pos_pose.yaw = yaw; 
            count += 7; continue;
        }
        ss >> val_str;
        count++;
    }
    return true;
}


Eigen::MatrixXf rot_y_matrix(double t){
    double c = cos(t);
    double s = sin(t);
    Eigen::MatrixXf R(3,3);
    R << c, 0, s,
         0, 1, 0,
        -s, 0, c;
    return R;
}



Eigen::MatrixXf compute_3d_box_corners(PosPose & pp){

// Compute the 8 cornners of 3d bounding box in camera coord
// Input: label infor (x,y,z), (h,w,l)
// Ouput: 8 corners coords in camera coord.  (8x3)

    //rotational matrix (3x3) around y axis in camera coord, input: yaw;
    Eigen::MatrixXf R = rot_y_matrix(pp.yaw);

    //length, width, height of 3d box
    double l = pp.lwh.x;
    double w = pp.lwh.y;
    double h = pp.lwh.z;
    //3d bounding box dimensions
    Eigen::Matrix<float,3,8> corners;
    Eigen::MatrixXf corners_3d;
    corners << l/2,l/2,-l/2,-l/2,l/2,l/2,-l/2,-l/2,
               0,0,0,0,-h,-h,-h,-h,
               w/2,-w/2,-w/2,w/2,w/2,-w/2,-w/2,w/2;
    corners_3d = R * corners;
    //3d position given in label (bottom-center)
    Eigen::VectorXf pos_bc(3);
    pos_bc << pp.pos.x, pp.pos.y, pp.pos.z;
    corners_3d.colwise() += pos_bc;
    return corners_3d.transpose();
}



void create_bbox_mesh(Eigen::MatrixXf & corners, visualization_msgs::Marker & marker, BoxColor & color){
    marker.header.frame_id = "kitti_track_frame";
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.10;

    marker.color.r = color.r;
    marker.color.g = color.g;
    marker.color.b = color.b;
    marker.color.a = 1.0;

    geometry_msgs::Point pre; geometry_msgs::Point nex;
    for(int k=0; k<4; k++){
        int i=k; int j=k+4;
        pre.x = corners(i,0); pre.y = corners(i,1); pre.z = corners(i,2);
        nex.x = corners(j,0); nex.y = corners(j,1); nex.z = corners(j,2);
        marker.points.push_back(pre); marker.points.push_back(nex); 

        i=k; j=(k+1)%4;
        pre.x = corners(i,0); pre.y = corners(i,1); pre.z = corners(i,2);
        nex.x = corners(j,0); nex.y = corners(j,1); nex.z = corners(j,2);
        marker.points.push_back(pre); marker.points.push_back(nex); 

        i=k+4; j=(k+1)%4+4;
        pre.x = corners(i,0); pre.y = corners(i,1); pre.z = corners(i,2);
        nex.x = corners(j,0); nex.y = corners(j,1); nex.z = corners(j,2);
        marker.points.push_back(pre); marker.points.push_back(nex); 
    }
}


bool create_2dbox_mesh(Eigen::MatrixXf & corners_2d, cv_bridge::CvImagePtr &cv_ptr, BoxColor & color)
{
    
    // cv::imshow("image",cv_ptr->image);
    double r=color.r*255.0,b=color.b*255.0,g=color.g*255.0;
    cv::Mat tmp = cv_ptr->image;
    cv::Point pre; cv::Point nex;
    for(int k=0; k<4; k++){
        int i = k, j = (k+1)%4;
        pre.x = corners_2d(i,0); pre.y = corners_2d(i,1); 
        nex.x = corners_2d(j,0); nex.y = corners_2d(j,1); 
        cv::line(cv_ptr->image, pre, nex, cv::Scalar(b,g,r),2);
        // cv::line(cv_ptr->image, pre, nex, cv::Scalar(0,255,0),2);
        i=k; j=(k+4);
        pre.x = corners_2d(i,0); pre.y = corners_2d(i,1); 
        nex.x = corners_2d(j,0); nex.y = corners_2d(j,1); 
        cv::line(cv_ptr->image, pre, nex, cv::Scalar(b,g,r),2);
        // cv::line(cv_ptr->image, pre, nex, cv::Scalar(0,255,0),2);
        i=k+4; j=(k+1)%4+4;
        pre.x = corners_2d(i,0); pre.y = corners_2d(i,1); 
        nex.x = corners_2d(j,0); nex.y = corners_2d(j,1); 
        cv::line(cv_ptr->image, pre, nex, cv::Scalar(b,g,r),2);
        // cv::line(cv_ptr->image, pre, nex, cv::Scalar(0,255,0),2);
    }
}





bool generate_line_marker(Eigen::MatrixXf & corners_3d, visualization_msgs::Marker & marker,BoxColor & box_color){
    create_bbox_mesh(corners_3d,marker,box_color);  
}

bool genrate_image_marker(Eigen::MatrixXf & corners_2d, cv_bridge::CvImagePtr &cv_ptr, BoxColor & box_color){
    create_2dbox_mesh(corners_2d, cv_ptr, box_color);
}




void generate_text_marker(PosPose &pp, visualization_msgs::Marker & marker, ros::Time now, 
    Eigen::MatrixXf &corners_3d){

    marker.header.frame_id = "kitti_track_frame";
    marker.header.stamp = now;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;

    Eigen::VectorXf centroid = corners_3d.colwise().sum()/corners_3d.rows();

    marker.pose.position.x = centroid(0);
    marker.pose.position.y = centroid(1);
    marker.pose.position.z = centroid(2) + pp.lwh.z;

    stringstream ss; ss << pp.track_id; string st_id;
    ss >> st_id;
    marker.text = pp.type + " " + st_id;

    marker.scale.x = 0.3*10;
    marker.scale.y = 0.3*10;
    marker.scale.z = 0.1*10;

    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
}



void generate_3dBoundingBox_markers(visualization_msgs::MarkerArray & bbox_markers, LabelFrame & objs, 
    Calibration & calib , cv_bridge::CvImagePtr & img_ptr, ros::Time now){
    int objs_num = objs.fra_data.size();
    for(int i=0; i<objs_num; i++){
        string data = (objs.fra_data)[i];
        PosPose pp;
        bool is_valid = frame_data_str_parser(data,pp);
        if(!is_valid) continue;
        // generate_marker(pp,marker,calib);
        // generate corners in 3d and create 3d bounding box marker
        Eigen::MatrixXf corners_cam = compute_3d_box_corners(pp);
        Eigen::MatrixXf corners_3d = calib.project_rect_to_velo(corners_cam);
        //generate 3d bounding boxes
        visualization_msgs::Marker marker;
        generate_line_marker(corners_3d, marker,box_color[object_map[pp.type]]);
        marker.header.stamp = now;
        marker.id = pp.track_id;
        marker.lifetime = ros::Duration(1.0/RATE);
        marker.text = pp.type;
        bbox_markers.markers.push_back(marker);

        //genetate text markers for 3d bounding boxes
        generate_text_marker(pp,marker,now,corners_3d);
        marker.id = pp.track_id+objs_num;
        marker.lifetime = ros::Duration(1.0/RATE);
        bbox_markers.markers.push_back(marker);

        //generate 2d image bounding boxes
        Eigen::MatrixXf corners_2d = calib.project_rect_to_image(corners_cam);
        create_2dbox_mesh(corners_2d, img_ptr, box_color[object_map[pp.type]]);
        
    }
}


void init_box_color(){
    box_color = new BoxColor[OBJ_TYPE_NUM+1];
    box_color[object_map["Car"]].assign(0.0,1.0,0.0);   //green
    box_color[object_map["Cyclist"]].assign(1.0,1.0,0);   //yellow
    box_color[object_map["Pedestrian"]].assign(1.0,0,1.0);   //pink
    box_color[object_map["Truck"]].assign(128.0/255.0,128.0/255.0,128.0/255.0);   //gray   128,128,128
    box_color[object_map["Van"]].assign(0,1.0,1.0);   //blue
    box_color[object_map["Person_sitting"]].assign(75.0/255,0.0,130.0/255);   //purple   75,0,130    
    box_color[object_map["Misc"]].assign(128.0/255.0,128.0/255.0,128.0/255.0);  //Maroon 128 0 0
    box_color[object_map["Tram"]].assign(1.0,140.0/255.0, 0.0);  //Orange 255,140,0
    
}



void test(){
    
    cout<<object_map["Truck"]<<endl;
}



int main(int argc, char** argv){
    ros::init(argc,argv,"kitti_viz_node");
    ros::NodeHandle nh("~");


    ros::Publisher ptcl_pub = nh.advertise<PointCloudT>("kitti_point_cloud",10);
    ros::Publisher img_pub = nh.advertise<sensor_msgs::Image>("kitti_image",10);
    ros::Publisher bbox_pub = nh.advertise<visualization_msgs::MarkerArray>("bbox_markers",10);
    ros::Publisher image2d_box_pub = nh.advertise<sensor_msgs::Image>("image_marker",10);

    PointCloudT point_cloud;
    PointCloudI point_cloud_i;
    cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage());
    cv_bridge::CvImagePtr cv_ptr_box(new cv_bridge::CvImage());
    sensor_msgs::Image sImg;

    // string files_dir = "/home/curry/study/kitti_data/Kitti/tracking/training/";
    string files_dir = "/home/curry/study/kitti_data/long_track_data/";
    string data_type [] = {"velodyne/", "image_2/"};
    string sequences [] = {"0000/","0001/"};
    string seq_label [] = {"0000", "0001"};
    
    
    //************fixed frame**************s*****//
    point_cloud.header.frame_id="kitti_track_frame";
    (cv_ptr->header).frame_id="kitti_track_frame";
    



    //******file_name_prefix generator********//
    vector<string> file_name_prefix;
    vector<string> label_file_name;
    stringstream ss;
    for(int i=0; i<FRAME_PER_SEQ; i++){
        string tmp;
        ss << std::right << std::setfill('0') << std::setw(6) <<i<<endl;
        ss>>tmp; 
        file_name_prefix.push_back(tmp);
    }

    //****************load labels*************************//
    LabelSeq label_seq[SEQ_NUM];
    for(int i=0;i<SEQ_NUM;i++){
        load_seq_label_files(files_dir,i,label_seq[i]);
    }
    
    //*************objects map***************************//
    map<string,int> obj_map;
    string objs[]= {"Car", "Van","Truck","Pedestrian","Person_sitting","Cyclist","Tram","Misc","DontCare"};
    for(int i=0;i<OBJ_TYPE_NUM;i++){
        obj_map.insert(std::pair<string,int>(objs[i],i));
    }
    object_map = obj_map;

    init_box_color();
    test();

    int frame_id = 0;
    int seq_id = 0;
    ros::Rate loop_rate(RATE);



    //***********************Load Calibration******************//
    string cal_filepath = "/home/curry/study/kitti_data/Kitti/tracking/training/calib/0000.txt";
    Calibration calib(cal_filepath);




    //**************************TEST AREA************************//
    cout<<"*****************THIS IS TEST RESULT**********************"<<endl;




    //**************************END TEST************************//

    while (nh.ok())
    {
        // seq_id = 0; frame_id = 0;
        //******load velodyne points and images*********//
        string velo_file_name = files_dir+data_type[0]+sequences[seq_id];
        string img_file_name =  files_dir+data_type[1]+sequences[seq_id];
        read_bin(point_cloud,velo_file_name,file_name_prefix[frame_id]);
        read_image(cv_ptr,img_file_name,file_name_prefix[frame_id]);
        //*****load labels**********//
        
        

        //*******add stamp to velodyne point cloud and images topics*************//
        ros::Time now = ros::Time::now();
        pcl_conversions::toPCL(now, point_cloud.header.stamp);
        cv_ptr->toImageMsg(sImg);
        sImg.header.stamp = now;

        //***************generate 2d bounding boxes and 3d bounding boxes**************************//
        sensor_msgs::Image sImg_2d_boxes(sImg);
        visualization_msgs::MarkerArray bbox_markers;
        cv_ptr_box->image = cv_ptr->image;
        generate_3dBoundingBox_markers(bbox_markers, label_seq[seq_id].seq_data[frame_id],calib,cv_ptr_box,now);
        cv_ptr_box->encoding = sensor_msgs::image_encodings::BGR8;
        cv_ptr_box->toImageMsg(sImg_2d_boxes);
        sImg_2d_boxes.header.stamp = now;

        
        //************publish the topic************************//
        bbox_pub.publish(bbox_markers);
        ptcl_pub.publish(point_cloud);
        img_pub.publish(sImg);
        image2d_box_pub.publish(sImg_2d_boxes);



        //**********frame_id and seq_id update************//
        frame_id++;
        if(frame_id % FRAME_PER_SEQ == 0){
            frame_id = frame_id % FRAME_PER_SEQ;
            seq_id++;
            seq_id = seq_id % SEQ_NUM; 
        }

        ros::spinOnce ();
        loop_rate.sleep ();
    }


}