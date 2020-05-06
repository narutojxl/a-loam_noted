// Author:   Tong Qin               qintonguav@gmail.com
// 	         Shaozu Cao 		    saozu.cao@connect.ust.hk

#include <cv_bridge/cv_bridge.h>
#include <eigen3/Eigen/Dense>
#include <fstream>
#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>
#include <iostream>
#include <iterator>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <string>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <vector>

std::vector<float> read_lidar_data(const std::string lidar_data_path) {
    std::ifstream lidar_data_file(lidar_data_path, std::ifstream::in | std::ifstream::binary);
    lidar_data_file.seekg(0, std::ios::end);
    const size_t num_elements = lidar_data_file.tellg() / sizeof(float);
    lidar_data_file.seekg(0, std::ios::beg);

    std::vector<float> lidar_data_buffer(num_elements);
    lidar_data_file.read(reinterpret_cast<char*>(&lidar_data_buffer[0]), num_elements * sizeof(float));
    return lidar_data_buffer;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "kitti_helper");
    ros::NodeHandle n("~");
    std::string dataset_folder("/media/jxl/0C3B04470C3B0447/KITTI_odometry/");
    std::string sequence_number("00");
    std::string output_bag_file("/media/jxl/0C3B04470C3B0447/KITTI_odometry/kitti.bag");

    n.getParam("dataset_folder", dataset_folder);
    n.getParam("sequence_number", sequence_number);
    std::cout << "Reading sequence " << sequence_number << " from " << dataset_folder << '\n';
    bool to_bag = false;
    n.getParam("to_bag", to_bag);
    if (to_bag)
        n.getParam("output_bag_file", output_bag_file);
    int publish_delay = 1;
    n.getParam("publish_delay", publish_delay);
    publish_delay = publish_delay <= 0 ? 1 : publish_delay;



    ros::Publisher pub_laser_cloud = n.advertise<sensor_msgs::PointCloud2>("/velodyne_points", 2);
    image_transport::ImageTransport it(n);
    image_transport::Publisher pub_image_left = it.advertise("/image_left", 2);
    image_transport::Publisher pub_image_right = it.advertise("/image_right", 2);

    ros::Publisher pubOdomGT = n.advertise<nav_msgs::Odometry>("/odometry_gt", 5);
    nav_msgs::Odometry odomGT;
    odomGT.header.frame_id = "/map";   //default: /camera_init
    odomGT.child_frame_id = "/camera"; //default: /ground_truth

    ros::Publisher pubPathGT = n.advertise<nav_msgs::Path>("/path_gt", 5);
    nav_msgs::Path pathGT;
    pathGT.header.frame_id = "/map"; //default: /camera_init




    std::string timestamp_path = "sequences/" + sequence_number + "/times.txt";
    std::ifstream timestamp_file(dataset_folder + timestamp_path, std::ifstream::in);

    std::string ground_truth_path = "results/" + sequence_number + ".txt";
    std::ifstream ground_truth_file(dataset_folder + ground_truth_path, std::ifstream::in);

    if (!timestamp_file.is_open()) {
        std::cout << timestamp_path.c_str() << " not exist!\n";
        return -1;
    }

    if (!ground_truth_file.is_open()) {
        std::cout << ground_truth_path.c_str() << " not exist!\n";
        return -1;
    }

    rosbag::Bag bag_out;
    if (to_bag)
        bag_out.open(output_bag_file, rosbag::bagmode::Write);

    Eigen::Matrix3d R_transform;



    // R_transform << 0, 0, 1, -1, 0, 0, 0, -1, 0; //?? TODO
    // R_transform << -1, 0, 0, 0, -1, 0, 0, 0, 1; //camera_init --->kitti_map
    R_transform << 0, 0, 1, -1, 0, 0, 0, -1, 0; //map --->kitti_map
    Eigen::Quaterniond q_transform(R_transform);
    q_transform.normalize(); //乘之前归一化和未归一化，对结果没有影响

    //根据Vision meets Robotics: The KITTI Dataset paper，   laser ---> left_gray_camera: (tx,ty,tz)=(0.27, 0, -(1.73-1.65))
    Eigen::Vector3d t_map_kittimap(0.27, 0, -0.08);


    Eigen::Matrix3d R_camera_laser = R_transform.inverse();
    Eigen::Quaterniond q_camera_laser(R_camera_laser);
    Eigen::Vector3d t_camera_laser = -R_camera_laser * t_map_kittimap ;


    // in lego_loam
    // camera_init(Z前，X左，Y上) ---> map(X前，Y左，Z上) x y z yaw pitch roll 0 0 0  1.570795  0   1.570795   /map    /camera_init
    // R: map--->camera_init        R: camera_init--->map
    // 0 0 1                        0 1  0
    // 1 0 0                        0 0  1
    // 0 1 0                        1 0  0

    // in kitti, kitti_map is left_camera_0th(Z前，X右，Y下)，x y z yaw pitch roll 0 -0.08  0.27  -3.14159  0  0  /camera_init  /kitti_map
    // R: kitti_map--->camera_init         R: camera_init--->kitti_map
    //-1  0 0                              -1   0  0
    // 0 -1 0                               0  -1  0
    // 0  0 1                               0   0  1

    std::string line;
    std::size_t line_num = 0;

    //jxl
    tf::StampedTransform map_cameraInit;
    map_cameraInit.frame_id_ = "map";
    map_cameraInit.child_frame_id_ = "camera_init";
    tf::TransformBroadcaster tfBroadcaster2;

    
    std::ofstream gt_path_inMap(dataset_folder + "gt_path_inMap_" + sequence_number + ".txt", std::ios::out);
    std::ostringstream ss;

    ros::Rate r(10.0 / publish_delay);
    while (std::getline(timestamp_file, line) && ros::ok()) {
        float timestamp = stof(line);
        std::stringstream left_image_path, right_image_path;
        left_image_path << dataset_folder << "sequences/" + sequence_number + "/image_0/" << std::setfill('0') << std::setw(6) << line_num << ".png";
        cv::Mat left_image = cv::imread(left_image_path.str(), CV_LOAD_IMAGE_GRAYSCALE);
        right_image_path << dataset_folder << "sequences/" + sequence_number + "/image_1/" << std::setfill('0') << std::setw(6) << line_num << ".png";
        cv::Mat right_image = cv::imread(left_image_path.str(), CV_LOAD_IMAGE_GRAYSCALE);

        std::getline(ground_truth_file, line);
        std::stringstream pose_stream(line);
        std::string s;
        Eigen::Matrix<double, 3, 4> gt_pose;
        for (std::size_t i = 0; i < 3; ++i) {
            for (std::size_t j = 0; j < 4; ++j) {
                std::getline(pose_stream, s, ' ');
                gt_pose(i, j) = stof(s);
            }
        }

        Eigen::Quaterniond q_w_i(gt_pose.topLeftCorner<3, 3>());
        q_w_i.normalize();
        Eigen::Quaterniond q = q_transform * q_w_i;       
        q.normalize();
        
        // Eigen::Vector3d t = q_transform * gt_pose.topRightCorner<3, 1>();
        Eigen::Vector3d t = q_transform * gt_pose.topRightCorner<3, 1>() + t_map_kittimap;


        t = t + q * t_camera_laser;
        q = q * q_camera_laser;
        q.normalize();
        Eigen::Matrix3d R = q.toRotationMatrix(); //The quaternion is required to be normalized, otherwise the result is undefined.


        ss.str(""); //clear() 函数不起作用
        ss << R(0, 0) << " " << R(0, 1) << " " << R(0, 2) << " " << t(0) << " "
           << R(1, 0) << " " << R(1, 1) << " " << R(1, 2) << " " << t(1) << " "
           << R(2, 0) << " " << R(2, 1) << " " << R(2, 2) << " " << t(2) << std::endl;
        gt_path_inMap << ss.str();

        odomGT.header.stamp = ros::Time().fromSec(timestamp);
        odomGT.pose.pose.orientation.x = q.x();
        odomGT.pose.pose.orientation.y = q.y();
        odomGT.pose.pose.orientation.z = q.z();
        odomGT.pose.pose.orientation.w = q.w();
        odomGT.pose.pose.position.x = t(0);
        odomGT.pose.pose.position.y = t(1);
        odomGT.pose.pose.position.z = t(2);
        pubOdomGT.publish(odomGT);

        geometry_msgs::PoseStamped poseGT;
        poseGT.header = odomGT.header;
        poseGT.pose = odomGT.pose.pose;
        pathGT.header.stamp = odomGT.header.stamp;
        pathGT.poses.push_back(poseGT);
        pubPathGT.publish(pathGT);

        //jxl publish TF：lego_loam map ---> camera_init
        map_cameraInit.stamp_ = ros::Time().fromSec(timestamp);
        geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw(1.570795, 0, 1.570795); //roll, pitch, yaw
        map_cameraInit.setRotation(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w));
        map_cameraInit.setOrigin(tf::Vector3(0, 0, 0));
        tfBroadcaster2.sendTransform(map_cameraInit);

        // read lidar point cloud
        std::stringstream lidar_data_path;
        lidar_data_path << dataset_folder << "velodyne/sequences/" + sequence_number + "/velodyne/" << std::setfill('0') << std::setw(6) << line_num << ".bin";
        std::vector<float> lidar_data = read_lidar_data(lidar_data_path.str());
        std::cout << "totally " << lidar_data.size() / 4.0 << " points in this lidar frame \n";

        std::vector<Eigen::Vector3d> lidar_points;
        std::vector<float> lidar_intensities;
        pcl::PointCloud<pcl::PointXYZI> laser_cloud;
        for (std::size_t i = 0; i < lidar_data.size(); i += 4) {
            lidar_points.emplace_back(lidar_data[i], lidar_data[i + 1], lidar_data[i + 2]);
            lidar_intensities.push_back(lidar_data[i + 3]);

            pcl::PointXYZI point;
            point.x = lidar_data[i];
            point.y = lidar_data[i + 1];
            point.z = lidar_data[i + 2];
            point.intensity = lidar_data[i + 3];
            laser_cloud.push_back(point);
        }

        sensor_msgs::PointCloud2 laser_cloud_msg;
        pcl::toROSMsg(laser_cloud, laser_cloud_msg);
        laser_cloud_msg.header.stamp = ros::Time().fromSec(timestamp);
        laser_cloud_msg.header.frame_id = "/velodyne"; // TODO  default: /camera_init
        pub_laser_cloud.publish(laser_cloud_msg);

        sensor_msgs::ImagePtr image_left_msg = cv_bridge::CvImage(laser_cloud_msg.header, "mono8", left_image).toImageMsg();
        sensor_msgs::ImagePtr image_right_msg = cv_bridge::CvImage(laser_cloud_msg.header, "mono8", right_image).toImageMsg();
        pub_image_left.publish(image_left_msg);
        pub_image_right.publish(image_right_msg);

        if (to_bag) {
            ros::Time now_stamp = ros::Time::now();
            ros::Time msg_stamp = ros::Time().fromSec(timestamp);
            bag_out.write("/image_left", now_stamp, image_left_msg); //default: ros::Time::now()
            bag_out.write("/image_right", now_stamp, image_right_msg);
            bag_out.write("/velodyne_points", now_stamp, laser_cloud_msg);
            bag_out.write("/path_gt", now_stamp, pathGT);
            bag_out.write("/odometry_gt", now_stamp, odomGT);
        }

        line_num++;
        r.sleep();
    }
    bag_out.close();
    gt_path_inMap.close();
    std::cout << "Done \n";

    return 0;
}
