#include <ros/ros.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/CameraInfo.h>
#include <math.h>
#include <yaml-cpp/yaml.h> // 包含yaml-cpp头文件

using namespace std;
using namespace cv;

sensor_msgs::CameraInfo getCameraInfo(void) {
    sensor_msgs::CameraInfo cam;

    // 读取YAML文件
    YAML::Node config = YAML::LoadFile("/userdata/workdir/config/camera.yaml");
    
    // 构造D向量（畸变系数）
    vector<double> D{config["k1"].as<double>(), config["k2"].as<double>(), 
                     config["p1"].as<double>(), config["p2"].as<double>(), 
                     config["k3"].as<double>()};
    
    // 构造K矩阵（内参矩阵）
    boost::array<double, 9> K = {
        config["fx"].as<double>(), 0.0, config["cx"].as<double>(),
        0.0, config["fy"].as<double>(), config["cy"].as<double>(),
        0.0, 0.0, 1.0
    };

    // 构造P矩阵（投影矩阵）
    // 通常情况下，P矩阵可以根据K矩阵和相机的尺寸手动构建，如果没有特定的平移信息，最后一列设置为0
    boost::array<double, 12> P = {
        config["fx"].as<double>(), 0.0, config["cx"].as<double>(), 0.0,
        0.0, config["fy"].as<double>(), config["cy"].as<double>(), 0.0,
        0.0, 0.0, 1.0, 0.0
    };

    // 设置相机信息
    cam.height = 512;  // 相机默认高度
    cam.width = 800;   // 相机默认宽度
    cam.distortion_model = "plumb_bob";
    cam.D = D;
    cam.K = K;
    cam.P = P;
    cam.R = {1, 0, 0, 0, 1, 0, 0, 0, 1}; // R矩阵设置为单位矩阵，假设没有旋转
    cam.binning_x = 0;
    cam.binning_y = 0;
    cam.header.frame_id = "camera";
    cam.header.stamp = ros::Time::now();

    return cam;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "camera_info_node");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<sensor_msgs::CameraInfo>("/camera/camera_info", 1000);
    sensor_msgs::CameraInfo camera_info_dyn;
    ros::Rate rate(1);

    while (ros::ok()) {
        camera_info_dyn = getCameraInfo();
        pub.publish(camera_info_dyn);
        rate.sleep();
    }
    ros::spin();
    return 0;
}
