//
// Created by xiaofan on 25-4-3.
//
#include "../include/common_lib.h"

/************ variability load from yaml ************/
Eigen::Vector3d imu_t_lidar; // lidar 系相对 IMU 系的外参   imu_t_lidar           T 4x4  (R t) -> T
Eigen::Matrix3d imu_R_lidar; // lidar 系相对 IMU 系的外参   imu_R_lidar
Eigen::Vector3d lidar_t_vehicle; // body 系相对 lidar 系的外参  lidar_t_vehicle
Eigen::Matrix3d lidar_R_vehicle; // body 系相对 lidar 系的外参  lidar_R_vehicle
int point_filter_num; // 点云订阅时每隔 point_filter_num 个点才读入一次
double blind; // 距离原点过近的点会被忽略，半径为 blind
bool use_box_blind; // 如果使用长方形盒作为待删除区域，设置为true
Eigen::Vector2d box_corner_x;
Eigen::Vector2d box_corner_y;
Eigen::Vector2d box_corner_z;
float filter_size = 0.5; // 点云降采样大小（ikdtree大小），默认值0.5
bool global_map_save_enable; // 是否启用点云保存
int every_n_frame_save; // 每隔多少帧保存一次单帧地图，-1表示不保存
bool timestamp_prefix; // 是否启用时间戳前缀
string pcd_save_name; // 点云保存的文件名
bool global_map_pub_enable; // 是否发布全局地图
bool ikdtree_pub_enable; // 是否发布全局地图

string lidar_topic_name;
string imu_topic_name;
string wheel_topic_name;
string image_topic_name;

string clouds_lidar_topic_name; // lidar系当前帧点云发布的话题名
string global_map_topic_name; // world系全局地图发布的话题名
string ikdtree_topic_name; // world系 ikdtree 发布的话题名
string odom_imu_topic_name; // imu位姿的话题名
string odom_vehicle_topic_name; // body位姿的话题名
bool relocation_enable; // 是否启用重定位
string pcd_load_name; // 重定位时，加载的点云文件名


bool wheel_enable; // 是否启用wheel轮速计更新


bool log_save_enable; // 是否记录日志，全局设置
bool wheel_data_log_enable; // 是否记录轮速计数据
bool trajectory_log_enable; // 是否记录位姿数据
bool time_log_enable; // 是否记录程序耗时
bool pub_time_log_enable; // 是否记录topic发送耗时
bool res_wheel_log_enable; // 是否记录轮速计残差，在 UpdateByWheel() 中使用
int lidar_type = 1; //     Livox = 1, Ouster = 2

bool high_frequency_odom = false;
float lidar_frequency = 10;


void load_lio_yaml( string file_name ) {
    YamlReader reader(std::string(PACKAGE_ROOT_DIR) +  "/yaml/" + file_name);
    if (reader.load()) {
        /******** lidar_type ********/
        if (reader.read("lidar_type", lidar_type)) {
            std::cout << "lidar_type: " << lidar_type << std::endl;
        } else {
            std::cerr << "Failed to read 'lidar_type'" << std::endl;
        }
        /******** frequency ********/
        if (reader.read("high_frequency_odom", high_frequency_odom)) {
            std::cout << "high_frequency_odom: " << high_frequency_odom << std::endl;
        } else {
            std::cerr << "Failed to read 'high_frequency_odom'" << std::endl;
        }
        if (reader.read("lidar_frequency", lidar_frequency)) {
            std::cout << "lidar_frequency: " << lidar_frequency << std::endl;
        } else {
            std::cerr << "Failed to read 'lidar_frequency'" << std::endl;
        }
        /******** offset ********/
        if (reader.readEigen("offset.imu_t_lidar", imu_t_lidar)) {
            std::cout << "imu_t_lidar: " << imu_t_lidar << std::endl;
        } else {
            std::cerr << "Failed to read 'imu_t_lidar'" << std::endl;
        }
        if (reader.readEigen("offset.imu_R_lidar", imu_R_lidar)) {
            std::cout << "imu_R_lidar: " << imu_R_lidar << std::endl;
        } else {
            std::cerr << "Failed to read 'imu_R_lidar'" << std::endl;
        }
        if (reader.readEigen("offset.lidar_t_vehicle", lidar_t_vehicle)) {
            std::cout << "lidar_t_vehicle: " << lidar_t_vehicle << std::endl;
        } else {
            std::cerr << "Failed to read 'lidar_t_vehicle'" << std::endl;
        }
        if (reader.readEigen("offset.lidar_R_vehicle", lidar_R_vehicle)) {
            std::cout << "lidar_R_vehicle: " << lidar_R_vehicle << std::endl;
        } else {
            std::cerr << "Failed to read 'lidar_R_vehicle'" << std::endl;
        }
        /******** downsample ********/
        if (reader.read("downsample.point_filter_num", point_filter_num)) {
            std::cout << "point_filter_num: " << point_filter_num << std::endl;
        } else {
            std::cerr << "Failed to read 'point_filter_num'" << std::endl;
        }
        if (reader.read("downsample.blind", blind)) {
            std::cout << "blind: " << blind << std::endl;
        } else {
            std::cerr << "Failed to read 'blind'" << std::endl;
        }
        if (reader.read("downsample.use_box_blind", use_box_blind)) {
            std::cout << "use_box_blind: " << use_box_blind << std::endl;
        } else {
            std::cerr << "Failed to read 'use_box_blind'" << std::endl;
        }
        if (reader.readEigen("downsample.box_corner.box_corner_x", box_corner_x)) {
            std::cout << "box_corner_x: " << box_corner_x << std::endl;
        } else {
            std::cerr << "Failed to read 'box_corner_x'" << std::endl;
        }
        if (reader.readEigen("downsample.box_corner.box_corner_y", box_corner_y)) {
            std::cout << "box_corner_y: " << box_corner_y << std::endl;
        } else {
            std::cerr << "Failed to read 'box_corner_y'" << std::endl;
        }
        if (reader.readEigen("downsample.box_corner.box_corner_z", box_corner_z)) {
            std::cout << "box_corner_z: " << box_corner_z << std::endl;
        } else {
            std::cerr << "Failed to read 'box_corner_z'" << std::endl;
        }
        if (reader.read("downsample.filter_size", filter_size)) {
            std::cout << "filter_size: " << filter_size << std::endl;
        } else {
            std::cerr << "Failed to read 'filter_size'" << std::endl;
        }
        /******** pcd_save ********/
        if (reader.read("pcd_save.global_map_save_enable", global_map_save_enable)) {
            std::cout << "global_map_save_enable: " << global_map_save_enable << std::endl;
        } else {
            std::cerr << "Failed to read 'global_map_save_enable'" << std::endl;
        }
        if (reader.read("pcd_save.every_n_frame_save", every_n_frame_save)) {
            std::cout << "every_n_frame_save: " << every_n_frame_save << std::endl;
        } else {
            std::cerr << "Failed to read 'every_n_frame_save'" << std::endl;
        }
        if (reader.read("pcd_save.timestamp_prefix", timestamp_prefix)) {
            std::cout << "timestamp_prefix: " << timestamp_prefix << std::endl;
        } else {
            std::cerr << "Failed to read 'timestamp_prefix'" << std::endl;
        }
        if (reader.read("pcd_save.pcd_save_name", pcd_save_name)) {
            std::cout << "pcd_save_name: " << pcd_save_name << std::endl;
        } else {
            std::cerr << "Failed to read 'pcd_save_name'" << std::endl;
        }
        /******** pub ********/
        if (reader.read("pub.global_map_pub_enable", global_map_pub_enable)) {
            std::cout << "global_map_pub_enable: " << global_map_pub_enable << std::endl;
        } else {
            std::cerr << "Failed to read 'global_map_pub_enable'" << std::endl;
        }
        if (reader.read("pub.ikdtree_pub_enable", ikdtree_pub_enable)) {
            std::cout << "ikdtree_pub_enable: " << ikdtree_pub_enable << std::endl;
        } else {
            std::cerr << "Failed to read 'ikdtree_pub_enable'" << std::endl;
        }
        /******** topic_sub ********/
        if (reader.read("topic_sub.lidar_topic_name", lidar_topic_name)) {
            std::cout << "lidar_topic_name: " << lidar_topic_name << std::endl;
        } else {
            std::cerr << "Failed to read 'lidar_topic_name'" << std::endl;
        }
        if (reader.read("topic_sub.imu_topic_name", imu_topic_name)) {
            std::cout << "imu_topic_name: " << imu_topic_name << std::endl;
        } else {
            std::cerr << "Failed to read 'imu_topic_name'" << std::endl;
        }
        if (reader.read("topic_sub.wheel_topic_name", wheel_topic_name)) {
            std::cout << "wheel_topic_name: " << wheel_topic_name << std::endl;
        } else {
            std::cerr << "Failed to read 'wheel_topic_name'" << std::endl;
        }
        if (reader.read("topic_sub.image_topic_name", image_topic_name)) {
            std::cout << "image_topic_name: " << image_topic_name << std::endl;
        } else {
            std::cerr << "Failed to read 'image_topic_name'" << std::endl;
        }
        /******** topic_pub ********/
        if (reader.read("topic_pub.clouds_lidar_topic_name", clouds_lidar_topic_name)) {
            std::cout << "clouds_lidar_topic_name: " << clouds_lidar_topic_name << std::endl;
        } else {
            std::cerr << "Failed to read 'clouds_lidar_topic_name'" << std::endl;
        }
        if (reader.read("topic_pub.global_map_topic_name", global_map_topic_name)) {
            std::cout << "global_map_topic_name: " << global_map_topic_name << std::endl;
        } else {
            std::cerr << "Failed to read 'global_map_topic_name'" << std::endl;
        }
        if (reader.read("topic_pub.ikdtree_topic_name", ikdtree_topic_name)) {
            std::cout << "ikdtree_topic_name: " << ikdtree_topic_name << std::endl;
        } else {
            std::cerr << "Failed to read 'ikdtree_topic_name'" << std::endl;
        }
        if (reader.read("topic_pub.odom_imu_topic_name", odom_imu_topic_name)) {
            std::cout << "odom_imu_topic_name: " << odom_imu_topic_name << std::endl;
        } else {
            std::cerr << "Failed to read 'odom_imu_topic_name'" << std::endl;
        }
        if (reader.read("topic_pub.odom_vehicle_topic_name", odom_vehicle_topic_name)) {
            std::cout << "odom_vehicle_topic_name: " << odom_vehicle_topic_name << std::endl;
        } else {
            std::cerr << "Failed to read 'odom_vehicle_topic_name'" << std::endl;
        }
        /***** relocation ******/
        if (reader.read("relocation.relocation_enable", relocation_enable)) {
            std::cout << "relocation_enable: " << relocation_enable << std::endl;
        } else {
            std::cerr << "Failed to read 'relocation_enable'" << std::endl;
        }
        if (reader.read("relocation.pcd_load_name", pcd_load_name)) {
            std::cout << "pcd_load_name: " << pcd_load_name << std::endl;
        } else {
            std::cerr << "Failed to read 'pcd_load_name'" << std::endl;
        }

        /***** wheel ******/
        if (reader.read("wheel.wheel_enable", wheel_enable)) {
            std::cout << "wheel_enable: " << wheel_enable << std::endl;
        } else {
            std::cerr << "Failed to read 'wheel_enable'" << std::endl;
        }
        /***** log ******/
        if (reader.read("log.log_save_enable", log_save_enable)) {
            std::cout << "log_save_enable: " << log_save_enable << std::endl;
        } else {
            std::cerr << "Failed to read 'log_save_enable'" << std::endl;
        }
        if (reader.read("log.wheel_data_log_enable", wheel_data_log_enable)) {
            std::cout << "wheel_data_log_enable: " << wheel_data_log_enable << std::endl;
        } else {
            std::cerr << "Failed to read 'wheel_data_log_enable'" << std::endl;
        }
        if (reader.read("log.trajectory_log_enable", trajectory_log_enable)) {
            std::cout << "trajectory_log_enable: " << trajectory_log_enable << std::endl;
        } else {
            std::cerr << "Failed to read 'trajectory_log_enable'" << std::endl;
        }
        if (reader.read("log.time_log_enable", time_log_enable)) {
            std::cout << "time_log_enable: " << time_log_enable << std::endl;
        } else {
            std::cerr << "Failed to read 'time_log_enable'" << std::endl;
        }
        if (reader.read("log.pub_time_log_enable", pub_time_log_enable)) {
            std::cout << "pub_time_log_enable: " << pub_time_log_enable << std::endl;
        } else {
            std::cerr << "Failed to read 'pub_time_log_enable'" << std::endl;
        }
        if (reader.read("log.res_wheel_log_enable", res_wheel_log_enable)) {
            std::cout << "res_wheel_log_enable: " << res_wheel_log_enable << std::endl;
        } else {
            std::cerr << "Failed to read 'res_wheel_log_enable'" << std::endl;
        }
    }
    else {
        std::cerr << "Failed to load YAML file" << std::endl;
    }
}

string lio_yaml;

void load_root_yaml( string file_name ) {
    YamlReader reader(std::string(PACKAGE_ROOT_DIR) +  "/yaml/" + file_name);
    if (reader.load()) {
        if (reader.read("lio_yaml", lio_yaml)) {
            std::cout << "lio_yaml: " << lio_yaml << std::endl;
        } else {
            std::cerr << "Failed to read 'lio_yaml'" << std::endl;
        }
        load_lio_yaml(lio_yaml);
    }
    else {
        std::cerr << "Failed to load ROOT YAML file" << std::endl;
    }
}