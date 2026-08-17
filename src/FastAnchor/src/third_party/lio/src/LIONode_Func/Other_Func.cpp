//
// Created by ss on 25-5-5.
//

#include "main.h"

extern std::shared_ptr<EskfEstimator> p_eskf_estimator;
extern vector<BoxPointType> cub_needrm;

/********************************************** Other Function **********************************************/

/**
 * @brief 根据当前位置动态移动局部地图，减少内存占用
 */
void LIONode::lasermap_fov_segment() {
    if (p_eskf_estimator -> get_prior_states().empty()) { // TODO: 这是一个补丁，暂时不知道为什么有的时候会为空
        return;
    }
    static BoxPointType LocalMap_Points; // ikd-tree中，局部地图的包围盒角点
    cub_needrm.clear(); // 初始化待删除点云的数组
    static int cnt = 0;
    cnt ++;
    /***********************************   获取当前位置   ***************************************/
    auto states = p_eskf_estimator -> get_prior_states();
    Eigen::Vector3d pos_LiD = states.back().p;  // world 系下lidar位置
    static bool Localmap_Initialized = false;
    double cube_len = 1500;
    /***********************************   初始化局部地图   ***************************************/
    // >>> 以下代码只运行一次 >>> //
    if (!Localmap_Initialized){
        // 初始化局部地图包围盒角点，以为w系下lidar位置为中心,得到长宽高均为cube_len的局部地图
        // 在yaml文件中，cube_len均被定义为1000
        for (int i = 0; i < 3; i++){ // 系统起始需要初始化局部地图的大小和位置
            LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
            LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
        }
        Localmap_Initialized = true;
        return;
    }
    // <<< 以上代码只运行一次 <<< //
    /************************************   计算与边界的距离   **********************************************/
    double dist_to_map_edge[3][2];// 各个方向上Lidar与局部地图边界的距离，或者说是lidar与立方体盒子六个面的距离
    bool need_move = false;
    // pos_LiD 会随载体移动而不断变化
    double DET_RANGE = 200.0f;         // 设置的当前雷达系中心到各个地图边缘的距离阈值
    const double MOV_THRESHOLD = 1.5f; // 设置的当前雷达系中心到各个地图边缘的权重
    for (int i = 0; i < 3; i++){      // 当前雷达系中心到各个地图边缘的距离【 6个 】
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]); // 与坐标较小的三个面的距离
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]); // 与坐标较大的三个面的距离
        // 与某个方向上的边界距离（例如1.5*300m）太小，标记需要移动need_move
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE || dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) need_move = true;
    }
    if (!need_move) return; // 如果不需要移动直接返回，如果距离太近，就会移动并更新局部地图位置
    /**********************************   计算需要移动的距离，移动局部box   ***********************************/
    BoxPointType New_LocalMap_Points, tmp_boxpoints;
    New_LocalMap_Points = LocalMap_Points;

    // 具体计算需要移动的距离，为了防止 cube_len 设置的太小，所以这里设定一个最小移动距离（150m）
    double mov_dist = std::max(
        0.9 * (0.5 * cube_len - MOV_THRESHOLD * DET_RANGE),
        DET_RANGE * (MOV_THRESHOLD - 1)
    );

    std::cout << "[DEBUG] mov_dist = " << mov_dist << std::endl;

    // 打印当前 LocalMap_Points 的初值
    std::cout << "[DEBUG] LocalMap_Points (before move): \n"
              << "         min = (" << LocalMap_Points.vertex_min[0] << ", "
                                   << LocalMap_Points.vertex_min[1] << ", "
                                   << LocalMap_Points.vertex_min[2] << ")\n"
              << "         max = (" << LocalMap_Points.vertex_max[0] << ", "
                                   << LocalMap_Points.vertex_max[1] << ", "
                                   << LocalMap_Points.vertex_max[2] << ")\n";

    for (int i = 0; i < 3; i++)
    {
        // 打印当前 i 轴上与边界的距离
        std::cout << "[DEBUG] Axis i=" << i
                  << ", dist_to_map_edge[i][0]=" << dist_to_map_edge[i][0]
                  << ", dist_to_map_edge[i][1]=" << dist_to_map_edge[i][1]
                  << std::endl;

        // 具体看一下需要向哪个方向移动
        tmp_boxpoints = LocalMap_Points; // 先复制旧的包围框

        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE)
        {
            // 如果与坐标较小的那个面较近
            New_LocalMap_Points.vertex_max[i] -= mov_dist;  // 包围框整体向负方向移动
            New_LocalMap_Points.vertex_min[i] -= mov_dist;

            // 构造要删除的 box
            tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;

            // 打印下要删除区域
            std::cout << "[DEBUG]   -> Removing box (negative side) on axis " << i << "\n"
                      << "            tmp_boxpoints min = (" << tmp_boxpoints.vertex_min[0] << ", "
                                                           << tmp_boxpoints.vertex_min[1] << ", "
                                                           << tmp_boxpoints.vertex_min[2] << ")\n"
                      << "            tmp_boxpoints max = (" << tmp_boxpoints.vertex_max[0] << ", "
                                                           << tmp_boxpoints.vertex_max[1] << ", "
                                                           << tmp_boxpoints.vertex_max[2] << ")\n";

            cub_needrm.push_back(tmp_boxpoints);

        }
        else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE)
        {
            // 如果与坐标较大的那个面较近
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;

            tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;

            // 打印下要删除区域
            std::cout << "[DEBUG]   -> Removing box (positive side) on axis " << i << "\n"
                      << "            tmp_boxpoints min = (" << tmp_boxpoints.vertex_min[0] << ", "
                                                           << tmp_boxpoints.vertex_min[1] << ", "
                                                           << tmp_boxpoints.vertex_min[2] << ")\n"
                      << "            tmp_boxpoints max = (" << tmp_boxpoints.vertex_max[0] << ", "
                                                           << tmp_boxpoints.vertex_max[1] << ", "
                                                           << tmp_boxpoints.vertex_max[2] << ")\n";

            cub_needrm.push_back(tmp_boxpoints);
        }
    }

    // 更新全局的 LocalMap_Points
    LocalMap_Points = New_LocalMap_Points;

    // 打印更新后的包围框
    std::cout << "[DEBUG] LocalMap_Points (after move): \n"
              << "         min = (" << LocalMap_Points.vertex_min[0] << ", "
                                   << LocalMap_Points.vertex_min[1] << ", "
                                   << LocalMap_Points.vertex_min[2] << ")\n"
              << "         max = (" << LocalMap_Points.vertex_max[0] << ", "
                                   << LocalMap_Points.vertex_max[1] << ", "
                                   << LocalMap_Points.vertex_max[2] << ")\n"
              << std::endl;
    /**********************************   删除 cub_needrm 中的点   ***********************************/
    // ikdtree.Delete_Point_Boxes(cub_needrm);
    /*****************************************   log 部分   ***********************************/
    int kdtree_delete_counter;
    if(cub_needrm.size() > 0) kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm); // 把移除部分对应的点删除
    // 将kdtree_delete_counter写入到文件中
    std::ofstream outfile;
    outfile.open(string(PACKAGE_ROOT_DIR) + "/kdtree_delete_counter.txt", std::ios::app);
    outfile << "cnt: " << cnt << " kdtree_delete_counter : " << kdtree_delete_counter << std::endl;
    outfile.close();
}


void LIONode::map_incremental(PointCloudXYZI & lidar_clouds) {
    /************************** 重定位模式不更新地图 ******************************/
    if (lidar_clouds.empty()) return;
    if (relocation_enable) return ;
    // auto world_points = p_eskf_estimator->transLidar2World(lidar_clouds)->points;
    // ikdtree.Add_Points(world_points, true);
    // return;

    /******************************** 建图模式 *********************************/
    auto world_points = p_eskf_estimator->transLidar2World(lidar_clouds)->points;
    // if (1) {
    //     ikdtree.Add_Points(world_points, true); // 测试使用 ikdtree 自带函数直接全部添加
    //     return;
    // }
    /**
     * 这里将待添加的点云分为两类，一类是需要降采样的点云，一类是不需要降采样的点云，
     * 等下会分别使用不同的参数来添加到ikdtree中，【这样比直接让ikdtree全部降采样会快零点几ms】
     * - ikdtree.Add_Points(PointNeedDownsample, true);
     * - ikdtree.Add_Points(PointNoNeedDownsample, false);
     */
    PointVector PointNeedDownsample;
    PointVector PointNoNeedDownsample;
    size_t feats_down_size = lidar_clouds.size();
    PointNeedDownsample.reserve(feats_down_size);   //构建的地图点，需要降采样的点云
    PointNoNeedDownsample.reserve(feats_down_size); //构建的地图点，不需要降采样的点云
    /***************** 根据点与所在包围盒中心点的距离，分类是否需要降采样 ***************/
    for (size_t i = 0; i < feats_down_size; i++)
    {
        // 判断是否有关键点需要加到地图中
        if (!p_eskf_estimator->Nearest_Points[i].empty()) // Nearest_Points 数组在 EskfEstimator::UpdateByLidarIESKF()函数中更新
        {
            const PointVector &points_near = p_eskf_estimator->Nearest_Points[i];//获取附近的点云
            bool need_add = true; //是否需要加入到地图中，初始化为true
            PointType  mid_point; // mid_point即为该特征点所属的栅格的中心点坐标

            // filter_size_map_min 是地图体素降采样的栅格边长，设为0.5m
            double filter_size_map_min = 0.5;
            mid_point.x = floor(world_points[i].x/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.y = floor(world_points[i].y/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.z = floor(world_points[i].z/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;

            // 当前点与box中心的距离
            float dist  = calc_dist2(world_points[i],mid_point);

            // 判断最近点在x、y、z三个方向上，与中心的距离，判断是否加入时需要降采样
            if (fabs(points_near[0].x - mid_point.x) > 0.5 * filter_size_map_min && fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_map_min && fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_map_min){
                //若三个方向距离都大于地图栅格半轴长，无需降采样
                PointNoNeedDownsample.push_back(world_points[i]);
                continue;
            }
            constexpr size_t NUM_MATCH_POINTS = 5;

            //判断当前点的 NUM_MATCH_POINTS 个邻近点与包围盒中心的范围
            for (size_t readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i ++)
            {
                if (points_near.size() < NUM_MATCH_POINTS) break;//若邻近点数小于NUM_MATCH_POINTS，则直接跳出，添加到PointToAdd中
                if (calc_dist2(points_near[readd_i], mid_point) < dist)// 如果存在邻近点到中心的距离小于当前点到中心的距离，则不需要添加当前点
                {
                    need_add = false;
                    break;
                }
            }
            if (need_add) PointNeedDownsample.push_back(world_points[i]);//加入到PointToAdd中
        }
        else
        {
            PointNeedDownsample.push_back(world_points[i]); // 如果周围没有点或者没有初始化EKF，则加入到PointToAdd中
        }
    }
    ikdtree.Add_Points(PointNeedDownsample, true);     //加入点时需要降采样
    ikdtree.Add_Points(PointNoNeedDownsample, false);  //加入点时不需要降采样
    // add_point_size = ikdtree.Add_Points(PointToAdd, true);
    // add_point_size = PointToAdd.size() + PointNoNeedDownsample.size();
}