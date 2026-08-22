#include "path_searching/dyn_a_star.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/point.hpp>

using namespace std;
using namespace Eigen;

AStar::~AStar()
{
    for (int i = 0; i < POOL_SIZE_(0); i++)
        for (int j = 0; j < POOL_SIZE_(1); j++)
            for (int k = 0; k < POOL_SIZE_(2); k++)
                delete GridNodeMap_[i][j][k];
}

void AStar::initGridMap(GridMap::Ptr occ_map, const Eigen::Vector3i pool_size, rclcpp::Node *node)
{
    POOL_SIZE_ = pool_size;
    CENTER_IDX_ = pool_size / 2;

    GridNodeMap_ = new GridNodePtr **[POOL_SIZE_(0)];
    for (int i = 0; i < POOL_SIZE_(0); i++)
    {
        GridNodeMap_[i] = new GridNodePtr *[POOL_SIZE_(1)];
        for (int j = 0; j < POOL_SIZE_(1); j++)
        {
            GridNodeMap_[i][j] = new GridNodePtr[POOL_SIZE_(2)];
            for (int k = 0; k < POOL_SIZE_(2); k++)
            {
                GridNodeMap_[i][j][k] = new GridNode;
            }
        }
    }

    grid_map_ = occ_map;
    node_ = node;
    if (node_ != nullptr)
    {
        if (!node_->has_parameter("astar.search_plane_z_offset"))
            node_->declare_parameter<double>("astar.search_plane_z_offset", 0.0);
        if (!node_->has_parameter("astar.visualize_search_plane"))
            node_->declare_parameter<bool>("astar.visualize_search_plane", false);

        node_->get_parameter("grid_map.frame_id", search_plane_frame_id_);
        const auto marker_qos = rclcpp::QoS(2).reliable().transient_local();
        search_plane_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
            "a_star_search_plane", marker_qos);
        updateSearchPlaneParameters();
        RCLCPP_INFO(node_->get_logger(),
                    "A-star search plane: z_offset=%.3f, visualization=%s, frame=%s",
                    search_plane_z_offset_, visualize_search_plane_ ? "enabled" : "disabled",
                    search_plane_frame_id_.c_str());
    }
}

void AStar::updateSearchPlaneParameters()
{
    if (node_ == nullptr)
        return;

    node_->get_parameter("astar.search_plane_z_offset", search_plane_z_offset_);
    node_->get_parameter("astar.visualize_search_plane", visualize_search_plane_);
    node_->get_parameter("grid_map.frame_id", search_plane_frame_id_);
}

void AStar::clearSearchPlaneVisualization()
{
    if (!search_plane_visible_ || search_plane_pub_ == nullptr)
        return;

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = search_plane_frame_id_;
    marker.header.stamp = node_->now();
    marker.ns = "a_star_search_plane";
    marker.action = visualization_msgs::msg::Marker::DELETEALL;
    search_plane_pub_->publish(marker);
    search_plane_visible_ = false;
}

void AStar::publishSearchPlane(const Eigen::Vector3d &start, const Eigen::Vector3d &end)
{
    if (!visualize_search_plane_)
    {
        clearSearchPlaneVisualization();
        return;
    }
    if (search_plane_pub_ == nullptr)
        return;

    Eigen::Vector2d longitudinal = end.head<2>() - start.head<2>();
    if (longitudinal.squaredNorm() < 1e-8)
        longitudinal = Eigen::Vector2d::UnitX();
    else
        longitudinal.normalize();
    const Eigen::Vector2d lateral(-longitudinal.y(), longitudinal.x());
    const double half_width = 0.5 * std::min(POOL_SIZE_(0), POOL_SIZE_(1)) * step_size_;

    Eigen::Vector3d start_left = start;
    Eigen::Vector3d start_right = start;
    Eigen::Vector3d end_left = end;
    Eigen::Vector3d end_right = end;
    start_left.head<2>() += lateral * half_width;
    start_right.head<2>() -= lateral * half_width;
    end_left.head<2>() += lateral * half_width;
    end_right.head<2>() -= lateral * half_width;
    start_left.z() += search_plane_z_offset_;
    start_right.z() += search_plane_z_offset_;
    end_left.z() += search_plane_z_offset_;
    end_right.z() += search_plane_z_offset_;

    const auto toPoint = [](const Eigen::Vector3d &point) {
        geometry_msgs::msg::Point msg;
        msg.x = point.x();
        msg.y = point.y();
        msg.z = point.z();
        return msg;
    };

    visualization_msgs::msg::Marker surface;
    surface.header.frame_id = search_plane_frame_id_;
    surface.header.stamp = node_->now();
    surface.ns = "a_star_search_plane";
    surface.id = 0;
    surface.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
    surface.action = visualization_msgs::msg::Marker::ADD;
    surface.pose.orientation.w = 1.0;
    surface.scale.x = surface.scale.y = surface.scale.z = 1.0;
    surface.color.r = 0.0f;
    surface.color.g = 0.65f;
    surface.color.b = 1.0f;
    surface.color.a = 0.22f;
    surface.points = {
        toPoint(start_left), toPoint(start_right), toPoint(end_right),
        toPoint(start_left), toPoint(end_right), toPoint(end_left)};
    search_plane_pub_->publish(surface);

    visualization_msgs::msg::Marker outline = surface;
    outline.id = 1;
    outline.type = visualization_msgs::msg::Marker::LINE_STRIP;
    outline.scale.x = std::max(0.02, step_size_ * 0.2);
    outline.color.r = 0.0f;
    outline.color.g = 0.85f;
    outline.color.b = 1.0f;
    outline.color.a = 0.9f;
    outline.points = {
        toPoint(start_left), toPoint(start_right), toPoint(end_right),
        toPoint(end_left), toPoint(start_left)};
    search_plane_pub_->publish(outline);
    search_plane_visible_ = true;
}

double AStar::getDiagHeu(GridNodePtr node1, GridNodePtr node2)
{
    double dx = abs(node1->index(0) - node2->index(0));
    double dy = abs(node1->index(1) - node2->index(1));
    double dz = abs(node1->index(2) - node2->index(2));

    double h = 0.0;
    int diag = min(min(dx, dy), dz);
    dx -= diag;
    dy -= diag;
    dz -= diag;

    if (dx == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dy, dz) + 1.0 * abs(dy - dz);
    }
    if (dy == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dz) + 1.0 * abs(dx - dz);
    }
    if (dz == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dy) + 1.0 * abs(dx - dy);
    }
    return h;
}

double AStar::getManhHeu(GridNodePtr node1, GridNodePtr node2)
{
    double dx = abs(node1->index(0) - node2->index(0));
    double dy = abs(node1->index(1) - node2->index(1));
    double dz = abs(node1->index(2) - node2->index(2));

    return dx + dy + dz;
}

double AStar::getEuclHeu(GridNodePtr node1, GridNodePtr node2)
{
    return (node2->index - node1->index).norm();
}

vector<GridNodePtr> AStar::retrievePath(GridNodePtr current)
{
    vector<GridNodePtr> path;
    path.push_back(current);

    while (current->cameFrom != NULL)
    {
        current = current->cameFrom;
        path.push_back(current);
    }

    return path;
}

bool AStar::ConvertToIndexAndAdjustStartEndPoints(Vector3d start_pt, Vector3d end_pt, Vector3i &start_idx, Vector3i &end_idx)
{
    if (!Coord2Index(start_pt, start_idx) || !Coord2Index(end_pt, end_idx))
        return false;

    Eigen::Vector3d start_to_end = end_pt - start_pt;
    if (start_to_end.norm() < 1e-6)
        return false;
    const double path_yaw = std::atan2(start_to_end(1), start_to_end(0));
    start_to_end.normalize();

    int occ = checkOccupancy(Index2Coord(start_idx), path_yaw);
    if (occ)
    {
        //ROS_WARN("Start point is insdide an obstacle.");
        do
        {
            start_pt -= start_to_end * step_size_;
            if (!Coord2Index(start_pt, start_idx))
                return false;

            occ = checkOccupancy(Index2Coord(start_idx), path_yaw);
            if (occ == -1)
            {
                RCLCPP_WARN(rclcpp::get_logger("path_searching"), "[Astar] Start point outside the map region.");
                return false;
            }
        } while (occ);
    }

    occ = checkOccupancy(Index2Coord(end_idx), path_yaw);
    if (occ)
    {
        //ROS_WARN("End point is insdide an obstacle.");
        do
        {
            end_pt += start_to_end * step_size_;
            if (!Coord2Index(end_pt, end_idx))
                return false;

            occ = checkOccupancy(Index2Coord(end_idx), path_yaw);
            if (occ == -1)
            {
                RCLCPP_WARN(rclcpp::get_logger("path_searching"), "[Astar] End point outside the map region.");
                return false;
            }
        } while (occ);
    }

    return true;
}

ASTAR_RET AStar::AstarSearch(const double step_size, Vector3d start_pt, Vector3d end_pt)
{
    const auto time_1 = std::chrono::steady_clock::now();
    ++rounds_;

    updateSearchPlaneParameters();

    step_size_ = step_size;
    inv_step_size_ = 1 / step_size;
    center_ = (start_pt + end_pt) / 2;

    Vector3i start_idx, end_idx;
    if (!ConvertToIndexAndAdjustStartEndPoints(start_pt, end_pt, start_idx, end_idx))
    {
        RCLCPP_ERROR(rclcpp::get_logger("path_searching"),
                     "Unable to handle the initial or end point, force return!");
        return ASTAR_RET::INIT_ERR;
    }

    const Eigen::Vector3d search_start = Index2Coord(start_idx);
    const Eigen::Vector3d search_end = Index2Coord(end_idx);
    const Eigen::Vector2d search_start_xy = search_start.head<2>();
    const Eigen::Vector2d search_xy_delta = search_end.head<2>() - search_start_xy;
    const double search_xy_len2 = search_xy_delta.squaredNorm();

    auto interpolateZIndexOnSearchPlane = [&](const int x_idx, const int y_idx) -> int {
        if (search_xy_len2 < 1e-8)
        {
            const double z = search_start(2) + search_plane_z_offset_;
            return static_cast<int>(std::lround((z - center_(2)) * inv_step_size_)) + CENTER_IDX_(2);
        }

        Eigen::Vector3i sample_idx(x_idx, y_idx, start_idx(2));
        const Eigen::Vector2d sample_xy = Index2Coord(sample_idx).head<2>();
        double ratio = (sample_xy - search_start_xy).dot(search_xy_delta) / search_xy_len2;
        ratio = std::max(0.0, std::min(1.0, ratio));

        const double z = search_start(2) + ratio * (search_end(2) - search_start(2)) +
                         search_plane_z_offset_;
        return static_cast<int>(std::lround((z - center_(2)) * inv_step_size_)) + CENTER_IDX_(2);
    };

    publishSearchPlane(search_start, search_end);

    start_idx(2) = interpolateZIndexOnSearchPlane(start_idx(0), start_idx(1));
    end_idx(2) = interpolateZIndexOnSearchPlane(end_idx(0), end_idx(1));
    if (start_idx(2) < 1 || start_idx(2) >= POOL_SIZE_(2) - 1 ||
        end_idx(2) < 1 || end_idx(2) >= POOL_SIZE_(2) - 1)
    {
        RCLCPP_ERROR(rclcpp::get_logger("path_searching"),
                     "A-star search plane offset %.3f moves an endpoint outside the search pool",
                     search_plane_z_offset_);
        return ASTAR_RET::INIT_ERR;
    }

    // if ( start_pt(0) > -1 && start_pt(0) < 0 )
    //     cout << "start_pt=" << start_pt.transpose() << " end_pt=" << end_pt.transpose() << endl;

    GridNodePtr startPtr = GridNodeMap_[start_idx(0)][start_idx(1)][start_idx(2)];
    GridNodePtr endPtr = GridNodeMap_[end_idx(0)][end_idx(1)][end_idx(2)];

    std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, NodeComparator> empty;
    openSet_.swap(empty);

    GridNodePtr neighborPtr = NULL;
    GridNodePtr current = NULL;

    endPtr->index = end_idx;

    startPtr->index = start_idx;
    startPtr->rounds = rounds_;
    startPtr->gScore = 0;
    startPtr->fScore = getHeu(startPtr, endPtr);
    startPtr->state = GridNode::OPENSET; //put start node in open set
    startPtr->cameFrom = NULL;
    openSet_.push(startPtr); //put start in open set

    double tentative_gScore;

    int num_iter = 0;
    while (!openSet_.empty())
    {
        num_iter++;
        current = openSet_.top();
        openSet_.pop();

        // if ( num_iter < 10000 )
        //     cout << "current=" << current->index.transpose() << endl;

        if (current->index(0) == endPtr->index(0) && current->index(1) == endPtr->index(1) && current->index(2) == endPtr->index(2))
        {
            // ros::Time time_2 = ros::Time::now();
            // printf("\033[34mA star iter:%d, time:%.3f\033[0m\n",num_iter, (time_2 - time_1).toSec()*1000);
            // if((time_2 - time_1).toSec() > 0.1)
            //     ROS_WARN("Time consume in A star path finding is %f", (time_2 - time_1).toSec() );
            gridPath_ = retrievePath(current);
            return ASTAR_RET::SUCCESS;
        }
        current->state = GridNode::CLOSEDSET; //move current node from open set to closed set.

        for (int dx = -1; dx <= 1; dx++)
            for (int dy = -1; dy <= 1; dy++)
            {
                if (dx == 0 && dy == 0)
                    continue;

                Vector3i neighborIdx;
                neighborIdx(0) = (current->index)(0) + dx;
                neighborIdx(1) = (current->index)(1) + dy;
                neighborIdx(2) = interpolateZIndexOnSearchPlane(neighborIdx(0), neighborIdx(1));

                if (neighborIdx(0) < 1 || neighborIdx(0) >= POOL_SIZE_(0) - 1 || neighborIdx(1) < 1 || neighborIdx(1) >= POOL_SIZE_(1) - 1 || neighborIdx(2) < 1 || neighborIdx(2) >= POOL_SIZE_(2) - 1)
                {
                    continue;
                }

                neighborPtr = GridNodeMap_[neighborIdx(0)][neighborIdx(1)][neighborIdx(2)];
                neighborPtr->index = neighborIdx;

                bool flag_explored = neighborPtr->rounds == rounds_;

                if (flag_explored && neighborPtr->state == GridNode::CLOSEDSET)
                {
                    continue; //in closed set.
                }

                neighborPtr->rounds = rounds_;

                const double neighbor_yaw = std::atan2(static_cast<double>(dy), static_cast<double>(dx));
                if (checkOccupancy(Index2Coord(neighborPtr->index), neighbor_yaw))
                {
                    continue;
                }

                const int dz = neighborIdx(2) - current->index(2);
                double static_cost = sqrt(dx * dx + dy * dy + dz * dz);
                tentative_gScore = current->gScore + static_cost;

                if (!flag_explored)
                {
                    //discover a new node
                    neighborPtr->state = GridNode::OPENSET;
                    neighborPtr->cameFrom = current;
                    neighborPtr->gScore = tentative_gScore;
                    neighborPtr->fScore = tentative_gScore + getHeu(neighborPtr, endPtr);
                    openSet_.push(neighborPtr); //put neighbor in open set and record it.
                }
                else if (tentative_gScore < neighborPtr->gScore)
                { //in open set and need update
                    neighborPtr->cameFrom = current;
                    neighborPtr->gScore = tentative_gScore;
                    neighborPtr->fScore = tentative_gScore + getHeu(neighborPtr, endPtr);
                }
            }
        const auto time_2 = std::chrono::steady_clock::now();
        if (std::chrono::duration<double>(time_2 - time_1).count() > 0.2)
        {
            RCLCPP_WARN(rclcpp::get_logger("path_searching"),
                        "Failed in A-star path search: 0.2 second time limit exceeded");
            return ASTAR_RET::SEARCH_ERR;
        }
    }

    const auto time_2 = std::chrono::steady_clock::now();

    const double elapsed = std::chrono::duration<double>(time_2 - time_1).count();
    if (elapsed > 0.1)
        RCLCPP_WARN(rclcpp::get_logger("path_searching"),
                    "A-star path search took %.3fs, iter=%d", elapsed, num_iter);

    return ASTAR_RET::SEARCH_ERR;
}

vector<Vector3d> AStar::getPath()
{
    vector<Vector3d> path;

    for (auto ptr : gridPath_)
        path.push_back(Index2Coord(ptr->index));

    reverse(path.begin(), path.end());
    return path;
}
