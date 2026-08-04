#include "plan_env/grid_map.h"
#include <array>
#include <cmath>
#include <chrono>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <unistd.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace
{
template <typename T>
void load_parameter(rclcpp::Node *node, const std::string &name, T &value, const T &default_value)
{
  if (!node->has_parameter(name))
    node->declare_parameter<T>(name, default_value);
  node->get_parameter(name, value);
}

double elapsedMilliseconds(const std::chrono::steady_clock::time_point &start)
{
  return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
}

void updateAtomicMaximum(std::atomic<uint64_t> &maximum, uint64_t value)
{
  uint64_t current = maximum.load(std::memory_order_relaxed);
  while (current < value &&
         !maximum.compare_exchange_weak(current, value, std::memory_order_relaxed))
  {
  }
}

uint64_t readPressureTotal(const char *path)
{
  std::ifstream input(path);
  std::string line;
  if (!std::getline(input, line) || line.rfind("some ", 0) != 0)
    return 0;
  const std::string key = "total=";
  const size_t position = line.find(key);
  if (position == std::string::npos)
    return 0;
  try
  {
    return std::stoull(line.substr(position + key.size()));
  }
  catch (const std::exception &)
  {
    return 0;
  }
}
}  // namespace

GridMap::~GridMap()
{
  {
    std::lock_guard<std::mutex> lock(visualization_mutex_);
    stop_visualization_worker_ = true;
    pending_visualization_.reset();
  }
  visualization_cv_.notify_one();
  if (visualization_worker_.joinable())
    visualization_worker_.join();
}

void GridMap::initializeRuntimeDiagnostics()
{
  if (!runtime_log_enabled_)
    return;

  runtime_window_start_ = std::chrono::steady_clock::now();
  previous_resource_sample_time_ = runtime_window_start_;
  (void)sampleSystemResources();

  if (!runtime_csv_path_.empty())
  {
    runtime_csv_stream_.open(runtime_csv_path_, std::ios::out | std::ios::app);
    if (!runtime_csv_stream_)
    {
      RCLCPP_ERROR(node_->get_logger(),
                   "[RuntimeLog] cannot open CSV path '%s'; ROS log output remains enabled",
                   runtime_csv_path_.c_str());
    }
    else if (runtime_csv_stream_.tellp() == std::streampos(0))
    {
      runtime_csv_stream_
          << "ros_time_s,window_s,status,fusion_status,visualization_status,reasons,input_hz,"
             "unique_input_hz,prepared_hz,fusion_hz,"
             "target_hz,duplicate_frames,no_pose_frames,empty_frames,coalesced_frames,avg_input_points,"
             "avg_prepared_points,decode_avg_ms,decode_max_ms,filter_avg_ms,filter_max_ms,sliding_avg_ms,"
             "sliding_max_ms,projection_avg_ms,projection_max_ms,raycast_avg_ms,raycast_max_ms,"
             "fusion_avg_ms,fusion_max_ms,queue_avg_ms,queue_max_ms,visualization_hz,visualization_target_hz,"
             "visualization_dropped,visualization_avg_ms,visualization_max_ms,visualization_avg_points,"
             "process_cpu_percent,system_cpu_percent,memory_percent,process_rss_mb,load_1m,"
             "cpu_pressure_percent,memory_pressure_percent\n";
      runtime_csv_stream_.flush();
    }
  }

  const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(runtime_report_interval_sec_));
  runtime_diagnostics_timer_ = node_->create_wall_timer(
      period, std::bind(&GridMap::runtimeDiagnosticsCallback, this));
  RCLCPP_INFO(node_->get_logger(),
              "[RuntimeLog] enabled: map target=%.1f Hz tolerance=%.0f%% report=%.1f s csv=%s",
              runtime_map_target_rate_hz_, runtime_rate_tolerance_ * 100.0,
              runtime_report_interval_sec_, runtime_csv_path_.empty() ? "disabled" : runtime_csv_path_.c_str());
}

GridMap::SystemSample GridMap::sampleSystemResources()
{
  SystemSample sample;
  const auto now = std::chrono::steady_clock::now();

  uint64_t system_total = 0;
  uint64_t system_idle = 0;
  {
    std::ifstream stat("/proc/stat");
    std::string cpu;
    std::array<uint64_t, 10> ticks{};
    if (stat >> cpu && cpu == "cpu")
    {
      for (auto &tick : ticks)
        stat >> tick;
      for (const uint64_t tick : ticks)
        system_total += tick;
      system_idle = ticks[3] + ticks[4];
    }
  }

  uint64_t process_ticks = 0;
  {
    std::ifstream stat("/proc/self/stat");
    std::string line;
    std::getline(stat, line);
    const size_t command_end = line.rfind(')');
    if (command_end != std::string::npos && command_end + 2 < line.size())
    {
      std::istringstream fields(line.substr(command_end + 2));
      std::vector<std::string> values;
      std::string value;
      while (fields >> value)
        values.push_back(value);
      // values[0] is field 3 (state); utime/stime are fields 14 and 15.
      if (values.size() > 12)
      {
        try
        {
          process_ticks = std::stoull(values[11]) + std::stoull(values[12]);
        }
        catch (const std::exception &)
        {
          process_ticks = 0;
        }
      }
    }
  }

  uint64_t memory_total_kb = 0;
  uint64_t memory_available_kb = 0;
  {
    std::ifstream meminfo("/proc/meminfo");
    std::string line;
    while (std::getline(meminfo, line))
    {
      std::istringstream fields(line);
      std::string key;
      uint64_t value = 0;
      fields >> key >> value;
      if (key == "MemTotal:")
        memory_total_kb = value;
      else if (key == "MemAvailable:")
        memory_available_kb = value;
    }
  }

  {
    std::ifstream status("/proc/self/status");
    std::string key;
    while (status >> key)
    {
      if (key == "VmRSS:")
      {
        uint64_t rss_kb = 0;
        std::string unit;
        status >> rss_kb >> unit;
        sample.process_rss_mb = static_cast<double>(rss_kb) / 1024.0;
        break;
      }
      std::string remainder;
      std::getline(status, remainder);
    }
  }

  {
    std::ifstream loadavg("/proc/loadavg");
    loadavg >> sample.load_1m;
  }

  const uint64_t cpu_pressure_us = readPressureTotal("/proc/pressure/cpu");
  const uint64_t memory_pressure_us = readPressureTotal("/proc/pressure/memory");
  const double elapsed_sec = std::chrono::duration<double>(now - previous_resource_sample_time_).count();
  if (have_previous_resource_sample_ && elapsed_sec > 0.0)
  {
    const uint64_t total_delta = system_total >= previous_system_total_ticks_
                                     ? system_total - previous_system_total_ticks_
                                     : 0;
    const uint64_t idle_delta = system_idle >= previous_system_idle_ticks_
                                    ? system_idle - previous_system_idle_ticks_
                                    : 0;
    if (total_delta > 0)
      sample.system_cpu_percent =
          100.0 * static_cast<double>(total_delta - std::min(total_delta, idle_delta)) /
          static_cast<double>(total_delta);

    const long clock_ticks = sysconf(_SC_CLK_TCK);
    const uint64_t process_delta = process_ticks >= previous_process_ticks_
                                       ? process_ticks - previous_process_ticks_
                                       : 0;
    if (clock_ticks > 0)
      sample.process_cpu_percent =
          100.0 * static_cast<double>(process_delta) /
          (static_cast<double>(clock_ticks) * elapsed_sec);

    const double elapsed_us = elapsed_sec * 1e6;
    if (cpu_pressure_us >= previous_cpu_pressure_us_)
      sample.cpu_pressure_percent =
          100.0 * static_cast<double>(cpu_pressure_us - previous_cpu_pressure_us_) / elapsed_us;
    if (memory_pressure_us >= previous_memory_pressure_us_)
      sample.memory_pressure_percent =
          100.0 * static_cast<double>(memory_pressure_us - previous_memory_pressure_us_) / elapsed_us;
    sample.valid = true;
  }

  if (memory_total_kb > 0)
    sample.memory_percent =
        100.0 * static_cast<double>(memory_total_kb - std::min(memory_total_kb, memory_available_kb)) /
        static_cast<double>(memory_total_kb);

  previous_system_total_ticks_ = system_total;
  previous_system_idle_ticks_ = system_idle;
  previous_process_ticks_ = process_ticks;
  previous_cpu_pressure_us_ = cpu_pressure_us;
  previous_memory_pressure_us_ = memory_pressure_us;
  previous_resource_sample_time_ = now;
  have_previous_resource_sample_ = system_total > 0;
  return sample;
}

void GridMap::runtimeDiagnosticsCallback()
{
  const auto now = std::chrono::steady_clock::now();
  const double window_sec = std::chrono::duration<double>(now - runtime_window_start_).count();
  if (window_sec <= 0.0)
    return;

  const RuntimeMetrics metrics = runtime_metrics_;
  runtime_metrics_ = RuntimeMetrics{};
  runtime_window_start_ = now;

  const uint64_t visualization_requested =
      visualization_requested_.exchange(0, std::memory_order_relaxed);
  const uint64_t visualization_dropped =
      visualization_dropped_.exchange(0, std::memory_order_relaxed);
  const uint64_t visualization_published =
      visualization_published_.exchange(0, std::memory_order_relaxed);
  const uint64_t visualization_points =
      visualization_points_.exchange(0, std::memory_order_relaxed);
  const uint64_t visualization_work_us =
      visualization_work_us_.exchange(0, std::memory_order_relaxed);
  const uint64_t visualization_work_max_us =
      visualization_work_max_us_.exchange(0, std::memory_order_relaxed);
  const SystemSample system = sampleSystemResources();

  const auto rate = [window_sec](uint64_t count) {
    return static_cast<double>(count) / window_sec;
  };
  const auto average = [](double sum, uint64_t count) {
    return count == 0 ? 0.0 : sum / static_cast<double>(count);
  };
  const double input_hz = rate(metrics.input_frames);
  const double unique_input_hz = rate(metrics.unique_input_frames);
  const double prepared_hz = rate(metrics.prepared_frames);
  const double fusion_hz = rate(metrics.fusion_frames);
  const double visualization_hz = rate(visualization_published);
  const double decode_avg_ms = average(metrics.decode_ms_sum, metrics.unique_input_frames);
  const double filter_avg_ms = average(metrics.filter_ms_sum, metrics.unique_input_frames);
  const double sliding_avg_ms = average(metrics.sliding_ms_sum, metrics.unique_input_frames);
  const double projection_avg_ms = average(metrics.projection_ms_sum, metrics.fusion_frames);
  const double raycast_avg_ms = average(metrics.raycast_ms_sum, metrics.fusion_frames);
  const double fusion_avg_ms = average(metrics.fusion_ms_sum, metrics.fusion_frames);
  const double queue_avg_ms = average(metrics.queue_wait_ms_sum, metrics.fusion_frames);
  const double visualization_avg_ms = visualization_requested == 0
                                          ? 0.0
                                          : static_cast<double>(visualization_work_us) /
                                                (1000.0 * visualization_requested);
  const double visualization_max_ms = static_cast<double>(visualization_work_max_us) / 1000.0;
  const double average_input_points = average(
      static_cast<double>(metrics.input_points), metrics.unique_input_frames);
  const double average_prepared_points = average(
      static_cast<double>(metrics.prepared_points), metrics.prepared_frames);
  const double average_visualization_points = average(
      static_cast<double>(visualization_points), visualization_published);

  const double required_hz = runtime_map_target_rate_hz_ * runtime_rate_tolerance_;
  const bool fusion_rate_ok = fusion_hz >= required_hz;
  const bool visualization_active = visualization_requested > 0;
  const bool visualization_rate_ok = !visualization_active ||
      visualization_hz >= runtime_visualization_target_rate_hz_ * runtime_rate_tolerance_;
  std::vector<std::string> reasons;
  const auto add_reason = [&reasons](const std::string &reason) {
    if (std::find(reasons.begin(), reasons.end(), reason) == reasons.end())
      reasons.push_back(reason);
  };
  if (!fusion_rate_ok)
  {
    if (metrics.no_pose_frames > 0)
      add_reason("missing_sensor_pose");
    if (unique_input_hz < required_hz)
      add_reason("upstream_unique_input_low");
    if (metrics.input_frames > 0 &&
        static_cast<double>(metrics.duplicate_input_frames) /
                static_cast<double>(metrics.input_frames) > 0.1)
      add_reason("upstream_repeated_cloud");
    if (metrics.empty_input_frames > 0)
      add_reason("empty_or_fully_filtered_input");
    if (metrics.coalesced_frames > 0 || prepared_hz > fusion_hz + 0.5)
      add_reason("fusion_queue_coalescing");

    const double budget_ms = 1000.0 / runtime_map_target_rate_hz_;
    const std::array<std::pair<const char *, double>, 5> stages{{
        {"pointcloud_decode", decode_avg_ms},
        {"pointcloud_filter", filter_avg_ms},
        {"sliding_map", sliding_avg_ms},
        {"depth_projection", projection_avg_ms},
        {"raycast_and_inflation", raycast_avg_ms},
    }};
    const auto dominant = std::max_element(
        stages.begin(), stages.end(),
        [](const auto &left, const auto &right) { return left.second < right.second; });
    if (dominant != stages.end() && dominant->second >= budget_ms * 0.35)
      add_reason(std::string("slow_") + dominant->first);
    // A fresh frame normally waits up to one 50 ms occupancy-timer period.
    // Only the excess is evidence that another callback delayed fusion.
    if (queue_avg_ms >= 50.0 + budget_ms * 0.25)
      add_reason("executor_callback_delay");
  }

  if (!visualization_rate_ok)
  {
    const double visualization_budget_ms = runtime_visualization_target_rate_hz_ > 0.0
                                               ? 1000.0 / runtime_visualization_target_rate_hz_
                                               : 0.0;
    if (visualization_dropped > 0)
      add_reason("visualization_worker_backlog");
    if (visualization_budget_ms > 0.0 && visualization_avg_ms >= visualization_budget_ms * 0.5)
      add_reason("visualization_serialization");
    if (visualization_dropped == 0 &&
        (visualization_budget_ms <= 0.0 || visualization_avg_ms < visualization_budget_ms * 0.5))
      add_reason("visualization_timer_or_executor_delay");
  }

  const bool overall_ok = fusion_rate_ok && visualization_rate_ok;
  if (!overall_ok)
  {
    if (system.valid && system.system_cpu_percent >= runtime_cpu_warn_percent_)
      add_reason("system_cpu_saturation");
    if (system.valid && system.process_cpu_percent >= 90.0)
      add_reason("planner_process_cpu_core_saturation");
    if (system.valid && system.cpu_pressure_percent >= 10.0)
      add_reason("cpu_scheduling_pressure");
    if (system.memory_percent >= runtime_memory_warn_percent_ ||
        (system.valid && system.memory_pressure_percent >= 5.0))
      add_reason("memory_pressure");
    if (reasons.empty())
      add_reason("callback_jitter_or_unmeasured_upstream_drop");
  }

  std::ostringstream reason_stream;
  if (reasons.empty())
  {
    reason_stream << "none";
  }
  else
  {
    for (size_t i = 0; i < reasons.size(); ++i)
    {
      if (i > 0)
        reason_stream << ';';
      reason_stream << reasons[i];
    }
  }
  const std::string reason_text = reason_stream.str();
  const char *status = overall_ok ? "OK" : "DEGRADED";
  const char *fusion_status = fusion_rate_ok ? "OK" : "DEGRADED";
  const char *visualization_status = !visualization_active
                                         ? "inactive"
                                         : (visualization_rate_ok ? "OK" : "DEGRADED");

  if (overall_ok)
  {
    RCLCPP_INFO(
        node_->get_logger(),
        "[RuntimeLog] map=%s fusion=%s(%.2f/%.2fHz) input=%.2fHz unique=%.2fHz prepared=%.2fHz "
        "coalesced=%lu reason=%s process_cpu=%.1f%% system_cpu=%.1f%% memory=%.1f%% rss=%.1fMiB "
        "load1=%.2f cpu_psi=%.1f%% mem_psi=%.1f%% visualization=%s(%.2f/%.2fHz,dropped=%lu)",
        status, fusion_status, fusion_hz, runtime_map_target_rate_hz_, input_hz, unique_input_hz, prepared_hz,
        static_cast<unsigned long>(metrics.coalesced_frames), reason_text.c_str(),
        system.process_cpu_percent, system.system_cpu_percent, system.memory_percent,
        system.process_rss_mb, system.load_1m, system.cpu_pressure_percent,
        system.memory_pressure_percent, visualization_status, visualization_hz,
        runtime_visualization_target_rate_hz_, static_cast<unsigned long>(visualization_dropped));
  }
  else
  {
    RCLCPP_WARN(
        node_->get_logger(),
        "[RuntimeLog] map=%s fusion=%s(%.2f/%.2fHz) input=%.2fHz unique=%.2fHz prepared=%.2fHz "
        "coalesced=%lu reason=%s process_cpu=%.1f%% system_cpu=%.1f%% memory=%.1f%% rss=%.1fMiB "
        "load1=%.2f cpu_psi=%.1f%% mem_psi=%.1f%% visualization=%s(%.2f/%.2fHz,dropped=%lu)",
        status, fusion_status, fusion_hz, runtime_map_target_rate_hz_, input_hz, unique_input_hz, prepared_hz,
        static_cast<unsigned long>(metrics.coalesced_frames), reason_text.c_str(),
        system.process_cpu_percent, system.system_cpu_percent, system.memory_percent,
        system.process_rss_mb, system.load_1m, system.cpu_pressure_percent,
        system.memory_pressure_percent, visualization_status, visualization_hz,
        runtime_visualization_target_rate_hz_, static_cast<unsigned long>(visualization_dropped));
  }

  RCLCPP_INFO(
      node_->get_logger(),
      "[RuntimeLog][Stages] points=%.0f->%.0f decode=%.2f/%.2fms filter=%.2f/%.2fms "
      "sliding=%.2f/%.2fms projection=%.2f/%.2fms raycast=%.2f/%.2fms fusion=%.2f/%.2fms "
      "queue=%.2f/%.2fms visualization=%.2f/%.2fms vis_points=%.0f duplicates=%lu no_pose=%lu empty=%lu",
      average_input_points, average_prepared_points,
      decode_avg_ms, metrics.decode_ms_max, filter_avg_ms, metrics.filter_ms_max,
      sliding_avg_ms, metrics.sliding_ms_max, projection_avg_ms, metrics.projection_ms_max,
      raycast_avg_ms, metrics.raycast_ms_max, fusion_avg_ms, metrics.fusion_ms_max,
      queue_avg_ms, metrics.queue_wait_ms_max, visualization_avg_ms, visualization_max_ms,
      average_visualization_points, static_cast<unsigned long>(metrics.duplicate_input_frames),
      static_cast<unsigned long>(metrics.no_pose_frames),
      static_cast<unsigned long>(metrics.empty_input_frames));

  if (runtime_csv_stream_)
  {
    runtime_csv_stream_
        << std::fixed << std::setprecision(6) << node_->now().seconds() << ',' << window_sec << ','
        << status << ',' << fusion_status << ',' << visualization_status << ',' << reason_text << ','
        << input_hz << ',' << unique_input_hz << ','
        << prepared_hz << ',' << fusion_hz << ',' << runtime_map_target_rate_hz_ << ','
        << metrics.duplicate_input_frames << ',' << metrics.no_pose_frames << ','
        << metrics.empty_input_frames << ',' << metrics.coalesced_frames << ','
        << average_input_points << ',' << average_prepared_points << ','
        << decode_avg_ms << ',' << metrics.decode_ms_max << ','
        << filter_avg_ms << ',' << metrics.filter_ms_max << ','
        << sliding_avg_ms << ',' << metrics.sliding_ms_max << ','
        << projection_avg_ms << ',' << metrics.projection_ms_max << ','
        << raycast_avg_ms << ',' << metrics.raycast_ms_max << ','
        << fusion_avg_ms << ',' << metrics.fusion_ms_max << ','
        << queue_avg_ms << ',' << metrics.queue_wait_ms_max << ','
        << visualization_hz << ',' << runtime_visualization_target_rate_hz_ << ','
        << visualization_dropped << ',' << visualization_avg_ms << ',' << visualization_max_ms << ','
        << average_visualization_points << ',' << system.process_cpu_percent << ','
        << system.system_cpu_percent << ',' << system.memory_percent << ',' << system.process_rss_mb << ','
        << system.load_1m << ',' << system.cpu_pressure_percent << ','
        << system.memory_pressure_percent << '\n';
    runtime_csv_stream_.flush();
  }
}

void GridMap::initMap(rclcpp::Node *node)
{
  node_ = node;

  /* get parameter */
  double x_size, y_size, z_size;
  load_parameter(node_, "grid_map.resolution", mp_.resolution_, -1.0);
  load_parameter(node_, "grid_map.sliding_map_size_x", x_size, -1.0);
  load_parameter(node_, "grid_map.sliding_map_size_y", y_size, -1.0);
  load_parameter(node_, "grid_map.sliding_map_size_z", z_size, -1.0);
  load_parameter(node_, "grid_map.local_update_range_x", mp_.local_update_range_(0), x_size / 2.0);
  load_parameter(node_, "grid_map.local_update_range_y", mp_.local_update_range_(1), y_size / 2.0);
  load_parameter(node_, "grid_map.local_update_range_z", mp_.local_update_range_(2), z_size / 2.0);
  
  load_parameter(node_, "grid_map.obstacles_inflation_z_up", mp_.obstacles_inflation_z_up, -1.0);
  load_parameter(node_, "grid_map.obstacles_inflation_z_down", mp_.obstacles_inflation_z_down, -1.0);
  load_parameter(node_, "grid_map.double_cylinder_radius", mp_.double_cylinder_radius_, -1.0);
  load_parameter(node_, "grid_map.double_cylinder_offset", mp_.double_cylinder_offset_, 0.0);
  load_parameter(node_, "grid_map.map_sliding_en", mp_.map_sliding_en_, true);
  load_parameter(node_, "grid_map.map_sliding_thresh", mp_.map_sliding_thresh_, mp_.resolution_);

  load_parameter(node_, "grid_map.fx", mp_.fx_, -1.0);
  load_parameter(node_, "grid_map.fy", mp_.fy_, -1.0);
  load_parameter(node_, "grid_map.cx", mp_.cx_, -1.0);
  load_parameter(node_, "grid_map.cy", mp_.cy_, -1.0);

  load_parameter(node_, "grid_map.depth_filter_maxdist", mp_.depth_filter_maxdist_, -1.0);
  load_parameter(node_, "grid_map.depth_filter_mindist", mp_.depth_filter_mindist_, -1.0);
  load_parameter(node_, "grid_map.depth_filter_margin", mp_.depth_filter_margin_, -1);
  load_parameter(node_, "grid_map.k_depth_scaling_factor", mp_.k_depth_scaling_factor_, -1.0);
  load_parameter(node_, "grid_map.skip_pixel", mp_.skip_pixel_, -1);

  load_parameter(node_, "grid_map.p_hit", mp_.p_hit_, -1.0);
  load_parameter(node_, "grid_map.p_miss", mp_.p_miss_, -1.0);
  load_parameter(node_, "grid_map.p_min", mp_.p_min_, -1.0);
  load_parameter(node_, "grid_map.p_max", mp_.p_max_, -1.0);
  load_parameter(node_, "grid_map.p_occ", mp_.p_occ_, -1.0);
  load_parameter(node_, "grid_map.max_ray_length", mp_.max_ray_length_, -0.1);

  load_parameter(node_, "grid_map.vis_height", mp_.vis_height_, 0.3);
  load_parameter(node_, "grid_map.show_occ_time", mp_.show_occ_time_, false);
  double visualization_rate_hz;
  load_parameter(node_, "grid_map.visualization_rate_hz", visualization_rate_hz, 5.0);
  runtime_visualization_target_rate_hz_ = std::max(0.0, visualization_rate_hz);

  load_parameter(node_, "runtime_log.enabled", runtime_log_enabled_, true);
  load_parameter(node_, "runtime_log.report_interval_sec", runtime_report_interval_sec_, 5.0);
  load_parameter(node_, "runtime_log.map_target_rate_hz", runtime_map_target_rate_hz_, 10.0);
  load_parameter(node_, "runtime_log.rate_tolerance", runtime_rate_tolerance_, 0.9);
  load_parameter(node_, "runtime_log.cpu_warn_percent", runtime_cpu_warn_percent_, 85.0);
  load_parameter(node_, "runtime_log.memory_warn_percent", runtime_memory_warn_percent_, 90.0);
  load_parameter(node_, "runtime_log.csv_path", runtime_csv_path_, std::string(""));

  runtime_report_interval_sec_ = std::max(1.0, runtime_report_interval_sec_);
  runtime_map_target_rate_hz_ = std::max(0.1, runtime_map_target_rate_hz_);
  runtime_rate_tolerance_ = std::clamp(runtime_rate_tolerance_, 0.1, 1.0);

  load_parameter(node_, "grid_map.frame_id", mp_.frame_id_, string("world"));
  load_parameter(node_, "grid_map.sliding_map_frame_id", mp_.sliding_map_frame_id_, string("sliding_map"));
  load_parameter(node_, "grid_map.ground_height", mp_.ground_height_, 0.0);

  load_parameter(node_, "grid_map.sensor_type", mp_.sensor_type_, string("lidar"));
  load_parameter(node_, "grid_map.cloud_is_world", mp_.cloud_is_world_, true);
  load_parameter(node_, "grid_map.need_extrinsic", mp_.need_extrinsic_, true);

  mp_.lidar_extrinsic_ <<
      1.0, 0.0, 0.0, -0.01100,
      0.0, 1.0, 0.0, -0.02329,
      0.0, 0.0, 1.0,  0.04412,
      0.0, 0.0, 0.0,  1.00000;

  mp_.depth_extrinsic_ <<
      0.0,  0.707107, 0.707107, -0.15170,
     -1.0,  0.000000, 0.000000,  0.00000,
      0.0, -0.707107, 0.707107,  0.07510,
      0.0,  0.000000, 0.000000,  1.00000;
      
  if (mp_.sensor_type_ != "lidar" && mp_.sensor_type_ != "depth")
  {
    RCLCPP_ERROR(node_->get_logger(), "[GridMap] invalid grid_map.sensor_type: %s; falling back to lidar",
                 mp_.sensor_type_.c_str());
    mp_.sensor_type_ = "lidar";
  }

  mp_.resolution_inv_ = 1 / mp_.resolution_;
  mp_.map_origin_ = Eigen::Vector3d(-x_size / 2.0, -y_size / 2.0, mp_.ground_height_);
  mp_.map_size_ = Eigen::Vector3d(x_size, y_size, z_size);

  mp_.prob_hit_log_ = logit(mp_.p_hit_);
  mp_.prob_miss_log_ = logit(mp_.p_miss_);
  mp_.clamp_min_log_ = logit(mp_.p_min_);
  mp_.clamp_max_log_ = logit(mp_.p_max_);
  mp_.min_occupancy_log_ = logit(mp_.p_occ_);
  mp_.unknown_flag_ = 0.01;
  mp_.map_sliding_thresh_vox_ = std::max(1, static_cast<int>(std::ceil(mp_.map_sliding_thresh_ * mp_.resolution_inv_)));

  cout << "hit: " << mp_.prob_hit_log_ << endl;
  cout << "miss: " << mp_.prob_miss_log_ << endl;
  cout << "min log: " << mp_.clamp_min_log_ << endl;
  cout << "max: " << mp_.clamp_max_log_ << endl;
  cout << "thresh log: " << mp_.min_occupancy_log_ << endl;

  for (int i = 0; i < 3; ++i)
    mp_.map_voxel_num_(i) = ceil(mp_.map_size_(i) / mp_.resolution_);

  mp_.map_min_boundary_ = mp_.map_origin_;
  mp_.map_max_boundary_ = mp_.map_origin_ + mp_.map_size_;
  posToIndex(mp_.map_origin_, mp_.map_bound_min_idx_);
  mp_.map_bound_max_idx_ = mp_.map_bound_min_idx_ + mp_.map_voxel_num_ - Eigen::Vector3i::Ones();
  mp_.map_origin_idx_ = mp_.map_bound_min_idx_ + mp_.map_voxel_num_ / 2;
  updateMapBoundaryFromIndex();

  // initialize data buffers

  int buffer_size = mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2);

  md_.occupancy_buffer_ = vector<double>(buffer_size, mp_.clamp_min_log_ - mp_.unknown_flag_);
  md_.occupancy_buffer_inflate_ = vector<char>(buffer_size, 0);
  md_.occupancy_buffer_inflate_cnt_ = vector<int>(buffer_size, 0);
  rebuildInflationOffsets();

  md_.count_hit_and_miss_ = vector<short>(buffer_size, 0);
  md_.count_hit_ = vector<short>(buffer_size, 0);
  md_.flag_rayend_ = vector<char>(buffer_size, -1);
  md_.flag_traverse_ = vector<char>(buffer_size, -1);

  md_.raycast_num_ = 0;

  md_.proj_points_.resize(640 * 480 / mp_.skip_pixel_ / mp_.skip_pixel_);
  md_.proj_points_cnt = 0;

  /* init callback */
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*node_);

  if (mp_.sensor_type_ == "depth")
  {
    depth_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>();
    depth_pose_sub_ = std::make_shared<message_filters::Subscriber<nav_msgs::msg::Odometry>>();
    depth_sub_->subscribe(node_, "depth", rmw_qos_profile_sensor_data);
    depth_pose_sub_->subscribe(node_, "sensor_pose", rmw_qos_profile_sensor_data);

    sync_image_pose_.reset(new message_filters::Synchronizer<SyncPolicyImagePose>(
        SyncPolicyImagePose(100), *depth_sub_, *depth_pose_sub_));
    sync_image_pose_->registerCallback(
        std::bind(&GridMap::depthPoseCallback, this, std::placeholders::_1, std::placeholders::_2));
  }
  else if (mp_.sensor_type_ == "lidar")
  {
    lidar_pose_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "sensor_pose", rclcpp::SensorDataQoS(),
        std::bind(&GridMap::sensorPoseCallback, this, std::placeholders::_1));
    cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        "cloud", rclcpp::SensorDataQoS(),
        std::bind(&GridMap::cloudCallback, this, std::placeholders::_1));
  }

  sliding_map_frame_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      "body_pose", rclcpp::SensorDataQoS(),
      std::bind(&GridMap::slidingMapFrameCallback, this, std::placeholders::_1));

  occ_timer_ = node_->create_wall_timer(std::chrono::milliseconds(50),
                                        std::bind(&GridMap::updateOccupancyCallback, this));

  map_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("grid_map/occupancy", rclcpp::SensorDataQoS());
  map_inf_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("grid_map/occupancy_inflate", rclcpp::SensorDataQoS());
  sliding_map_bbox_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("grid_map/sliding_map_bbox", 10);

  unknown_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("grid_map/unknown", rclcpp::SensorDataQoS());
  depth_cloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("grid_map/depth_cloud", rclcpp::SensorDataQoS());
  extrinsic_pose_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("grid_map/sensor_pose_extrinsic", 10);

  if (visualization_rate_hz > 0.0)
  {
    const auto visualization_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / visualization_rate_hz));
    vis_timer_ = node_->create_wall_timer(visualization_period,
                                          std::bind(&GridMap::visCallback, this));
    visualization_worker_ = std::thread(&GridMap::visualizationWorkerLoop, this);
  }

  md_.occ_need_update_ = false;
  md_.use_cloud_update_ = false;
  md_.has_first_depth_ = false;
  md_.has_ray_pose_ = false;
  md_.has_cloud_ = false;
  md_.image_cnt_ = 0;
  md_.ray_pos_.setZero();
  md_.sliding_map_frame_pos_.setZero();
  md_.ray_q_ = Eigen::Quaterniond::Identity();

  md_.fuse_time_ = 0.0;
  md_.update_num_ = 0;
  md_.max_fuse_time_ = 0.0;
  md_.local_bound_min_ = mp_.map_bound_min_idx_;
  md_.local_bound_max_ = mp_.map_bound_max_idx_;

  initializeRuntimeDiagnostics();

  // rand_noise_ = uniform_real_distribution<double>(-0.2, 0.2);
  // rand_noise2_ = normal_distribution<double>(0, 0.2);
  // random_device rd;
  // eng_ = default_random_engine(rd());
}

void GridMap::updateMapBoundaryFromIndex()
{
  mp_.map_bound_min_idx_ = mp_.map_origin_idx_ - mp_.map_voxel_num_ / 2;
  mp_.map_bound_max_idx_ = mp_.map_bound_min_idx_ + mp_.map_voxel_num_ - Eigen::Vector3i::Ones();

  mp_.map_min_boundary_ = mp_.map_bound_min_idx_.cast<double>() * mp_.resolution_;
  mp_.map_max_boundary_ = (mp_.map_bound_max_idx_.cast<double>() + Eigen::Vector3d::Ones()) * mp_.resolution_;
  mp_.map_origin_ = mp_.map_min_boundary_;
}

void GridMap::rebuildInflationOffsets()
{
  const double double_radius = std::max(0.0, mp_.double_cylinder_radius_);
  const int inf_step_xy = ceil(double_radius / mp_.resolution_);
  const int inf_step_z_up = ceil(mp_.obstacles_inflation_z_up / mp_.resolution_);
  const int inf_step_z_down = ceil(mp_.obstacles_inflation_z_down / mp_.resolution_);

  md_.inflate_offsets_.clear();
  for (int x = -inf_step_xy; x <= inf_step_xy; ++x)
    for (int y = -inf_step_xy; y <= inf_step_xy; ++y)
    {
      Eigen::Vector2d offset_xy(x * mp_.resolution_, y * mp_.resolution_);
      if (offset_xy.norm() >= double_radius)
        continue;

      for (int z = -inf_step_z_down; z <= inf_step_z_up; ++z)
        md_.inflate_offsets_.push_back(Eigen::Vector3i(x, y, z));
    }
}

void GridMap::resetAllMapData()
{
  std::fill(md_.occupancy_buffer_.begin(), md_.occupancy_buffer_.end(), mp_.clamp_min_log_ - mp_.unknown_flag_);
  std::fill(md_.occupancy_buffer_inflate_.begin(), md_.occupancy_buffer_inflate_.end(), 0);
  std::fill(md_.occupancy_buffer_inflate_cnt_.begin(), md_.occupancy_buffer_inflate_cnt_.end(), 0);
  std::fill(md_.count_hit_and_miss_.begin(), md_.count_hit_and_miss_.end(), 0);
  std::fill(md_.count_hit_.begin(), md_.count_hit_.end(), 0);
  std::fill(md_.flag_rayend_.begin(), md_.flag_rayend_.end(), -1);
  std::fill(md_.flag_traverse_.begin(), md_.flag_traverse_.end(), -1);
  std::queue<Eigen::Vector3i> empty;
  std::swap(md_.cache_voxel_, empty);
}

void GridMap::hashIdToGlobalIndex(int addr, Eigen::Vector3i& id_g) const
{
  Eigen::Vector3i id_l;
  id_l(0) = addr / (mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2));
  id_l(1) = (addr - id_l(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2)) / mp_.map_voxel_num_(2);
  id_l(2) = addr - id_l(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) -
            id_l(1) * mp_.map_voxel_num_(2);

  for (int i = 0; i < 3; ++i)
  {
    const int min_l = getLocalIndex(mp_.map_bound_min_idx_(i), i);
    int dist = id_l(i) - min_l;
    if (dist < 0)
      dist += mp_.map_voxel_num_(i);
    id_g(i) = mp_.map_bound_min_idx_(i) + dist;
  }
}

void GridMap::updateInflationLayer(const Eigen::Vector3i& id, int delta,
                                   const vector<Eigen::Vector3i>& offsets,
                                   std::vector<int>& cnt_buffer,
                                   std::vector<char>& flag_buffer,
                                   const std::vector<char>* ignore_mask)
{
  for (const auto& offset : offsets)
  {
    const Eigen::Vector3i inf_id = id + offset;
    if (!isInMap(inf_id))
      continue;

    const int addr = toAddress(inf_id);
    if (ignore_mask && (*ignore_mask)[addr])
      continue;

    cnt_buffer[addr] += delta;
    if (cnt_buffer[addr] < 0)
      cnt_buffer[addr] = 0;
    flag_buffer[addr] = cnt_buffer[addr] > 0 ? 1 : 0;
  }
}

void GridMap::updateInflation(const Eigen::Vector3i& id, int delta, const std::vector<char>* ignore_mask)
{
  updateInflationLayer(id, delta, md_.inflate_offsets_, md_.occupancy_buffer_inflate_cnt_,
                       md_.occupancy_buffer_inflate_, ignore_mask);
}

void GridMap::applyOccupancyUpdate(const Eigen::Vector3i& id, double new_log_odds)
{
  if (!isInMap(id))
    return;

  const int addr = toAddress(id);
  const bool was_occ = md_.occupancy_buffer_[addr] > mp_.min_occupancy_log_;
  const bool now_occ = new_log_odds > mp_.min_occupancy_log_;

  md_.occupancy_buffer_[addr] = new_log_odds;
  if (was_occ != now_occ)
    updateInflation(id, now_occ ? 1 : -1);
}

void GridMap::resetCellByAddress(int addr)
{
  Eigen::Vector3i id_g;
  hashIdToGlobalIndex(addr, id_g);
  if (md_.occupancy_buffer_[addr] > mp_.min_occupancy_log_)
    updateInflation(id_g, -1);

  md_.occupancy_buffer_[addr] = mp_.clamp_min_log_ - mp_.unknown_flag_;
  md_.count_hit_[addr] = 0;
  md_.count_hit_and_miss_[addr] = 0;
  md_.flag_rayend_[addr] = -1;
  md_.flag_traverse_[addr] = -1;
}

void GridMap::resetCellByAddressForSliding(int addr, const std::vector<char>& clear_mask)
{
  Eigen::Vector3i id_g;
  hashIdToGlobalIndex(addr, id_g);
  if (md_.occupancy_buffer_[addr] > mp_.min_occupancy_log_)
    updateInflation(id_g, -1, &clear_mask);
}

void GridMap::updateSlidingMap(const Eigen::Vector3d& center)
{
  if (!mp_.map_sliding_en_)
    return;

  Eigen::Vector3i new_origin_idx;
  posToIndex(center, new_origin_idx);
  const Eigen::Vector3i shift_num = new_origin_idx - mp_.map_origin_idx_;
  if (shift_num.cwiseAbs().maxCoeff() < mp_.map_sliding_thresh_vox_)
    return;

  if ((shift_num.cwiseAbs().array() >= mp_.map_voxel_num_.array()).any())
  {
    resetAllMapData();
    mp_.map_origin_idx_ = new_origin_idx;
    updateMapBoundaryFromIndex();
    md_.local_bound_min_ = mp_.map_bound_min_idx_;
    md_.local_bound_max_ = mp_.map_bound_max_idx_;
    return;
  }

  const int buffer_size = mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2);
  std::vector<char> clear_mask(buffer_size, 0);
  std::vector<int> clear_addrs;
  clear_addrs.reserve(buffer_size / 8);

  auto add_clear_addr = [&](const Eigen::Vector3i& id_l) {
    const int addr = toAddressLocal(id_l);
    if (!clear_mask[addr])
    {
      clear_mask[addr] = 1;
      clear_addrs.push_back(addr);
    }
  };

  for (int dim = 0; dim < 3; ++dim)
  {
    const int shift = shift_num(dim);
    if (shift == 0)
      continue;

    if (shift > 0)
    {
      for (int k = 0; k < shift; ++k)
      {
        const int clear_g = mp_.map_bound_min_idx_(dim) + k;
        const int clear_l = getLocalIndex(clear_g, dim);

        for (int a = 0; a < mp_.map_voxel_num_((dim + 1) % 3); ++a)
          for (int b = 0; b < mp_.map_voxel_num_((dim + 2) % 3); ++b)
          {
            Eigen::Vector3i id_l;
            id_l(dim) = clear_l;
            id_l((dim + 1) % 3) = a;
            id_l((dim + 2) % 3) = b;
            add_clear_addr(id_l);
          }
      }
    }
    else
    {
      for (int k = 0; k < -shift; ++k)
      {
        const int clear_g = mp_.map_bound_max_idx_(dim) - k;
        const int clear_l = getLocalIndex(clear_g, dim);

        for (int a = 0; a < mp_.map_voxel_num_((dim + 1) % 3); ++a)
          for (int b = 0; b < mp_.map_voxel_num_((dim + 2) % 3); ++b)
          {
            Eigen::Vector3i id_l;
            id_l(dim) = clear_l;
            id_l((dim + 1) % 3) = a;
            id_l((dim + 2) % 3) = b;
            add_clear_addr(id_l);
          }
      }
    }
  }

  for (int addr : clear_addrs)
    resetCellByAddressForSliding(addr, clear_mask);

  for (int addr : clear_addrs)
  {
    md_.occupancy_buffer_[addr] = mp_.clamp_min_log_ - mp_.unknown_flag_;
    md_.occupancy_buffer_inflate_cnt_[addr] = 0;
    md_.occupancy_buffer_inflate_[addr] = 0;
    md_.count_hit_[addr] = 0;
    md_.count_hit_and_miss_[addr] = 0;
    md_.flag_rayend_[addr] = -1;
    md_.flag_traverse_[addr] = -1;
  }

  mp_.map_origin_idx_ = new_origin_idx;
  updateMapBoundaryFromIndex();
  boundIndex(md_.local_bound_min_);
  boundIndex(md_.local_bound_max_);
}

void GridMap::resetBuffer()
{
  resetAllMapData();
  md_.local_bound_min_ = mp_.map_bound_min_idx_;
  md_.local_bound_max_ = mp_.map_bound_max_idx_;
}

void GridMap::resetBuffer(Eigen::Vector3d min_pos, Eigen::Vector3d max_pos)
{
  Eigen::Vector3i min_id, max_id;
  posToIndex(min_pos, min_id);
  posToIndex(max_pos, max_id);

  boundIndex(min_id);
  boundIndex(max_id);

  for (int x = min_id(0); x <= max_id(0); ++x)
    for (int y = min_id(1); y <= max_id(1); ++y)
    {
      for (int z = min_id(2); z <= max_id(2); ++z)
      {
        resetCellByAddress(toAddress(x, y, z));
      }
    }
}

int GridMap::setCacheOccupancy(Eigen::Vector3d pos, int occ)
{
  if (occ != 1 && occ != 0)
    return INVALID_IDX;

  Eigen::Vector3i id;
  posToIndex(pos, id);
  if (!isInMap(id))
    return INVALID_IDX;

  int idx_ctns = toAddress(id);

  md_.count_hit_and_miss_[idx_ctns] += 1;

  if (md_.count_hit_and_miss_[idx_ctns] == 1)
  {
    md_.cache_voxel_.push(id);
  }

  if (occ == 1)
    md_.count_hit_[idx_ctns] += 1;

  return idx_ctns;
}

void GridMap::projectDepthImage()
{
  // md_.proj_points_.clear();
  md_.proj_points_cnt = 0;

  uint16_t *row_ptr;
  // int cols = current_img_.cols, rows = current_img_.rows;
  int cols = md_.depth_image_.cols;
  int rows = md_.depth_image_.rows;

  double depth;

  Eigen::Matrix3d sensor_r = md_.ray_q_.toRotationMatrix();

  // cout << "rotate: " << md_.ray_q_.toRotationMatrix() << endl;
  // std::cout << "pos in proj: " << md_.ray_pos_ << std::endl;

  if (!md_.has_first_depth_)
  {
    md_.has_first_depth_ = true;
    return;
  }

  Eigen::Vector3d pt_cur, pt_world;
  const double inv_factor = 1.0 / mp_.k_depth_scaling_factor_;

  for (int v = mp_.depth_filter_margin_; v < rows - mp_.depth_filter_margin_; v += mp_.skip_pixel_)
  {
    row_ptr = md_.depth_image_.ptr<uint16_t>(v) + mp_.depth_filter_margin_;

    for (int u = mp_.depth_filter_margin_; u < cols - mp_.depth_filter_margin_; u += mp_.skip_pixel_)
    {
      const uint16_t raw_depth = *row_ptr;
      depth = raw_depth * inv_factor;
      row_ptr = row_ptr + mp_.skip_pixel_;

      // filter depth
      // depth += rand_noise_(eng_);
      // if (depth > 0.01) depth += rand_noise2_(eng_);

      if (raw_depth == 0)
      {
        depth = mp_.max_ray_length_ + 0.1;
      }
      else if (depth < mp_.depth_filter_mindist_)
      {
        continue;
      }
      else if (depth > mp_.depth_filter_maxdist_)
      {
        depth = mp_.max_ray_length_ + 0.1;
      }

      // project to world frame
      pt_cur(0) = (u - mp_.cx_) * depth / mp_.fx_;
      pt_cur(1) = (v - mp_.cy_) * depth / mp_.fy_;
      pt_cur(2) = depth;

      pt_world = sensor_r * pt_cur + md_.ray_pos_;
      // if (!isInMap(pt_world)) {
      //   pt_world = closetPointInMap(pt_world, md_.ray_pos_);
      // }

      md_.proj_points_[md_.proj_points_cnt++] = pt_world;
    }
  }
}

void GridMap::raycastProcess()
{
  // if (md_.proj_points_.size() == 0)
  if (md_.proj_points_cnt == 0)
    return;

  updateSlidingMap(md_.ray_pos_);

  md_.raycast_num_ += 1;

  int vox_idx;
  double length;

  // bounding box of updated region
  double min_x = mp_.map_max_boundary_(0);
  double min_y = mp_.map_max_boundary_(1);
  double min_z = mp_.map_max_boundary_(2);

  double max_x = mp_.map_min_boundary_(0);
  double max_y = mp_.map_min_boundary_(1);
  double max_z = mp_.map_min_boundary_(2);

  RayCaster raycaster;
  Eigen::Vector3d half = Eigen::Vector3d(0.5, 0.5, 0.5);
  Eigen::Vector3d ray_pt, pt_w;

  for (int i = 0; i < md_.proj_points_cnt; ++i)
  {
    pt_w = md_.proj_points_[i];

    // set flag for projected point

    if (!isInMap(pt_w))
    {
      pt_w = closetPointInMap(pt_w, md_.ray_pos_);

      length = (pt_w - md_.ray_pos_).norm();
      if (length > mp_.max_ray_length_)
      {
        pt_w = (pt_w - md_.ray_pos_) / length * mp_.max_ray_length_ + md_.ray_pos_;
      }
      vox_idx = setCacheOccupancy(pt_w, 0);
    }
    else
    {
      length = (pt_w - md_.ray_pos_).norm();

      if (length > mp_.max_ray_length_)
      {
        pt_w = (pt_w - md_.ray_pos_) / length * mp_.max_ray_length_ + md_.ray_pos_;
        vox_idx = setCacheOccupancy(pt_w, 0);
      }
      else
      {
        vox_idx = setCacheOccupancy(pt_w, 1);
      }
    }

    max_x = max(max_x, pt_w(0));
    max_y = max(max_y, pt_w(1));
    max_z = max(max_z, pt_w(2));

    min_x = min(min_x, pt_w(0));
    min_y = min(min_y, pt_w(1));
    min_z = min(min_z, pt_w(2));

    // raycasting between ray origin and point

    if (vox_idx != INVALID_IDX)
    {
      if (md_.flag_rayend_[vox_idx] == md_.raycast_num_)
      {
        continue;
      }
      else
      {
        md_.flag_rayend_[vox_idx] = md_.raycast_num_;
      }
    }

    raycaster.setInput(pt_w / mp_.resolution_, md_.ray_pos_ / mp_.resolution_);

    while (raycaster.step(ray_pt))
    {
      Eigen::Vector3d tmp = (ray_pt + half) * mp_.resolution_;
      length = (tmp - md_.ray_pos_).norm();

      vox_idx = setCacheOccupancy(tmp, 0);

      if (vox_idx != INVALID_IDX)
      {
        if (md_.flag_traverse_[vox_idx] == md_.raycast_num_)
        {
          break;
        }
        else
        {
          md_.flag_traverse_[vox_idx] = md_.raycast_num_;
        }
      }
    }
  }

  min_x = min(min_x, md_.ray_pos_(0));
  min_y = min(min_y, md_.ray_pos_(1));
  min_z = min(min_z, md_.ray_pos_(2));

  max_x = max(max_x, md_.ray_pos_(0));
  max_y = max(max_y, md_.ray_pos_(1));
  max_z = max(max_z, md_.ray_pos_(2));
  max_z = max(max_z, mp_.ground_height_);

  posToIndex(Eigen::Vector3d(max_x, max_y, max_z), md_.local_bound_max_);
  posToIndex(Eigen::Vector3d(min_x, min_y, min_z), md_.local_bound_min_);
  boundIndex(md_.local_bound_min_);
  boundIndex(md_.local_bound_max_);

  // update occupancy cached in queue
  Eigen::Vector3d local_range_min = md_.ray_pos_ - mp_.local_update_range_;
  Eigen::Vector3d local_range_max = md_.ray_pos_ + mp_.local_update_range_;

  Eigen::Vector3i min_id, max_id;
  posToIndex(local_range_min, min_id);
  posToIndex(local_range_max, max_id);
  boundIndex(min_id);
  boundIndex(max_id);

  // std::cout << "cache all: " << md_.cache_voxel_.size() << std::endl;

  while (!md_.cache_voxel_.empty())
  {

    Eigen::Vector3i idx = md_.cache_voxel_.front();
    int idx_ctns = toAddress(idx);
    md_.cache_voxel_.pop();

    double log_odds_update =
        md_.count_hit_[idx_ctns] >= md_.count_hit_and_miss_[idx_ctns] - md_.count_hit_[idx_ctns] ? mp_.prob_hit_log_ : mp_.prob_miss_log_;

    md_.count_hit_[idx_ctns] = md_.count_hit_and_miss_[idx_ctns] = 0;

    if (log_odds_update >= 0 && md_.occupancy_buffer_[idx_ctns] >= mp_.clamp_max_log_)
    {
      continue;
    }
    else if (log_odds_update <= 0 && md_.occupancy_buffer_[idx_ctns] <= mp_.clamp_min_log_)
    {
      applyOccupancyUpdate(idx, mp_.clamp_min_log_);
      continue;
    }

    bool in_local = idx(0) >= min_id(0) && idx(0) <= max_id(0) && idx(1) >= min_id(1) &&
                    idx(1) <= max_id(1) && idx(2) >= min_id(2) && idx(2) <= max_id(2);
    if (!in_local)
    {
      applyOccupancyUpdate(idx, mp_.clamp_min_log_);
    }

    const double new_log_odds =
        std::min(std::max(md_.occupancy_buffer_[idx_ctns] + log_odds_update, mp_.clamp_min_log_),
                 mp_.clamp_max_log_);
    applyOccupancyUpdate(idx, new_log_odds);
  }
}

Eigen::Vector3d GridMap::closetPointInMap(const Eigen::Vector3d &pt, const Eigen::Vector3d &ray_pos)
{
  Eigen::Vector3d diff = pt - ray_pos;
  Eigen::Vector3d max_tc = mp_.map_max_boundary_ - ray_pos;
  Eigen::Vector3d min_tc = mp_.map_min_boundary_ - ray_pos;

  double min_t = 1000000;

  for (int i = 0; i < 3; ++i)
  {
    if (fabs(diff[i]) > 0)
    {

      double t1 = max_tc[i] / diff[i];
      if (t1 > 0 && t1 < min_t)
        min_t = t1;

      double t2 = min_tc[i] / diff[i];
      if (t2 > 0 && t2 < min_t)
        min_t = t2;
    }
  }

  return ray_pos + (min_t - 1e-3) * diff;
}

void GridMap::visCallback()
{
  queueVisualizationSnapshot();
  publishSlidingMapFrame();
  publishSlidingMapBBox();
  publishDepthCloud();
}

void GridMap::queueVisualizationSnapshot()
{
  const bool publish_occupancy = map_pub_->get_subscription_count() > 0;
  const bool publish_inflated = map_inf_pub_->get_subscription_count() > 0;
  if (!publish_occupancy && !publish_inflated)
    return;

  const auto work_start = std::chrono::steady_clock::now();
  visualization_requested_.fetch_add(1, std::memory_order_relaxed);

  VisualizationSnapshot snapshot;
  if (publish_occupancy)
    snapshot.occupancy = md_.occupancy_buffer_;
  if (publish_inflated)
    snapshot.inflated = md_.occupancy_buffer_inflate_;
  snapshot.map_voxel_num = mp_.map_voxel_num_;
  snapshot.min_index = mp_.map_bound_min_idx_;
  snapshot.max_index = mp_.map_bound_max_idx_;
  snapshot.ray_position = md_.ray_pos_;
  snapshot.stamp = node_->now();
  snapshot.frame_id = mp_.frame_id_;
  snapshot.resolution = mp_.resolution_;
  snapshot.min_occupancy_log = mp_.min_occupancy_log_;
  snapshot.visualization_height = mp_.vis_height_;
  snapshot.has_ray_pose = md_.has_ray_pose_;
  snapshot.publish_occupancy = publish_occupancy;
  snapshot.publish_inflated = publish_inflated;

  {
    std::lock_guard<std::mutex> lock(visualization_mutex_);
    // Keep only the newest frame when serialization is slower than the requested
    // rate. Visualization must never build an unbounded backlog behind planning.
    if (pending_visualization_)
      visualization_dropped_.fetch_add(1, std::memory_order_relaxed);
    pending_visualization_ = std::move(snapshot);
  }
  const uint64_t work_us = static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now() - work_start).count());
  visualization_work_us_.fetch_add(work_us, std::memory_order_relaxed);
  updateAtomicMaximum(visualization_work_max_us_, work_us);
  visualization_cv_.notify_one();
}

void GridMap::visualizationWorkerLoop()
{
  while (true)
  {
    VisualizationSnapshot snapshot;
    {
      std::unique_lock<std::mutex> lock(visualization_mutex_);
      visualization_cv_.wait(lock, [this]() {
        return stop_visualization_worker_ || pending_visualization_.has_value();
      });
      if (stop_visualization_worker_)
        return;
      snapshot = std::move(*pending_visualization_);
      pending_visualization_.reset();
    }

    const auto work_start = std::chrono::steady_clock::now();

    pcl::PointCloud<pcl::PointXYZ> occupancy_cloud;
    pcl::PointCloud<pcl::PointXYZ> inflated_cloud;
    const int size_y = snapshot.map_voxel_num.y();
    const int size_z = snapshot.map_voxel_num.z();
    auto local_index = [](int value, int size) {
      int result = value % size;
      return result < 0 ? result + size : result;
    };
    auto address = [&](int x, int y, int z) {
      return local_index(x, snapshot.map_voxel_num.x()) * size_y * size_z +
             local_index(y, size_y) * size_z + local_index(z, size_z);
    };

    for (int x = snapshot.min_index.x(); x <= snapshot.max_index.x(); ++x)
      for (int y = snapshot.min_index.y(); y <= snapshot.max_index.y(); ++y)
        for (int z = snapshot.min_index.z(); z <= snapshot.max_index.z(); ++z)
        {
          const int cell = address(x, y, z);
          const double point_z = (z + 0.5) * snapshot.resolution;
          if (snapshot.has_ray_pose &&
              point_z > snapshot.ray_position.z() + snapshot.visualization_height)
            continue;

          pcl::PointXYZ point;
          point.x = static_cast<float>((x + 0.5) * snapshot.resolution);
          point.y = static_cast<float>((y + 0.5) * snapshot.resolution);
          point.z = static_cast<float>(point_z);
          if (snapshot.publish_occupancy &&
              snapshot.occupancy[cell] >= snapshot.min_occupancy_log)
            occupancy_cloud.push_back(point);
          if (snapshot.publish_inflated && snapshot.inflated[cell] != 0)
            inflated_cloud.push_back(point);
        }

    auto publish_cloud = [&](pcl::PointCloud<pcl::PointXYZ> &cloud,
                             const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &publisher) {
      cloud.width = cloud.points.size();
      cloud.height = 1;
      cloud.is_dense = true;
      cloud.header.frame_id = snapshot.frame_id;
      sensor_msgs::msg::PointCloud2 cloud_msg;
      pcl::toROSMsg(cloud, cloud_msg);
      cloud_msg.header.stamp = snapshot.stamp;
      publisher->publish(cloud_msg);
    };
    if (snapshot.publish_occupancy)
      publish_cloud(occupancy_cloud, map_pub_);
    if (snapshot.publish_inflated)
      publish_cloud(inflated_cloud, map_inf_pub_);

    const uint64_t point_count = occupancy_cloud.size() + inflated_cloud.size();
    const uint64_t work_us = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - work_start).count());
    visualization_published_.fetch_add(1, std::memory_order_relaxed);
    visualization_points_.fetch_add(point_count, std::memory_order_relaxed);
    visualization_work_us_.fetch_add(work_us, std::memory_order_relaxed);
    updateAtomicMaximum(visualization_work_max_us_, work_us);
  }
}

void GridMap::updateOccupancyCallback()
{
  if (!md_.occ_need_update_)
    return;

  const auto fusion_start = std::chrono::steady_clock::now();
  if (pending_map_ready_time_valid_)
  {
    const double queue_wait_ms = std::chrono::duration<double, std::milli>(
        fusion_start - pending_map_ready_time_).count();
    runtime_metrics_.queue_wait_ms_sum += queue_wait_ms;
    runtime_metrics_.queue_wait_ms_max = std::max(runtime_metrics_.queue_wait_ms_max, queue_wait_ms);
  }

  /* update occupancy */
  // ros::Time t1, t2, t3, t4;
  // t1 = ros::Time::now();

  const auto projection_start = std::chrono::steady_clock::now();
  const bool depth_update = !md_.use_cloud_update_;
  if (depth_update)
    projectDepthImage();
  if (depth_update)
    runtime_metrics_.prepared_points += static_cast<uint64_t>(std::max(0, md_.proj_points_cnt));
  const double projection_ms = elapsedMilliseconds(projection_start);
  runtime_metrics_.projection_ms_sum += projection_ms;
  runtime_metrics_.projection_ms_max = std::max(runtime_metrics_.projection_ms_max, projection_ms);
  // t2 = ros::Time::now();
  const auto raycast_start = std::chrono::steady_clock::now();
  raycastProcess();
  const double raycast_ms = elapsedMilliseconds(raycast_start);
  runtime_metrics_.raycast_ms_sum += raycast_ms;
  runtime_metrics_.raycast_ms_max = std::max(runtime_metrics_.raycast_ms_max, raycast_ms);
  // t3 = ros::Time::now();

  // t4 = ros::Time::now();

  // cout << setprecision(7);
  // cout << "t2=" << (t2-t1).toSec() << " t3=" << (t3-t2).toSec() << " t4=" << (t4-t3).toSec() << endl;;

  // md_.fuse_time_ += (t2 - t1).toSec();
  // md_.max_fuse_time_ = max(md_.max_fuse_time_, (t2 - t1).toSec());

  // if (mp_.show_occ_time_)
  //   ROS_WARN("Fusion: cur t = %lf, avg t = %lf, max t = %lf", (t2 - t1).toSec(),
  //            md_.fuse_time_ / md_.update_num_, md_.max_fuse_time_);

  md_.occ_need_update_ = false;
  md_.use_cloud_update_ = false;
  pending_map_ready_time_valid_ = false;

  const double fusion_ms = elapsedMilliseconds(fusion_start);
  runtime_metrics_.fusion_frames++;
  runtime_metrics_.fusion_ms_sum += fusion_ms;
  runtime_metrics_.fusion_ms_max = std::max(runtime_metrics_.fusion_ms_max, fusion_ms);
}

void GridMap::depthPoseCallback(const sensor_msgs::msg::Image::ConstSharedPtr &img,
                                const nav_msgs::msg::Odometry::ConstSharedPtr &pose)
{
  if (mp_.sensor_type_ != "depth")
    return;

  runtime_metrics_.input_frames++;
  runtime_metrics_.unique_input_frames++;

  /* get depth image */
  const auto decode_start = std::chrono::steady_clock::now();
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(img, img->encoding);

  if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
  {
    (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, mp_.k_depth_scaling_factor_);
  }
  cv_ptr->image.copyTo(md_.depth_image_);
  const double decode_ms = elapsedMilliseconds(decode_start);
  runtime_metrics_.decode_ms_sum += decode_ms;
  runtime_metrics_.decode_ms_max = std::max(runtime_metrics_.decode_ms_max, decode_ms);
  runtime_metrics_.input_points += static_cast<uint64_t>(
      std::max(0, md_.depth_image_.rows) * std::max(0, md_.depth_image_.cols));

  // std::cout << "depth: " << md_.depth_image_.cols << ", " << md_.depth_image_.rows << std::endl;

  /* get pose */
  const geometry_msgs::msg::Pose &sensor_pose = pose->pose.pose;
  Eigen::Quaterniond ray_q(sensor_pose.orientation.w, sensor_pose.orientation.x,
                           sensor_pose.orientation.y, sensor_pose.orientation.z);
  if (ray_q.norm() < 1e-6)
  {
    runtime_metrics_.no_pose_frames++;
    return;
  }
  ray_q.normalize();

  Eigen::Vector3d ray_pos(sensor_pose.position.x, sensor_pose.position.y, sensor_pose.position.z);
  if (mp_.need_extrinsic_)
  {
    const Eigen::Matrix3d pose_r = ray_q.toRotationMatrix();
    ray_pos += pose_r * mp_.depth_extrinsic_.block<3, 1>(0, 3);
    ray_q = Eigen::Quaterniond(pose_r * mp_.depth_extrinsic_.block<3, 3>(0, 0));
    ray_q.normalize();
  }

  nav_msgs::msg::Odometry extrinsic_pose = *pose;
  extrinsic_pose.pose.pose.position.x = ray_pos.x();
  extrinsic_pose.pose.pose.position.y = ray_pos.y();
  extrinsic_pose.pose.pose.position.z = ray_pos.z();
  extrinsic_pose.pose.pose.orientation.x = ray_q.x();
  extrinsic_pose.pose.pose.orientation.y = ray_q.y();
  extrinsic_pose.pose.pose.orientation.z = ray_q.z();
  extrinsic_pose.pose.pose.orientation.w = ray_q.w();
  extrinsic_pose.child_frame_id =
      pose->child_frame_id.empty() ? "sensor_extrinsic" : pose->child_frame_id + "_extrinsic";
  extrinsic_pose_pub_->publish(extrinsic_pose);

  md_.ray_pos_ = ray_pos;
  md_.ray_q_ = ray_q;
  md_.use_cloud_update_ = false;
  const auto sliding_start = std::chrono::steady_clock::now();
  updateSlidingMap(md_.ray_pos_);
  const double sliding_ms = elapsedMilliseconds(sliding_start);
  runtime_metrics_.sliding_ms_sum += sliding_ms;
  runtime_metrics_.sliding_ms_max = std::max(runtime_metrics_.sliding_ms_max, sliding_ms);
  if (isInMap(md_.ray_pos_))
  {
    md_.has_ray_pose_ = true;
    md_.update_num_ += 1;
    if (md_.occ_need_update_)
      runtime_metrics_.coalesced_frames++;
    md_.occ_need_update_ = true;
    runtime_metrics_.prepared_frames++;
    pending_map_ready_time_ = std::chrono::steady_clock::now();
    pending_map_ready_time_valid_ = true;
  }
  else
  {
    md_.occ_need_update_ = false;
  }
}

void GridMap::sensorPoseCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &pose_msg)
{
  if (mp_.sensor_type_ != "lidar")
    return;

  const geometry_msgs::msg::Pose &sensor_pose = pose_msg->pose.pose;
  Eigen::Quaterniond ray_q(sensor_pose.orientation.w, sensor_pose.orientation.x,
                           sensor_pose.orientation.y, sensor_pose.orientation.z);
  if (ray_q.norm() < 1e-6)
    return;
  ray_q.normalize();

  Eigen::Vector3d ray_pos(sensor_pose.position.x, sensor_pose.position.y, sensor_pose.position.z);
  if (mp_.need_extrinsic_)
  {
    const Eigen::Matrix3d pose_r = ray_q.toRotationMatrix();
    ray_pos += pose_r * mp_.lidar_extrinsic_.block<3, 1>(0, 3);
    ray_q = Eigen::Quaterniond(pose_r * mp_.lidar_extrinsic_.block<3, 3>(0, 0));
    ray_q.normalize();
  }
  if (!std::isfinite(ray_pos.x()) || !std::isfinite(ray_pos.y()) || !std::isfinite(ray_pos.z()))
    return;

  md_.ray_pos_ = ray_pos;
  md_.ray_q_ = ray_q;
  md_.has_ray_pose_ = true;
  updateSlidingMap(md_.ray_pos_);
}

void GridMap::slidingMapFrameCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &pose)
{
  const geometry_msgs::msg::Point &pos = pose->pose.pose.position;
  md_.sliding_map_frame_pos_ = Eigen::Vector3d(pos.x, pos.y, pos.z);
}

void GridMap::cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &img)
{
  if (mp_.sensor_type_ != "lidar")
    return;

  runtime_metrics_.input_frames++;

  if (!md_.has_ray_pose_)
  {
    runtime_metrics_.no_pose_frames++;
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                         "[GridMap] no sensor_pose received for lidar cloud update");
    return;
  }

  const rclcpp::Time cloud_stamp(img->header.stamp);
  if (cloud_stamp.nanoseconds() != 0 && cloud_stamp == last_cloud_stamp_)
  {
    runtime_metrics_.duplicate_input_frames++;
    return;
  }
  if (cloud_stamp.nanoseconds() != 0)
    last_cloud_stamp_ = cloud_stamp;

  runtime_metrics_.unique_input_frames++;

  const auto decode_start = std::chrono::steady_clock::now();
  pcl::PointCloud<pcl::PointXYZ> latest_cloud;
  pcl::fromROSMsg(*img, latest_cloud);
  const double decode_ms = elapsedMilliseconds(decode_start);
  runtime_metrics_.decode_ms_sum += decode_ms;
  runtime_metrics_.decode_ms_max = std::max(runtime_metrics_.decode_ms_max, decode_ms);
  runtime_metrics_.input_points += latest_cloud.points.size();

  md_.has_cloud_ = true;

  if (latest_cloud.points.size() == 0)
  {
    runtime_metrics_.empty_input_frames++;
    return;
  }

  const Eigen::Matrix3d sensor_r = md_.ray_q_.toRotationMatrix();
  const Eigen::Vector3d ray_pos = md_.ray_pos_;
  if (!std::isfinite(ray_pos.x()) || !std::isfinite(ray_pos.y()) || !std::isfinite(ray_pos.z()))
    return;

  const auto sliding_start = std::chrono::steady_clock::now();
  updateSlidingMap(ray_pos);
  const double sliding_ms = elapsedMilliseconds(sliding_start);
  runtime_metrics_.sliding_ms_sum += sliding_ms;
  runtime_metrics_.sliding_ms_max = std::max(runtime_metrics_.sliding_ms_max, sliding_ms);

  md_.proj_points_cnt = 0;

  const auto filter_start = std::chrono::steady_clock::now();
  for (size_t i = 0; i < latest_cloud.points.size(); ++i)
  {
    const pcl::PointXYZ &pt = latest_cloud.points[i];
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z))
      continue;

    Eigen::Vector3d pt_world;
    if (mp_.cloud_is_world_)
    {
      pt_world = Eigen::Vector3d(pt.x, pt.y, pt.z);
    }
    else
    {
      const Eigen::Vector3d pt_sensor(pt.x, pt.y, pt.z);
      pt_world = sensor_r * pt_sensor + ray_pos;
    }
    const Eigen::Vector3d devi = pt_world - ray_pos;
    const double ray_length = devi.norm();
    const bool in_local_range =
        fabs(devi(0)) <= mp_.local_update_range_(0) && fabs(devi(1)) <= mp_.local_update_range_(1) &&
        fabs(devi(2)) <= mp_.local_update_range_(2);
    if (!in_local_range && ray_length <= mp_.max_ray_length_)
      continue;

    if (md_.proj_points_cnt >= static_cast<int>(md_.proj_points_.size()))
      md_.proj_points_.push_back(pt_world);
    else
      md_.proj_points_[md_.proj_points_cnt] = pt_world;

    md_.proj_points_cnt++;
  }
  const double filter_ms = elapsedMilliseconds(filter_start);
  runtime_metrics_.filter_ms_sum += filter_ms;
  runtime_metrics_.filter_ms_max = std::max(runtime_metrics_.filter_ms_max, filter_ms);

  if (md_.proj_points_cnt == 0)
  {
    runtime_metrics_.empty_input_frames++;
    return;
  }

  if (md_.occ_need_update_)
    runtime_metrics_.coalesced_frames++;
  md_.use_cloud_update_ = true;
  md_.occ_need_update_ = true;
  runtime_metrics_.prepared_frames++;
  runtime_metrics_.prepared_points += static_cast<uint64_t>(md_.proj_points_cnt);
  pending_map_ready_time_ = std::chrono::steady_clock::now();
  pending_map_ready_time_valid_ = true;
}

void GridMap::publishMap()
{

  if (map_pub_->get_subscription_count() == 0)
    return;

  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  Eigen::Vector3i min_cut = mp_.map_bound_min_idx_;
  Eigen::Vector3i max_cut = mp_.map_bound_max_idx_;

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z)
      {
        if (md_.occupancy_buffer_[toAddress(x, y, z)] < mp_.min_occupancy_log_)
          continue;

        Eigen::Vector3d pos;
        indexToPos(Eigen::Vector3i(x, y, z), pos);
        if (md_.has_ray_pose_ && pos(2) > md_.ray_pos_(2) + mp_.vis_height_)
          continue;
        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        cloud.push_back(pt);
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;
  sensor_msgs::msg::PointCloud2 cloud_msg;

  pcl::toROSMsg(cloud, cloud_msg);
  cloud_msg.header.stamp = node_->now();
  map_pub_->publish(cloud_msg);
}

void GridMap::publishMapInflate(bool all_info)
{

  if (map_inf_pub_->get_subscription_count() == 0)
    return;

  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  Eigen::Vector3i min_cut = mp_.map_bound_min_idx_;
  Eigen::Vector3i max_cut = mp_.map_bound_max_idx_;

  const std::vector<char> &inflate_buffer = md_.occupancy_buffer_inflate_;
  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z)
      {
        if (inflate_buffer[toAddress(x, y, z)] == 0)
          continue;

        Eigen::Vector3d pos;
        indexToPos(Eigen::Vector3i(x, y, z), pos);
        if (md_.has_ray_pose_ && pos(2) > md_.ray_pos_(2) + mp_.vis_height_)
          continue;

        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        cloud.push_back(pt);
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;
  sensor_msgs::msg::PointCloud2 cloud_msg;

  pcl::toROSMsg(cloud, cloud_msg);
  cloud_msg.header.stamp = node_->now();
  map_inf_pub_->publish(cloud_msg);

  // ROS_INFO("pub map");
}

void GridMap::publishSlidingMapFrame()
{
  if (mp_.sliding_map_frame_id_.empty())
    return;

  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = node_->now();
  transform.header.frame_id = mp_.frame_id_;
  transform.child_frame_id = mp_.sliding_map_frame_id_;
  transform.transform.translation.x = md_.sliding_map_frame_pos_.x();
  transform.transform.translation.y = md_.sliding_map_frame_pos_.y();
  transform.transform.translation.z = md_.sliding_map_frame_pos_.z();
  transform.transform.rotation.w = 1.0;
  tf_broadcaster_->sendTransform(transform);
}

void GridMap::publishSlidingMapBBox()
{
  if (sliding_map_bbox_pub_->get_subscription_count() == 0)
    return;

  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = mp_.frame_id_;
  marker.header.stamp = node_->now();
  marker.ns = "sliding_map";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.04;
  marker.color.r = 0.0;
  marker.color.g = 0.8;
  marker.color.b = 1.0;
  marker.color.a = 1.0;

  const Eigen::Vector3d& min_pt = mp_.map_min_boundary_;
  const Eigen::Vector3d& max_pt = mp_.map_max_boundary_;
  Eigen::Vector3d corners[8] = {
      {min_pt.x(), min_pt.y(), min_pt.z()},
      {max_pt.x(), min_pt.y(), min_pt.z()},
      {max_pt.x(), max_pt.y(), min_pt.z()},
      {min_pt.x(), max_pt.y(), min_pt.z()},
      {min_pt.x(), min_pt.y(), max_pt.z()},
      {max_pt.x(), min_pt.y(), max_pt.z()},
      {max_pt.x(), max_pt.y(), max_pt.z()},
      {min_pt.x(), max_pt.y(), max_pt.z()},
  };

  auto pushPoint = [&](const Eigen::Vector3d& p) {
    geometry_msgs::msg::Point point;
    point.x = p.x();
    point.y = p.y();
    point.z = p.z();
    marker.points.push_back(point);
  };

  const int edges[12][2] = {
      {0, 1}, {1, 2}, {2, 3}, {3, 0},
      {4, 5}, {5, 6}, {6, 7}, {7, 4},
      {0, 4}, {1, 5}, {2, 6}, {3, 7},
  };

  for (const auto& edge : edges)
  {
    pushPoint(corners[edge[0]]);
    pushPoint(corners[edge[1]]);
  }

  sliding_map_bbox_pub_->publish(marker);
}

void GridMap::publishUnknown()
{
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  Eigen::Vector3i min_cut = md_.local_bound_min_;
  Eigen::Vector3i max_cut = md_.local_bound_max_;

  boundIndex(max_cut);
  boundIndex(min_cut);

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z)
      {

        if (md_.occupancy_buffer_[toAddress(x, y, z)] < mp_.clamp_min_log_ - 1e-3)
        {
          Eigen::Vector3d pos;
          indexToPos(Eigen::Vector3i(x, y, z), pos);
          if (md_.has_ray_pose_ && pos(2) > md_.ray_pos_(2) + mp_.vis_height_)
            continue;

          pt.x = pos(0);
          pt.y = pos(1);
          pt.z = pos(2);
          cloud.push_back(pt);
        }
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;

  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  cloud_msg.header.stamp = node_->now();
  unknown_pub_->publish(cloud_msg);
}

bool GridMap::odomValid() { return md_.has_ray_pose_; }

bool GridMap::hasDepthObservation() { return md_.has_first_depth_; }

Eigen::Vector3d GridMap::getOrigin() { return mp_.map_origin_; }

// int GridMap::getVoxelNum() {
//   return mp_.map_voxel_num_[0] * mp_.map_voxel_num_[1] * mp_.map_voxel_num_[2];
// }

void GridMap::getRegion(Eigen::Vector3d &ori, Eigen::Vector3d &size)
{
  ori = mp_.map_origin_, size = mp_.map_size_;
}

// GridMap

void GridMap::publishDepthCloud()
{
  if (depth_cloud_pub_->get_subscription_count() == 0)
    return;

  if (md_.proj_points_cnt == 0)
    return;

  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::PointXYZ pt;

  for (int i = 0; i < md_.proj_points_cnt; ++i)
  {
    pt.x = md_.proj_points_[i](0);
    pt.y = md_.proj_points_[i](1);
    pt.z = md_.proj_points_[i](2);
    cloud.push_back(pt);
  }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;

  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  cloud_msg.header.stamp = node_->now();
  depth_cloud_pub_->publish(cloud_msg);
}
