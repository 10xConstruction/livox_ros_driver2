//
// The MIT License (MIT)
//
// Copyright (c) 2022 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include <iostream>
#include <chrono>
#include <vector>
#include <csignal>
#include <thread>
#include <mutex>
#include <atomic>
#include <condition_variable>

#include "include/livox_ros_driver2.h"
#include "include/ros_headers.h"
#include "driver_node.h"
#include "lddc.h"
#include "lds_lidar.h"
#include "comm/pub_handler.h"

using namespace livox_ros;

#ifdef BUILDING_ROS1
int main(int argc, char **argv) {
  /** Ros related */
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }

  ros::init(argc, argv, "livox_lidar_publisher");

  // ros::NodeHandle livox_node;
  livox_ros::DriverNode livox_node;

  DRIVER_INFO(livox_node, "Livox Ros Driver2 Version: %s", LIVOX_ROS_DRIVER2_VERSION_STRING);

  /** Init default system parameter */
  int xfer_format = kPointCloud2Msg;
  int multi_topic = 0;
  int data_src = kSourceRawLidar;
  double publish_freq  = 10.0; /* Hz */
  int output_type      = kOutputToRos;
  bool lidar_bag = true;
  bool imu_bag   = false;

  livox_node.GetNode().getParam("xfer_format", xfer_format);
  livox_node.GetNode().getParam("multi_topic", multi_topic);
  livox_node.GetNode().getParam("data_src", data_src);
  livox_node.GetNode().getParam("publish_freq", publish_freq);
  livox_node.GetNode().getParam("output_data_type", output_type);
  livox_node.GetNode().getParam("enable_lidar_bag", lidar_bag);
  livox_node.GetNode().getParam("enable_imu_bag", imu_bag);

  printf("data source:%u.\n", data_src);

  if (publish_freq > 100.0) {
    publish_freq = 100.0;
  } else if (publish_freq < 0.5) {
    publish_freq = 0.5;
  } else {
    publish_freq = publish_freq;
  }

  livox_node.future_ = livox_node.exit_signal_.get_future();

  /** Lidar data distribute control and lidar data source set */
  livox_node.lddc_ptr_ = std::make_unique<Lddc>(xfer_format, multi_topic, data_src, output_type,
                        publish_freq, lidar_bag, imu_bag);
  livox_node.lddc_ptr_->SetRosNode(&livox_node);

  if (data_src == kSourceRawLidar) {
    DRIVER_INFO(livox_node, "Data Source is raw lidar.");

    std::string user_config_path;
    livox_node.getParam("user_config_path", user_config_path);
    DRIVER_INFO(livox_node, "Config file : %s", user_config_path.c_str());

    LdsLidar *read_lidar = LdsLidar::GetInstance(publish_freq);
    livox_node.lddc_ptr_->RegisterLds(static_cast<Lds *>(read_lidar));

    if ((read_lidar->InitLdsLidar(user_config_path))) {
      DRIVER_INFO(livox_node, "Init lds lidar successfully!");
    } else {
      DRIVER_ERROR(livox_node, "Init lds lidar failed!");
    }
  } else {
    DRIVER_ERROR(livox_node, "Invalid data src (%d), please check the launch file", data_src);
  }

  livox_node.pointclouddata_poll_thread_ = std::make_shared<std::thread>(&DriverNode::PointCloudDataPollThread, &livox_node);
  livox_node.imudata_poll_thread_ = std::make_shared<std::thread>(&DriverNode::ImuDataPollThread, &livox_node);
  while (ros::ok()) { usleep(10000); }

  return 0;
}

#elif defined BUILDING_ROS2
namespace livox_ros
{
DriverNode::DriverNode(const rclcpp::NodeOptions & node_options)
: Node("livox_driver_node", node_options),
  restart_requested_(false),
  polling_paused_(false)
{
  DRIVER_INFO(*this, "Livox Ros Driver2 Version: %s", LIVOX_ROS_DRIVER2_VERSION_STRING);

  /** Init default system parameter */
  int xfer_format = kPointCloud2Msg;
  int multi_topic = 0;
  int data_src = kSourceRawLidar;
  double publish_freq = 10.0; /* Hz */
  int output_type = kOutputToRos;

  this->declare_parameter("xfer_format", xfer_format);
  this->declare_parameter("multi_topic", 0);
  this->declare_parameter("data_src", data_src);
  this->declare_parameter("publish_freq", 10.0);
  this->declare_parameter("output_data_type", output_type);
  this->declare_parameter("user_config_path", "path_default");
  this->declare_parameter("cmdline_input_bd_code", "000000000000001");
  this->declare_parameter("lvx_file_path", "/home/livox/livox_test.lvx");

  this->get_parameter("xfer_format", xfer_format);
  this->get_parameter("multi_topic", multi_topic);
  this->get_parameter("data_src", data_src);
  this->get_parameter("publish_freq", publish_freq);
  this->get_parameter("output_data_type", output_type);

  if (publish_freq > 100.0) {
    publish_freq = 100.0;
  } else if (publish_freq < 0.5) {
    publish_freq = 0.5;
  } else {
    publish_freq = publish_freq;
  }

  // Store publish_freq for restart
  publish_freq_ = publish_freq;

  future_ = exit_signal_.get_future();

  /** Lidar data distribute control and lidar data source set */
  lddc_ptr_ = std::make_unique<Lddc>(xfer_format, multi_topic, data_src, output_type, publish_freq);
  lddc_ptr_->SetRosNode(this);

  if (data_src == kSourceRawLidar) {
    DRIVER_INFO(*this, "Data Source is raw lidar.");

    this->get_parameter("user_config_path", user_config_path_);
    DRIVER_INFO(*this, "Config file : %s", user_config_path_.c_str());

    std::string cmdline_bd_code;
    this->get_parameter("cmdline_input_bd_code", cmdline_bd_code);

    LdsLidar *read_lidar = LdsLidar::GetInstance(publish_freq);
    lddc_ptr_->RegisterLds(static_cast<Lds *>(read_lidar));

    if ((read_lidar->InitLdsLidar(user_config_path_))) {
      DRIVER_INFO(*this, "Init lds lidar success!");
    } else {
      DRIVER_ERROR(*this, "Init lds lidar fail!");
    }
  } else {
    DRIVER_ERROR(*this, "Invalid data src (%d), please check the launch file", data_src);
  }

  pointclouddata_poll_thread_ = std::make_shared<std::thread>(&DriverNode::PointCloudDataPollThread, this);
  imudata_poll_thread_ = std::make_shared<std::thread>(&DriverNode::ImuDataPollThread, this);
  
  // Create restart service
  restart_service_ = this->create_service<std_srvs::srv::Trigger>(
      "~/restart_lidar",
      std::bind(&DriverNode::RestartLidarCallback, this, std::placeholders::_1, std::placeholders::_2));
  DRIVER_INFO(*this, "LiDAR restart service created at ~/restart_lidar");
}

}  // namespace livox_ros

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(livox_ros::DriverNode)

#endif  // defined BUILDING_ROS2


void DriverNode::PointCloudDataPollThread()
{
  std::future_status status;
  std::this_thread::sleep_for(std::chrono::seconds(3));
  do {
    // Check if polling is paused
    {
      std::unique_lock<std::mutex> lock(polling_pause_mutex_);
      polling_pause_cv_.wait(lock, [this] { return !polling_paused_.load(); });
    }
    
    if (lddc_ptr_) {
      lddc_ptr_->DistributePointCloudData();
    }
    status = future_.wait_for(std::chrono::microseconds(0));
  } while (status == std::future_status::timeout);
}

void DriverNode::ImuDataPollThread()
{
  std::future_status status;
  std::this_thread::sleep_for(std::chrono::seconds(3));
  do {
    // Check if polling is paused
    {
      std::unique_lock<std::mutex> lock(polling_pause_mutex_);
      polling_pause_cv_.wait(lock, [this] { return !polling_paused_.load(); });
    }
    
    if (lddc_ptr_) {
      lddc_ptr_->DistributeImuData();
    }
    status = future_.wait_for(std::chrono::microseconds(0));
  } while (status == std::future_status::timeout);
}

void DriverNode::PausePollingThreads()
{
  DRIVER_INFO(*this, "Pausing polling threads...");
  polling_paused_.store(true);
  // Give threads time to finish current iteration and reach the wait
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
}

void DriverNode::ResumePollingThreads()
{
  DRIVER_INFO(*this, "Resuming polling threads...");
  polling_paused_.store(false);
  polling_pause_cv_.notify_all();
}

void DriverNode::RestartLidarCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;  // Unused parameter
  std::lock_guard<std::mutex> lock(restart_mutex_);
  
  DRIVER_INFO(*this, "LiDAR restart service called - performing FULL SDK-level reset");
  
  try {
    // Get the LiDAR instance
    LdsLidar *lds_lidar = dynamic_cast<LdsLidar*>(lddc_ptr_->lds_);
    if (!lds_lidar) {
      response->success = false;
      response->message = "Failed to get LiDAR instance";
      DRIVER_ERROR(*this, "%s", response->message.c_str());
      return;
    }
    
    // Check if initialized
    if (!lds_lidar->IsInitialized()) {
      response->success = false;
      response->message = "LiDAR is not initialized, cannot restart";
      DRIVER_WARN(*this, "%s", response->message.c_str());
      return;
    }
    
    // Step 1: Pause polling threads to prevent data access during reset
    DRIVER_INFO(*this, "Step 1: Pausing polling threads...");
    PausePollingThreads();
    
    // Step 2: Request LDS to stop processing
    DRIVER_INFO(*this, "Step 2: Requesting driver to stop processing...");
    lds_lidar->RequestExit();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    
    // Step 3: Complete SDK teardown (closes network sockets, destroys device manager)
    DRIVER_INFO(*this, "Step 3: Performing full SDK deinitialization (breaking hardware connection)...");
    int deinit_result = lds_lidar->DeInitLdsLidar();
    if (deinit_result != 0) {
      DRIVER_WARN(*this, "DeInitLdsLidar returned non-zero: %d, continuing anyway...", deinit_result);
    }
    
    // Step 4: Reset all driver state to clean slate
    DRIVER_INFO(*this, "Step 4: Resetting all driver state...");
    lds_lidar->CleanRequestExit();
    lds_lidar->ResetForRestart();
    
    // Step 5: Reinitialize SDK from config (reopens sockets, reconnects to hardware)
    DRIVER_INFO(*this, "Step 5: Reinitializing SDK and reconnecting to hardware...");
    if (!lds_lidar->InitLdsLidar(user_config_path_)) {
      ResumePollingThreads();
      response->success = false;
      response->message = "Failed to reinitialize LiDAR SDK after reset";
      DRIVER_ERROR(*this, "%s", response->message.c_str());
      return;
    }
    
    // Step 6: Resume polling threads
    DRIVER_INFO(*this, "Step 6: Resuming polling threads...");
    ResumePollingThreads();
    
    DRIVER_INFO(*this, "LiDAR full SDK reset completed successfully!");
    response->success = true;
    response->message = "LiDAR SDK reset successfully with complete hardware reconnection";
    
  } catch (const std::exception& e) {
    ResumePollingThreads();
    response->success = false;
    response->message = std::string("Exception during SDK reset: ") + e.what();
    DRIVER_ERROR(*this, "%s", response->message.c_str());
  }
}





















