#ifndef VINS_IMU_FILTER_HPP
#define VINS_IMU_FILTER_HPP

#include <memory>
#include <mutex>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <laser_uav_lib/filter/imu_filter.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace vins_imu_filter
{
class VinsImuFilter : public rclcpp_lifecycle::LifecycleNode {
public:
  /* VinsImuFilter() //{ */
  explicit VinsImuFilter(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  /*//}*/

  /* ~VinsImuFilter() //{ */
  ~VinsImuFilter() override;
  /*//}*/

private:
  /* CONFIGURATION //{ */

  /* on_configure() //{ */
  CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;
  /*//}*/

  /* on_activate() //{ */
  CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;
  /*//}*/

  /* on_deactivate() //{ */
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;
  /*//}*/

  /* on_cleanup() //{ */
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override;
  /*//}*/

  /* on_shutdown() //{ */
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override;
  /*//}*/

  /* getParameters() //{ */
  void getParameters();
  /*//}*/

  /* configPubSub() //{ */
  void configPubSub();
  /*//}*/

  /* configTimers() //{ */
  void configTimers();
  /*//}*/

  /*//}*/

  /* SUBSCRIBERS //{ */
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_accel_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_gyro_;
  /*//}*/

  /* PUBLISHERS //{ */
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
  /*//}*/

  /* CALLBACKS //{ */

  /* subImuCallback() //{ */
  void subImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void subAccelCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void subGyroCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  /*//}*/

  /*//}*/

  /* VARIABLES //{ */
  bool is_initialized_{false};
  bool acc_received_{false};
  bool gyro_received_{false};
  bool imu_received_{false};

  rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr tf_broadcaster_;
  bool                                                               _acc_iir_enable_;
  std::vector<double>                                                _acc_iir_a_;
  std::vector<double>                                                _acc_iir_b_;
  bool                                                               _acc_notch_enable_;
  int                                                                _acc_notch_sample_rate_;
  std::vector<int>                                                   _acc_notch_frequencies_;
  int                                                                _acc_notch_bandwidth_;
  bool                                                               _gyro_iir_enable_;
  std::vector<double>                                                _gyro_iir_a_;
  std::vector<double>                                                _gyro_iir_b_;
  bool                                                               _gyro_notch_enable_;
  int                                                                _gyro_notch_sample_rate_;
  std::vector<int>                                                   _gyro_notch_frequencies_;
  int                                                                _gyro_notch_bandwidth_;

  bool        _change_frame_id_enabled_;
  std::string _frame_id_;

  bool                                      _imu_data_united_{false};
  std::unique_ptr<laser_uav_lib::ImuFilter> imu_filter_;

  sensor_msgs::msg::Imu last_accel_msg_;
  std::mutex            mutex_last_accel_msg_;

  sensor_msgs::msg::Imu last_gyro_msg_;

  /*//}*/

  /* FUNCTIONS //{ */
  void                   initializeFilter();
  laser_uav_lib::ImuData convertToImuData(const sensor_msgs::msg::Imu &imu_msg);
  sensor_msgs::msg::Imu  convertToSensorMsg(const laser_uav_lib::ImuData &imu_data);

  /*//}*/
};

}  // namespace vins_imu_filter

#endif  // VINS_IMU_FILTER_HPP
