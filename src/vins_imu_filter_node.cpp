#include <laser_vins_imu_filter/vins_imu_filter.hpp>

namespace vins_imu_filter
{

    /* VinsImuFilter() //{ */
    VinsImuFilter::VinsImuFilter(const rclcpp::NodeOptions &options)
        : rclcpp_lifecycle::LifecycleNode("vins_imu_filter", "", options)
    {
        RCLCPP_INFO(get_logger(), "Creating VINS IMU Filter Node");

        // Declare parameters (optional here, can be done in on_configure as well)
        declare_parameter("accelerometer.iir_filter.enable", rclcpp::ParameterValue(false));
        declare_parameter("accelerometer.iir_filter.a", rclcpp::ParameterValue(std::vector<double>{}));
        declare_parameter("accelerometer.iir_filter.b", rclcpp::ParameterValue(std::vector<double>{}));

        declare_parameter("accelerometer.notch_filter.enable", rclcpp::ParameterValue(false));
        declare_parameter("accelerometer.notch_filter.sample_rate", rclcpp::ParameterValue(400));
        declare_parameter("accelerometer.notch_filter.frequencies", rclcpp::ParameterValue(std::vector<int>{}));
        declare_parameter("accelerometer.notch_filter.bandwidth", rclcpp::ParameterValue(50));

        declare_parameter("gyro.iir_filter.enable", rclcpp::ParameterValue(false));
        declare_parameter("gyro.iir_filter.a", rclcpp::ParameterValue(std::vector<double>{}));
        declare_parameter("gyro.iir_filter.b", rclcpp::ParameterValue(std::vector<double>{}));

        declare_parameter("gyro.notch_filter.enable", rclcpp::ParameterValue(false));
        declare_parameter("gyro.notch_filter.sample_rate", rclcpp::ParameterValue(400));
        declare_parameter("gyro.notch_filter.frequencies", rclcpp::ParameterValue(std::vector<int>{}));
        declare_parameter("gyro.notch_filter.bandwidth", rclcpp::ParameterValue(50));

        declare_parameter("change_frame_id.enable", false);
        declare_parameter("change_frame_id.frame_id", std::string("imu_link"));

        RCLCPP_INFO(get_logger(), "VINS IMU Filter Node initialized.");
    }
    /*//}*/

    /* ~VinsImuFilter() //{ */
    VinsImuFilter::~VinsImuFilter()
    {
        RCLCPP_INFO(get_logger(), "Destroying VINS IMU Filter Node");
    }
    /*//}*/

    /* on_configure() //{ */
    CallbackReturn VinsImuFilter::on_configure(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "Configuring VINS IMU Filter Node...");

        getParameters();
        configPubSub();
        configTimers();
        initializeFilter();

        is_initialized_ = true;

        RCLCPP_INFO(get_logger(), "VINS IMU Filter Node configured.");
        return CallbackReturn::SUCCESS;
    }
    /*//}*/

    /* on_activate() //{ */
    CallbackReturn VinsImuFilter::on_activate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "Activating VINS IMU Filter Node...");

        // Activate publishers (if any)
        pub_imu_->on_activate();

        // Activate subscribers

        RCLCPP_INFO(get_logger(), "VINS IMU Filter Node activated.");
        return CallbackReturn::SUCCESS;
    }
    /*//}*/

    /* on_deactivate() //{ */
    CallbackReturn VinsImuFilter::on_deactivate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "Deactivating VINS IMU Filter Node...");

        // Deactivate publishers (if any)
        pub_imu_->on_deactivate();

        RCLCPP_INFO(get_logger(), "VINS IMU Filter Node deactivated.");
        return CallbackReturn::SUCCESS;
    }
    /*//}*/

    /* on_cleanup() //{ */
    CallbackReturn VinsImuFilter::on_cleanup(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "Cleaning up VINS IMU Filter Node...");

        // Reset publishers
        pub_imu_.reset();

        // Reset subscribers
        sub_imu_.reset();

        // Reset timers
        // timer_.reset();

        RCLCPP_INFO(get_logger(), "VINS IMU Filter Node cleaned up.");
        return CallbackReturn::SUCCESS;
    }
    /*//}*/

    /* on_shutdown() //{ */
    CallbackReturn VinsImuFilter::on_shutdown(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "Shutting down VINS IMU Filter Node...");
        return CallbackReturn::SUCCESS;
    }
    /*//}*/

    /* getParameters() //{ */
    void VinsImuFilter::getParameters()
    {
        RCLCPP_INFO(get_logger(), "Getting filter parameters...");

        // Accelerometer IIR Filter
        get_parameter("accelerometer.iir_filter.enable", _acc_iir_enable_);
        RCLCPP_INFO(get_logger(), "Accelerometer IIR Filter Enable: %s", _acc_iir_enable_ ? "true" : "false");

        get_parameter("accelerometer.iir_filter.a", _acc_iir_a_);
        RCLCPP_INFO(get_logger(), "Accelerometer IIR Filter a: [%s]",
                    std::accumulate(_acc_iir_a_.begin(), _acc_iir_a_.end(), std::string(),
                                    [](const std::string &a, double b)
                                    { return a.empty() ? std::to_string(b) : a + ", " + std::to_string(b); })
                        .c_str());

        get_parameter("accelerometer.iir_filter.b", _acc_iir_b_);
        RCLCPP_INFO(get_logger(), "Accelerometer IIR Filter b: [%s]",
                    std::accumulate(_acc_iir_b_.begin(), _acc_iir_b_.end(), std::string(),
                                    [](const std::string &a, double b)
                                    { return a.empty() ? std::to_string(b) : a + ", " + std::to_string(b); })
                        .c_str());

        // Accelerometer Notch Filter
        get_parameter("accelerometer.notch_filter.enable", _acc_notch_enable_);
        RCLCPP_INFO(get_logger(), "Accelerometer Notch Filter Enable: %s", _acc_notch_enable_ ? "true" : "false");

        get_parameter("accelerometer.notch_filter.sample_rate", _acc_notch_sample_rate_);
        RCLCPP_INFO(get_logger(), "Accelerometer Notch Filter Sample Rate: %d", _acc_notch_sample_rate_);

        std::vector<int64_t> temp_frequencies;
        get_parameter("accelerometer.notch_filter.frequencies", temp_frequencies);
        _acc_notch_frequencies_.clear();
        _acc_notch_frequencies_.reserve(temp_frequencies.size());
        for (const auto &freq : temp_frequencies)
        {
            _acc_notch_frequencies_.push_back(static_cast<int>(freq));
        }

        RCLCPP_INFO(get_logger(), "Accelerometer Notch Filter Frequencies: [%s]",
                    std::accumulate(_acc_notch_frequencies_.begin(), _acc_notch_frequencies_.end(), std::string(),
                                    [](const std::string &a, int b)
                                    { return a.empty() ? std::to_string(b) : a + ", " + std::to_string(b); })
                        .c_str());

        get_parameter("accelerometer.notch_filter.bandwidth", _acc_notch_bandwidth_);
        RCLCPP_INFO(get_logger(), "Accelerometer Notch Filter Bandwidth: %d", _acc_notch_bandwidth_);

        // Gyro IIR Filter (similar to accelerometer)
        get_parameter("gyro.iir_filter.enable", _gyro_iir_enable_);
        RCLCPP_INFO(get_logger(), "Gyro IIR Filter Enable: %s", _gyro_iir_enable_ ? "true" : "false");

        get_parameter("gyro.iir_filter.a", _gyro_iir_a_);
        RCLCPP_INFO(get_logger(), "Gyro IIR Filter a: [%s]",
                    std::accumulate(_gyro_iir_a_.begin(), _gyro_iir_a_.end(), std::string(),
                                    [](const std::string &a, double b)
                                    { return a.empty() ? std::to_string(b) : a + ", " + std::to_string(b); })
                        .c_str());

        get_parameter("gyro.iir_filter.b", _gyro_iir_b_);
        RCLCPP_INFO(get_logger(), "Gyro IIR Filter b: [%s]",
                    std::accumulate(_gyro_iir_b_.begin(), _gyro_iir_b_.end(), std::string(),
                                    [](const std::string &a, double b)
                                    { return a.empty() ? std::to_string(b) : a + ", " + std::to_string(b); })
                        .c_str());

        // Gyro Notch Filter (similar to accelerometer)
        get_parameter("gyro.notch_filter.enable", _gyro_notch_enable_);
        RCLCPP_INFO(get_logger(), "Gyro Notch Filter Enable: %s", _gyro_notch_enable_ ? "true" : "false");

        get_parameter("gyro.notch_filter.sample_rate", _gyro_notch_sample_rate_);
        RCLCPP_INFO(get_logger(), "Gyro Notch Filter Sample Rate: %d", _gyro_notch_sample_rate_);

        std::vector<int64_t> temp_gyro_notch_frequencies_;
        get_parameter("accelerometer.notch_filter.frequencies", temp_gyro_notch_frequencies_);
        _acc_notch_frequencies_.clear();
        _acc_notch_frequencies_.reserve(temp_gyro_notch_frequencies_.size());
        for (const auto &freq : temp_gyro_notch_frequencies_)
        {
            _acc_notch_frequencies_.push_back(static_cast<int>(freq));
        }
        RCLCPP_INFO(get_logger(), "Gyro Notch Filter Frequencies: [%s]",
                    std::accumulate(_gyro_notch_frequencies_.begin(), _gyro_notch_frequencies_.end(), std::string(),
                                    [](const std::string &a, int b)
                                    { return a.empty() ? std::to_string(b) : a + ", " + std::to_string(b); })
                        .c_str());

        get_parameter("gyro.notch_filter.bandwidth", _gyro_notch_bandwidth_);
        RCLCPP_INFO(get_logger(), "Gyro Notch Filter Bandwidth: %d", _gyro_notch_bandwidth_);

        // Change Frame ID
        get_parameter("change_frame_id.enable", _change_frame_id_enabled_);
        RCLCPP_INFO(get_logger(), "Change Frame ID Enable: %s", _change_frame_id_enabled_ ? "true" : "false");

        if (_change_frame_id_enabled_)
        {
            get_parameter("change_frame_id.frame_id", _frame_id_);
            RCLCPP_INFO(get_logger(), "Change Frame ID: %s", _frame_id_.c_str());
        }

        RCLCPP_INFO(get_logger(), "Filter parameters loaded.");
    }
    /*//}*/

    /* configPubSub() //{ */
    void VinsImuFilter::configPubSub()
    {
        RCLCPP_INFO(get_logger(), "Configuring publishers and subscribers...");

        // Subscriber for IMU data
        sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
            "/uav1/rgbd/imu", rclcpp::QoS(10).best_effort(),
            std::bind(&VinsImuFilter::subImuCallback, this, std::placeholders::_1));

        // Publisher for TF
        pub_imu_ = create_publisher<sensor_msgs::msg::Imu>("/uav1/rgbd/imu_filtered", 10);

        RCLCPP_INFO(get_logger(), "Publishers and subscribers configured.");
    }
    /*//}*/

    /* configTimers() //{ */
    void VinsImuFilter::configTimers()
    {
        RCLCPP_INFO(get_logger(), "Configuring timers...");
        // Add any timers you need here
        RCLCPP_INFO(get_logger(), "Timers configured.");
    }
    /*//}*/

    /* initializeFilter() //{ */
    void VinsImuFilter::initializeFilter()
    {
        RCLCPP_INFO(get_logger(), "Initializing IMU filter...");

        Eigen::MatrixXd acc_notch_freq_matrix;
        if (!_acc_notch_frequencies_.empty())
        {
            acc_notch_freq_matrix.resize(1, _acc_notch_frequencies_.size());
            for (size_t i = 0; i < _acc_notch_frequencies_.size(); ++i)
            {
                acc_notch_freq_matrix(0, i) = _acc_notch_frequencies_[i];
            }
        }

        Eigen::MatrixXd gyro_notch_freq_matrix;
        if (!_gyro_notch_frequencies_.empty())
        {
            gyro_notch_freq_matrix.resize(1, _gyro_notch_frequencies_.size());
            for (size_t i = 0; i < _gyro_notch_frequencies_.size(); ++i)
            {
                gyro_notch_freq_matrix(0, i) = _gyro_notch_frequencies_[i];
            }
        }

        imu_filter_ = std::make_unique<laser_uav_lib::ImuFilter>(
            _acc_iir_enable_, _acc_iir_a_, _acc_iir_b_,
            _acc_notch_enable_, _acc_notch_sample_rate_, acc_notch_freq_matrix, _acc_notch_bandwidth_,
            _gyro_iir_enable_, _gyro_iir_a_, _gyro_iir_b_,
            _gyro_notch_enable_, _gyro_notch_sample_rate_, gyro_notch_freq_matrix, _gyro_notch_bandwidth_);

        RCLCPP_INFO(get_logger(), "IMU filter initialized.");
    }
    /*//}*/

    /* subImuCallback() //{ */
    void VinsImuFilter::subImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        if (!is_initialized_)
        {
            RCLCPP_WARN_ONCE(get_logger(), "IMU Filter not fully initialized yet.");
            return;
        }

        RCLCPP_DEBUG(get_logger(), "Received IMU message");

        laser_uav_lib::ImuData imu_data;
        imu_data.linear_acceleration_x = msg->linear_acceleration.x;
        imu_data.linear_acceleration_y = msg->linear_acceleration.y;
        imu_data.linear_acceleration_z = msg->linear_acceleration.z;
        imu_data.angular_velocity_x = msg->angular_velocity.x;
        imu_data.angular_velocity_y = msg->angular_velocity.y;
        imu_data.angular_velocity_z = msg->angular_velocity.z;

        laser_uav_lib::ImuData filtered_imu_data = imu_filter_->filter(imu_data);

        sensor_msgs::msg::Imu filtered_msg = *msg; // Copy the original message
        filtered_msg.linear_acceleration.x = filtered_imu_data.linear_acceleration_x;
        filtered_msg.linear_acceleration.y = filtered_imu_data.linear_acceleration_y;
        filtered_msg.linear_acceleration.z = filtered_imu_data.linear_acceleration_z;
        filtered_msg.angular_velocity.x = filtered_imu_data.angular_velocity_x;
        filtered_msg.angular_velocity.y = filtered_imu_data.angular_velocity_y;
        filtered_msg.angular_velocity.z = filtered_imu_data.angular_velocity_z;

        if (_change_frame_id_enabled_)
        {
            filtered_msg.header.frame_id = _frame_id_;
        }

        pub_imu_->publish(filtered_msg);
        RCLCPP_DEBUG(get_logger(), "Published filtered IMU message");
    }
    /*//}*/

} // namespace vins_imu_filter
