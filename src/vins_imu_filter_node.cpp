#include <laser_vins_imu_filter/vins_imu_filter.hpp>

namespace vins_imu_filter
{

    /* VinsImuFilter() //{ */
    VinsImuFilter::VinsImuFilter(const rclcpp::NodeOptions &options)
        : rclcpp_lifecycle::LifecycleNode("vins_imu_filter", "", options)
    {
        RCLCPP_INFO(get_logger(), "Creating VINS IMU Filter Node");

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

        declare_parameter("change_frame_id.enable", rclcpp::ParameterValue(false));
        declare_parameter("change_frame_id.frame_id", rclcpp::ParameterValue(std::string("imu_link")));

        declare_parameter("imu_data_united", rclcpp::ParameterValue(false));

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
        if (_imu_data_united_){
          sub_imu_.reset();
        } else {
          sub_gyro_.reset();
          sub_accel_.reset();
        }

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

        get_parameter("imu_data_united", _imu_data_united_);

        RCLCPP_INFO(get_logger(), "Filter parameters loaded.");
    }
    /*//}*/

    /* configPubSub() //{ */
    void VinsImuFilter::configPubSub()
    {
        RCLCPP_INFO(get_logger(), "Configuring publishers and subscribers...");

        if (_imu_data_united_) {
            sub_imu_ = create_subscription<sensor_msgs::msg::Imu>("imu_in", rclcpp::QoS(10).best_effort(),
                std::bind(&VinsImuFilter::subImuCallback, this, std::placeholders::_1));
        } else {
            sub_gyro_ = create_subscription<sensor_msgs::msg::Imu>("gyro_in", rclcpp::QoS(10).best_effort(),
                std::bind(&VinsImuFilter::subGyroCallback, this, std::placeholders::_1));

            sub_accel_ = create_subscription<sensor_msgs::msg::Imu>("accel_in", rclcpp::QoS(10).best_effort(),
                std::bind(&VinsImuFilter::subAccelCallback, this, std::placeholders::_1));
        }

        pub_imu_ = create_publisher<sensor_msgs::msg::Imu>("imu_out", 10);

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

        // Usando as funções de conversão
        laser_uav_lib::ImuData imu_data = convertToImuData(*msg);

        // Filtrar os dados
        laser_uav_lib::ImuData filtered_imu_data = imu_filter_->filter(imu_data);

        // Converter de volta para mensagem ROS
        sensor_msgs::msg::Imu filtered_msg = convertToSensorMsg(filtered_imu_data);

        if (_change_frame_id_enabled_)
        {
            filtered_msg.header.frame_id = _frame_id_;
        }

        pub_imu_->publish(filtered_msg);
        RCLCPP_DEBUG(get_logger(), "Published filtered IMU message");
    }
    /*//}*/

    /**
     * @brief Callback function for accelerometer data
     * @param msg The accelerometer message
     * @details This function filters the accelerometer data and publishes the filtered message.
     *          It also updates the frame ID if the change_frame_id_enabled_ parameter is set.
     *          The filtered message is published at a rate of 1 Hz.
     */
    /* subAccelCallback() //{ */
    void VinsImuFilter::subAccelCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        if (!is_initialized_)
        {
            RCLCPP_WARN_ONCE(get_logger(), "IMU Filter not fully initialized yet.");
            return;
        }

        RCLCPP_DEBUG(get_logger(), "Received accelerometer message");

        acc_received_ = true;
        laser_uav_lib::ImuData imu_data = convertToImuData(*msg);
        sensor_msgs::msg::Imu filtered_msg = convertToSensorMsg(imu_filter_->filterAccelerometer(imu_data));

        if (_change_frame_id_enabled_)
        {
            filtered_msg.header.frame_id = _frame_id_;
        }
        {
            std::scoped_lock lock(mutex_last_accel_msg_);
            last_accel_msg_ = filtered_msg;
        }

        RCLCPP_INFO_THROTTLE(
            get_logger(),
            *get_clock(), // Adicionar o clock como segundo argumento
            1000,         // Período em milissegundos (1.0 segundo = 1000 ms)
            "Filtered accelerometer message salved");
    }
    /*//}*/

    /**
     * @brief Callback function for gyroscope data
     * @param msg The gyroscope message
     * @details This function filters the gyroscope data and publishes the filtered message.
     *          It also updates the frame ID if the change_frame_id_enabled_ parameter is set.
     *          The filtered message is published at a rate of 1 Hz.
     * @note This function is called when a gyroscope message is received.
     * @warning This function assumes that the IMU filter is fully initialized.
     */
    /* subGyroCallback() //{ */
    void VinsImuFilter::subGyroCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        if (!is_initialized_)
        {
            RCLCPP_WARN_ONCE(get_logger(), "IMU Filter not fully initialized yet.");
            return;
        }

        RCLCPP_DEBUG(get_logger(), "Received gyroscope message");

        gyro_received_ = true;

        laser_uav_lib::ImuData imu_data = convertToImuData(*msg);
        sensor_msgs::msg::Imu filtered_msg = convertToSensorMsg(imu_filter_->filterGyro(imu_data));

        if (_change_frame_id_enabled_)
        {
            filtered_msg.header.frame_id = _frame_id_;
        }
        {
            std::scoped_lock lock(mutex_last_accel_msg_);
            filtered_msg.linear_acceleration = last_accel_msg_.linear_acceleration;
        }

        RCLCPP_INFO_THROTTLE(
            get_logger(),
            *get_clock(), // Adicionar o clock como segundo argumento
            1000,         // Período em milissegundos (1.0 segundo = 1000 ms)
            "Filtered gyroscope message published");

        pub_imu_->publish(filtered_msg);
    }
    /*//}*/

    laser_uav_lib::ImuData VinsImuFilter::convertToImuData(const sensor_msgs::msg::Imu &imu_msg)
    {
        laser_uav_lib::ImuData imu_data;

        // Copiar aceleração
        imu_data.linear_acceleration_x = imu_msg.linear_acceleration.x;
        imu_data.linear_acceleration_y = imu_msg.linear_acceleration.y;
        imu_data.linear_acceleration_z = imu_msg.linear_acceleration.z;

        // Copiar giroscópio (se necessário)
        imu_data.angular_velocity_x = imu_msg.angular_velocity.x;
        imu_data.angular_velocity_y = imu_msg.angular_velocity.y;
        imu_data.angular_velocity_z = imu_msg.angular_velocity.z;

        // Copiar timestamp (se necessário)
        imu_data.timestamp = imu_msg.header.stamp.sec + imu_msg.header.stamp.nanosec * 1e-9;

        return imu_data;
    }

    sensor_msgs::msg::Imu VinsImuFilter::convertToSensorMsg(const laser_uav_lib::ImuData &imu_data)
    {
        sensor_msgs::msg::Imu imu_msg;

        // Configurar cabeçalho
        imu_msg.header.stamp = rclcpp::Time(static_cast<int64_t>(imu_data.timestamp * 1e9)); // converte segundos para nanosegundos
        imu_msg.header.frame_id = imu_data.frame_id;

        // Orientação (quaternion)
        imu_msg.orientation.x = imu_data.orientation[0];
        imu_msg.orientation.y = imu_data.orientation[1];
        imu_msg.orientation.z = imu_data.orientation[2];
        imu_msg.orientation.w = imu_data.orientation[3];

        // Matriz de covariância da orientação (row-major)
        std::copy(imu_data.orientation_covariance.begin(),
                  imu_data.orientation_covariance.end(),
                  imu_msg.orientation_covariance.begin());

        // Velocidade angular
        imu_msg.angular_velocity.x = imu_data.angular_velocity_x;
        imu_msg.angular_velocity.y = imu_data.angular_velocity_y;
        imu_msg.angular_velocity.z = imu_data.angular_velocity_z;

        // Matriz de covariância da velocidade angular
        std::copy(imu_data.angular_velocity_covariance.begin(),
                  imu_data.angular_velocity_covariance.end(),
                  imu_msg.angular_velocity_covariance.begin());

        // Aceleração linear
        imu_msg.linear_acceleration.x = imu_data.linear_acceleration_x;
        imu_msg.linear_acceleration.y = imu_data.linear_acceleration_y;
        imu_msg.linear_acceleration.z = imu_data.linear_acceleration_z;

        // Matriz de covariância da aceleração linear
        std::copy(imu_data.linear_acceleration_covariance.begin(),
                  imu_data.linear_acceleration_covariance.end(),
                  imu_msg.linear_acceleration_covariance.begin());

        return imu_msg;
    }

} // namespace vins_imu_filter
