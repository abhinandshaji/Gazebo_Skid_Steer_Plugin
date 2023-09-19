#include <gazebo/common/Time.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>

#include <skid_steer_drive_plugin/skid_steer_drive_plugin.hpp>

#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <gazebo_ros/node.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sdf/sdf.hh>

// #ifdef NO_ERROR
// // NO_ERROR is a macro defined in Windows that's used as an enum in tf2
// #undef NO_ERROR
// #endif

#ifdef IGN_PROFILER_ENABLE
#include <ignition/common/Profiler.hh>
#endif

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <utility>

namespace gazebo
{
  class SkidSteerDrivePrivate
  {
  public:
    // indicate where the odom is calculate from
    enum OdomSource
    {
      ENCODER = 0,
      WORLD = 1,
    };

    /// Callback to be called at every simulation iteration.
    /// \param[in] _info Updated simulation info.
    void OnUpdate(const gazebo::common::UpdateInfo &_info);

    /// Callback when a velocity command is received.
    /// \param[in] _msg Twist command message.
    void OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr _msg);

    /// Update wheel velocities according to latest target velocities.
    void UpdateWheelVelocities();

    /// Odometry updation
    /// \param[in] _current_time Twist command message.
    void UpdateOdometryEncoder(const gazebo::common::Time &_current_time);

    // update odom according to world
    void UpdateOdometryWorld();

    /// to publish odom transforms
    /// \param[in] _current_time Twist command message.
    void PublishOdometryTf(const gazebo::common::Time &_current_time);

    /// to publish wheel transforms
    /// \param[in] _current_time Twist command message.
    void PublishWheelsTf(const gazebo::common::Time &_current_time);

    /// to publish odom msgs calulated in the update Encoder funtion
    /// \param[in] _current_time Twist command message.
    void PublishOdometryMsg(const gazebo::common::Time &_current_time);

    /// A pointer to the GazeboROS node.
    gazebo_ros::Node::SharedPtr ros_node_;

    /// Subscriber to command velocities
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

    /// Odometry publisher
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;

    /// Connection to event called at every world iteration.
    gazebo::event::ConnectionPtr update_connection_;

    double wheel_separation_;
    double wheel_sep_length_;
    double wheel_radius_;
    double max_wheel_torque_; // torque in [Nm]
    double max_wheel_accel_;

    /// Desired wheel speed.
    std::vector<double> desired_wheel_speed_;

    /// Speed sent to wheel.
    std::vector<double> wheel_speed_instr_;

    /// Pointers to wheel joints.
    std::vector<gazebo::physics::JointPtr> joints_;
    /// Pointer to model.
    gazebo::physics::ModelPtr model_;

    // pointer to brodcast transform msgs
    std::shared_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;

    std::mutex lock_;
    double target_x_{0.0};   // in m/s
    double target_y_{0.0};   // in m/s
    double target_rot_{0.0}; // in rad/s

    unsigned int num_wheel_;
    unsigned int num_wheel_pairs_;

    // time params and msgs update frequency
    double update_period_;

    // previous update time (gazebo clock)
    gazebo::common::Time last_update_time_;

    // variable to store encoder readings from position feedback
    geometry_msgs::msg::Pose2D pose_encoder_;

    /// Odometry frame ID
    std::string odometry_frame_;

    /// Last time the encoder was updated
    gazebo::common::Time last_encoder_update_;

    /// Either ENCODER or WORLD
    OdomSource odom_source_;

    /// Keep latest odometry message
    nav_msgs::msg::Odometry odom_;

    std::string robot_base_frame_;

    bool publish_odom_;
    bool publish_odom_tf_;
    bool publish_wheel_tf_;

    double covariance_[3];
  };

  SkidSteerDrive::SkidSteerDrive()
      : impl_(std::make_unique<SkidSteerDrivePrivate>()) {}

  SkidSteerDrive::~SkidSteerDrive() {}

  void SkidSteerDrive::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {
    impl_->model_ = _model;

    // Initialize ROS node
    impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

    // Get QoS profiles
    const gazebo_ros::QoS &qos = impl_->ros_node_->get_qos();
    if (impl_->num_wheel_ < 4)
    {
      impl_->num_wheel_ = 4;
      RCLCPP_WARN(impl_->ros_node_->get_logger(), "This plugin requires exactly [4 wheels] to work so setting [num_wheel]=4\n");
    }

    // loading params part-1

    impl_->num_wheel_ = static_cast<unsigned int>(_sdf->Get<int>("num_wheel"));
    impl_->max_wheel_accel_ = _sdf->Get<double>("max_wheel_acceleration");
    impl_->max_wheel_torque_ = _sdf->Get<double>("max_wheel_torque");

    // loading joints
    std::vector<gazebo::physics::JointPtr> front_left_joints, front_right_joints, back_left_joints, back_right_joints;
    for (auto front_left_joint_elem = _sdf->GetElement("front_left_joint"); front_left_joint_elem != nullptr; front_left_joint_elem = front_left_joint_elem->GetNextElement("front_left_joint"))
    {
      auto front_left_joint_name = front_left_joint_elem->Get<std::string>();
      auto front_left_joint = _model->GetJoint(front_left_joint_name);

      if (!front_left_joint)
      {
        RCLCPP_ERROR(impl_->ros_node_->get_logger(), "Joint [%s] not found, plugin will not work.", front_left_joint_name.c_str());
        impl_->ros_node_.reset();
        return;
      }
      front_left_joint->SetParam("fmax", 0, impl_->max_wheel_torque_);
      front_left_joints.push_back(front_left_joint);
    }
    for (auto front_right_joint_elem = _sdf->GetElement("front_right_joint"); front_right_joint_elem != nullptr; front_right_joint_elem = front_right_joint_elem->GetNextElement("front_right_joint"))
    {
      auto front_right_joint_name = front_right_joint_elem->Get<std::string>();
      auto front_right_joint = _model->GetJoint(front_right_joint_name);

      if (!front_right_joint)
      {
        RCLCPP_ERROR(impl_->ros_node_->get_logger(), "Joint [%s] not found, plugin will not work.", front_right_joint_name.c_str());
        impl_->ros_node_.reset();
        return;
      }
      front_right_joint->SetParam("fmax", 0, impl_->max_wheel_torque_);
      front_right_joints.push_back(front_right_joint);
    }
    for (auto back_left_joint_elem = _sdf->GetElement("back_left_joint"); back_left_joint_elem != nullptr; back_left_joint_elem = back_left_joint_elem->GetNextElement("back_left_joint"))
    {
      auto back_left_joint_name = back_left_joint_elem->Get<std::string>();
      auto back_left_joint = _model->GetJoint(back_left_joint_name);

      if (!back_left_joint)
      {
        RCLCPP_ERROR(impl_->ros_node_->get_logger(), "Joint [%s] not found, plugin will not work.", back_left_joint_name.c_str());
        impl_->ros_node_.reset();
        return;
      }
      back_left_joint->SetParam("fmax", 0, impl_->max_wheel_torque_);
      back_left_joints.push_back(back_left_joint);
    }
    for (auto back_right_joint_elem = _sdf->GetElement("back_right_joint"); back_right_joint_elem != nullptr; back_right_joint_elem = back_right_joint_elem->GetNextElement("back_right_joint"))
    {
      auto back_right_joint_name = back_right_joint_elem->Get<std::string>();
      auto back_right_joint = _model->GetJoint(back_right_joint_name);

      if (!back_right_joint)
      {
        RCLCPP_ERROR(impl_->ros_node_->get_logger(), "Joint [%s] not found, plugin will not work.", back_right_joint_name.c_str());
        impl_->ros_node_.reset();
        return;
      }
      back_right_joint->SetParam("fmax", 0, impl_->max_wheel_torque_);
      back_right_joints.push_back(back_right_joint);
    }

    unsigned int index; // unsigned is optional its juts that the index is best to be kept >= 0
    // sending joints to joints_ vector storage space
    for (index = 0; index < impl_->num_wheel_ / 4; index++)
    {
      impl_->joints_.push_back(front_left_joints[index]);
      impl_->joints_.push_back(front_right_joints[index]);
      impl_->joints_.push_back(back_left_joints[index]);
      impl_->joints_.push_back(back_right_joints[index]);
    }

    impl_->wheel_separation_ = _sdf->Get<double>("wheel_separation");
    impl_->wheel_sep_length_ = _sdf->Get<double>("wheel_separation_length");
    impl_->wheel_radius_ = _sdf->Get<double>("wheel_radius");

    // setting desired and intial wheel velocities to zero
    impl_->wheel_speed_instr_.assign(impl_->num_wheel_, 0);
    impl_->desired_wheel_speed_.assign(impl_->num_wheel_, 0);

    // setting the Update rate
    auto update_rate = _sdf->Get<double>("update_rate");
    if (update_rate > 0.0)
    {
      impl_->update_period_ = 1.0 / update_rate;
    }
    else
    {
      impl_->update_period_ = 0.0;
    }
    // previous simulation time
    impl_->last_update_time_ = _model->GetWorld()->SimTime();
    printf("previous sim time: [%f]\n", impl_->last_update_time_);

    // creating s subscription to twist msgs topic /cmd_vel
    impl_->cmd_vel_sub_ = impl_->ros_node_->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", qos.get_subscription_qos("cmd_vel", rclcpp::QoS(1)),
        std::bind(&SkidSteerDrivePrivate::OnCmdVel, impl_.get(), std::placeholders::_1));

    RCLCPP_INFO(impl_->ros_node_->get_logger(), "Subscribed to [%s]", impl_->cmd_vel_sub_->get_topic_name());

    // Odometry
    impl_->odometry_frame_ = _sdf->Get<std::string>("odometry_frame", "odom").first;
    impl_->robot_base_frame_ = _sdf->Get<std::string>("robot_base_frame", "base_footprint").first;
    impl_->odom_source_ = static_cast<SkidSteerDrivePrivate::OdomSource>(_sdf->Get<int>("odometry_source", 1).first);

    // Publish odom if true passed from URDF
    impl_->publish_odom_ = _sdf->Get<bool>("publish_odom", false).first;
    if (impl_->publish_odom_)
    {
      impl_->odometry_pub_ = impl_->ros_node_->create_publisher<nav_msgs::msg::Odometry>(
          "odom", qos.get_publisher_qos("odom", rclcpp::QoS(1)));

      RCLCPP_INFO(
          impl_->ros_node_->get_logger(), "Advertise odometry on [%s]",
          impl_->odometry_pub_->get_topic_name());
    }

    // Create TF broadcaster if needed
    impl_->publish_wheel_tf_ = _sdf->Get<bool>("publish_wheel_tf", false).first;
    impl_->publish_odom_tf_ = _sdf->Get<bool>("publish_odom_tf", false).first;
    if (impl_->publish_wheel_tf_ || impl_->publish_odom_tf_)
    {
      impl_->transform_broadcaster_ =
          std::make_shared<tf2_ros::TransformBroadcaster>(impl_->ros_node_);

      if (impl_->publish_odom_tf_)
      {
        RCLCPP_INFO(
            impl_->ros_node_->get_logger(),
            "Publishing odom transforms between [%s] and [%s]", impl_->odometry_frame_.c_str(),
            impl_->robot_base_frame_.c_str());
      }

      for (index = 0; index < impl_->num_wheel_; ++index)
      {
        if (impl_->publish_wheel_tf_)
        {
          RCLCPP_INFO(
              impl_->ros_node_->get_logger(),
              "Publishing wheel transforms between [%s], [%s], [%s], [%s], [%s]",
              impl_->robot_base_frame_.c_str(),
              impl_->joints_[0]->GetName().c_str(),
              impl_->joints_[1]->GetName().c_str(),
              impl_->joints_[2]->GetName().c_str(),
              impl_->joints_[3]->GetName().c_str());
        }
      }
    }

    impl_->covariance_[0] = _sdf->Get<double>("covariance_x", 0.00001).first;
    impl_->covariance_[1] = _sdf->Get<double>("covariance_y", 0.00001).first;
    impl_->covariance_[2] = _sdf->Get<double>("covariance_yaw", 0.001).first;

    // Listen to the update event (broadcast every simulation iteration)
    impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&SkidSteerDrivePrivate::OnUpdate, impl_.get(), std::placeholders::_1));
  }

  void SkidSteerDrivePrivate::OnUpdate(const gazebo::common::UpdateInfo &_info)
  {
#ifdef IGN_PROFILER_ENABLE
    IGN_PROFILE("SkidSteerDrivePrivate::OnUpdate");
#endif

    // Update encoder even if we're going to skip this update
    if (odom_source_ == ENCODER)
    {
      UpdateOdometryEncoder(_info.simTime);
    }

    double seconds_since_last_update = (_info.simTime - last_update_time_).Double();

    if (seconds_since_last_update < update_period_)
    {
      return;
    }

#ifdef IGN_PROFILER_ENABLE
    IGN_PROFILE_BEGIN("UpdateOdometryWorld");
#endif
    // Update odom message if using ground truth
    if (odom_source_ == WORLD)
    {
      UpdateOdometryWorld();
    }
#ifdef IGN_PROFILER_ENABLE
    IGN_PROFILE_END();
    IGN_PROFILE_BEGIN("PublishOdometryMsg");
#endif
    if (publish_odom_)
    {
      PublishOdometryMsg(_info.simTime);
    }
#ifdef IGN_PROFILER_ENABLE
    IGN_PROFILE_END();
    IGN_PROFILE_BEGIN("PublishWheelsTf");
#endif
    if (publish_wheel_tf_)
    {
      PublishWheelsTf(_info.simTime);
    }
#ifdef IGN_PROFILER_ENABLE
    IGN_PROFILE_END();
    IGN_PROFILE_BEGIN("PublishOdometryTf");
#endif
    if (publish_odom_tf_)
    {
      PublishOdometryTf(_info.simTime);
    }

#ifdef IGN_PROFILER_ENABLE
      IGN_PROFILE_END();
      IGN_PROFILE_BEGIN("UpdateWheelVelocities");
#endif
      // Update robot in case new velocities have been requested
      UpdateWheelVelocities();
#ifdef IGN_PROFILER_ENABLE
      IGN_PROFILE_END();
#endif
      // current speed of wheels
      std::vector<double> current_speed(num_wheel_);

      for (unsigned int i = 0; i < num_wheel_; i++)
      {
        current_speed[i] = joints_[i]->GetVelocity(0) * (wheel_radius_);
        // current_speed[2*i+LEFT] = joints_[2*i+LEFT]->GetVelocity(0); // in [rad/s]
      }

      // If max_accel == 0, or target speed is reached
      for (unsigned int i = 0; i < num_wheel_; i++)
      {
        // when max_accel == 0 or if the wheel velocity reaches its desired value
        if (max_wheel_accel_ == 0 || (fabs(desired_wheel_speed_[i] - current_speed[i]) < 0.01))
        {
          joints_[i]->SetParam("vel", 0, desired_wheel_speed_[i] / (wheel_radius_));
        }
        else
        {
          if (desired_wheel_speed_[i] >= current_speed[i])
          {
            wheel_speed_instr_[i] += fmin(desired_wheel_speed_[i] - current_speed[i], max_wheel_accel_ * seconds_since_last_update);
          }
          else
          {
            wheel_speed_instr_[i] += fmax(desired_wheel_speed_[i] - current_speed[i], -max_wheel_accel_ * seconds_since_last_update);
          }

          joints_[i]->SetParam("vel", 0, wheel_speed_instr_[i] / (wheel_radius_));
        }
      }

      last_update_time_ = _info.simTime;
    }

    // Robot wheel kinematics constrain implementation
    void SkidSteerDrivePrivate::UpdateWheelVelocities()
    {
      // kinematics (with desired wheel velocities)
      std::lock_guard<std::mutex> scoped_lock(lock_);

      double u = target_x_;
      double v = target_y_;
      double r = target_rot_;

      /*wheel_sep_length --> l
        wheel_seperation_ --> d
        wheel_radius_ --> a   */

      // Left Front wheel
      desired_wheel_speed_[0] = (u - r * wheel_separation_) / wheel_radius_; // in m/s
      // Right Front Wheel
      desired_wheel_speed_[1] = (u + r * wheel_separation_) / wheel_radius_; // in m/s
      // Left Back Wheel
      desired_wheel_speed_[2] = (u - r * wheel_separation_) / wheel_radius_; // in m/s
      // Right Back Wheel
      desired_wheel_speed_[3] = (u + r * wheel_separation_) / wheel_radius_; // in m/s
    }

    // cmd_vel reciever function
    void SkidSteerDrivePrivate::OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr _msg)
    {
      std::lock_guard<std::mutex> scoped_lock(lock_);
      target_x_ = _msg->linear.x;
      target_y_ = _msg->linear.y;
      target_rot_ = _msg->angular.z;
    }

    // Reset function
    void SkidSteerDrive::Reset()
    {
      impl_->last_update_time_ = impl_->joints_[3]->GetWorld()->SimTime();

      for (unsigned int i = 0; i < impl_->num_wheel_pairs_; i++)
      {
        if (impl_->joints_[i])
        {
          impl_->joints_[i]->SetParam("fmax", 0, impl_->max_wheel_torque_);
        }

        impl_->target_x_ = 0.0;
        impl_->target_y_ = 0.0;
        impl_->target_rot_ = 0.0;
      }
    }

    void SkidSteerDrivePrivate::UpdateOdometryEncoder(const gazebo::common::Time &_current_time)
    {
      double vfl = joints_[0]->GetVelocity(0);
      double vfr = joints_[1]->GetVelocity(0);
      double vbl = joints_[2]->GetVelocity(0);
      double vbr = joints_[3]->GetVelocity(0);

      double seconds_since_last_update = (_current_time - last_encoder_update_).Double();
      last_encoder_update_ = _current_time;

      double b = wheel_separation_;

      // Book: Sigwart 2011 Autonompus Mobile Robots page:337
      double sfl = vfl * (wheel_radius_)*seconds_since_last_update;
      double sfr = vfr * (wheel_radius_)*seconds_since_last_update;
      double sbl = vbl * (wheel_radius_)*seconds_since_last_update;
      double sbr = vbr * (wheel_radius_)*seconds_since_last_update;

      // front left and front right have different values of distance moved
      // front and back wheels on same side have same value of distance moved
      double ssum = sfl + sfr;

      double sdiff = sfr - sfl;

      double dx = (ssum) / 2.0 * cos(pose_encoder_.theta + (sdiff) / (2.0 * b));
      double dy = (ssum) / 2.0 * sin(pose_encoder_.theta + (sdiff) / (2.0 * b));
      double dtheta = (sdiff) / b;

      pose_encoder_.x += dx;
      pose_encoder_.y += dy;
      pose_encoder_.theta += dtheta;

      double w = dtheta / seconds_since_last_update;
      double v = sqrt(dx * dx + dy * dy) / seconds_since_last_update;

      tf2::Quaternion qt;
      tf2::Vector3 vt;
      qt.setRPY(0, 0, pose_encoder_.theta);
      vt = tf2::Vector3(pose_encoder_.x, pose_encoder_.y, 0);

      odom_.pose.pose.position.x = vt.x();
      odom_.pose.pose.position.y = vt.y();
      odom_.pose.pose.position.z = vt.z();

      odom_.pose.pose.orientation.x = qt.x();
      odom_.pose.pose.orientation.y = qt.y();
      odom_.pose.pose.orientation.z = qt.z();
      odom_.pose.pose.orientation.w = qt.w();

      odom_.twist.twist.angular.z = w;
      odom_.twist.twist.linear.x = v;
      odom_.twist.twist.linear.y = 0;
    }

    void SkidSteerDrivePrivate::UpdateOdometryWorld()
    {
      auto pose = model_->WorldPose();
      odom_.pose.pose.position = gazebo_ros::Convert<geometry_msgs::msg::Point>(pose.Pos());
      odom_.pose.pose.orientation = gazebo_ros::Convert<geometry_msgs::msg::Quaternion>(pose.Rot());

      // Get velocity in odom frame
      auto linear = model_->WorldLinearVel();
      odom_.twist.twist.angular.z = model_->WorldAngularVel().Z();

      // Convert velocity to child_frame_id(aka base_footprint)
      float yaw = pose.Rot().Yaw();
      odom_.twist.twist.linear.x = cosf(yaw) * linear.X() + sinf(yaw) * linear.Y();
      odom_.twist.twist.linear.y = cosf(yaw) * linear.Y() - sinf(yaw) * linear.X();
    }

    void SkidSteerDrivePrivate::PublishOdometryTf(const gazebo::common::Time &_current_time)
    {
      geometry_msgs::msg::TransformStamped msg;
      msg.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(_current_time);
      msg.header.frame_id = odometry_frame_;
      msg.child_frame_id = robot_base_frame_;
      msg.transform.translation =
          gazebo_ros::Convert<geometry_msgs::msg::Vector3>(odom_.pose.pose.position);
      msg.transform.rotation = odom_.pose.pose.orientation;

      transform_broadcaster_->sendTransform(msg);
    }

    void SkidSteerDrivePrivate::PublishWheelsTf(const gazebo::common::Time &_current_time)
    {
      for (unsigned int i = 0; i < num_wheel_; ++i)
      {
        auto pose_wheel = joints_[i]->GetChild()->RelativePose();

        geometry_msgs::msg::TransformStamped msg;
        msg.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(_current_time);
        msg.header.frame_id = joints_[i]->GetParent()->GetName();
        msg.child_frame_id = joints_[i]->GetChild()->GetName();
        msg.transform.translation = gazebo_ros::Convert<geometry_msgs::msg::Vector3>(pose_wheel.Pos());
        msg.transform.rotation = gazebo_ros::Convert<geometry_msgs::msg::Quaternion>(pose_wheel.Rot());

        transform_broadcaster_->sendTransform(msg);
      }
    }

    void SkidSteerDrivePrivate::PublishOdometryMsg(const gazebo::common::Time &_current_time)
    {
      // Set covariance
      odom_.pose.covariance[0] = covariance_[0];
      odom_.pose.covariance[7] = covariance_[1];
      odom_.pose.covariance[14] = 1000000000000.0;
      odom_.pose.covariance[21] = 1000000000000.0;
      odom_.pose.covariance[28] = 1000000000000.0;
      odom_.pose.covariance[35] = covariance_[2];

      odom_.twist.covariance[0] = covariance_[0];
      odom_.twist.covariance[7] = covariance_[1];
      odom_.twist.covariance[14] = 1000000000000.0;
      odom_.twist.covariance[21] = 1000000000000.0;
      odom_.twist.covariance[28] = 1000000000000.0;
      odom_.twist.covariance[35] = covariance_[2];

      // Set header
      odom_.header.frame_id = odometry_frame_;
      odom_.child_frame_id = robot_base_frame_;
      odom_.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(_current_time);

      // Publish
      odometry_pub_->publish(odom_);
    }

    GZ_REGISTER_MODEL_PLUGIN(SkidSteerDrive)
  }