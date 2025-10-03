#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

class SendTrajectoryNode : public rclcpp::Node
{
public:
  SendTrajectoryNode() : Node("send_trajectory")
  {
    // Setting parameter to get robot description
    this->declare_parameter("robot_description", rclcpp::ParameterType::PARAMETER_STRING);
    this->get_parameter("robot_description", robot_param_);
    robot_description_ = robot_param_.as_string();

    // Create publisher
    pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/r6bot_controller/joint_trajectory", 10);

    // Initialize kinematics
    if (!initializeKinematics()) {
      RCLCPP_ERROR(get_logger(), "Failed to initialize kinematics");
      return;
    }

    // Initialize timer
    timer_ = this->create_wall_timer(
      std::chrono::seconds(5), std::bind(&SendTrajectoryNode::generateAndPublishTrajectory, this));
  }

private:
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_;
  rclcpp::Parameter robot_param_;
  std::string robot_description_;

  rclcpp::TimerBase::SharedPtr timer_;

  KDL::Tree robot_tree_;
  KDL::Chain chain_;
  KDL::JntArray joint_positions_;
  KDL::JntArray joint_velocities_;
  KDL::Twist target_twist_;

  std::shared_ptr<KDL::ChainIkSolverVel_pinv> ik_vel_solver_;

  bool initializeKinematics()
  {
    // Create kinematic chain
    kdl_parser::treeFromString(robot_description_, robot_tree_);
    robot_tree_.getChain("base_link", "tool0", chain_);
    joint_positions_.resize(chain_.getNrOfJoints());
    joint_velocities_.resize(chain_.getNrOfJoints());

    // Create solver
    ik_vel_solver_ = std::make_shared<KDL::ChainIkSolverVel_pinv>(chain_, 1e-7);

    RCLCPP_INFO(get_logger(), "Kinematics initialized");

    return true;
  }

  void generateAndPublishTrajectory()
  {
    trajectory_msgs::msg::JointTrajectory trajectory_msg;
    trajectory_msg.header.stamp = this->now();

    // Populate joint names
    for (unsigned int i = 0; i < chain_.getNrOfSegments(); i++) {
      auto joint = chain_.getSegment(i).getJoint();
      if (joint.getType() != KDL::Joint::Fixed) {
        trajectory_msg.joint_names.push_back(joint.getName());        
      }
    }

    trajectory_msgs::msg::JointTrajectoryPoint trajectory_point_msg;
    trajectory_point_msg.positions.resize(chain_.getNrOfJoints());
    trajectory_point_msg.velocities.resize(chain_.getNrOfJoints());

    // Set number of points
    double total_time = 3.0;
    int trajectory_len = 200;
    double dt = total_time / static_cast<double>(trajectory_len - 1);

    // Generate trajectory points
    for (int i = 0; i < trajectory_len; i++) {
      double t = i / static_cast<double>(trajectory_len - 1);

      // Set endpoint twist (circular motion in XY plane)
      target_twist_.vel.x(2 * 0.3 * cos(2 * M_PI * t));
      target_twist_.vel.y(-0.3 * sin(2 * M_PI * t));
      target_twist_.vel.z(0.0);
      target_twist_.rot.x(0.0);
      target_twist_.rot.y(0.0);
      target_twist_.rot.z(0.0);

      // Convert cartesian to joint velocities
      ik_vel_solver_->CartToJnt(joint_positions_, target_twist_, joint_velocities_);

      // Copy joint data to trajectory_point_msg
      std::memcpy(
        trajectory_point_msg.positions.data(), joint_positions_.data.data(),
        trajectory_point_msg.positions.size() * sizeof(double)
      );
      std::memcpy(
        trajectory_point_msg.velocities.data(), joint_velocities_.data.data(),
        trajectory_point_msg.velocities.size() * sizeof(double)
      );

      // Integrate joint velocities
      joint_positions_.data += joint_velocities_.data * dt;

      // Set timing information using rclcpp::Duration
      double time_point = total_time * t;
      trajectory_point_msg.time_from_start = rclcpp::Duration::from_seconds(time_point);
      trajectory_msg.points.push_back(trajectory_point_msg);      
    }

    // Set the final velocity to zero to stop the robot smoothly
    auto & last_point_msg = trajectory_msg.points.back();
    std::fill(last_point_msg.velocities.begin(), last_point_msg.velocities.end(), 0.0);

    // Publish trajectory
    pub_->publish(trajectory_msg);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SendTrajectoryNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}