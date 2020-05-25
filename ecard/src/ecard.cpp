/// ECARD

////////////////////
/// DEPENDENCIES ///
////////////////////

// ROS 2
#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// ROS 2 interfaces
#include <std_srvs/srv/empty.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometric_primitive_msgs/msg/geometric_primitive_stamped.hpp>

// MoveIt
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

// Eigen
#include <Eigen/Geometry>

//////////////////
/// NAMESPACES ///
//////////////////

using namespace std::chrono_literals;
using geometric_primitive_msgs::msg::GeometricPrimitive;

///////////////
/// DEFINES ///
///////////////

// If defined, the parameters of gaze correlation node will be changed according to the next action
// #define INFLUENCE_GAZE_CORRELATION_MODE

/////////////
/// TYPES ///
/////////////

/// Policy of the synchronizer
typedef message_filters::sync_policies::ExactTime<geometry_msgs::msg::PointStamped,
                                                  geometric_primitive_msgs::msg::GeometricPrimitiveStamped>
    synchronizer_policy;

/////////////////
/// CONSTANTS ///
/////////////////

/// The name of this (ECARD) node
const std::string NODE_NAME = "ecard";
/// The name of node responsible for moveit
const std::string NODE_NAME_MOVEIT = "ecard_moveit";
/// The name of gaze correlation node
const std::string NODE_NAME_GAZE_CORRELATION = "gaze_correlation";
/// Size of the queue size used by the synchronizer in its policy
const uint8_t SYNCHRONIZER_QUEUE_SIZE = 50;

/// Distance between end effector flange and fingertips, in metres
const double FLANGE_TO_FINGERTIP = 0.05;

const bool visualise_trajectories = false;

//////////////////////
/// CLASS - MoveIt ///
//////////////////////

class EcardMoveIt : public rclcpp::Node
{
public:
  /// Constructor
  EcardMoveIt();

  /// Publisher of the trajectories
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_publisher_;
  /// Publisher of the robot state
  rclcpp::Publisher<moveit_msgs::msg::DisplayRobotState>::SharedPtr robot_state_publisher_;

  /// Interface for MoveIt2
  moveit::planning_interface::MoveItCppPtr moveit_interface_;
  /// Planning component for manipulator
  moveit::planning_interface::PlanningComponentPtr arm_;

  bool move_to_named_target(const std::string &named_target);
  bool move_to_pose(const geometry_msgs::msg::PoseStamped &pose);
  bool execute(bool wait_until_finished = true);
  void visualize_trajectory(const robot_trajectory::RobotTrajectory &trajectory);
  bool grasp();
  bool release();
  void add_attached_collision_object(const geometric_primitive_msgs::msg::GeometricPrimitiveStamped &object);
  void detach_collision_objects();
  void remove_collision_object(const std::string &id);
};

EcardMoveIt::EcardMoveIt() : Node(NODE_NAME_MOVEIT, rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)),
                             trajectory_publisher_(this->create_publisher<trajectory_msgs::msg::JointTrajectory>("fake_joint_trajectory_controller/joint_trajectory", 1)),
                             robot_state_publisher_(this->create_publisher<moveit_msgs::msg::DisplayRobotState>("display_robot_state", 1))
{
  // Setup MoveIt
  moveit_interface_ = std::make_shared<moveit::planning_interface::MoveItCpp>(std::shared_ptr<rclcpp::Node>(std::move(this)));
  moveit_interface_->getPlanningSceneMonitor()->setPlanningScenePublishingFrequency(100);
  arm_ = std::make_shared<moveit::planning_interface::PlanningComponent>("panda_arm", moveit_interface_);
}

bool EcardMoveIt::move_to_named_target(const std::string &named_target)
{
  RCLCPP_INFO(this->get_logger(), "Setting goal to named target \"" + named_target + "\"");
  arm_->setGoal(named_target);
  return execute();
}

bool EcardMoveIt::move_to_pose(const geometry_msgs::msg::PoseStamped &pose)
{
  RCLCPP_INFO(this->get_logger(), "Setting goal to custom pose");
  arm_->setGoal(pose, "panda_hand");
  return execute();
}

bool EcardMoveIt::execute(bool wait_until_finished)
{
  const auto plan_solution = arm_->plan();
  if (plan_solution)
  {
    RCLCPP_INFO(this->get_logger(), "Planning successful, executing ...");

    if (visualise_trajectories)
    {
      visualize_trajectory(*plan_solution.trajectory);
    }
    moveit_msgs::msg::RobotTrajectory robot_trajectory;
    plan_solution.trajectory->getRobotTrajectoryMsg(robot_trajectory);
    trajectory_publisher_->publish(robot_trajectory.joint_trajectory);

    // Wait until execution ends, if desired
    if (wait_until_finished)
    {
      auto duration = rclcpp::Duration::from_seconds(plan_solution.trajectory->getDuration());
      rclcpp::sleep_for(std::chrono::nanoseconds(duration.nanoseconds()));
    }

    RCLCPP_INFO(this->get_logger(), "Motion successful");
    return true;
  }
  else
  {
    RCLCPP_WARN(this->get_logger(), "Planning failed");
    return false;
  }
}

void EcardMoveIt::visualize_trajectory(const robot_trajectory::RobotTrajectory &trajectory)
{
  moveit_msgs::msg::DisplayRobotState waypoint;
  const auto start_time = this->now();
  for (size_t i = 0; i < trajectory.getWayPointCount(); ++i)
  {
    moveit::core::robotStateToRobotStateMsg(trajectory.getWayPoint(i), waypoint.state);
    const auto waypoint_time = start_time + rclcpp::Duration::from_seconds(trajectory.getWayPointDurationFromStart(i));
    const auto now = this->now();
    if (waypoint_time > now)
    {
      rclcpp::sleep_for(std::chrono::nanoseconds((waypoint_time - now).nanoseconds()));
    }
    robot_state_publisher_->publish(waypoint);
  }
}

void EcardMoveIt::add_attached_collision_object(const geometric_primitive_msgs::msg::GeometricPrimitiveStamped &object)
{
  moveit_msgs::msg::AttachedCollisionObject attached_collision_object;
  attached_collision_object.link_name = "panda_hand";
  // TODO: touch_links not working as intended
  attached_collision_object.touch_links.push_back("panda_hand");
  attached_collision_object.touch_links.push_back("panda_leftfinger");
  attached_collision_object.touch_links.push_back("panda_rightfinger");
  attached_collision_object.touch_links.push_back("panda_link7");

  attached_collision_object.object.header = object.header;
  attached_collision_object.object.operation = attached_collision_object.object.ADD;

  switch (object.primitive.type)
  {
  case GeometricPrimitive::PLANE:
    return;

  case GeometricPrimitive::SPHERE:
  {
    auto sphere = object.primitive.sphere[0];
    attached_collision_object.object.id = std::to_string(sphere.id);

    shape_msgs::msg::SolidPrimitive collision_primitive;
    collision_primitive.type = collision_primitive.SPHERE;
    collision_primitive.dimensions.resize(1);
    collision_primitive.dimensions[collision_primitive.SPHERE_RADIUS] = sphere.radius;
    attached_collision_object.object.primitives.push_back(collision_primitive);

    geometry_msgs::msg::Pose collision_primitive_pose;
    collision_primitive_pose.position = sphere.centre;
    attached_collision_object.object.primitive_poses.push_back(collision_primitive_pose);
  }
  break;

  case GeometricPrimitive::CYLINDER:
  {
    auto cylinder = object.primitive.cylinder[0];
    attached_collision_object.object.id = std::to_string(cylinder.id);

    shape_msgs::msg::SolidPrimitive collision_primitive;
    collision_primitive.type = collision_primitive.CYLINDER;
    collision_primitive.dimensions.resize(2);
    collision_primitive.dimensions[collision_primitive.CYLINDER_RADIUS] = cylinder.radius;
    collision_primitive.dimensions[collision_primitive.CYLINDER_HEIGHT] = cylinder.height;
    attached_collision_object.object.primitives.push_back(collision_primitive);

    attached_collision_object.object.primitive_poses.push_back(cylinder.pose);
  }
  break;
  }

  // Add object to planning scene
  { // Lock PlanningScene
    planning_scene_monitor::LockedPlanningSceneRW scene(moveit_interface_->getPlanningSceneMonitor());
    scene->processAttachedCollisionObjectMsg(attached_collision_object);
  } // Unlock PlanningScene
}

void EcardMoveIt::detach_collision_objects()
{
  moveit_msgs::msg::AttachedCollisionObject attached_collision_object;
  attached_collision_object.object.operation = attached_collision_object.object.REMOVE;

  // Add object to planning scene
  { // Lock PlanningScene
    planning_scene_monitor::LockedPlanningSceneRW scene(moveit_interface_->getPlanningSceneMonitor());
    scene->processAttachedCollisionObjectMsg(attached_collision_object);
  } // Unlock PlanningScene
}

void EcardMoveIt::remove_collision_object(const std::string &id)
{
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.operation = collision_object.REMOVE;
  collision_object.id = id;

  // Add object to planning scene
  { // Lock PlanningScene
    planning_scene_monitor::LockedPlanningSceneRW scene(moveit_interface_->getPlanningSceneMonitor());
    scene->processCollisionObjectMsg(collision_object);
  } // Unlock PlanningScene
}

/////////////////////
/// CLASS - ECARD ///
/////////////////////

class Ecard : public rclcpp::Node
{
public:
  /// Constructor
  Ecard(std::shared_ptr<EcardMoveIt> &ecard_moveit);

private:
  /// Subscriber to point of gaze
  message_filters::Subscriber<geometry_msgs::msg::PointStamped> sub_point_of_gaze_;
  /// Subscriber to object of interest
  message_filters::Subscriber<geometric_primitive_msgs::msg::GeometricPrimitiveStamped> sub_object_of_interest_;
  /// Synchronizer of the subscribers
  message_filters::Synchronizer<synchronizer_policy> synchronizer_;

  /// Service for triggering actions
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr trigger_action_service_;
  /// Flag that determines whether to trigger an action on the next synchronized callback
  bool trigger_action_;

  /// Pointer to node handling interfacing with MoveIt2
  std::shared_ptr<EcardMoveIt> ecard_moveit2_;

  /// Flag that determines the next action - either "pick" or "place"
  std::string next_action_;
  /// Determines what kind of object was picked up last time
  geometric_primitive_msgs::msg::GeometricPrimitiveStamped grasped_object_;
  /// Pointer to the ID of the grasped object
  std::string grasped_object_id_;

  /// Buffer for tf2 transforms
  tf2_ros::Buffer tf2_buffer_;
  /// Listener of tf2 transforms
  tf2_ros::TransformListener tf2_listener_;

#ifdef INFLUENCE_GAZE_CORRELATION_MODE
  rclcpp::SyncParametersClient::SharedPtr parameters_client_;
#endif

  void handle_trigger_action_service(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<std_srvs::srv::Empty::Request> request,
      const std::shared_ptr<std_srvs::srv::Empty::Response> response);
  /// Callback called each time a message is received on all topics
  void synchronized_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg_point_of_gaze,
                             const geometric_primitive_msgs::msg::GeometricPrimitiveStamped::SharedPtr msg_object_of_interest);
  bool pick(const geometry_msgs::msg::PointStamped::SharedPtr msg_point_of_gaze,
            const geometric_primitive_msgs::msg::GeometricPrimitiveStamped::SharedPtr msg_object_of_interest);
  bool place(const geometry_msgs::msg::PointStamped::SharedPtr msg_point_of_gaze,
             const geometric_primitive_msgs::msg::GeometricPrimitiveStamped::SharedPtr msg_object_of_interest);
  bool setup_correlation_for_picking();
  bool setup_correlation_for_placing();
  geometry_msgs::msg::PoseStamped transform_to_robot_base(const geometry_msgs::msg::PoseStamped &pose);
};

Ecard::Ecard(std::shared_ptr<EcardMoveIt> &ecard_moveit) : Node(NODE_NAME),
                                                           ecard_moveit2_(ecard_moveit),
                                                           sub_point_of_gaze_(this, "point_of_gaze"),
                                                           sub_object_of_interest_(this, "object_of_interest"),
                                                           synchronizer_(synchronizer_policy(SYNCHRONIZER_QUEUE_SIZE), sub_point_of_gaze_, sub_object_of_interest_),
                                                           tf2_buffer_(this->get_clock()),
                                                           tf2_listener_(tf2_buffer_),
                                                           trigger_action_(false),
                                                           next_action_("pick")
{
  // Move home (wait until RViz is ready)
  rclcpp::sleep_for(10s);
  ecard_moveit2_->move_to_named_target("home");

#ifdef INFLUENCE_GAZE_CORRELATION_MODE
  parameters_client_ = std::make_shared<rclcpp::SyncParametersClient>(this, NODE_NAME_GAZE_CORRELATION);
  // Make sure volumetric objects are correlated first for picking
  setup_correlation_for_picking();
#endif

  // Register a service that triggest pick or place actions
  trigger_action_service_ = this->create_service<std_srvs::srv::Empty>("trigger_action",
                                                                       std::bind(&Ecard::handle_trigger_action_service, this,
                                                                                 std::placeholders::_1,
                                                                                 std::placeholders::_2,
                                                                                 std::placeholders::_3));

  // Synchronize the subscriptions under a single callback
  synchronizer_.registerCallback(&Ecard::synchronized_callback, this);

  RCLCPP_INFO(this->get_logger(), "Node initialised");
}

void Ecard::handle_trigger_action_service(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    const std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
  (void)request_header;
  (void)request;
  (void)response;
  trigger_action_ = true;
}

void Ecard::synchronized_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg_point_of_gaze,
                                  const geometric_primitive_msgs::msg::GeometricPrimitiveStamped::SharedPtr msg_object_of_interest)
{
  // Continue only if requested by service
  if (!trigger_action_)
  {
    return;
  }
  trigger_action_ = false;

  RCLCPP_DEBUG(this->get_logger(), "Received synchronized messages for processing");

  if (next_action_ == "pick")
  {
    RCLCPP_ERROR(this->get_logger(), "Picking");
    bool result = pick(msg_point_of_gaze, msg_object_of_interest);
    if (result)
    {
#ifdef INFLUENCE_GAZE_CORRELATION_MODE
      setup_correlation_for_placing();
#endif
      next_action_ = "place";
    }
    else
    {
      ecard_moveit2_->detach_collision_objects();
      ecard_moveit2_->remove_collision_object(grasped_object_id_);
      ecard_moveit2_->move_to_named_target("home");
    }
  }
  else if (next_action_ == "place")
  {
    RCLCPP_ERROR(this->get_logger(), "Placing");
    bool result = place(msg_point_of_gaze, msg_object_of_interest);
    if (result)
    {
#ifdef INFLUENCE_GAZE_CORRELATION_MODE
      setup_correlation_for_picking();
#endif
      next_action_ = "pick";
    }
    else
    {
      ecard_moveit2_->detach_collision_objects();
      ecard_moveit2_->remove_collision_object(grasped_object_id_);
      ecard_moveit2_->move_to_named_target("home");
    }
  }
}

bool Ecard::pick(const geometry_msgs::msg::PointStamped::SharedPtr msg_point_of_gaze,
                 const geometric_primitive_msgs::msg::GeometricPrimitiveStamped::SharedPtr msg_object_of_interest)
{
  // Create pose for grasping
  geometry_msgs::msg::PoseStamped pose;
  pose.header = msg_object_of_interest->header;

  std::string post_grasp_named_pose;

  // Determine grasp pose based on the object
  switch (msg_object_of_interest->primitive.type)
  {
  case GeometricPrimitive::PLANE:
    return false;

  case GeometricPrimitive::SPHERE:
  {
    auto sphere = msg_object_of_interest->primitive.sphere[0];
    pose.pose.position = sphere.centre;

    pose = transform_to_robot_base(pose);
    pose.pose.position.z += sphere.radius;
    pose.pose.position.z += FLANGE_TO_FINGERTIP;

    tf2::Quaternion quat;
    quat.setRPY(M_PI, 0, 0);
    pose.pose.orientation.x = quat.x();
    pose.pose.orientation.y = quat.y();
    pose.pose.orientation.z = quat.z();
    pose.pose.orientation.w = quat.w();

    grasped_object_id_ = std::to_string(sphere.id);
    post_grasp_named_pose = "home";
  }
  break;

  case GeometricPrimitive::CYLINDER:
  {
    auto cylinder = msg_object_of_interest->primitive.cylinder[0];
    pose.pose = cylinder.pose;

    pose = transform_to_robot_base(pose);
    pose.pose.position.x -= cylinder.radius;
    pose.pose.position.x -= FLANGE_TO_FINGERTIP;

    tf2::Quaternion quat;
    quat.setRPY(M_PI, -M_PI_2, 0);

    pose.pose.orientation.x = quat.x();
    pose.pose.orientation.y = quat.y();
    pose.pose.orientation.z = quat.z();
    pose.pose.orientation.w = quat.w();

    grasped_object_id_ = std::to_string(cylinder.id);
    post_grasp_named_pose = "home_forward";
  }
  break;
  }

  // Move to grasp position
  bool res = ecard_moveit2_->move_to_pose(pose);

  // Add collision object attached to the end effector
  ecard_moveit2_->add_attached_collision_object(*msg_object_of_interest);

  if (res)
  {
    grasped_object_ = *msg_object_of_interest;
    return ecard_moveit2_->move_to_named_target(post_grasp_named_pose);
  }
  return false;
}

bool Ecard::place(const geometry_msgs::msg::PointStamped::SharedPtr msg_point_of_gaze,
                  const geometric_primitive_msgs::msg::GeometricPrimitiveStamped::SharedPtr msg_object_of_interest)
{
  // Create pose for placing
  geometry_msgs::msg::PoseStamped pose;
  pose.header = msg_point_of_gaze->header;

  // Set the point of gaze as target
  pose.pose.position = msg_point_of_gaze->point;

  // Determine grasp pose based on the object
  switch (grasped_object_.primitive.type)
  {
  case GeometricPrimitive::SPHERE:
  {
    auto sphere = grasped_object_.primitive.sphere[0];

    pose = transform_to_robot_base(pose);
    pose.pose.position.z += sphere.radius;
    pose.pose.position.z += FLANGE_TO_FINGERTIP;

    tf2::Quaternion quat;
    quat.setRPY(M_PI, 0, 0);
    pose.pose.orientation.x = quat.x();
    pose.pose.orientation.y = quat.y();
    pose.pose.orientation.z = quat.z();
    pose.pose.orientation.w = quat.w();
  }
  break;

  case GeometricPrimitive::CYLINDER:
  {
    auto cylinder = grasped_object_.primitive.cylinder[0];

    pose = transform_to_robot_base(pose);
    pose.pose.position.z += cylinder.height / 2.0;
    pose.pose.position.x -= cylinder.radius;
    pose.pose.position.x -= FLANGE_TO_FINGERTIP;

    tf2::Quaternion quat;
    quat.setRPY(M_PI, -M_PI_2, 0);

    pose.pose.orientation.x = quat.x();
    pose.pose.orientation.y = quat.y();
    pose.pose.orientation.z = quat.z();
    pose.pose.orientation.w = quat.w();
  }
  break;
  }

  // Move to place position
  bool res = ecard_moveit2_->move_to_pose(pose);

  // Remove the collision object attached to the end effector
  ecard_moveit2_->detach_collision_objects();

  if (res)
  {
    return ecard_moveit2_->move_to_named_target("home");
  }
  return false;
}

#ifdef INFLUENCE_GAZE_CORRELATION_MODE
bool Ecard::setup_correlation_for_picking()
{
  // Create a parameter client and wait until it is ready
  while (!parameters_client_->wait_for_service(1s))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
      exit(EXIT_FAILURE);
    }
  }

  // Setup parameter change requests
  std::vector<rclcpp::Parameter> parameters;
  parameters.push_back(rclcpp::Parameter("enable.plane", false));
  parameters.push_back(rclcpp::Parameter("enable.sphere", true));
  parameters.push_back(rclcpp::Parameter("enable.cylinder", true));

  // Set the parameters
  std::vector<rcl_interfaces::msg::SetParametersResult> set_parameters_results = parameters_client_->set_parameters(parameters);

  // Make sure it was successful
  for (auto &&result : set_parameters_results)
  {
    if (!result.successful)
    {
      RCLCPP_WARN(this->get_logger(), "Setting properties of " + NODE_NAME_GAZE_CORRELATION + " failed");
      return false;
    }
  }
  return true;
}

bool Ecard::setup_correlation_for_placing()
{
  // Create a parameter client and wait until it is ready
  while (!parameters_client_->wait_for_service(1s))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
      exit(EXIT_FAILURE);
    }
  }

  // Setup parameter change requests
  std::vector<rclcpp::Parameter> parameters;
  parameters.push_back(rclcpp::Parameter("enable.plane", true));
  parameters.push_back(rclcpp::Parameter("enable.sphere", false));
  parameters.push_back(rclcpp::Parameter("enable.cylinder", false));

  // Set the parameters
  std::vector<rcl_interfaces::msg::SetParametersResult> set_parameters_results = parameters_client_->set_parameters(parameters);

  // Make sure it was successful
  for (auto &&result : set_parameters_results)
  {
    if (!result.successful)
    {
      RCLCPP_WARN(this->get_logger(), "Setting properties of " + NODE_NAME_GAZE_CORRELATION + " failed");
      return false;
    }
  }
  return true;
}
#endif

geometry_msgs::msg::PoseStamped Ecard::transform_to_robot_base(const geometry_msgs::msg::PoseStamped &pose)
{
  geometry_msgs::msg::PoseStamped output;
  try
  {
    return tf2_buffer_.transform(pose, output, "panda_link0");
  }
  catch (tf2::TransformException &ex)
  {
    RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
  }
  return output;
}

////////////
/// MAIN ///
////////////

/// Main function that initiates nodes of this process
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  // Create a multi-threaded executor such that MoveIt thread is separated from ECARD specifics
  auto ecard_moveit = std::make_shared<EcardMoveIt>();
  auto ecard = std::make_shared<Ecard>(ecard_moveit);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(ecard_moveit);
  executor.add_node(ecard);
  executor.spin();

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
