#include <ros/ros.h>
#include <memory>
#include <fstream>
// MoveitCpp
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <sstream>

using namespace std;
namespace rvt = rviz_visual_tools;

void drawRvizStuff(ros::NodeHandle nh,double robot_base_heigth, double table_base_heigth);

int main(int argc, char** argv)
{
  ros::init(argc, argv, "kr90_moving");
  ros::NodeHandle nh("/kr90_moving");
  ros::AsyncSpinner spinner(4);
  spinner.start();

  //{{{{{{{{{{{{{{{{{{{{{{{{}}}}}}}}}}}}}}}}}}}}}}}}
  /* Initialize kr90 base and kp2 substract plate as markers*/
  //TODO: Include the base and substract in the URDFs
  double robot_base_heigth = 0.5;
  double table_base_heigth = 0.01;
  
  //{{{{{{{{{{{{{{{{}}}}}}}}}}}}}}}}
  // Initialize moveit variables

  static const std::string PLANNING_GROUP = "kr90_arm";
  static const std::string LOGNAME = "kr90_moving";
  /* Otherwise robot with zeros joint_states */
  ros::Duration(5.0).sleep(); //too long?
  
  drawRvizStuff(nh,robot_base_heigth,table_base_heigth);//draw robots bases

  ROS_INFO_STREAM_NAMED(LOGNAME, "Starting Planning objects...");

  auto moveit_cpp_ptr = std::make_shared<moveit::planning_interface::MoveItCpp>(nh);
  auto planning_components =
      std::make_shared<moveit::planning_interface::PlanningComponent>(PLANNING_GROUP, moveit_cpp_ptr);
  auto robot_model_ptr = moveit_cpp_ptr->getRobotModel();
  auto robot_start_state = planning_components->getStartState();
  auto joint_model_group_ptr = robot_model_ptr->getJointModelGroup(PLANNING_GROUP);
  const vector<string>& joint_names = joint_model_group_ptr->getVariableNames();
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group.getPlanningFrame().c_str());
  
  ifstream inFile; //for waamplanner file
  
  // {{{{{{{{{{{{{{{}}}}}}}}}}}}}}}
  // Initialize visualization variables
  moveit_visual_tools::MoveItVisualTools visual_tools("world", rvt::RVIZ_MARKER_TOPIC,
                                                      moveit_cpp_ptr->getPlanningSceneMonitor());
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl(); //Enable Rviz buttons
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 2.0+robot_base_heigth; 
  visual_tools.publishText(text_pose, "MoveIt AM", rvt::WHITE, rvt::XLARGE);//just puts a text on rviz screen
  visual_tools.trigger(); //draws what needs to be drawn


  // {{{{{{{{{{{{{{{}}}}}}}}}}}}}}}
  // PLanning first point (to the positioner table center)
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the planning");
  planning_components->setStartStateToCurrentState();

  // The first way to set the goal of the plan is by using geometry_msgs::PoseStamped ROS message type as follow
  geometry_msgs::PoseStamped target_pose1;
  geometry_msgs::Pose target_pose_way;
  target_pose1.header.frame_id = "world";
  target_pose1.pose.orientation.w=0.34202;
  target_pose1.pose.orientation.x=0; 
  target_pose1.pose.orientation.y = 0.93969;
  target_pose1.pose.orientation.z = 0;
  target_pose1.pose.position.x = 1.73;
  target_pose1.pose.position.y = 0.5;
  target_pose1.pose.position.z = 1.09;
  planning_components->setGoal(target_pose1, "tool0");

  // Just planning
  auto plan_solution1 = planning_components->plan();

  // Check if PlanningComponents succeeded in finding the plan
  if (plan_solution1)
  {
    // Visualize the start pose in Rviz
    visual_tools.publishAxisLabeled(robot_start_state->getGlobalLinkTransform("tool0"), "start_pose");
    visual_tools.publishText(text_pose, "Start Pose", rvt::WHITE, rvt::XLARGE);
    // Visualize the goal pose in Rviz
    visual_tools.publishAxisLabeled(target_pose1.pose, "target_pose");
    visual_tools.publishText(text_pose, "Goal Pose", rvt::WHITE, rvt::XLARGE);
    // Visualize the trajectory in Rviz 
    visual_tools.publishTrajectoryLine(plan_solution1.trajectory, joint_model_group_ptr);
    visual_tools.trigger();
    //TODO: Show this trajectory for the end-effector
    visual_tools.prompt("Press 'next' to follow the planned trajectory");

    planning_components->execute(); // Execute the plan
    visual_tools.trigger();
  }

visual_tools.prompt("Press 'next' to create the way points");
move_group.setStartStateToCurrentState();
//Creating the Trajectory with the waypoints
vector<geometry_msgs::Pose> waypoints;
vector<double> cartesianPositionTemp{0.0,0.0,0.0};
inFile.open("/home/fernando/Documents/Arquivos WAAM Planner/Basic1/0001.cpi");

//{{{{{{{{{{{{{}}}}}}}}}}}}}
/*Readin waamplanner file*/
if (!inFile) {
    ROS_INFO_STREAM_NAMED(LOGNAME,"Unable to open file");
    exit(1);   // call system to stop
} else if (inFile.is_open())
  { string line;
    while ( getline(inFile,line) )
    {
      vector<double> cartesianPosition;
      int i=0;
      std::string delimiter= " ";
      std::string token;
      while (i <=2) {
        token = line.substr(0, line.find(delimiter));
        cartesianPosition.push_back(stod(token)*0.001);
        line.erase(0, line.find(delimiter) + delimiter.length());
        i++;
      }
      /*The points consider the frame origin to be the touch sense calibrated
      position, so they need to be changed considering the /world origin*/
      transform(cartesianPosition.begin(), cartesianPosition.end(), cartesianPositionTemp.begin(), cartesianPositionTemp.begin(), minus<double>());//cartesianPosition = cartesianPosition-cartesianPositionTemp;
      target_pose1.pose.position.x += cartesianPositionTemp[0];
      target_pose1.pose.position.y += cartesianPositionTemp[1];
      target_pose1.pose.position.z += cartesianPositionTemp[2];
      cartesianPositionTemp = cartesianPosition;
      waypoints.push_back(target_pose1.pose);
    }
    
    inFile.close();
  }

move_group.setMaxVelocityScalingFactor(0.1);
moveit_msgs::RobotTrajectory kr90Trajectory;
const double jump_threshold = 0.0;
const double eef_step = 0.1;
double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, kr90Trajectory);
visual_tools.deleteAllMarkers();
visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::XXSMALL);
ROS_INFO_NAMED("tutorial", "Visualizing plan cartesian path (%.2f%% acheived)", fraction * 100.0);
visual_tools.trigger();

//{{{{{{{{{{{{{{{{{{{{{{}}}}}}}}}}}}}}}}}}}}}}
//Execute new trajectory
visual_tools.prompt("Press 'next' to follow the path");
ROS_INFO_STREAM("cartesian path .....");
moveit::planning_interface::MoveGroupInterface::Plan my_plan;
my_plan.trajectory_= kr90Trajectory;
move_group.execute(my_plan);
visual_tools.trigger();

// END
visual_tools.deleteAllMarkers();
visual_tools.prompt("Press 'next' to end the path generator");
ROS_INFO_STREAM_NAMED(LOGNAME, "Shutting down.");
ros::waitForShutdown();
}


void drawRvizStuff(ros::NodeHandle nh,double robot_base_heigth, double table_base_heigth)
{
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("/visualization_marker", 1);
  
  uint32_t shape_robot_base = visualization_msgs::Marker::CYLINDER;
  uint32_t shape_table_base = visualization_msgs::Marker::CUBE;
  uint32_t shape_table_plate = visualization_msgs::Marker::CYLINDER;
  
  std_msgs::ColorRGBA black;
  black.a=1.0f;
  black.r=0.1f;
  black.g=0.1f;
  black.b=0.1f;

  std_msgs::ColorRGBA gray;
  gray.a=1.0f;
  gray.r=0.4f;
  gray.g=0.4f;
  gray.b=0.4f;

  visualization_msgs::Marker robot_base;
  visualization_msgs::Marker table_base;
  visualization_msgs::Marker table_plate;
  
  robot_base.header.frame_id = "/world";
  robot_base.header.stamp = ros::Time::now();
  robot_base.ns="basic_shapes";
  robot_base.id=0;
  robot_base.type=shape_robot_base;
  robot_base.action = visualization_msgs::Marker::ADD;
  robot_base.pose.position.x = 0;
  robot_base.pose.position.y = 0;
  robot_base.pose.position.z = robot_base_heigth/2;
  robot_base.pose.orientation.x = 0.0;
  robot_base.pose.orientation.y = 0.0;
  robot_base.pose.orientation.z = 0.0;
  robot_base.pose.orientation.w = 1.0;
  robot_base.scale.x=1.2;
  robot_base.scale.y=1.2;
  robot_base.scale.z=robot_base_heigth;
  robot_base.color=black;
  robot_base.lifetime = ros::Duration();

  table_base.header.frame_id = "/world";
  table_base.header.stamp = ros::Time::now();
  table_base.ns="basic_shapes";
  table_base.id=1;//so it does not erase the previous marker
  table_base.type=shape_table_base;
  table_base.action = visualization_msgs::Marker::ADD;
  table_base.pose.position.x = 1.725;
  table_base.pose.position.y = 0.4735;
  table_base.pose.position.z = table_base_heigth*0.001/2;
  table_base.pose.orientation.x = 0;
  table_base.pose.orientation.y = 0;
  table_base.pose.orientation.z = 0;
  table_base.pose.orientation.w = 1.0;
  table_base.scale.x=1.2;
  table_base.scale.y=1.2;
  table_base.scale.z=table_base_heigth*0.001;//does not have that base
  table_base.color=black;
  table_base.lifetime = ros::Duration();

  table_plate.header.frame_id = "/world";
  table_plate.header.stamp = ros::Time::now();
  table_plate.ns="basic_shapes";
  table_plate.id=2;//so it does not erase the previous marker
  table_plate.type=shape_table_plate;
  table_plate.action = visualization_msgs::Marker::ADD;
  table_plate.pose.position.x = 1.73;
  table_plate.pose.position.y = 0.5;
  table_plate.pose.position.z = 1.0876;
  table_plate.pose.orientation.x = 0;
  table_plate.pose.orientation.y = 0;
  table_plate.pose.orientation.z = 0;
  table_plate.pose.orientation.w = 1.0;
  table_plate.scale.x=1;
  table_plate.scale.y=1;
  table_plate.scale.z=0.005;
  table_plate.color=gray;
  table_plate.lifetime = ros::Duration();

  int i=0;
  
      while (marker_pub.getNumSubscribers() < 1)
    {
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
      i++;
      if(i==10)
      {
        ROS_WARN_ONCE("crashou");
        break;
      }
    }
  marker_pub.publish(robot_base);
  //marker_pub.publish(table_base);
  marker_pub.publish(table_plate);
}