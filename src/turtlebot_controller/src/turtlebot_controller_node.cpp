#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#define LINEAR_VELOCITY 0.3
#define ANGULAR_VELOCITY 0.0

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
    MoveBaseClient;

class TurtlebotController {
public:
  TurtlebotController() {
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    scan_sub_ =
        nh_.subscribe("/scan", 10, &TurtlebotController::scanCallback, this);
    move_base_client_.reset(new MoveBaseClient("move_base", true));
    target_distance_ = 0.3;          // Target distance to keep from obstacles
    angular_vel_ = ANGULAR_VELOCITY; // Angular velocity of the robot
    linear_vel_ = LINEAR_VELOCITY;   // Linear velocity of the robot
  }

  void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan) {
    // Find the closest obstacle in front of the robot
    float angle_min = scan->angle_min;
    float angle_increment = scan->angle_increment;
    std::vector<float> ranges = scan->ranges;

    float closest_obstacle = *std::min_element(ranges.begin(), ranges.end());
    float obstacle_angle =
        angle_min +
        (std::distance(ranges.begin(),
                       std::min_element(ranges.begin(), ranges.end())) *
         angle_increment);

    ROS_INFO("Obstacle angle %f , Closest obstacle %f", obstacle_angle,
             closest_obstacle);

    // Calculate the angular velocity based on the obstacle angle
    if (closest_obstacle < target_distance_) {
      // Turn away from the obstacle
      if (obstacle_angle > 0) {
        angular_vel_ = -0.5;
      } else if (obstacle_angle < 0) {
        angular_vel_ = 0.5;
      }
    } else {
      angular_vel_ = ANGULAR_VELOCITY; // Continue moving straight

        ROS_INFO("go strainghr");
    }
  }

  void move() {
    move_base_msgs::MoveBaseGoal goal;

    // Set the goal position and orientation
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.pose.position.x = 1.0;
    goal.target_pose.pose.position.y = 1.0;
    goal.target_pose.pose.orientation.w = 1.0;

    // Send the goal to the move_base action server
    move_base_client_->sendGoal(goal);

    while (ros::ok()) {
      // Create a Twist message and set the linear and angular velocities
      geometry_msgs::Twist cmd_vel;
      cmd_vel.linear.x = linear_vel_;
      cmd_vel.angular.z = angular_vel_;

      // Publish the Twist message
      vel_pub_.publish(cmd_vel);

      // ROS_INFO("Angular vel %f linear vel %f", angular_vel_, linear_vel_);

      // Check if the goal has been reached
      if (move_base_client_->getState() ==
          actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Goal reached!");
        break;
      }

      ros::spinOnce();
    }
  }

private:
  ros::NodeHandle nh_;
  ros::Publisher vel_pub_;
  ros::Subscriber scan_sub_;
  std::unique_ptr<MoveBaseClient> move_base_client_;

  double target_distance_;
  double angular_vel_;
  double linear_vel_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "turtlebot_controller");
  TurtlebotController controller;
  controller.move();
  return 0;
}
