#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/PoseStamped.h>
#include "tough_common/tough_common_names.h"
#include "tough_moveit_planners/taskspace_planner.h"
#include "tough_controller_interface/wholebody_control_interface.h"

//Number of datapoints to average for scale calibration
#define MAX_TPOSE_READS 10

//Values for each axis when ATLAS does a T-pose for calibration
#define ATLAS_RARM_TPOSE_X 0.127
#define ATLAS_RARM_TPOSE_Y 0.88
#define ATLAS_RARM_TPOSE_Z 0.443

#define ATLAS_LARM_TPOSE_X 0.127
#define ATLAS_LARM_TPOSE_Y 0.88
#define ATLAS_LARM_TPOSE_Z 0.443

#define ATLAS_TORSO_TPOSE_X 0
#define ATLAS_TORSO_TPOSE_Y 0
#define ATLAS_TORSO_TPOSE_Z 0

//Max values for ATLAS relation to each axis
#define ATLAS_RARM_MAX_X 1.1
#define ATLAS_RARM_MAX_Y -0.2
#define ATLAS_RARM_MAX_Z 0.9

#define ATLAS_LARM_MAX_X 1.1
#define ATLAS_LARM_MAX_Y 0.88
#define ATLAS_LARM_MAX_Z 0.9

#define ATLAS_TORSO_MAX_X 0
#define ATLAS_TORSO_MAX_Y 0
#define ATLAS_TORSO_MAX_Z 0

//Min values for ATLAS relation to each axis
#define ATLAS_RARM_MIN_X 0.1
#define ATLAS_RARM_MIN_Y -0.88
#define ATLAS_RARM_MIN_Z -0.1

#define ATLAS_LARM_MIN_X 0.1
#define ATLAS_LARM_MIN_Y 0.0
#define ATLAS_LARM_MIN_Z -0.1

#define ATLAS_TORSO_MIN_X 0
#define ATLAS_TORSO_MIN_Y 0
#define ATLAS_TORSO_MIN_Z 0

double rarm_xscale;
double rarm_yscale;
double rarm_zscale;

double larm_xscale;
double larm_yscale;
double larm_zscale;

double torso_xscale;
double torso_yscale;
double torso_zscale;






/*
*@brief Gives boundaries for incoming position values of the indicated axis
*@param min           Minimum value allowed for position
*@param max           Maximum value allowed for position
*@param pose          Pose containing the position in question
*@param axis          What axis to set a boundary for ("x", "y", "z")
*@param return
*/
void capPosition(double min, double max, geometry_msgs::PoseStamped& pose, std::string axis){
  
  if(axis == "x"){
    if(pose.pose.position.x > max){
      pose.pose.position.x = max;
    }
    else if(pose.pose.position.x < min){
      pose.pose.position.x = min;
    }
  }
  else if(axis == "y"){
    if(pose.pose.position.y > max){
      pose.pose.position.y = max;
    }
    else if(pose.pose.position.y < min){
      pose.pose.position.y = min;
    }
  }
  else if(axis == "z"){
    if(pose.pose.position.z > max){
      pose.pose.position.z = max;
    }
    else if(pose.pose.position.z < min){
      pose.pose.position.z = min;
    }
  }
}
 

void transformToPose(const tf::StampedTransform& tf, geometry_msgs::PoseStamped& result){
    
    double xscale;
    double yscale;
    double zscale;
    double xmax;
    double ymax;
    double zmax;
    double xmin;
    double ymin;
    double zmin;
  
    if(tf.child_frame_id_ == "left_palm"){
      xscale = rarm_xscale;
      yscale = rarm_yscale;
      zscale = rarm_zscale;

      xmax = ATLAS_RARM_MAX_X;
      ymax = ATLAS_RARM_MAX_Y;
      zmax = ATLAS_RARM_MAX_Z;

      xmin = ATLAS_RARM_MIN_X;
      ymin = ATLAS_RARM_MIN_Y;
      zmin = ATLAS_RARM_MIN_Z;


    }

    else if(tf.child_frame_id_ == "right_palm"){
      xscale = larm_xscale;
      yscale = larm_yscale;
      zscale = larm_zscale;

      xmax = ATLAS_LARM_MAX_X;
      ymax = ATLAS_LARM_MAX_Y;
      zmax = ATLAS_LARM_MAX_Z;

      xmin = ATLAS_LARM_MIN_X;
      ymin = ATLAS_LARM_MIN_Y;
      zmin = ATLAS_LARM_MIN_Z;

    }

    
    result.header.frame_id = "pelvis";
    result.pose.position.x = tf.getOrigin().getX(); //* xscale;
    result.pose.position.y = tf.getOrigin().getY(); //* yscale;
    result.pose.position.z = tf.getOrigin().getZ(); //* zscale;
    result.pose.orientation.x = 0.0; // tf.getRotation().getX();
    result.pose.orientation.y = 0.0; //tf.getRotation().getY();
    result.pose.orientation.z = 0.0; //tf.getRotation().getZ();
    result.pose.orientation.w = 1.0; //tf.getRotation().getW();
    if (tf.child_frame_id_ != "torso"){
      result.pose.position.y *= -1;
    }
    ROS_INFO("%f : %f : %f", result.pose.position.x,result.pose.position.y,result.pose.position.z);
    capPosition(xmin,xmax,result,"x");
    capPosition(ymin,ymax,result,"y");
    capPosition(zmin,zmax,result,"z");
}


/*
* @brief Updates all past poses to the current poses
* @param old_l_arm                Old pose for left arm                        
* @param old_r_arm                Old pose for right arm
* @param old_torso                Old pose for torso
* @param l_arm                    Current pose for left arm
* @param r_arm                    Current pose for right arm
* @param torso                    Current pose for torso
*
* @return  
*/

void updatePastPoses(geometry_msgs::PoseStamped& old_l_arm, geometry_msgs::PoseStamped& old_r_arm, geometry_msgs::PoseStamped& old_torso, geometry_msgs::PoseStamped& l_arm, geometry_msgs::PoseStamped& r_arm, geometry_msgs::PoseStamped& torso ){
  old_l_arm = l_arm;
  old_r_arm = r_arm;
  old_l_arm = torso;
}

bool isNewPos(double new_position, double old_position, double tolerance){
    double diff = fabs(old_position - new_position);
    if(diff > tolerance){
      return true;
    }
    else{
      return false;
    }
}

/*
* @brief Compares the newest pose to the previous pose to determine if the change is worth planning a trajectory
* @param pose                     Newest pose msg
* @param past_pose                Previous pose msg
* @param tolerance                Minimum difference between current and past pose to be considered worth planning a trajectory
*
* @return  bool                   returns a true if at least one of the positions is different from the previous 
*/
bool isNewGoal(const geometry_msgs::PoseStamped& pose, const geometry_msgs::PoseStamped& past_pose, double tolerance){
  double x_pose = pose.pose.position.x;
  double y_pose = pose.pose.position.y;
  double z_pose = pose.pose.position.z;
  
  double x_past = past_pose.pose.position.x;
  double y_past = past_pose.pose.position.y;
  double z_past = past_pose.pose.position.z;

  if(isNewPos(x_pose,x_past,tolerance) || isNewPos(y_pose,y_past,tolerance) || isNewPos(z_pose,z_past,tolerance)){
    return true;
  }

  else{
    return false;
  }
}


/*
*@brief Adds values their respective sums to be averaged for calibration
*@param tf            Inputed stamped transform with position values to be added
*@param xsum          Current sum of values in the x position
*@param ysum          Current sum of values in the y postiion
*@param zsum          Current sum of values in the z position
*@param avg           Current number of values stored
*
*@return
*/
void addToSum(const tf::StampedTransform& tf, double& xsum, double& ysum, double& zsum, int avg){
    xsum += tf.getOrigin().getX();
    ysum += tf.getOrigin().getY();
    zsum += tf.getOrigin().getZ();
}

/*
*@brief Updates the scale from human limb locations to robot locations
*@param xsum          The total sum of values in the x position
*@param ysum          The total sum of values in the y position
*@param zsum          The total sum of values in the z position
*@param avg           Total number of data points in each sum (stops on number to average by)
*@param type          The body part to update the scales for (rarm, larm, torso)
*@return
*/
void updateScale( double& xsum, double& ysum, double& zsum, int avg, std::string type){
  if(avg + 1 == MAX_TPOSE_READS){ //if the number of readings have been met, average sums and calculate scale
    xsum /= (double)MAX_TPOSE_READS;
    ysum /= (double)MAX_TPOSE_READS;
    zsum /= (double)MAX_TPOSE_READS;

    if(type == "rarm"){
        rarm_xscale = (double)(ATLAS_RARM_TPOSE_X) / xsum;
        rarm_yscale = (double)(ATLAS_RARM_TPOSE_Y) / ysum;
        rarm_zscale = (double)(ATLAS_RARM_TPOSE_Z) / zsum;
    }

    else if(type == "larm"){
      larm_xscale = (double)(ATLAS_LARM_TPOSE_X) / xsum;
      larm_yscale = (double)(ATLAS_LARM_TPOSE_Y) / ysum;
      larm_zscale = (double)(ATLAS_LARM_TPOSE_Z) / zsum;
    }

    else if(type == "torso"){
      torso_xscale = (double)(ATLAS_TORSO_TPOSE_X) / xsum;
      torso_yscale = (double)(ATLAS_TORSO_TPOSE_Y) / ysum;
      torso_zscale = (double)(ATLAS_TORSO_TPOSE_Z) / zsum;
    }

  }
}




int main(int argc, char** argv)
{
  

  ros::init(argc, argv, "shadow_planner");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  tf::TransformListener right_arm, left_arm, chest;
  tf::StampedTransform  right_palm, left_palm, torso;
  geometry_msgs::PoseStamped right_arm_pose, left_arm_pose, torso_pose,
                              old_rarm_pose, old_larm_pose, old_torso_pose;
  
  WholebodyControlInterface wb_controller(nh);
  TaskspacePlanner atlas_taskspace(nh);
  ChestControlInterface chestTraj(nh);
  moveit_msgs::RobotTrajectory trajectory_msg;
  std::string larm_planner_group = TOUGH_COMMON_NAMES::LEFT_ARM_7DOF_GROUP;
  std::string rarm_planner_group = TOUGH_COMMON_NAMES::RIGHT_ARM_7DOF_GROUP;

  atlas_taskspace.waitForMoveGroupInitialization();

  atlas_taskspace.setPlanningTime(2.0);

//TODO add T-Pose calibration for human
  ROS_INFO("Callibrating: HOLD T-POSE UNTIL NOTIFIED");
  int reading = 0;
  double rx_sum = 0;
  double ry_sum = 0;
  double rz_sum = 0;
  double lx_sum = 0;
  double ly_sum = 0;
  double lz_sum = 0;
  double tx_sum = 0;
  double ty_sum = 0;
  double tz_sum = 0;
  
  while(reading < MAX_TPOSE_READS){
    right_arm.waitForTransform("/human_pelvis", "/left_palm", ros::Time(0.0), ros::Duration(3.0));
    right_arm.lookupTransform( "/human_pelvis","/left_palm",ros::Time(0.0), right_palm);
    addToSum(right_palm,rx_sum,ry_sum,rz_sum,reading);
    updateScale(rx_sum,ry_sum,rz_sum,reading, "rarm");

    left_arm.waitForTransform("/human_pelvis", "/right_palm", ros::Time(0.0), ros::Duration(3.0));
    left_arm.lookupTransform( "/human_pelvis","/right_palm",ros::Time(0.0), left_palm);
    addToSum(left_palm,lx_sum,ly_sum,lz_sum,reading);
    updateScale(lx_sum,ly_sum,lz_sum,reading, "larm");

    reading++;
  }

  ROS_INFO("Finished Calibrating!");
  ROS_INFO("YOU ARE FREE TO POSE NOW");

 while(ros::ok()){ 
  try{
   /* left_arm.lookupTransform( "/human_pelvis","/right_palm", ros::Time(0.0),left_palm);
    right_arm.waitForTransform("/human_pelvis", "/left_palm", ros::Time(0.0), ros::Duration(3.0));
    transformToPose(left_palm, left_arm_pose);
    */

    right_arm.waitForTransform("/human_pelvis", "/left_palm", ros::Time(0.0), ros::Duration(3.0));
    right_arm.lookupTransform( "/human_pelvis","/left_palm",ros::Time(0.0), right_palm);
    transformToPose(right_palm, right_arm_pose);
    // chest.lookupTransform( "/human_pelvis","/torso", ros::Time(0.0), torso);
    // transformToPose(torso, torso_pose);

   // chestTraj.controlChest(torso_pose.pose.orientation);
   
    if(isNewGoal(right_arm_pose,old_rarm_pose, 0.03)){
      atlas_taskspace.getTrajectory(right_arm_pose, rarm_planner_group, trajectory_msg);
      wb_controller.executeTrajectory(trajectory_msg);
      ROS_INFO("Attmepted: %f, %f, %f for %s",right_arm_pose.pose.position.x,right_arm_pose.pose.position.y,right_arm_pose.pose.position.z, "right arm");
      updatePastPoses(old_larm_pose, old_rarm_pose, old_torso_pose, left_arm_pose, right_arm_pose, torso_pose);
    }
   /* if(isNewGoal(left_arm_pose,old_larm_pose, 0.03)){
      atlas_taskspace.getTrajectory(left_arm_pose, larm_planner_group, trajectory_msg);
      wb_controller.executeTrajectory(trajectory_msg);
      ROS_INFO("Attmepted: %f, %f, %f for %s",left_arm_pose.pose.position.x,right_arm_pose.pose.position.y,right_arm_pose.pose.position.z, "left arm");
      updatePastPoses(old_larm_pose, old_rarm_pose, old_torso_pose, left_arm_pose, right_arm_pose, torso_pose);
    }*/
    //atlas_taskspace.getTrajectory(left_arm_pose, larm_planner_group, trajectory_msg);
    //wb_controller.executeTrajectory(trajectory_msg);

    ROS_INFO("SENDING COMMAND!!");
    ros::Duration(2.0f).sleep();
  }
  catch(tf2::LookupException error){
      ROS_ERROR("Following error: %s",error.what());
  }
 }
}
