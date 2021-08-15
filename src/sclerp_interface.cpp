/* Author: Dasharadhan Mahalingam */

#include "sclerp_motion_planner/sclerp_interface.h"

#if DEBUG

#include "eigen_matrix_formatting.h"
#include <fstream>
#include <signal.h>
#include <ctime>
#include <sys/stat.h>
#include <errno.h>

#include <rosbag/bag.h>

namespace sclerp_interface
{

Eigen::IOFormat CSVFormat(Eigen::FullPrecision,0,",","\n","","","","");

std::string log_folder_name = "sclerp_interface";
std::string home_dir(getenv("HOME"));
std::string log_dir = home_dir + "/logs/" + log_folder_name;

std::string timestamp_str = "Y%YM%mD%d_T%H%M%S";

bool log_folder_error = false;
bool log_file_error = false;

rosbag::Bag bag;

}

#endif

namespace sclerp_interface
{

ScLERPInterface::ScLERPInterface( const std::string base_link,
                                  const std::string tip_link,
                                  const ros::NodeHandle &nh)
                                  
  : nh_(nh), name_("ScLERPInterface"), base_link_name(base_link), tip_link_name(tip_link)
{

  init_failed = false;
  
  ROS_INFO("Reading URDF from Parameter Server");

  if(!urdf_model.initParam("/robot_description"))
  {
    ROS_ERROR("Failed to read URDF file from Parameter Server");
    init_failed = true;
  }
  
  if(!init_failed)
  {
    jnt_list = urdf_model.joints_;
  
    nh.param("/robot_description", robot_desc_string, std::string());
    if(!kdl_parser::treeFromString(robot_desc_string, robot_tree))
    {
      ROS_ERROR("Failed to construct kdl tree");
      init_failed = true;
    }
  }

  if(!init_failed)
  {
    std::map<std::string, urdf::JointSharedPtr> jnt_list = urdf_model.joints_;
  
    if(!robot_tree.getChain(base_link_name, tip_link_name, manip_chain))
    {
      init_failed = true;
      ROS_ERROR("Failed to construct kdl chain from tree");
    }
  }
  
  if(init_failed)
  {
    return;  
  }

  Eigen::Matrix4d t_ref;
  Eigen::Matrix4d t_jnt;
  Eigen::Matrix4d t_tip;

  Eigen::Vector4d p_jnt;
  Eigen::Vector4d v_jnt;

  t_ref <<  1, 0, 0, 0, 
            0, 1, 0, 0, 
            0, 0, 1, 0, 
            0, 0, 0, 1;

  t_tip <<  1, 0, 0, 0, 
            0, 1, 0, 0, 
            0, 0, 1, 0, 
            0, 0, 0, 1;

  p_jnt << 0, 0, 0, 1;

  v_jnt << 0, 0, 0, 0;

  KDL::Frame frame_to_tip;
  
  kinlib::Manipulator manip;
  kinlib::JointType jnt_type;
  kinlib::JointLimits jnt_lim;
  
  for(int itr = 0; itr < manip_chain.getNrOfSegments(); itr++)
  {
    KDL::Segment chain_seg = manip_chain.getSegment(itr);
    std::string seg_name = chain_seg.getName();
    KDL::Joint jnt = chain_seg.getJoint();
    KDL::RigidBodyInertia inertia_prop = chain_seg.getInertia();
    KDL::RotationalInertia rot_inertia = inertia_prop.getRotationalInertia();

    Eigen::Matrix3d rot_inertia_mat;

    frame_to_tip = chain_seg.getFrameToTip();

    urdf::JointSharedPtr jnt_ptr(jnt_list[jnt.getName()]);

    for(int r_itr = 0; r_itr < 3; r_itr++)
    {
      for(int c_itr = 0; c_itr < 3; c_itr++)
      {
        t_tip(r_itr, c_itr) = frame_to_tip.M.data[(r_itr * 3) + c_itr];
        rot_inertia_mat(r_itr, c_itr) = rot_inertia.data[(r_itr * 3) + c_itr];
      }

      t_tip(r_itr, 3) = frame_to_tip.p.data[r_itr];

      p_jnt(r_itr) = jnt.JointOrigin().data[r_itr];
      v_jnt(r_itr) = jnt.JointAxis().data[r_itr];
    }

    Eigen::Vector4d w_p_jnt = t_ref * p_jnt;
    Eigen::Vector4d w_v_jnt = t_ref * v_jnt;

    t_ref = t_ref * t_tip;

    if(jnt.getType() == KDL::Joint::JointType::RotAxis)
    {

      jnt_lim.upper_limit_ = jnt_ptr->limits->upper;
      jnt_lim.lower_limit_ = jnt_ptr->limits->lower;

      jnt_type = kinlib::JointType::Revolute;
      manip.addJoint(jnt_type, jnt.getName(), w_v_jnt, w_p_jnt, jnt_lim, t_ref);
    }
    else
    {
      if(itr < (manip_chain.getNrOfSegments()-1))
      {
        manip.modifyEndJointTipPose(t_ref);
      }
    }
    
  }
  
  kinlib_solver_ = kinlib::KinematicsSolver(manip);
}

bool ScLERPInterface::solve(const Eigen::VectorXd &init_jnt_values,
                            const Eigen::Matrix4d &g_f,
                            trajectory_msgs::JointTrajectory &jnt_trajectory)
{
  Eigen::Matrix4d g_i;
  
  kinlib_solver_.getFK(init_jnt_values, g_i);

  kinlib::ErrorCodes plan_result = kinlib_solver_.getMotionPlan(
                                      init_jnt_values,
                                      g_i,
                                      g_f,
                                      jnt_trajectory);
                                      
  if(plan_result == kinlib::ErrorCodes::OPERATION_SUCCESS)
  {
    return true;
  }
  else
  {
    return false;  
  }
                                      
}

bool ScLERPInterface::solve(const Eigen::VectorXd &init_jnt_values,
                            const Eigen::Matrix4d &g_f,
                            trajectory_msgs::JointTrajectory &jnt_trajectory,
                            std::vector<geometry_msgs::Pose> &ee_trajectory)
{
  Eigen::Matrix4d g_i;
  
  kinlib_solver_.getFK(init_jnt_values, g_i);

  kinlib::ErrorCodes plan_result = kinlib_solver_.getMotionPlan(
                                      init_jnt_values,
                                      g_i,
                                      g_f,
                                      jnt_trajectory,
                                      ee_trajectory);
                                      
  if(plan_result == kinlib::ErrorCodes::OPERATION_SUCCESS)
  {
    return true;
  }
  else
  {
    return false;  
  }
                                      
}

}
