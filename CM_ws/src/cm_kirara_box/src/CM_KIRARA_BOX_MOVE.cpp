 






















 
 
 
#define PINOCCHIO_WITH_HPP_FCL true
#include "pinocchio/config.hpp"
 
#include "pinocchio/parsers/urdf.hpp"
 
#include "pinocchio/multibody/fcl.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/geometry.hpp"
#include "pinocchio/algorithm/geometry.hxx"
#include "pinocchio/parsers/urdf/geometry.hxx"
#include "pinocchio/algorithm/jacobian.hpp"

 
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <Eigen/Dense>
#include <cmath>
#include <vector>
#include <valarray>
#include <limits>
#include <urdf_parser/urdf_parser.h>
 
#include "gcopter.hpp"
#include "kirara.hpp"
#include "dual_kirara.hpp"
#include "dual_trajectory_validator.hpp"
#include "dual_smoothing_pipeline.hpp"
#include "bidirectional_connect_planner.hpp"

 
#include <ompl/config.h>
 
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/geometric/SimpleSetup.h>

 
 
 
 
 
 
 
 

 
#include <kdl/chain.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/tree.hpp>

 
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

 
 
#include "MyRobotKinematics.h"  

#include "yaml-cpp/yaml.h"
#include "cm_box_attachment.hpp"

 
 
namespace ob = ompl::base;
namespace og = ompl::geometric;
 

 

     Eigen::Vector3d Obs(0.5, 0, 1.2);
    double radius = 0.06;
    double radius_arm = 0.05;
    double d_obs[14] = {0};
     
     
    double param_eff_min = 0.180;
    double param_eff_max = 0.185;
    double param_cos ;
    double param_cos_2 ;
    double param_rot = 0.05;
    double param_object = 0.005;
     
     
     
    double max_wrist_change = 1.20;
     
     
     
    double max_relative_transverse = 0.035;
    double max_relative_abs_z = 0.016;
    double min_relative_y_cos = 0.98;
    double min_relative_y = 0.150;
    double plantime = 30;
    std::string urdf_filename;
    Eigen::Matrix<double,12,1> q_init_yaml;
    Eigen::Matrix<double,12,1> q_final_yaml;
    Eigen::Matrix<double, 7, 1> ball_joint;
    Eigen::VectorXi collision_kit;
    bool verbose_state_validity = false;
    cm_box::Attachment carried_box_attachment;
 



double distanceToSegment_obs(const Eigen::Vector3d &A, const Eigen::Vector3d &B, const Eigen::Vector3d &P)
{
    Eigen::Vector3d AB = B - A;
    Eigen::Vector3d AP = P - A;

    double dotProduct = AP.dot(AB);
    double abLengthSquared = AB.dot(AB);

    double t = dotProduct / abLengthSquared;

    if (t >= 0 && t <= 1)
    {
         
        Eigen::Vector3d projection = A + t * AB;
         
        Eigen::Vector3d APProjection = projection - P;
         
        double distance = APProjection.norm();
        return distance;
    }
    else if (t < 0)
        return AP.norm();
    else
        return (P - B).norm();
}

double Clamp(double value, double min, double max)
{
    return std::max(min, std::min(max, value));
}



double distanceToSegment_capsule(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2, const Eigen::Vector3d &q1, const Eigen::Vector3d &q2)
{
    Eigen::Vector3d s1 = p2 - p1;  
    Eigen::Vector3d s2 = q2 - q1;  
    Eigen::Vector3d p1_q1 = p1 - q1;
    double a = s1.dot(s1);  
    double b = s1.dot(s2);
    double c = s2.dot(s2);  
    double d = p1_q1.dot(s1);
    double e = p1_q1.dot(s2);
    double D = a * c - b * b;  
    double sc, tc;


    if (D < 1e-8)
    {
            double d_p1 = distanceToSegment_obs(q1, q2, p1);
            double d_p2 = distanceToSegment_obs(q1, q2, p2);
            double d_q1 = distanceToSegment_obs(p1, p2, q1);
            double d_q2 = distanceToSegment_obs(p1, p2, q2);
            return std::min(std::min(d_p1, d_p2), std::min(d_q1, d_q2));
    }
    else{
        sc = (b * e - c * d) / D;
        tc = -(b * d - a * e) / D;
        if(sc >= 0 && sc <= 1 && tc >= 0 && tc <= 1 )
        {
            Eigen::Vector3d d_min = p1 + sc * s1 - (q1 + tc * s2);
            return d_min.norm();
        }
        else
        {
            double d_p1 = distanceToSegment_obs(q1, q2, p1);
            double d_p2 = distanceToSegment_obs(q1, q2, p2);
            double d_q1 = distanceToSegment_obs(p1, p2, q1);
            double d_q2 = distanceToSegment_obs(p1, p2, q2);
            return std::min(std::min(d_p1, d_p2), std::min(d_q1, d_q2));
        }
    }

     

     
}
 
bool isStateValid(
                    const ob::State *state, const pinocchio::Model &model,
                    pinocchio::Data &data,
                    const pinocchio::GeometryModel &geom_model,
                    pinocchio::GeometryData &geom_data
                )
{

     
     
    const ob::RealVectorStateSpace::StateType &pos = *state->as<ob::RealVectorStateSpace::StateType>();
     
     
    Eigen::Matrix<double, 6, 1> theta,theta2;
    for (int joint = 3; joint < 6; ++joint)
    {
        const double left_change = std::abs(std::remainder(
            pos[joint] - q_init_yaml[joint], 2.0 * M_PI));
        const double right_change = std::abs(std::remainder(
            pos[joint + 6] - q_init_yaml[joint + 6], 2.0 * M_PI));
        if (left_change > max_wrist_change ||
            right_change > max_wrist_change)
        {
            if (verbose_state_validity)
                std::cout << "TCST kirara_box MOVE wrist-change reject: joint="
                          << joint + 1 << " left=" << left_change
                          << " right=" << right_change
                          << " limit=" << max_wrist_change << std::endl;
            return false;
        }
    }
    Eigen::Matrix<double, 7, 1> ball_joint;
    Eigen::Matrix<double, 19, 1> theta_pinocchio;
    for (std::size_t q = 0; q < 6; ++q)
    {
        theta[q] = pos[q];
        theta2[q] = pos[q + 6];
    }
    
 
    
     
    Eigen::Matrix<double, 6, 1>  d_, alpha_, a_;
    Eigen::Matrix<double, 6, 1>  d_2, alpha_2, a_2;
    Eigen::Matrix4d Trans_L,Trans_R;
    Trans_L.setZero() ;
    Trans_R.setZero() ;
    Trans_L(0,0) = -1;
    Trans_L(1,1) = -1;
    Trans_L(2,2) = 1;
    Trans_L(3,3) = 1;
    Trans_R(0,0) = 1;
    Trans_R(0,3) = 0.0;
    Trans_R(1,1) = 1;
    Trans_R(1,3) = 0.437;
    Trans_R(2,2) = 1;
    Trans_R(2,3) = 0.0;
    Trans_R(3,3) = 1;
     
     
    d_ <<0.1625, 0, 0, 0.1333, 0.0997, 0.0996 + 0.0375 + 0.0063;
    a_ <<0, -0.425, -0.39225, 0, 0, 0;
    alpha_ <<0.5*M_PI, 0, 0, 0.5*M_PI, -0.5*M_PI, 0;
     
    d_2 <<0.1519, 0, 0, 0.11235, 0.08535, 0.0819 + 0.0375 +0.0063;
    a_2 <<0, -0.24365, -0.21325, 0, 0, 0;
    alpha_2<<0.5*M_PI, 0, 0, 0.5*M_PI, -0.5*M_PI, 0;

     
    Eigen::Matrix<double, 4, 4> J6e,J12e;
     
     
    J6e = Trans_L * tran(theta,d_,alpha_,a_);
    J12e = Trans_R * tran(theta2,d_2,alpha_2,a_2);
    ball_joint.setZero();
    ball_joint[6] = 1.0;
     
    theta_pinocchio  <<theta,theta2,ball_joint;
    carried_box_attachment.apply(model, data, theta_pinocchio);
     
    pinocchio::forwardKinematics(model, data, theta_pinocchio);


    
    
     
    
    
     



    int count = 0;
    bool robot_collision = false;
    pinocchio::computeCollisions(model,data,geom_model,geom_data,theta_pinocchio);
    for(size_t k = 0; k < geom_model.collisionPairs.size(); ++k)
    {   
        if(count < collision_kit.size() && k == static_cast<size_t>(collision_kit[count]))
        {
            count++;
             
        }
        else
        {
            const hpp::fcl::CollisionResult & cr = geom_data.collisionResults[k];
            robot_collision = cr.isCollision();
            if(robot_collision)
            {
                if (verbose_state_validity)
                {
                    const pinocchio::CollisionPair &pair =
                        geom_model.collisionPairs[k];
                    std::cerr << "TCST kirara_box invalid collision pair: "
                              << geom_model.geometryObjects[pair.first].name
                              << " && "
                              << geom_model.geometryObjects[pair.second].name
                              << std::endl;
                }
                 
                 
                break;
            }
        }
    }

    double d_eff2 = (J6e.block<3,1>(0,3) - J12e.block<3,1>(0,3)).norm();
     
     
     
    Eigen::Matrix3d rot1 = J6e.block<3,3>(0,0);
    Eigen::Matrix3d rot2 = J12e.block<3,3>(0,0);   

    double d_rot = (rot1.col(2) + rot2.col(2)).norm();
    Eigen::Vector3d e_end = J12e.block<3,1>(0,3) - J6e.block<3,1>(0,3) ;
    const double relative_transverse =
        std::hypot(e_end.x(), e_end.z());
    const double relative_y_cos = e_end.y() / e_end.norm();
    Eigen::Vector3d left_z = rot1.col(2);
    Eigen::Vector3d right_z = rot2.col(2);
    double dis_angle_cos = e_end.dot(left_z) / (left_z.norm()) / (e_end.norm());
    double dis_angle2_cos =-1* e_end.dot(right_z) / (right_z.norm()) / (e_end.norm());

     
     
     
     
     
     
    double radius_obj = 0.09;
     
     
     
    if (verbose_state_validity)
    {
        std::cerr << "TCST kirara_box validity metrics: distance=" << d_eff2
                  << " left_cos=" << dis_angle_cos
                  << " right_cos=" << dis_angle2_cos
                  << " relative_xyz=" << e_end.transpose()
                  << " transverse=" << relative_transverse
                  << " y_cos=" << relative_y_cos << std::endl;
    }
    return !robot_collision &&
           d_eff2 >= param_eff_min && d_eff2 <= param_eff_max &&
           dis_angle_cos > param_cos && dis_angle2_cos > param_cos_2 &&
           e_end.y() >= min_relative_y &&
           relative_transverse <= max_relative_transverse &&
           std::abs(e_end.z()) <= max_relative_abs_z &&
           relative_y_cos >= min_relative_y_cos;
}

KDL::JntArray KIRARAe2k(const Eigen::VectorXd &v)
{
    int num = v.size();
    KDL::JntArray q(num);
    for (int i = 0; i < num; i++)
    {
        q(i) = v(i);
    }
    return q;
}

bool KIRARAKDLInverChange(const KDL::Chain &chain, const KDL::JntArray &init_q, KDL::JntArray &fin_q, const Eigen::Matrix4d &TargetHOMO)
{
    int num = chain.getNrOfJoints();
    KDL::ChainFkSolverPos_recursive fk(chain);
    KDL::ChainIkSolverVel_pinv ik(chain);
    KDL::ChainIkSolverPos_NR solver(chain, fk, ik);
    KDL::Rotation R;
    KDL::Vector V;
    for (int i = 0; i < 3; i++)
    {
        V(i) = TargetHOMO(i, 3);
        for (int j = 0; j < 3; j++)
        {
            R(i, j) = TargetHOMO(i, j);
        }
    }
     
     
    KDL::Frame frame(R, V);

    int state = solver.CartToJnt(init_q, frame, fin_q);
    if (state)
    {
        return true;
    }
    else
    {
        return false;
    }
}
 
Eigen:: VectorXi KIRARA_PINO_COLLISION_KIT
(
    const pinocchio::Model &model,
    pinocchio::Data &data,
    const pinocchio::GeometryModel &geom_model,
    pinocchio::GeometryData &geom_data,
    const Eigen::VectorXd &q, 
    const bool &print_)
{
    std::vector<int> ki_;
    
    pinocchio::computeCollisions(model,data,geom_model,geom_data,q);
    if (print_ == true)
    {
            for(int ll =0;ll<geom_model.geometryObjects.size();ll++ )
        {
            std::cout <<"collision ID = "<<ll <<" ";
            std::cout <<"link name = " <<geom_model.geometryObjects[ll].name<< std::endl;
        }
    }
    
    for(size_t k = 0; k < geom_model.collisionPairs.size(); ++k)
    {
        const pinocchio::CollisionPair & cp = geom_model.collisionPairs[k];
        const hpp::fcl::CollisionResult & cr = geom_data.collisionResults[k];
        if(cr.isCollision())        
        {
            if(print_ == true)
            {
                std::cout << "collision pair: " << cp.first << " , " << cp.second << " collision: check! "<<std::endl;
                std::cout << "pair name: "<<  geom_model.geometryObjects[cp.first].name <<" && " << geom_model.geometryObjects[cp.second].name<<std::endl;
                std::cout << "please record k"<<" = " ;
                std::cout << k <<std::endl;
            }
            ki_.push_back(k);
        }
    }
    Eigen::Matrix<int, Eigen::Dynamic, 1> ki__(ki_.size());
    for (size_t i = 0; i < ki_.size(); ++i)
    {
        ki__[i] = ki_[i];
    }    
    return ki__;
}

int main(int argc, char **argv)
{

     
    const char *config_override = std::getenv("CM_KIRARA_BOX_MOVE_CONFIG");
    const std::string config_path = config_override
        ? std::string(config_override)
        : std::string("/home/mas-2/3rd_party/CM_ws/src/cm_kirara_box/config/move_kirara_box.yaml");
    YAML::Node config = YAML::LoadFile(config_path);
    Obs[0] = config["ball_collision"]["x"].as<double>();
    Obs[1] = config["ball_collision"]["y"].as<double>();
    Obs[2] = config["ball_collision"]["z"].as<double>();
    radius = config["ball_collision"]["r"].as<double>();
    radius_arm = config["arm"]["r"].as<double>();
    param_eff_max = config["cost"]["eff"].as<double>();
    if (config["move_grasp_distance_dh"])
    {
        param_eff_min =
            config["move_grasp_distance_dh"]["min_m"].as<double>();
        param_eff_max =
            config["move_grasp_distance_dh"]["max_m"].as<double>();
    }
    double physical_box_length = 0.220;
    if (config["move_grasp_distance_dh"] &&
        config["move_grasp_distance_dh"]["physical_box_length_m"])
    {
        physical_box_length =
            config["move_grasp_distance_dh"]["physical_box_length_m"].as<double>();
    }
    if (!(param_eff_min > 0.0 && param_eff_min < param_eff_max))
    {
        std::cerr << "Invalid kirara_box Move DH distance interval ["
                  << param_eff_min << ", " << param_eff_max
                  << "]; bounds must be positive and strictly ordered."
                  << std::endl;
        return EXIT_FAILURE;
    }
    param_rot = config["cost"]["rot"].as<double>();
    param_cos = config["cost"]["cos"].as<double>();
    param_cos_2 = config["cost"]["cos_2"].as<double>();
    param_object = config["cost"]["object"].as<double>();
    if (config["move_joint_change_limit"] &&
        config["move_joint_change_limit"]["max_wrist_rad"])
    {
        max_wrist_change =
            config["move_joint_change_limit"]["max_wrist_rad"].as<double>();
    }
    if (config["move_relative_line_constraint"])
    {
        const YAML::Node relative = config["move_relative_line_constraint"];
        max_relative_transverse = relative["max_transverse_m"].as<double>();
        max_relative_abs_z = relative["max_abs_z_m"].as<double>();
        min_relative_y_cos = relative["min_y_cos"].as<double>();
        min_relative_y = relative["min_y_m"].as<double>();
    }
    plantime = config["ompl"]["plantime"].as<double>();
    urdf_filename = config["file_path"]["pinocchio_file"].as<std::string>();
    q_init_yaml[0] = config["Joint_init"]["joint_1"].as<double>();
    q_init_yaml[1] = config["Joint_init"]["joint_2"].as<double>();
    q_init_yaml[2] = config["Joint_init"]["joint_3"].as<double>();
    q_init_yaml[3] = config["Joint_init"]["joint_4"].as<double>();
    q_init_yaml[4] = config["Joint_init"]["joint_5"].as<double>();
    q_init_yaml[5] = config["Joint_init"]["joint_6"].as<double>();
    q_init_yaml[6] = config["Joint_init"]["joint_7"].as<double>();
    q_init_yaml[7] = config["Joint_init"]["joint_8"].as<double>();
    q_init_yaml[8] = config["Joint_init"]["joint_9"].as<double>();
    q_init_yaml[9] = config["Joint_init"]["joint_10"].as<double>();
    q_init_yaml[10] = config["Joint_init"]["joint_11"].as<double>();
    q_init_yaml[11] = config["Joint_init"]["joint_12"].as<double>();
    q_final_yaml[0] = config["Joint_final"]["joint_1"].as<double>();
    q_final_yaml[1] = config["Joint_final"]["joint_2"].as<double>();
    q_final_yaml[2] = config["Joint_final"]["joint_3"].as<double>();
    q_final_yaml[3] = config["Joint_final"]["joint_4"].as<double>();
    q_final_yaml[4] = config["Joint_final"]["joint_5"].as<double>();
    q_final_yaml[5] = config["Joint_final"]["joint_6"].as<double>();
    q_final_yaml[6] = config["Joint_final"]["joint_7"].as<double>();
    q_final_yaml[7] = config["Joint_final"]["joint_8"].as<double>();
    q_final_yaml[8] = config["Joint_final"]["joint_9"].as<double>();
    q_final_yaml[9] = config["Joint_final"]["joint_10"].as<double>();
    q_final_yaml[10] = config["Joint_final"]["joint_11"].as<double>();
    q_final_yaml[11] = config["Joint_final"]["joint_12"].as<double>();

     
     
     
     
    const std::string urdf_filename = config["file_path"]["pinocchio_file"].as<std::string>();
    const std::string robots_model_path = config["model_path"]["pinocchio_path"].as<std::string>();
    
    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_filename, model);
    pinocchio::Data data(model);
    carried_box_attachment.load(config, model);
     
    pinocchio::GeometryModel geom_model;
    pinocchio::urdf::buildGeom(model, urdf_filename, pinocchio::COLLISION, geom_model, robots_model_path);
    geom_model.addAllCollisionPairs();
    pinocchio::GeometryData geom_data(geom_model); 
   
     
    Eigen::Matrix<double, 6, 1>  d_, alpha_, a_;
    Eigen::Matrix<double, 6, 1>  d_2, alpha_2, a_2;
    Eigen::Matrix4d Trans_L,Trans_R;
    Trans_L.setZero() ;
    Trans_R.setZero() ;
    Trans_L(0,0) = -1;
    Trans_L(1,1) = -1;
    Trans_L(2,2) = 1;
    Trans_L(3,3) = 1;
    Trans_R(0,0) = 1;
    Trans_R(0,3) = 0.0;
    Trans_R(1,1) = 1;
    Trans_R(1,3) = 0.437;
    Trans_R(2,2) = 1;
    Trans_R(2,3) = 0.0;
    Trans_R(3,3) = 1;
     
     
    d_ <<0.1625, 0, 0, 0.1333, 0.0997, 0.0996 + 0.0375 + 0.0063;
    a_ <<0, -0.425, -0.39225, 0, 0, 0;
    alpha_ <<0.5*M_PI, 0, 0, 0.5*M_PI, -0.5*M_PI, 0;
     
    d_2 <<0.1519, 0, 0, 0.11235, 0.08535, 0.0819 + 0.0375 +0.0063;
    a_2 <<0, -0.24365, -0.21325, 0, 0, 0;
    alpha_2<<0.5*M_PI, 0, 0, 0.5*M_PI, -0.5*M_PI, 0;

    Eigen::VectorXd  theta = q_init_yaml.segment<6>(0);
    Eigen::VectorXd  theta2 = q_init_yaml.segment<6>(6);
    Eigen::Matrix<double, 4, 4> J6e,J12e;
    J6e = Trans_L * tran(theta,d_,alpha_,a_);
    J12e = Trans_R * tran(theta2,d_2,alpha_2,a_2);
     
    ball_joint.setZero();
    ball_joint[6] = 1.0;
     
     
    Eigen::Matrix<double, 19, 1>  theta_pinocchio;
    theta_pinocchio<<theta,theta2,ball_joint;
    carried_box_attachment.apply(model, data, theta_pinocchio);
    pinocchio::forwardKinematics(model, data, theta_pinocchio);
    collision_kit = KIRARA_PINO_COLLISION_KIT(model,data,geom_model,geom_data,theta_pinocchio,true);
    
    std::cout << "Initial collision check completed." << std::endl;
    
   
     
     
     
     
     
     
     
     
     
     
     
     
    Eigen::Matrix<double, 6, 1> init_q, fin_q;
    Eigen::Matrix<double, 6, 1> init_q2, fin_q2;
    
    init_q = q_init_yaml.block<6,1>(0,0);
    init_q2 = q_init_yaml.block<6,1>(6,0);
    fin_q = q_final_yaml.block<6,1>(0,0);
    fin_q2 =q_final_yaml.block<6,1>(6,0);


     
    double angle_lowerBound_13 = -2*M_PI;
    double angle_upperBound_13 =  2*M_PI;
    double angle_lowerBound_46 = -1 * M_PI;
    double angle_upperBound_46 = 1 * M_PI;
    int num = 0;
     
     
    ob::StateSpacePtr space(new ob::RealVectorStateSpace(12));
     
     
    ob::RealVectorBounds bounds(12);
    for (std::size_t i = 0; i < 6; ++i)
    {
        if(i<3)
        {
            bounds.setLow(i, angle_lowerBound_13);
            bounds.setLow(i+6, angle_lowerBound_13);
            bounds.setHigh(i, angle_upperBound_13);
            bounds.setHigh(i+6, angle_upperBound_13);
        }
        else{
            bounds.setLow(i, angle_lowerBound_46);
            bounds.setLow(i+6, angle_lowerBound_46);
            bounds.setHigh(i, angle_upperBound_46);
            bounds.setHigh(i+6, angle_upperBound_46);
        }
    }
    space->as<ob::RealVectorStateSpace>()->setBounds(bounds);
     
    og::SimpleSetup ss(space);
     
     
    ss.getSpaceInformation()->setStateValidityCheckingResolution(1.0e-4);
     

    ss.setStateValidityChecker(std::bind(&isStateValid, std::placeholders::_1, model, data, geom_model,geom_data));
    std::cout << "TCST kirara_box MOVE hard DH endpoint distance=["
              << param_eff_min << ", " << param_eff_max
              << "] m; physical box length=" << physical_box_length
              << " m; max wrist change=" << max_wrist_change
              << " rad; relative-line: dy>=" << min_relative_y
              << " m, transverse<=" << max_relative_transverse
              << " m, |dz|<=" << max_relative_abs_z
              << " m, y_cos>=" << min_relative_y_cos
              << "; config=" << config_path << std::endl;

     
    ob::ScopedState<> start(space);
    start[0] = init_q(0);
    start[1] = init_q(1);
    start[2] = init_q(2);
    start[3] = init_q(3);
    start[4] = init_q(4);
    start[5] = init_q(5);
    start[6] = init_q2(0);
    start[7] = init_q2(1);
    start[8] = init_q2(2);
    start[9] = init_q2(3);
    start[10] = init_q2(4);
    start[11] = init_q2(5);
     
     
     
     
    ob::ScopedState<> virtualGoal(space);
    virtualGoal[0] = fin_q(0);  
    virtualGoal[1] = fin_q(1);
    virtualGoal[2] = fin_q(2);
    virtualGoal[3] = fin_q(3);
    virtualGoal[4] = fin_q(4);
    virtualGoal[5] = fin_q(5);
    virtualGoal[6] = fin_q2(0);  
    virtualGoal[7] = fin_q2(1);
    virtualGoal[8] = fin_q2(2);
    virtualGoal[9] = fin_q2(3);
    virtualGoal[10] = fin_q2(4);
    virtualGoal[11] = fin_q2(5);
    if (argc > 3 && std::string(argv[1]) == "--validate-samples")
    {
        std::ifstream left_samples(argv[2]);
        std::ifstream right_samples(argv[3]);
        if (!left_samples || !right_samples)
        {
            std::cerr << "Cannot open trajectory sample files." << std::endl;
            return EXIT_FAILURE;
        }
        std::size_t sample_index = 0;
        while (true)
        {
            double left_values[6];
            double right_values[6];
            if (!(left_samples >> left_values[0]))
                break;
            for (int joint = 1; joint < 6; ++joint)
                if (!(left_samples >> left_values[joint]))
                {
                    std::cerr << "Incomplete left sample row." << std::endl;
                    return EXIT_FAILURE;
                }
            for (int joint = 0; joint < 6; ++joint)
                if (!(right_samples >> right_values[joint]))
                {
                    std::cerr << "Left/right sample count mismatch." << std::endl;
                    return EXIT_FAILURE;
                }
            ob::ScopedState<> sample(space);
            for (int joint = 0; joint < 6; ++joint)
            {
                sample[joint] = left_values[joint];
                sample[joint + 6] = right_values[joint];
            }
            if (!ss.getSpaceInformation()->isValid(sample.get()))
            {
                std::cerr << "TCST kirara_box MOVE sample invalid at index "
                          << sample_index << std::endl;
                verbose_state_validity = true;
                ss.getSpaceInformation()->isValid(sample.get());
                verbose_state_validity = false;
                return EXIT_FAILURE;
            }
            ++sample_index;
        }
        double extra = 0.0;
        if (right_samples >> extra || sample_index < 2)
        {
            std::cerr << "Invalid right sample count or empty trajectory."
                      << std::endl;
            return EXIT_FAILURE;
        }
        std::cout << "TCST kirara_box MOVE samples valid: count="
                  << sample_index << std::endl;
        return EXIT_SUCCESS;
    }
    if (argc > 1 && std::string(argv[1]) == "--validate-only")
    {
        verbose_state_validity = false;
        const bool start_valid =
            ss.getSpaceInformation()->isValid(start.get());
        verbose_state_validity = true;
        const bool goal_valid =
            ss.getSpaceInformation()->isValid(virtualGoal.get());
        verbose_state_validity = false;
        std::cout << "TCST kirara_box MOVE validity: start="
                  << start_valid << " goal=" << goal_valid << std::endl;
        return start_valid && goal_valid ? EXIT_SUCCESS : EXIT_FAILURE;
    }
    ss.setStartAndGoalStates(start, virtualGoal);
     
    ob::PlannerPtr planner(
        new cm_planning::BidirectionalConnectPlanner(ss.getSpaceInformation()));
     
    planner->as<cm_planning::BidirectionalConnectPlanner>()->setRange(0.08 * M_PI);
    ss.setPlanner(planner);
     
    ob::PlannerStatus solved = ss.solve(plantime);
    Eigen::Matrix<double, 6, Eigen::Dynamic> innerPts;
    Eigen::Matrix<double, 6, Eigen::Dynamic> innerPts2;
    if (solved == ob::PlannerStatus::EXACT_SOLUTION)
    {
        std::cout << "~ฅKIRARAฅ~ Found solution" << std::endl;

         
         
        ss.simplifySolution();

         
         
        og::PathGeometric solution_path = ss.getSolutionPath();

         
         

         
         
         
        std::string plannedPathFile1 = config["planned_path"]["left"].as<std::string>();
        std::string plannedPathFile2 = config["planned_path"]["right"].as<std::string>();
        std::ofstream jointfile(plannedPathFile1);
        std::ofstream jointfile2(plannedPathFile2);

        Eigen::Matrix<double, 6, 1> theta;
        Eigen::Matrix<double, 6, 1> theta2;
        std::vector<double> path_x, path_y, path_z;
        std::vector<double> J_1, J_2, J_3, J_4, J_5, J_6;
        std::vector<double> J_7, J_8, J_9, J_10, J_11, J_12;
         
        std::cout << "solution_path" << solution_path.getStateCount() << std::endl;

        for (std::size_t i = 0; i < solution_path.getStateCount(); ++i)
        {
            const ob::State *state = solution_path.getState(i);

            for (std::size_t j = 0; j < 6; ++j)
            {
                theta[j] = state->as<ob::RealVectorStateSpace::StateType>()->values[j];
                theta2[j] = state->as<ob::RealVectorStateSpace::StateType>()->values[j + 6];
            }
             
             
             
             
             
            J_1.push_back(theta[0]);
            J_2.push_back(theta[1]);
            J_3.push_back(theta[2]);
            J_4.push_back(theta[3]);
            J_5.push_back(theta[4]);
            J_6.push_back(theta[5]);
            J_7.push_back(theta2[0]);
            J_8.push_back(theta2[1]);
            J_9.push_back(theta2[2]);
            J_10.push_back(theta2[3]);
            J_11.push_back(theta2[4]);
            J_12.push_back(theta2[5]);
             
            if (innerPts.cols() <= i)
            {
                 
                innerPts.conservativeResize(6, i + 1);
                innerPts2.conservativeResize(6, i + 1);
            }
            innerPts.col(i) = theta;
            innerPts2.col(i) = theta2;
             
            num++;
        }
        for (size_t k = 0; k < J_1.size(); ++k)
        {
            jointfile << J_1[k] << " " << J_2[k] << " " << J_3[k]<<" " << J_4[k] << " " << J_5[k] << " " << J_6[k]  << std::endl;
            jointfile2 << J_7[k] << " " << J_8[k] << " " << J_9[k]<<" " << J_10[k] << " " << J_11[k] << " " << J_12[k]  << std::endl;

        }
        jointfile.close();
        jointfile2.close();
    }
    else
    {
        std::cerr << "No exact solution found; skip trajectory smoothing." << std::endl;
        return EXIT_FAILURE;
    }

    if (innerPts.cols() < 2 || innerPts2.cols() < 2)
    {
        std::cerr << "Planning solution has too few waypoints for smoothing." << std::endl;
        return EXIT_FAILURE;
    }

    if (argc > 1 && std::string(argv[1]) == "--planning-only")
    {
        std::cout << "TCST kirara_box MOVE planning-only path accepted: waypoints="
                  << innerPts.cols() << std::endl;
        return EXIT_SUCCESS;
    }

    Eigen::Matrix<double, 6, 3> headState, tailState, headState2, tailState2;
    headState.setZero();
    tailState.setZero();
    headState2.setZero();
    tailState2.setZero();

    headState.col(0) = innerPts.col(0);
    tailState.col(0) = innerPts.col(innerPts.cols() - 1);
    headState2.col(0) = innerPts2.col(0);
    tailState2.col(0) = innerPts2.col(innerPts2.cols() - 1);

    const int innerX = 4;
    Eigen::VectorXi innerNum(num - 1);
    innerNum.array() = innerX;

    Eigen::Matrix<double, 6, 1> qMin;
    Eigen::Matrix<double, 6, 1> qMax;
    for (int joint = 0; joint < 6; ++joint)
    {
        qMin(joint) = joint < 3 ? angle_lowerBound_13 : angle_lowerBound_46;
        qMax(joint) = joint < 3 ? angle_upperBound_13 : angle_upperBound_46;
    }

    gcopter::DualKIRARAOptions optimizerOptions;
    optimizerOptions.maxIterations = 4096;
    optimizerOptions.weightTime = 0.05;
    optimizerOptions.weightReference = 20.0;
    optimizerOptions.leftPenalty.qMin = qMin;
    optimizerOptions.leftPenalty.qMax = qMax;
    optimizerOptions.rightPenalty.qMin = qMin;
    optimizerOptions.rightPenalty.qMax = qMax;
    optimizerOptions.leftPenalty.weightPosition = 100.0;
    optimizerOptions.rightPenalty.weightPosition = 100.0;
    optimizerOptions.leftPenalty.weightVelocity = 100.0;
    optimizerOptions.rightPenalty.weightVelocity = 100.0;

    tcst::DualTrajectoryValidationOptions validationOptions;
    validationOptions.qMinLeft = qMin;
    validationOptions.qMaxLeft = qMax;
    validationOptions.qMinRight = qMin;
    validationOptions.qMaxRight = qMax;
    validationOptions.vMaxLeft = optimizerOptions.leftPenalty.vMax;
    validationOptions.vMaxRight = optimizerOptions.rightPenalty.vMax;
    validationOptions.samplePeriod = 0.01;

    const tcst::DualStateValidityFunction collisionAndTaskValid =
        [&](const Eigen::Matrix<double, 6, 1> &qLeft,
            const Eigen::Matrix<double, 6, 1> &qRight)
    {
        ob::ScopedState<> state(space);
        for (int joint = 0; joint < 6; ++joint)
        {
            state[joint] = qLeft(joint);
            state[joint + 6] = qRight(joint);
        }
        return ss.getSpaceInformation()->isValid(state.get());
    };

    const std::string kirarafile1 =
        config["kirara_path"]["left"].as<std::string>();
    const std::string kirarafile2 =
        config["kirara_path"]["right"].as<std::string>();
    if (!tcst::runDualSmoothingPipeline(
            headState, tailState, headState2, tailState2,
            innerNum, innerPts, innerPts2,
            optimizerOptions, validationOptions,
            collisionAndTaskValid, kirarafile1, kirarafile2))
    {
        return EXIT_FAILURE;
    }

    return 0;
}
