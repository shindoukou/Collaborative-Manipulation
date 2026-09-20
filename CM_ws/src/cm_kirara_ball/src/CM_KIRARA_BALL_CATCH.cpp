 






















 
 
 
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
#include "tcst_ik_goal_generator.hpp"
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

 
 
namespace ob = ompl::base;
namespace og = ompl::geometric;
 

 

     Eigen::Vector3d Obs(0.5, 0, 1.2);
    double radius = 0.06;
    double radius_arm = 0.05;
    double d_obs[14] = {0};
    double param_eff = 0.18;
    double param_cos ;
    double param_cos_2 ;
    double param_rot = 0.05;
    double param_object = 0.005;
    double plantime = 30;
    std::string urdf_filename;
    Eigen::Matrix<double,12,1> q_init_yaml;
    Eigen::Matrix<double,12,1> q_final_yaml;
 

bool isExpectedAdjacentArmPair(const std::string &first,
                               const std::string &second)
{
    const auto matches = [&](const std::string &a, const std::string &b)
    {
        return (first == a && second == b) || (first == b && second == a);
    };
    return
        matches("left_base_link_0", "left_shoulder_link_0") ||
        matches("left_shoulder_link_0", "left_upper_arm_link_0") ||
        matches("left_upper_arm_link_0", "left_forearm_link_0") ||
        matches("left_forearm_link_0", "left_wrist_1_link_0") ||
        matches("left_wrist_1_link_0", "left_wrist_2_link_0") ||
        matches("left_wrist_2_link_0", "left_wrist_3_link_0") ||
        matches("right_base_link_0", "right_shoulder_link_0") ||
        matches("right_shoulder_link_0", "right_upper_arm_link_0") ||
        matches("right_upper_arm_link_0", "right_forearm_link_0") ||
        matches("right_forearm_link_0", "right_wrist_1_link_0") ||
        matches("right_wrist_1_link_0", "right_wrist_2_link_0") ||
        matches("right_wrist_2_link_0", "right_wrist_3_link_0");
}



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
    Eigen::Matrix<double, 12, 1> theta_pinocchio;
    for (std::size_t q = 0; q < 6; ++q)
    {
        theta[q] = pos[q];
        theta2[q] = pos[q + 6];
    }
    for (std::size_t q = 0; q < 12; ++q)
    {
        theta_pinocchio[q] = pos[q];
        
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
     
     
    pinocchio::forwardKinematics(model, data, theta_pinocchio);

    J6e = Trans_L * tran(theta,d_,alpha_,a_);
    J12e = Trans_R * tran(theta2,d_2,alpha_2,a_2);

     




    bool robot_collision = false;
    pinocchio::updateGeometryPlacements(
        model, data, geom_model, geom_data, theta_pinocchio);
    for(size_t k = 0; k < geom_model.collisionPairs.size(); ++k)
    {   
        const pinocchio::CollisionPair &pair = geom_model.collisionPairs[k];
        const std::string &firstName = geom_model.geometryObjects[pair.first].name;
        const std::string &secondName = geom_model.geometryObjects[pair.second].name;
         
         
        if (isExpectedAdjacentArmPair(firstName, secondName))
            continue;
        else
        {
             
             
            geom_data.collisionResults[k].clear();
            robot_collision = pinocchio::computeCollision(
                geom_model, geom_data, k);
            if (robot_collision)
            {
                 
                 
                 
                robot_collision = pinocchio::computeDistance(
                    geom_model, geom_data, k).min_distance <= 0.0;
            }
            if(robot_collision)
            {
                std::cout << "collision_number" << k << " "
                          << firstName << " && " << secondName << std::endl;
                break;
            }
        }
    }

    double d_eff2 = (J6e.block<3,1>(0,3) - J12e.block<3,1>(0,3)).norm();
     
     
     
    Eigen::Matrix3d rot1 = J6e.block<3,3>(0,0);
    Eigen::Matrix3d rot2 = J12e.block<3,3>(0,0);   

    double d_rot = (rot1.col(2) + rot2.col(2)).norm();
    Eigen::Vector3d e_end = J12e.block<3,1>(0,3) - J6e.block<3,1>(0,3) ;
    Eigen::Vector3d left_z = rot1.col(2);
    Eigen::Vector3d right_z = rot2.col(2);
    double dis_angle_cos = e_end.dot(left_z) / (left_z.norm()) / (e_end.norm());
    double dis_angle2_cos =-1* e_end.dot(right_z) / (right_z.norm()) / (e_end.norm());

     
    std::cout << "d_eff2 "<<d_eff2 << std::endl;
    std::cout << "d_rot "<<d_rot << std::endl;
    std::cout << "d_angle_cos "<<dis_angle_cos << std::endl;
    std::cout << "d_angle2_cos "<<dis_angle2_cos << std::endl;
     
    double radius_obj = 0.09;
    Eigen::Vector3d object_p =  (J12e.block<3,1>(0,3) + J6e.block<3,1>(0,3) ) / 2;
     
     
    return !robot_collision;  
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

int main(int argc, char **argv)
{

     
    const std::string configPath = argc > 1
        ? argv[1]
        : "/home/mas-2/3rd_party/CM_ws/src/cm_kirara_ball/config/catch_template.yaml";
    YAML::Node config = YAML::LoadFile(configPath);
    Obs[0] = config["ball_collision"]["x"].as<double>();
    Obs[1] = config["ball_collision"]["y"].as<double>();
    Obs[2] = config["ball_collision"]["z"].as<double>();
    radius = config["ball_collision"]["r"].as<double>();
    radius_arm = config["arm"]["r"].as<double>();
    param_eff = config["cost"]["eff"].as<double>();
    param_rot = config["cost"]["rot"].as<double>();
    param_cos = config["cost"]["cos"].as<double>();
    param_cos_2 = config["cost"]["cos_2"].as<double>();
    param_object = config["cost"]["object"].as<double>();
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
     
    pinocchio::GeometryModel geom_model;
    pinocchio::urdf::buildGeom(model, urdf_filename, pinocchio::COLLISION, geom_model, robots_model_path);
    geom_model.addAllCollisionPairs();
    pinocchio::GeometryData geom_data(geom_model); 
   

   
     
     
     
     
     
     
     
     
     
     
     
     
    Eigen::Matrix<double, 6, 1> init_q, fin_q;
    Eigen::Matrix<double, 6, 1> init_q2, fin_q2;
    init_q = q_init_yaml.block<6,1>(0,0);
    init_q2 = q_init_yaml.block<6,1>(6,0);
    fin_q = q_final_yaml.block<6,1>(0,0);
    fin_q2 =q_final_yaml.block<6,1>(6,0);
     
     
     
     
     
     
     
     

     
    double angle_lowerBound_13 = -M_PI;
    double angle_upperBound_13 =  M_PI;
    double angle_lowerBound_46 = -2 * M_PI;
    double angle_upperBound_46 = 2 * M_PI;
    int num = 0;
     
     
    ob::StateSpacePtr space(new ob::RealVectorStateSpace(12));
     
     
    ob::RealVectorBounds bounds(12);
    const double maxRightWristExcursion =
        config["goal_generator"]["max_right_wrist_excursion_rad"]
            ? config["goal_generator"]["max_right_wrist_excursion_rad"].as<double>()
            : 1.3;
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
            bounds.setHigh(i, angle_upperBound_46);
            bounds.setLow(i+6, std::max(
                angle_lowerBound_46, init_q2(i) - maxRightWristExcursion));
            bounds.setHigh(i+6, std::min(
                angle_upperBound_46, init_q2(i) + maxRightWristExcursion));
        }
    }
    space->as<ob::RealVectorStateSpace>()->setBounds(bounds);
     
    og::SimpleSetup ss(space);
     

    ss.setStateValidityChecker(std::bind(&isStateValid, std::placeholders::_1, model, data, geom_model,geom_data));
     
     
     
    ss.getSpaceInformation()->setStateValidityCheckingResolution(0.0002);

    const tcst::ik_goal::GoalGeneratorOptions goalOptions =
        tcst::ik_goal::loadOptions(config, tcst::ik_goal::TaskMode::CATCH);
    if (goalOptions.enabled)
    {
        tcst::ik_goal::DualArmGoalGenerator goalGenerator(
            urdf_filename, goalOptions, q_init_yaml);
        const tcst::ik_goal::DualArmGoal generatedGoal = goalGenerator.generate(
            [&](const tcst::ik_goal::Vec12 &candidateQ)
            {
                ob::ScopedState<> candidate(space);
                for (int joint = 0; joint < 12; ++joint)
                    candidate[joint] = candidateQ(joint);
                for (int joint = 3; joint < 6; ++joint)
                {
                    if (std::abs(candidateQ(joint + 6) - init_q2(joint)) >
                        maxRightWristExcursion)
                        return false;
                }
                return ss.getSpaceInformation()->isValid(candidate.get());
            });
        std::cout << "IK goal generation: " << generatedGoal.message
                  << ", time_ms=" << generatedGoal.generation_time_ms
                  << ", valid_pairs=" << generatedGoal.valid_pair_count
                  << std::endl;
        if (generatedGoal.success)
        {
            q_final_yaml = generatedGoal.q;
            fin_q = generatedGoal.q.head<6>();
            fin_q2 = generatedGoal.q.tail<6>();
        }
        else if (!goalOptions.fallback_to_joint_final)
        {
            std::cerr << "Automatic IK goal generation failed; refusing the old-base fallback."
                      << std::endl;
            return 1;
        }
    }

     
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
    ss.setStartAndGoalStates(start, virtualGoal);
     
    ob::PlannerPtr planner(
        new cm_planning::BidirectionalConnectPlanner(ss.getSpaceInformation()));
     
    planner->as<cm_planning::BidirectionalConnectPlanner>()->setRange(0.08 * M_PI);
    ss.setPlanner(planner);
     
    ob::PlannerStatus solved = ss.solve(plantime);
    Eigen::Matrix<double, 6, Eigen::Dynamic> innerPts;
    Eigen::Matrix<double, 6, Eigen::Dynamic> innerPts2;
     
     
    if (solved && ss.haveExactSolutionPath())
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
        std::cerr << "No exact solution found; approximate planning paths are rejected; "
                     "skip trajectory smoothing."
                  << std::endl;
        return EXIT_FAILURE;
    }

    if (innerPts.cols() < 2 || innerPts2.cols() < 2)
    {
        std::cerr << "Planning solution has too few waypoints for smoothing." << std::endl;
        return EXIT_FAILURE;
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
