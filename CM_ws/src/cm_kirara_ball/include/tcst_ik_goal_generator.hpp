#pragma once

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/frames.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <functional>
#include <iomanip>
#include <limits>
#include <random>
#include <sstream>
#include <string>
#include <vector>

namespace tcst
{
namespace ik_goal
{

using Vec6 = Eigen::Matrix<double, 6, 1>;
using Vec12 = Eigen::Matrix<double, 12, 1>;

enum class TaskMode
{
    CATCH,
    MOVE
};

struct IKCandidate
{
    Vec6 q = Vec6::Zero();
    double position_error = std::numeric_limits<double>::infinity();
    double orientation_error = std::numeric_limits<double>::infinity();
};

struct LocalGoalCandidate
{
    int sample_index = -1;
    int axis_index = -1;
    int roll_index = -1;
    IKCandidate ik;
};

struct TaskMetrics
{
    double distance = 0.0;
    double left_facing_cos = -1.0;
    double right_facing_cos = -1.0;
    double left_target_cos = -1.0;
    double right_target_cos = -1.0;
};

struct GoalGeneratorOptions
{
    bool enabled = true;
    bool fallback_to_joint_final = true;
    bool target_from_reference = false;
     
     
     
     
     
    bool preserve_left_reference = false;
     
     
    bool preserve_right_world_pose = false;
    Eigen::Vector3d right_base_translation_delta = Eigen::Vector3d::Zero();
    Eigen::Vector3d target_offset = Eigen::Vector3d::Zero();
    bool stop_at_first_valid = false;
    TaskMode mode = TaskMode::CATCH;
    Eigen::Vector3d target = Eigen::Vector3d::Zero();
    double distance_min = 0.16;
    double distance_max = 0.18;
    double theta_left = 0.318;
    double theta_right = 0.4;
    double psi_left = 0.25;
    double psi_right = 0.20;
    int axis_samples = 24;
    int roll_samples = 4;
    int ik_restarts = 32;
    int ik_max_iterations = 250;
    double local_seed_ratio = 0.5;
    double local_seed_range = 0.8;
    double ik_position_tolerance = 1e-4;
    double ik_orientation_tolerance = 2e-3;
    double dedup_tolerance = 2e-3;
    double joint_score_weight = 1.0;
    double pose_score_weight = 20.0;
    Vec12 joint_score_weights = Vec12::Ones();
    unsigned int random_seed = 20260728u;
    std::string left_tip = "left_tool0";
    std::string right_tip = "right_tool0";
    std::string debug_file;
    std::string output_file;
};

struct DualArmGoal
{
    bool success = false;
    Vec12 q = Vec12::Zero();
    TaskMetrics metrics;
    double score = std::numeric_limits<double>::infinity();
    std::size_t left_candidate_count = 0;
    std::size_t right_candidate_count = 0;
    std::size_t tested_pair_count = 0;
    std::size_t valid_pair_count = 0;
    double generation_time_ms = 0.0;
    std::string message;
};

using PairValidityFunction = std::function<bool(const Vec12 &)>;

inline double clampUnit(double value)
{
    return std::max(-1.0, std::min(1.0, value));
}

inline double periodicDifference(double a, double b)
{
    return std::atan2(std::sin(a - b), std::cos(a - b));
}

inline Eigen::Matrix4d dhTransform(double theta, double d, double alpha, double a)
{
    Eigen::Matrix4d t;
    const double ct = std::cos(theta);
    const double st = std::sin(theta);
    const double ca = std::cos(alpha);
    const double sa = std::sin(alpha);
    t << ct, -st * ca, st * sa, a * ct,
         st, ct * ca, -ct * sa, a * st,
         0.0, sa, ca, d,
         0.0, 0.0, 0.0, 1.0;
    return t;
}

inline Eigen::Matrix4d dhForward(const Vec6 &q, bool left)
{
    static const std::array<double, 6> alpha = {
        0.5 * M_PI, 0.0, 0.0, 0.5 * M_PI, -0.5 * M_PI, 0.0};
    static const std::array<double, 6> left_d = {
        0.1625, 0.0, 0.0, 0.1333, 0.0997, 0.0996 + 0.0375 + 0.0063};
    static const std::array<double, 6> left_a = {
        0.0, -0.425, -0.39225, 0.0, 0.0, 0.0};
    static const std::array<double, 6> right_d = {
        0.1519, 0.0, 0.0, 0.11235, 0.08535, 0.0819 + 0.0375 + 0.0063};
    static const std::array<double, 6> right_a = {
        0.0, -0.24365, -0.21325, 0.0, 0.0, 0.0};

    Eigen::Matrix4d base = Eigen::Matrix4d::Identity();
    if (left)
    {
        base(0, 0) = -1.0;
        base(1, 1) = -1.0;
    }
    else
    {
         
        base(0, 3) = 0.0;
        base(1, 3) = 0.437;
        base(2, 3) = 0.0;
    }

    Eigen::Matrix4d result = base;
    for (int i = 0; i < 6; ++i)
    {
        result *= dhTransform(
            q(i), left ? left_d[i] : right_d[i], alpha[i],
            left ? left_a[i] : right_a[i]);
    }
    return result;
}

inline KDL::Frame eigenToKdl(const Eigen::Matrix4d &t)
{
    return KDL::Frame(
        KDL::Rotation(
            t(0, 0), t(0, 1), t(0, 2),
            t(1, 0), t(1, 1), t(1, 2),
            t(2, 0), t(2, 1), t(2, 2)),
        KDL::Vector(t(0, 3), t(1, 3), t(2, 3)));
}

inline Eigen::Matrix4d kdlToEigen(const KDL::Frame &frame)
{
    Eigen::Matrix4d result = Eigen::Matrix4d::Identity();
    for (int row = 0; row < 3; ++row)
    {
        result(row, 3) = frame.p(row);
        for (int col = 0; col < 3; ++col)
            result(row, col) = frame.M(row, col);
    }
    return result;
}

inline KDL::JntArray eigenToKdl(const Vec6 &q)
{
    KDL::JntArray result(6);
    for (int i = 0; i < 6; ++i)
        result(i) = q(i);
    return result;
}

inline Vec6 kdlToEigen(const KDL::JntArray &q)
{
    Vec6 result;
    for (int i = 0; i < 6; ++i)
        result(i) = q(i);
    return result;
}

inline Eigen::Matrix3d rotationWithZAxis(const Eigen::Vector3d &z, double roll)
{
    const Eigen::Vector3d zn = z.normalized();
    const Eigen::Vector3d helper =
        std::abs(zn.z()) < 0.9 ? Eigen::Vector3d::UnitZ() : Eigen::Vector3d::UnitY();
    const Eigen::Vector3d x0 = helper.cross(zn).normalized();
    const Eigen::Vector3d y0 = zn.cross(x0).normalized();
    Eigen::Matrix3d rotation;
    rotation.col(0) = std::cos(roll) * x0 + std::sin(roll) * y0;
    rotation.col(1) = -std::sin(roll) * x0 + std::cos(roll) * y0;
    rotation.col(2) = zn;
    return rotation;
}

inline double rotationError(
    const Eigen::Matrix3d &actual, const Eigen::Matrix3d &desired)
{
    const Eigen::Matrix3d delta = desired.transpose() * actual;
    return std::acos(clampUnit(0.5 * (delta.trace() - 1.0)));
}

inline GoalGeneratorOptions loadOptions(
    const YAML::Node &config, TaskMode default_mode)
{
    GoalGeneratorOptions options;
    options.mode = default_mode;
    if (!config["goal_generator"])
        return options;

    const YAML::Node node = config["goal_generator"];
    if (node["enabled"])
        options.enabled = node["enabled"].as<bool>();
    if (node["fallback_to_joint_final"])
        options.fallback_to_joint_final = node["fallback_to_joint_final"].as<bool>();
    if (node["target_from_reference"])
        options.target_from_reference = node["target_from_reference"].as<bool>();
    if (node["preserve_left_reference"])
        options.preserve_left_reference = node["preserve_left_reference"].as<bool>();
    if (node["preserve_right_world_pose"])
        options.preserve_right_world_pose = node["preserve_right_world_pose"].as<bool>();
    if (node["right_base_translation_delta"])
    {
        options.right_base_translation_delta <<
            node["right_base_translation_delta"]["x"].as<double>(),
            node["right_base_translation_delta"]["y"].as<double>(),
            node["right_base_translation_delta"]["z"].as<double>();
    }
    if (node["target_offset"])
    {
        options.target_offset <<
            node["target_offset"]["x"].as<double>(),
            node["target_offset"]["y"].as<double>(),
            node["target_offset"]["z"].as<double>();
    }
    if (node["stop_at_first_valid"])
        options.stop_at_first_valid = node["stop_at_first_valid"].as<bool>();
    if (node["mode"])
    {
        std::string mode = node["mode"].as<std::string>();
        std::transform(mode.begin(), mode.end(), mode.begin(), ::toupper);
        options.mode = mode == "MOVE" ? TaskMode::MOVE : TaskMode::CATCH;
    }
    if (node["target"])
    {
        options.target << node["target"]["x"].as<double>(),
            node["target"]["y"].as<double>(),
            node["target"]["z"].as<double>();
    }
    if (node["distance_min"])
        options.distance_min = node["distance_min"].as<double>();
    if (node["distance_max"])
        options.distance_max = node["distance_max"].as<double>();
    if (node["theta_left"])
        options.theta_left = node["theta_left"].as<double>();
    if (node["theta_right"])
        options.theta_right = node["theta_right"].as<double>();
    if (node["psi_left"])
        options.psi_left = node["psi_left"].as<double>();
    if (node["psi_right"])
        options.psi_right = node["psi_right"].as<double>();
    if (node["axis_samples"])
        options.axis_samples = node["axis_samples"].as<int>();
    if (node["roll_samples"])
        options.roll_samples = node["roll_samples"].as<int>();
    if (node["ik_restarts"])
        options.ik_restarts = node["ik_restarts"].as<int>();
    if (node["ik_max_iterations"])
        options.ik_max_iterations = node["ik_max_iterations"].as<int>();
    if (node["local_seed_ratio"])
        options.local_seed_ratio = node["local_seed_ratio"].as<double>();
    if (node["local_seed_range"])
        options.local_seed_range = node["local_seed_range"].as<double>();
    if (node["ik_position_tolerance"])
        options.ik_position_tolerance = node["ik_position_tolerance"].as<double>();
    if (node["ik_orientation_tolerance"])
        options.ik_orientation_tolerance = node["ik_orientation_tolerance"].as<double>();
    if (node["dedup_tolerance"])
        options.dedup_tolerance = node["dedup_tolerance"].as<double>();
    if (node["joint_score_weight"])
        options.joint_score_weight = node["joint_score_weight"].as<double>();
    if (node["pose_score_weight"])
        options.pose_score_weight = node["pose_score_weight"].as<double>();
    if (node["joint_score_weights"])
    {
        const YAML::Node weights = node["joint_score_weights"];
        if (!weights.IsSequence() || weights.size() != 12)
            throw std::runtime_error(
                "goal_generator.joint_score_weights must contain 12 values");
        for (int joint = 0; joint < 12; ++joint)
            options.joint_score_weights(joint) = weights[joint].as<double>();
    }
    if (node["random_seed"])
        options.random_seed = node["random_seed"].as<unsigned int>();
    if (node["left_tip"])
        options.left_tip = node["left_tip"].as<std::string>();
    if (node["right_tip"])
        options.right_tip = node["right_tip"].as<std::string>();
    if (node["debug_file"])
        options.debug_file = node["debug_file"].as<std::string>();
    if (node["output_file"])
        options.output_file = node["output_file"].as<std::string>();
    return options;
}

inline Vec12 loadJointVector(const YAML::Node &node)
{
    Vec12 q;
    for (int i = 0; i < 12; ++i)
        q(i) = node["joint_" + std::to_string(i + 1)].as<double>();
    return q;
}

class DualArmGoalGenerator
{
public:
    DualArmGoalGenerator(
        const std::string &urdf_file,
        const GoalGeneratorOptions &options,
        const Vec12 &reference)
        : options_(options),
          reference_(reference),
          rng_(options.random_seed),
          q_min_(6),
          q_max_(6)
    {
        initialized_ = kdl_parser::treeFromFile(urdf_file, tree_) &&
                       tree_.getChain("world", options_.left_tip, left_chain_) &&
                       tree_.getChain("world", options_.right_tip, right_chain_) &&
                       left_chain_.getNrOfJoints() == 6 &&
                       right_chain_.getNrOfJoints() == 6;
        if (!initialized_)
        {
            error_ = "cannot build two six-joint KDL chains from URDF world";
            return;
        }

        for (int i = 0; i < 6; ++i)
        {
            q_min_(i) = i < 3 ? -M_PI : -2.0 * M_PI;
            q_max_(i) = i < 3 ? M_PI : 2.0 * M_PI;
        }

        left_tool_to_dh_.setIdentity();
        right_tool_to_dh_.setIdentity();
        if (options_.target_from_reference)
        {
            const Eigen::Matrix4d left =
                chainForward(left_chain_, reference_.head<6>());
            Eigen::Matrix4d right =
                chainForward(right_chain_, reference_.tail<6>());
            if (options_.preserve_right_world_pose)
                right.block<3, 1>(0, 3) -= options_.right_base_translation_delta;
            if (options_.preserve_left_reference)
            {
                Eigen::Vector3d axis =
                    right.block<3, 1>(0, 3) - left.block<3, 1>(0, 3);
                if (axis.norm() < 1e-9)
                    axis = Eigen::Vector3d::UnitY();
                axis.normalize();
                const double requested_distance = std::max(
                    options_.distance_min,
                    std::min(options_.distance_max,
                             (right.block<3, 1>(0, 3) -
                              left.block<3, 1>(0, 3)).norm()));
                options_.target = left.block<3, 1>(0, 3) +
                                  0.5 * requested_distance * axis;
            }
            else
            {
                options_.target =
                    0.5 * (left.block<3, 1>(0, 3) + right.block<3, 1>(0, 3));
            }
        }
        options_.target += options_.target_offset;
    }

    bool initialized() const { return initialized_; }
    const std::string &error() const { return error_; }
    const Eigen::Vector3d &target() const { return options_.target; }
    Eigen::Matrix4d forwardPose(bool left, const Vec6 &q) const
    {
        return chainForward(left ? left_chain_ : right_chain_, q);
    }

    std::vector<LocalGoalCandidate> generateLocalCandidates(bool left)
    {
        std::vector<LocalGoalCandidate> result;
        if (!initialized_)
            return result;

        const Eigen::Matrix4d reference_left =
            chainForward(left_chain_, reference_.head<6>());
        Eigen::Matrix4d reference_right =
            chainForward(right_chain_, reference_.tail<6>());
        if (options_.preserve_right_world_pose)
            reference_right.block<3, 1>(0, 3) -=
                options_.right_base_translation_delta;
        Eigen::Vector3d reference_axis =
            reference_right.block<3, 1>(0, 3) -
            reference_left.block<3, 1>(0, 3);
        if (reference_axis.norm() < 1e-9)
            reference_axis = Eigen::Vector3d::UnitY();
        reference_axis.normalize();

        const int axis_count = std::max(1, options_.axis_samples);
        const int roll_count = std::max(1, options_.roll_samples);
        for (int axis_index = 0; axis_index < axis_count; ++axis_index)
        {
            const Eigen::Vector3d axis =
                axis_index == 0
                    ? reference_axis
                    : sampleGraspAxis(axis_index, axis_count);
            const double u = axis_count == 1
                                 ? 0.5
                                 : static_cast<double>(axis_index) /
                                       static_cast<double>(axis_count - 1);
            const double distance =
                axis_index == 0
                    ? std::max(options_.distance_min,
                               std::min(options_.distance_max,
                                        (reference_right.block<3, 1>(0, 3) -
                                         reference_left.block<3, 1>(0, 3))
                                            .norm()))
                    : options_.distance_min +
                          u * (options_.distance_max - options_.distance_min);
            for (int roll_index = 0; roll_index < roll_count; ++roll_index)
            {
                Eigen::Matrix4d desired = Eigen::Matrix4d::Identity();
                desired.block<3, 1>(0, 3) =
                    options_.target +
                    (left ? -0.5 : 0.5) * distance * axis;
                if (axis_index == 0 && roll_index == 0)
                {
                    desired.block<3, 3>(0, 0) =
                        (left ? reference_left : reference_right)
                            .block<3, 3>(0, 0);
                }
                else
                {
                    const double roll =
                        2.0 * M_PI * static_cast<double>(roll_index) /
                        static_cast<double>(roll_count);
                    desired.block<3, 3>(0, 0) =
                        left ? rotationWithZAxis(axis, roll)
                             : rotationWithZAxis(-axis, -roll);
                }
                const Vec6 local_reference =
                    left ? Vec6(reference_.head<6>())
                         : Vec6(reference_.tail<6>());
                const std::vector<IKCandidate> candidates = solveForPose(
                    left, desired, local_reference);
                for (const IKCandidate &candidate : candidates)
                {
                    LocalGoalCandidate local;
                    local.sample_index = axis_index * roll_count + roll_index;
                    local.axis_index = axis_index;
                    local.roll_index = roll_index;
                    local.ik = candidate;
                    result.push_back(local);
                }
            }
        }
        return result;
    }

    DualArmGoal generate(const PairValidityFunction &pair_valid = {})
    {
        const auto started = std::chrono::steady_clock::now();
        DualArmGoal result;
        if (!initialized_)
        {
            result.message = error_;
            return result;
        }

        const Eigen::Matrix4d reference_left =
            chainForward(left_chain_, reference_.head<6>());
        Eigen::Matrix4d reference_right =
            chainForward(right_chain_, reference_.tail<6>());
        if (options_.preserve_right_world_pose)
            reference_right.block<3, 1>(0, 3) -=
                options_.right_base_translation_delta;
        Eigen::Vector3d reference_axis =
            reference_right.block<3, 1>(0, 3) - reference_left.block<3, 1>(0, 3);
        if (reference_axis.norm() < 1e-9)
            reference_axis = Eigen::Vector3d::UnitY();
        reference_axis.normalize();

        std::ofstream debug;
        if (!options_.debug_file.empty())
            debug.open(options_.debug_file, std::ios::out | std::ios::app);
        if (debug)
        {
            debug << "\n=== TCST IK goal generation ===\n"
                  << "target: " << options_.target.transpose() << "\n"
                  << "distance_range: [" << options_.distance_min << ", "
                  << options_.distance_max << "]\n";
        }

        const int axis_count = std::max(1, options_.axis_samples);
        const int roll_count = std::max(1, options_.roll_samples);
        bool stop_search = false;
        for (int axis_index = 0; axis_index < axis_count; ++axis_index)
        {
            Eigen::Vector3d axis =
                axis_index == 0 ? reference_axis : sampleGraspAxis(axis_index, axis_count);
            const double u = axis_count == 1
                                 ? 0.5
                                 : static_cast<double>(axis_index) /
                                       static_cast<double>(axis_count - 1);
            const double distance =
                axis_index == 0
                    ? std::max(options_.distance_min,
                               std::min(options_.distance_max,
                                        (reference_right.block<3, 1>(0, 3) -
                                         reference_left.block<3, 1>(0, 3))
                                            .norm()))
                    : options_.distance_min +
                          u * (options_.distance_max - options_.distance_min);

            for (int roll_index = 0; roll_index < roll_count; ++roll_index)
            {
                Eigen::Matrix4d desired_left = Eigen::Matrix4d::Identity();
                Eigen::Matrix4d desired_right = Eigen::Matrix4d::Identity();
                desired_left.block<3, 1>(0, 3) =
                    options_.target - 0.5 * distance * axis;
                desired_right.block<3, 1>(0, 3) =
                    options_.target + 0.5 * distance * axis;

                if (axis_index == 0 && roll_index == 0)
                {
                    desired_left.block<3, 3>(0, 0) =
                        reference_left.block<3, 3>(0, 0);
                    desired_right.block<3, 3>(0, 0) =
                        reference_right.block<3, 3>(0, 0);
                }
                else
                {
                    const double roll =
                        2.0 * M_PI * static_cast<double>(roll_index) /
                        static_cast<double>(roll_count);
                    desired_left.block<3, 3>(0, 0) = rotationWithZAxis(axis, roll);
                    desired_right.block<3, 3>(0, 0) =
                        rotationWithZAxis(-axis, -roll);
                }

                const std::vector<IKCandidate> left_candidates =
                    solveIKMultiStart(left_chain_, left_tool_to_dh_, desired_left,
                                      reference_.head<6>(), true);
                const std::vector<IKCandidate> right_candidates =
                    solveIKMultiStart(right_chain_, right_tool_to_dh_, desired_right,
                                      reference_.tail<6>(), false);
                result.left_candidate_count += left_candidates.size();
                result.right_candidate_count += right_candidates.size();

                if (debug)
                {
                    double left_min_position_error =
                        std::numeric_limits<double>::infinity();
                    double right_min_position_error =
                        std::numeric_limits<double>::infinity();
                    double left_min_orientation_error =
                        std::numeric_limits<double>::infinity();
                    double right_min_orientation_error =
                        std::numeric_limits<double>::infinity();
                    for (const IKCandidate &candidate : left_candidates)
                    {
                        left_min_position_error = std::min(
                            left_min_position_error, candidate.position_error);
                        left_min_orientation_error = std::min(
                            left_min_orientation_error, candidate.orientation_error);
                    }
                    for (const IKCandidate &candidate : right_candidates)
                    {
                        right_min_position_error = std::min(
                            right_min_position_error, candidate.position_error);
                        right_min_orientation_error = std::min(
                            right_min_orientation_error, candidate.orientation_error);
                    }
                    debug << "sample axis=" << axis.transpose()
                          << " dEE=" << distance
                          << " roll_index=" << roll_index
                          << " candidates=" << left_candidates.size() << "/"
                          << right_candidates.size()
                          << " min_pos_err=" << left_min_position_error << "/"
                          << right_min_position_error
                          << " min_rot_err=" << left_min_orientation_error << "/"
                          << right_min_orientation_error << "\n";
                }

                for (const IKCandidate &left : left_candidates)
                {
                    for (const IKCandidate &right : right_candidates)
                    {
                        ++result.tested_pair_count;
                        Vec12 q;
                        q << left.q, right.q;
                        TaskMetrics metrics;
                        if (!checkDualGoalConstraint(q, metrics))
                            continue;
                        if (pair_valid && !pair_valid(q))
                            continue;
                        ++result.valid_pair_count;
                        const double score =
                            scoreGoalPair(q, left, right);
                        if (score < result.score)
                        {
                            result.success = true;
                            result.q = q;
                            result.metrics = metrics;
                            result.score = score;
                        }
                        if (options_.stop_at_first_valid)
                        {
                            stop_search = true;
                            break;
                        }
                    }
                    if (stop_search)
                        break;
                }
                if (stop_search)
                    break;
            }
            if (stop_search)
                break;
        }

        result.generation_time_ms =
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - started)
                .count();
        result.message = result.success ? "automatic IK goal selected"
                                        : "no collision-free dual-arm IK pair";
        if (result.success && !options_.output_file.empty())
        {
            std::ofstream output(options_.output_file, std::ios::out | std::ios::app);
            if (output)
            {
                output << std::setprecision(12) << "q: [";
                for (int i = 0; i < 12; ++i)
                    output << (i == 0 ? "" : ", ") << result.q(i);
                output << "]  # score=" << result.score
                       << " time_ms=" << result.generation_time_ms << "\n";
            }
        }
        if (debug)
        {
            debug << std::setprecision(12)
                  << "success: " << result.success << "\n"
                  << "selected_q: " << result.q.transpose() << "\n"
                  << "distance: " << result.metrics.distance << "\n"
                  << "cosines: " << result.metrics.left_facing_cos << " "
                  << result.metrics.right_facing_cos << " "
                  << result.metrics.left_target_cos << " "
                  << result.metrics.right_target_cos << "\n"
                  << "pair_counts: " << result.tested_pair_count << " "
                  << result.valid_pair_count << "\n"
                  << "score: " << result.score << "\n"
                  << "time_ms: " << result.generation_time_ms << "\n";
        }
        return result;
    }

    std::vector<IKCandidate> solveForPose(
        bool left, const Eigen::Matrix4d &desired_dh, const Vec6 &reference)
    {
        return solveIKMultiStart(
            left ? left_chain_ : right_chain_,
            left ? left_tool_to_dh_ : right_tool_to_dh_,
            desired_dh, reference, left);
    }

    bool checkDualGoalConstraint(const Vec12 &q, TaskMetrics &metrics) const
    {
        const Eigen::Matrix4d left = chainForward(left_chain_, q.head<6>());
        const Eigen::Matrix4d right = chainForward(right_chain_, q.tail<6>());
        const Eigen::Vector3d p_left = left.block<3, 1>(0, 3);
        const Eigen::Vector3d p_right = right.block<3, 1>(0, 3);
        const Eigen::Vector3d between = p_right - p_left;
        metrics.distance = between.norm();
        if (metrics.distance < 1e-9)
            return false;

        const Eigen::Vector3d direction = between / metrics.distance;
        const Eigen::Vector3d left_axis = left.block<3, 1>(0, 2).normalized();
        const Eigen::Vector3d right_axis = right.block<3, 1>(0, 2).normalized();
        const Eigen::Vector3d left_to_target = options_.target - p_left;
        const Eigen::Vector3d right_to_target = options_.target - p_right;
        if (left_to_target.norm() < 1e-9 || right_to_target.norm() < 1e-9)
            return false;

        metrics.left_facing_cos = left_axis.dot(direction);
        metrics.right_facing_cos = right_axis.dot(-direction);
        metrics.left_target_cos =
            left_axis.dot(left_to_target.normalized());
        metrics.right_target_cos =
            right_axis.dot(right_to_target.normalized());

        return metrics.distance >= options_.distance_min - 1e-6 &&
               metrics.distance <= options_.distance_max + 1e-6 &&
               metrics.left_facing_cos >= std::cos(options_.theta_left) &&
               metrics.right_facing_cos >= std::cos(options_.theta_right) &&
               metrics.left_target_cos >= std::cos(options_.psi_left) &&
               metrics.right_target_cos >= std::cos(options_.psi_right);
    }

private:
    Eigen::Matrix4d chainForward(const KDL::Chain &chain, const Vec6 &q) const
    {
        KDL::ChainFkSolverPos_recursive fk(chain);
        KDL::Frame frame;
        if (fk.JntToCart(eigenToKdl(q), frame) < 0)
            return Eigen::Matrix4d::Identity();
        return kdlToEigen(frame);
    }

    Eigen::Vector3d sampleGraspAxis(int index, int count) const
    {
        const double golden_angle = M_PI * (3.0 - std::sqrt(5.0));
        const double y = 1.0 - 2.0 *
                                   (static_cast<double>(index) + 0.5) /
                                   static_cast<double>(count);
        const double radius = std::sqrt(std::max(0.0, 1.0 - y * y));
        const double azimuth = golden_angle * static_cast<double>(index);
        return Eigen::Vector3d(
                   radius * std::cos(azimuth), y, radius * std::sin(azimuth))
            .normalized();
    }

    std::vector<IKCandidate> solveIKMultiStart(
        const KDL::Chain &chain,
        const Eigen::Matrix4d &tool_to_dh,
        const Eigen::Matrix4d &desired_dh,
        const Vec6 &reference,
        bool left)
    {
        std::vector<IKCandidate> candidates;
        KDL::ChainFkSolverPos_recursive fk(chain);
        KDL::ChainIkSolverVel_pinv velocity_solver(chain);
        KDL::ChainIkSolverPos_NR_JL solver(
            chain, q_min_, q_max_, fk, velocity_solver,
            options_.ik_max_iterations, 1e-7);
         
         
         
         
        KDL::ChainIkSolverPos_LMA lma_solver(
            chain, 1e-7, options_.ik_max_iterations, 1e-15);
        Eigen::Matrix<double, 6, 1> task_weights;
        task_weights << 1.0, 1.0, 1.0, 0.001, 0.001, 0.001;
        KDL::ChainIkSolverPos_LMA task_lma_solver(
            chain, task_weights, 1e-7, options_.ik_max_iterations, 1e-15);

        const Eigen::Matrix4d desired_tool = desired_dh * tool_to_dh.inverse();
        const KDL::Frame desired_frame = eigenToKdl(desired_tool);
        std::uniform_real_distribution<double> unit(0.0, 1.0);
        std::uniform_real_distribution<double> local(
            -options_.local_seed_range, options_.local_seed_range);

        const int restart_count = std::max(1, options_.ik_restarts);
        for (int restart = 0; restart < restart_count; ++restart)
        {
            Vec6 seed = reference;
            if (restart > 0)
            {
                const bool local_seed = unit(rng_) < options_.local_seed_ratio;
                for (int joint = 0; joint < 6; ++joint)
                {
                    if (local_seed)
                        seed(joint) = std::max(
                            q_min_(joint),
                            std::min(q_max_(joint), reference(joint) + local(rng_)));
                    else
                    {
                        std::uniform_real_distribution<double> global(
                            q_min_(joint), q_max_(joint));
                        seed(joint) = global(rng_);
                    }
                }
            }

            KDL::JntArray solution(6);
            int ret = solver.CartToJnt(eigenToKdl(seed), desired_frame, solution);
            if (ret < 0)
                ret = lma_solver.CartToJnt(eigenToKdl(seed), desired_frame, solution);
            if (ret < 0)
                ret = task_lma_solver.CartToJnt(
                    eigenToKdl(seed), desired_frame, solution);
            Vec6 q = kdlToEigen(solution);
            bool within_limits = true;
            for (int joint = 0; joint < 6; ++joint)
            {
                if (!std::isfinite(q(joint)))
                {
                    within_limits = false;
                    continue;
                }
                while (q(joint) > M_PI)
                    q(joint) -= 2.0 * M_PI;
                while (q(joint) < -M_PI)
                    q(joint) += 2.0 * M_PI;
                if (q(joint) < q_min_(joint) - 1e-9 ||
                    q(joint) > q_max_(joint) + 1e-9)
                    within_limits = false;
            }
            if (!within_limits)
                continue;
            const Eigen::Matrix4d actual = chainForward(chain, q);
            IKCandidate candidate;
            candidate.q = q;
            candidate.position_error =
                (actual.block<3, 1>(0, 3) - desired_dh.block<3, 1>(0, 3)).norm();
            candidate.orientation_error = rotationError(
                actual.block<3, 3>(0, 0), desired_dh.block<3, 3>(0, 0));
            if (candidate.position_error > options_.ik_position_tolerance ||
                candidate.orientation_error > options_.ik_orientation_tolerance)
                continue;

            bool duplicate = false;
            for (const IKCandidate &existing : candidates)
            {
                double difference_squared = 0.0;
                for (int joint = 0; joint < 6; ++joint)
                {
                    const double difference =
                        periodicDifference(q(joint), existing.q(joint));
                    difference_squared += difference * difference;
                }
                if (std::sqrt(difference_squared) < options_.dedup_tolerance)
                {
                    duplicate = true;
                    break;
                }
            }
            if (!duplicate)
                candidates.push_back(candidate);
        }
        return candidates;
    }

    double scoreGoalPair(
        const Vec12 &q,
        const IKCandidate &left,
        const IKCandidate &right) const
    {
        double joint_cost = 0.0;
        for (int joint = 0; joint < 12; ++joint)
        {
            const double difference =
                periodicDifference(q(joint), reference_(joint));
            joint_cost += options_.joint_score_weights(joint) *
                          difference * difference;
        }
        const double pose_cost =
            left.position_error + right.position_error +
            left.orientation_error + right.orientation_error;
        return options_.joint_score_weight * joint_cost +
               options_.pose_score_weight * pose_cost;
    }

    GoalGeneratorOptions options_;
    Vec12 reference_;
    std::mt19937 rng_;
    KDL::Tree tree_;
    KDL::Chain left_chain_;
    KDL::Chain right_chain_;
    KDL::JntArray q_min_;
    KDL::JntArray q_max_;
    Eigen::Matrix4d left_tool_to_dh_ = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d right_tool_to_dh_ = Eigen::Matrix4d::Identity();
    bool initialized_ = false;
    std::string error_;
};

}  
}  
