#pragma once

#include "tcst_ik_goal_generator.hpp"

#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <string>
#include <vector>

namespace tcst
{
namespace box_goal
{

using ik_goal::IKCandidate;
using ik_goal::PairValidityFunction;
using ik_goal::TaskMode;
using ik_goal::Vec6;
using ik_goal::Vec12;

struct BoxGoalOptions
{
    TaskMode mode = TaskMode::CATCH;
    bool target_from_reference = true;
    bool orientation_from_reference = false;
    bool grasp_along_long_axis = true;
    bool include_reference_candidate = true;
    double minimum_joint_change_from_reference = 0.0;
    bool stop_at_first_valid = true;
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    Eigen::Vector3d dimensions = Eigen::Vector3d(0.17, 0.193, 0.101);
    Eigen::Vector3d rpy = Eigen::Vector3d::Zero();
    int long_axis = 1;
    int grasp_normal_axis = 0;
    int parallel_tool_axis = -1;  
    double distance_min = 0.31;
    double distance_max = 0.34;
    double theta_left = 0.318;
    double theta_right = 0.25;
    double normal_tolerance = 0.20;
    double parallel_tolerance = 0.20;
    double midpoint_normal_tolerance = 0.02;
    double longitudinal_margin = 0.01;
    int longitudinal_samples = 3;
    int roll_samples = 8;
    std::string output_file;
    std::string debug_file;
};

struct BoxMetrics
{
    double distance = 0.0;
    double normal_alignment = -1.0;
    double left_facing_cos = -1.0;
    double right_facing_cos = -1.0;
    double left_parallel_cos = -1.0;
    double right_parallel_cos = -1.0;
    double midpoint_normal_error = std::numeric_limits<double>::infinity();
    double longitudinal_offset = std::numeric_limits<double>::infinity();
};

struct BoxGoal
{
    bool success = false;
    Vec12 q = Vec12::Zero();
    BoxMetrics metrics;
    double score = std::numeric_limits<double>::infinity();
    std::size_t tested_pairs = 0;
    std::size_t valid_pairs = 0;
    double time_ms = 0.0;
    std::string message;
};

inline int axisIndex(const std::string &name, int fallback)
{
    if (name == "x" || name == "X")
        return 0;
    if (name == "y" || name == "Y")
        return 1;
    if (name == "z" || name == "Z")
        return 2;
    return fallback;
}

inline Eigen::Matrix3d rpyRotation(const Eigen::Vector3d &rpy)
{
    return (Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX()))
        .toRotationMatrix();
}

inline BoxGoalOptions loadBoxOptions(const YAML::Node &config)
{
    BoxGoalOptions options;
    const YAML::Node node = config["box_goal_generator"];
    if (!node)
        return options;

    if (node["mode"])
    {
        std::string mode = node["mode"].as<std::string>();
        std::transform(mode.begin(), mode.end(), mode.begin(), ::toupper);
        options.mode = mode == "MOVE" ? TaskMode::MOVE : TaskMode::CATCH;
    }
    if (node["target_from_reference"])
        options.target_from_reference = node["target_from_reference"].as<bool>();
    if (node["orientation_from_reference"])
        options.orientation_from_reference =
            node["orientation_from_reference"].as<bool>();
    if (node["grasp_along_long_axis"])
        options.grasp_along_long_axis =
            node["grasp_along_long_axis"].as<bool>();
    if (node["include_reference_candidate"])
        options.include_reference_candidate =
            node["include_reference_candidate"].as<bool>();
    if (node["minimum_joint_change_from_reference"])
        options.minimum_joint_change_from_reference =
            node["minimum_joint_change_from_reference"].as<double>();
    if (node["stop_at_first_valid"])
        options.stop_at_first_valid = node["stop_at_first_valid"].as<bool>();
    if (node["center"])
        options.center << node["center"]["x"].as<double>(),
            node["center"]["y"].as<double>(),
            node["center"]["z"].as<double>();
    if (node["dimensions"])
        options.dimensions << node["dimensions"]["x"].as<double>(),
            node["dimensions"]["y"].as<double>(),
            node["dimensions"]["z"].as<double>();
    if (node["rpy"])
        options.rpy << node["rpy"]["roll"].as<double>(),
            node["rpy"]["pitch"].as<double>(),
            node["rpy"]["yaw"].as<double>();
    if (node["long_axis"])
        options.long_axis = axisIndex(node["long_axis"].as<std::string>(), 1);
    if (node["grasp_normal_axis"])
        options.grasp_normal_axis =
            axisIndex(node["grasp_normal_axis"].as<std::string>(), 0);
    if (node["parallel_tool_axis"])
    {
        const std::string axis = node["parallel_tool_axis"].as<std::string>();
        options.parallel_tool_axis =
            axis == "auto" ? -1 : axisIndex(axis, -1);
    }
    if (node["distance_min"])
        options.distance_min = node["distance_min"].as<double>();
    if (node["distance_max"])
        options.distance_max = node["distance_max"].as<double>();
    if (node["theta_left"])
        options.theta_left = node["theta_left"].as<double>();
    if (node["theta_right"])
        options.theta_right = node["theta_right"].as<double>();
    if (node["normal_tolerance"])
        options.normal_tolerance = node["normal_tolerance"].as<double>();
    if (node["parallel_tolerance"])
        options.parallel_tolerance = node["parallel_tolerance"].as<double>();
    if (node["midpoint_normal_tolerance"])
        options.midpoint_normal_tolerance =
            node["midpoint_normal_tolerance"].as<double>();
    if (node["longitudinal_margin"])
        options.longitudinal_margin = node["longitudinal_margin"].as<double>();
    if (node["longitudinal_samples"])
        options.longitudinal_samples = node["longitudinal_samples"].as<int>();
    if (node["roll_samples"])
        options.roll_samples = node["roll_samples"].as<int>();
    if (node["output_file"])
        options.output_file = node["output_file"].as<std::string>();
    if (node["debug_file"])
        options.debug_file = node["debug_file"].as<std::string>();
    return options;
}

class BoxDualArmGoalGenerator
{
public:
    BoxDualArmGoalGenerator(
        ik_goal::DualArmGoalGenerator &ik,
        const BoxGoalOptions &options,
        const Vec12 &reference)
        : ik_(ik), options_(options), reference_(reference)
    {
        const Eigen::Matrix4d left =
            ik_.forwardPose(true, reference_.head<6>());
        const Eigen::Matrix4d right =
            ik_.forwardPose(false, reference_.tail<6>());
        reference_left_rotation_ = left.block<3, 3>(0, 0);
        reference_right_rotation_ = right.block<3, 3>(0, 0);
        reference_left_position_ = left.block<3, 1>(0, 3);
        reference_right_position_ = right.block<3, 1>(0, 3);
        reference_distance_ =
            (reference_right_position_ - reference_left_position_).norm();
        reference_direction_ =
            (right.block<3, 1>(0, 3) - left.block<3, 1>(0, 3)).normalized();
        if (options_.target_from_reference)
            options_.center =
                0.5 * (left.block<3, 1>(0, 3) + right.block<3, 1>(0, 3));

        if (options_.orientation_from_reference)
        {
            normal_ = reference_direction_;
            if (options_.grasp_along_long_axis)
            {
                long_axis_ = normal_;
                tangent_axis_ = left.block<3, 1>(0, 0);
                tangent_axis_ -=
                    normal_ * normal_.dot(tangent_axis_);
                if (tangent_axis_.norm() < 1e-9)
                    tangent_axis_ = left.block<3, 1>(0, 1);
                tangent_axis_.normalize();
                box_rotation_ = left.block<3, 3>(0, 0);
            }
            else
            {
                const int preferred_axis =
                    options_.parallel_tool_axis == 1 ? 1 : 0;
                long_axis_ = left.block<3, 1>(0, preferred_axis);
                long_axis_ -= normal_ * normal_.dot(long_axis_);
                if (long_axis_.norm() < 1e-9)
                    long_axis_ = left.block<3, 1>(0, 1 - preferred_axis);
                long_axis_.normalize();
                tangent_axis_ = long_axis_;
            }
        }
        else
        {
            box_rotation_ = rpyRotation(options_.rpy);
            long_axis_ =
                box_rotation_.col(options_.long_axis).normalized();
            normal_ = options_.grasp_along_long_axis
                          ? long_axis_
                          : box_rotation_
                                .col(options_.grasp_normal_axis)
                                .normalized();
            if (normal_.dot(reference_direction_) < 0.0)
                normal_ = -normal_;
            if (options_.grasp_along_long_axis &&
                std::abs(long_axis_.dot(reference_direction_)) >=
                    std::cos(options_.normal_tolerance))
                normal_ = reference_direction_;
            int tangent_index = 2;
            if (tangent_index == options_.long_axis)
                tangent_index = 0;
            tangent_axis_ = box_rotation_.col(tangent_index).normalized();
        }
        if (!options_.grasp_along_long_axis)
        {
            long_axis_ -= normal_ * normal_.dot(long_axis_);
            long_axis_.normalize();
            tangent_axis_ = long_axis_;
        }
        tangent_axis_ -= normal_ * normal_.dot(tangent_axis_);
        tangent_axis_.normalize();
    }

    const Eigen::Vector3d &center() const { return options_.center; }
    const Eigen::Vector3d &normal() const { return normal_; }
    const Eigen::Vector3d &longAxis() const { return long_axis_; }
    const Eigen::Matrix3d &boxRotation() const { return box_rotation_; }

    BoxGoal generate(const PairValidityFunction &pair_valid = {})
    {
        const auto started = std::chrono::steady_clock::now();
        BoxGoal result;
        std::ofstream debug;
        if (!options_.debug_file.empty())
            debug.open(options_.debug_file, std::ios::out | std::ios::app);

        const std::vector<int> tool_axes =
            options_.parallel_tool_axis < 0
                ? std::vector<int>{0, 1}
                : std::vector<int>{options_.parallel_tool_axis};
        const int offset_count =
            options_.grasp_along_long_axis
                ? 1
                : std::max(1, options_.longitudinal_samples);
        bool stop = false;

        const int first_distance_index =
            options_.include_reference_candidate ? 0 : 1;
        for (int distance_index = first_distance_index;
             distance_index < 4 && !stop;
             ++distance_index)
        {
            const double fraction =
                distance_index == 0 ? 0.0 : 0.5 * (distance_index - 1);
            const double distance =
                distance_index == 0
                    ? reference_distance_
                    : options_.distance_min +
                          fraction *
                              (options_.distance_max - options_.distance_min);
            for (int offset_index = 0; offset_index < offset_count && !stop;
                 ++offset_index)
            {
                const double centered =
                    offset_count == 1
                        ? 0.0
                        : 2.0 * static_cast<double>(offset_index) /
                                  static_cast<double>(offset_count - 1) -
                              1.0;
                const double maximum_offset =
                    std::max(0.0,
                             0.5 * options_.dimensions(options_.long_axis) -
                                 options_.longitudinal_margin);
                const double longitudinal_offset =
                    centered * 0.5 * maximum_offset;
                const Eigen::Vector3d grasp_center =
                    options_.center + longitudinal_offset *
                                          (options_.grasp_along_long_axis
                                               ? tangent_axis_
                                               : long_axis_);
                const Eigen::Vector3d left_position =
                    distance_index == 0
                        ? reference_left_position_
                        : grasp_center - 0.5 * distance * normal_;
                const Eigen::Vector3d right_position =
                    distance_index == 0
                        ? reference_right_position_
                        : grasp_center + 0.5 * distance * normal_;

                for (const int tool_axis : tool_axes)
                {
                    for (int tangent_sign = -1; tangent_sign <= 1;
                         tangent_sign += 2)
                    {
                        Eigen::Matrix4d desired_left =
                            Eigen::Matrix4d::Identity();
                        Eigen::Matrix4d desired_right =
                            Eigen::Matrix4d::Identity();
                        desired_left.block<3, 1>(0, 3) = left_position;
                        desired_right.block<3, 1>(0, 3) = right_position;
                        if (tool_axis == 0 && tangent_sign == -1)
                        {
                             
                             
                             
                            desired_left.block<3, 3>(0, 0) =
                                reference_left_rotation_;
                            desired_right.block<3, 3>(0, 0) =
                                reference_right_rotation_;
                        }
                        else
                        {
                            desired_left.block<3, 3>(0, 0) =
                                makeToolRotation(
                                    normal_, tangent_sign * tangent_axis_,
                                    tool_axis);
                            desired_right.block<3, 3>(0, 0) =
                                makeToolRotation(
                                    -normal_, tangent_sign * tangent_axis_,
                                    tool_axis);
                        }

                        const std::vector<IKCandidate> left_candidates =
                            ik_.solveForPose(
                                true, desired_left, reference_.head<6>());
                        const std::vector<IKCandidate> right_candidates =
                            ik_.solveForPose(
                                false, desired_right, reference_.tail<6>());
                        if (debug)
                            debug << "d=" << distance
                                  << " offset=" << longitudinal_offset
                                  << " tool_axis=" << tool_axis
                                  << " sign=" << tangent_sign
                                  << " candidates=" << left_candidates.size()
                                  << "/" << right_candidates.size() << "\n";

                        for (const IKCandidate &left : left_candidates)
                        {
                            for (const IKCandidate &right : right_candidates)
                            {
                                ++result.tested_pairs;
                                Vec12 q;
                                q << left.q, right.q;
                                double joint_change_squared = 0.0;
                                for (int joint = 0; joint < 12; ++joint)
                                {
                                    const double difference =
                                        ik_goal::periodicDifference(
                                            q(joint), reference_(joint));
                                    joint_change_squared +=
                                        difference * difference;
                                }
                                if (std::sqrt(joint_change_squared) <
                                    options_
                                        .minimum_joint_change_from_reference)
                                    continue;
                                BoxMetrics metrics;
                                if (!checkConstraint(q, tool_axis, metrics))
                                    continue;
                                if (pair_valid && !pair_valid(q))
                                    continue;
                                ++result.valid_pairs;
                                const double score =
                                    scorePair(q, left, right);
                                if (score < result.score)
                                {
                                    result.success = true;
                                    result.q = q;
                                    result.metrics = metrics;
                                    result.score = score;
                                }
                                if (options_.stop_at_first_valid)
                                {
                                    stop = true;
                                    break;
                                }
                            }
                            if (stop)
                                break;
                        }
                        if (stop)
                            break;
                    }
                    if (stop)
                        break;
                }
            }
        }

        result.time_ms =
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - started)
                .count();
        result.message =
            result.success ? "parallel-long-edge box goal selected"
                           : "no feasible parallel-long-edge box goal";
        if (result.success && !options_.output_file.empty())
        {
            std::ofstream output(
                options_.output_file, std::ios::out | std::ios::app);
            output << std::setprecision(12) << "q: [";
            for (int i = 0; i < 12; ++i)
                output << (i == 0 ? "" : ", ") << result.q(i);
            output << "]  # score=" << result.score
                   << " time_ms=" << result.time_ms << "\n";
        }
        if (debug)
        {
            debug << std::setprecision(12)
                  << "success=" << result.success
                  << " q=" << result.q.transpose()
                  << " metrics=" << result.metrics.distance << " "
                  << result.metrics.normal_alignment << " "
                  << result.metrics.left_facing_cos << " "
                  << result.metrics.right_facing_cos << " "
                  << result.metrics.left_parallel_cos << " "
                  << result.metrics.right_parallel_cos
                  << " time_ms=" << result.time_ms << "\n";
        }
        return result;
    }

    bool checkConstraint(
        const Vec12 &q, int tool_axis, BoxMetrics &metrics) const
    {
        const Eigen::Matrix4d left =
            ik_.forwardPose(true, q.head<6>());
        const Eigen::Matrix4d right =
            ik_.forwardPose(false, q.tail<6>());
        const Eigen::Vector3d p_left = left.block<3, 1>(0, 3);
        const Eigen::Vector3d p_right = right.block<3, 1>(0, 3);
        const Eigen::Vector3d between = p_right - p_left;
        metrics.distance = between.norm();
        if (metrics.distance < 1e-9)
            return false;
        const Eigen::Vector3d direction = between / metrics.distance;
        metrics.normal_alignment = std::abs(direction.dot(normal_));
        metrics.left_facing_cos =
            left.block<3, 1>(0, 2).normalized().dot(direction);
        metrics.right_facing_cos =
            right.block<3, 1>(0, 2).normalized().dot(-direction);
        if (options_.grasp_along_long_axis)
        {
            metrics.left_parallel_cos = std::abs(
                left.block<3, 1>(0, 2).normalized().dot(long_axis_));
            metrics.right_parallel_cos = std::abs(
                right.block<3, 1>(0, 2).normalized().dot(long_axis_));
        }
        else
        {
            metrics.left_parallel_cos = std::abs(
                left.block<3, 1>(0, tool_axis).normalized().dot(long_axis_));
            metrics.right_parallel_cos = std::abs(
                right.block<3, 1>(0, tool_axis).normalized().dot(long_axis_));
        }
        const Eigen::Vector3d midpoint = 0.5 * (p_left + p_right);
        const Eigen::Vector3d center_error = midpoint - options_.center;
        metrics.midpoint_normal_error = std::abs(center_error.dot(normal_));
        metrics.longitudinal_offset =
            std::abs(center_error.dot(long_axis_));
        const double maximum_longitudinal =
            0.5 * options_.dimensions(options_.long_axis) -
            options_.longitudinal_margin + 1e-6;

        return metrics.distance >= options_.distance_min - 1e-6 &&
               metrics.distance <= options_.distance_max + 1e-6 &&
               metrics.normal_alignment >=
                   std::cos(options_.normal_tolerance) &&
               metrics.left_facing_cos >= std::cos(options_.theta_left) &&
               metrics.right_facing_cos >= std::cos(options_.theta_right) &&
               metrics.left_parallel_cos >=
                   std::cos(options_.parallel_tolerance) &&
               metrics.right_parallel_cos >=
                   std::cos(options_.parallel_tolerance) &&
               metrics.midpoint_normal_error <=
                   options_.midpoint_normal_tolerance &&
               metrics.longitudinal_offset <= maximum_longitudinal;
    }

private:
    Eigen::Matrix3d makeToolRotation(
        const Eigen::Vector3d &inward,
        const Eigen::Vector3d &parallel,
        int tool_axis) const
    {
        const Eigen::Vector3d z = inward.normalized();
        Eigen::Vector3d tangent =
            parallel - z * z.dot(parallel);
        tangent.normalize();
        Eigen::Matrix3d rotation;
        if (tool_axis == 0)
        {
            rotation.col(0) = tangent;
            rotation.col(1) = z.cross(tangent).normalized();
            rotation.col(2) = z;
        }
        else
        {
            rotation.col(1) = tangent;
            rotation.col(0) = tangent.cross(z).normalized();
            rotation.col(2) = z;
        }
        return rotation;
    }

    double scorePair(
        const Vec12 &q,
        const IKCandidate &left,
        const IKCandidate &right) const
    {
        double score = 0.0;
        for (int i = 0; i < 12; ++i)
        {
            const double difference =
                ik_goal::periodicDifference(q(i), reference_(i));
            score += difference * difference;
        }
        return score + 20.0 *
                           (left.position_error + right.position_error +
                            left.orientation_error + right.orientation_error);
    }

    ik_goal::DualArmGoalGenerator &ik_;
    BoxGoalOptions options_;
    Vec12 reference_;
    Eigen::Vector3d normal_ = Eigen::Vector3d::UnitX();
    Eigen::Vector3d long_axis_ = Eigen::Vector3d::UnitY();
    Eigen::Vector3d reference_direction_ = Eigen::Vector3d::UnitX();
    Eigen::Vector3d tangent_axis_ = Eigen::Vector3d::UnitZ();
    Eigen::Matrix3d box_rotation_ = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d reference_left_rotation_ = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d reference_right_rotation_ = Eigen::Matrix3d::Identity();
    Eigen::Vector3d reference_left_position_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d reference_right_position_ = Eigen::Vector3d::Zero();
    double reference_distance_ = 0.0;
};

}  
}  
