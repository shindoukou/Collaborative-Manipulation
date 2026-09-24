#ifndef TCST_DUAL_TRAJECTORY_VALIDATOR_HPP
#define TCST_DUAL_TRAJECTORY_VALIDATOR_HPP

#include "joint_trajectory.hpp"

#include <Eigen/Eigen>

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <sstream>
#include <string>

namespace tcst
{
    struct DualTrajectoryValidationOptions
    {
        Eigen::Matrix<double, 6, 1> qMinLeft =
            Eigen::Matrix<double, 6, 1>::Constant(-2.0 * M_PI);
        Eigen::Matrix<double, 6, 1> qMaxLeft =
            Eigen::Matrix<double, 6, 1>::Constant(2.0 * M_PI);
        Eigen::Matrix<double, 6, 1> qMinRight =
            Eigen::Matrix<double, 6, 1>::Constant(-2.0 * M_PI);
        Eigen::Matrix<double, 6, 1> qMaxRight =
            Eigen::Matrix<double, 6, 1>::Constant(2.0 * M_PI);
        Eigen::Matrix<double, 6, 1> vMaxLeft =
            Eigen::Matrix<double, 6, 1>::Constant(0.5 * M_PI);
        Eigen::Matrix<double, 6, 1> vMaxRight =
            Eigen::Matrix<double, 6, 1>::Constant(0.5 * M_PI);
        double samplePeriod = 0.01;
        double tolerance = 1.0e-6;
    };

    struct DualTrajectoryValidationResult
    {
        bool valid = false;
        double durationDifference = std::numeric_limits<double>::infinity();
        double maxJointViolation = 0.0;
        double maxVelocityViolation = 0.0;
        double firstInvalidTime = -1.0;
        std::string failureReason;
    };

    using DualStateValidityFunction = std::function<bool(
        const Eigen::Matrix<double, 6, 1> &,
        const Eigen::Matrix<double, 6, 1> &)>;

    inline DualTrajectoryValidationResult validateDualTrajectory(
        const JointTrajectory6 &left,
        const JointTrajectory6 &right,
        const DualTrajectoryValidationOptions &options,
        const DualStateValidityFunction &isStateValid = {})
    {
        DualTrajectoryValidationResult result;
        if (left.empty() || right.empty())
        {
            result.failureReason = "trajectory is empty";
            return result;
        }
        if (left.size() != right.size())
        {
            result.failureReason = "left/right piece counts differ";
            return result;
        }

        for (std::size_t i = 0; i < left.size(); ++i)
        {
            const double difference =
                std::abs(left.pieces[i].duration - right.pieces[i].duration);
            result.durationDifference =
                std::min(result.durationDifference, difference);
            if (difference > options.tolerance)
            {
                result.failureReason = "left/right piece durations differ";
                return result;
            }
        }

        result.durationDifference =
            std::abs(left.getTotalDuration() - right.getTotalDuration());
        if (result.durationDifference > options.tolerance)
        {
            result.failureReason = "left/right total durations differ";
            return result;
        }

        const double totalDuration = left.getTotalDuration();
        const double dt = std::max(options.samplePeriod, 1.0e-4);
        const int sampleCount =
            std::max(1, static_cast<int>(std::ceil(totalDuration / dt)));

        for (int sample = 0; sample <= sampleCount; ++sample)
        {
            const double time =
                totalDuration * static_cast<double>(sample) /
                static_cast<double>(sampleCount);
            const auto qLeft = left.getPos(time);
            const auto qRight = right.getPos(time);
            const auto vLeft = left.getVel(time);
            const auto vRight = right.getVel(time);

            for (int joint = 0; joint < 6; ++joint)
            {
                result.maxJointViolation = std::max(
                    result.maxJointViolation,
                    std::max({
                        qLeft(joint) - options.qMaxLeft(joint),
                        options.qMinLeft(joint) - qLeft(joint),
                        qRight(joint) - options.qMaxRight(joint),
                        options.qMinRight(joint) - qRight(joint),
                        0.0}));
                result.maxVelocityViolation = std::max(
                    result.maxVelocityViolation,
                    std::max({
                        std::abs(vLeft(joint)) - options.vMaxLeft(joint),
                        std::abs(vRight(joint)) - options.vMaxRight(joint),
                        0.0}));
            }

            if (result.maxJointViolation > options.tolerance)
            {
                result.firstInvalidTime = time;
                result.failureReason = "joint position limit violated";
                return result;
            }
            if (result.maxVelocityViolation > options.tolerance)
            {
                result.firstInvalidTime = time;
                result.failureReason = "joint velocity limit violated";
                return result;
            }
            if (isStateValid && !isStateValid(qLeft, qRight))
            {
                result.firstInvalidTime = time;
                result.failureReason =
                    "collision or dual-arm task constraint violated";
                return result;
            }
        }

        result.valid = true;
        result.failureReason.clear();
        return result;
    }
}

#endif
