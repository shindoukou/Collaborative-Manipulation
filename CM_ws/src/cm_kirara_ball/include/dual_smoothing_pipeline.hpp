#ifndef TCST_DUAL_SMOOTHING_PIPELINE_HPP
#define TCST_DUAL_SMOOTHING_PIPELINE_HPP

#include "dual_kirara.hpp"
#include "dual_trajectory_validator.hpp"

#include <Eigen/Eigen>

#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

namespace tcst
{
    inline bool runDualSmoothingPipeline(
        const Eigen::Matrix<double, 6, 3> &headLeft,
        const Eigen::Matrix<double, 6, 3> &tailLeft,
        const Eigen::Matrix<double, 6, 3> &headRight,
        const Eigen::Matrix<double, 6, 3> &tailRight,
        const Eigen::VectorXi &intervalCounts,
        const Eigen::Matrix<double, 6, Eigen::Dynamic> &waypointsLeft,
        const Eigen::Matrix<double, 6, Eigen::Dynamic> &waypointsRight,
        const gcopter::DualKIRARAOptions &optimizerOptions,
        const DualTrajectoryValidationOptions &validationOptions,
        const DualStateValidityFunction &stateValidity,
        const std::string &leftOutputPath,
        const std::string &rightOutputPath)
    {
        using Clock = std::chrono::steady_clock;
        const auto smoothingStageStarted = Clock::now();
        double accumulatedOptimizationSeconds = 0.0;
        double accumulatedValidationSeconds = 0.0;
        constexpr int maximumAttempts = 4;
        for (int attempt = 0; attempt < maximumAttempts; ++attempt)
        {
            gcopter::DualKIRARAOptions attemptOptions = optimizerOptions;
            Eigen::VectorXi attemptIntervals = intervalCounts;
            if (attempt >= 1)
            {
                attemptOptions.weightReference *= 5.0;
            }
            if (attempt >= 2)
            {
                attemptOptions.weightReference *= 5.0;
                attemptIntervals *= 2;
            }
            if (attempt >= 3)
            {
                attemptOptions.weightReference *= 5.0;
                attemptIntervals *= 2;
            }

            std::cout << "DualKIRARA smoothing attempt "
                      << (attempt + 1) << '/' << maximumAttempts
                      << ", reference_weight="
                      << attemptOptions.weightReference
                      << ", pieces=" << attemptIntervals.sum()
                      << std::endl;

            gcopter::DualKIRARA optimizer;
            if (!optimizer.setup(
                    headLeft, tailLeft, headRight, tailRight,
                    attemptIntervals, attemptOptions) ||
                !optimizer.setWaypoints(waypointsLeft, waypointsRight))
            {
                std::cerr << "DualKIRARA setup failed: "
                          << optimizer.lastError() << std::endl;
                continue;
            }

            JointTrajectory6 trajectoryLeft;
            JointTrajectory6 trajectoryRight;
            double finalCost = 0.0;
            const auto optimizationStarted = Clock::now();
            if (!optimizer.optimize(
                    trajectoryLeft, trajectoryRight, finalCost))
            {
                accumulatedOptimizationSeconds +=
                    std::chrono::duration<double>(
                        Clock::now() - optimizationStarted).count();
                std::cerr << "DualKIRARA optimization failed: "
                          << optimizer.lastError() << std::endl;
                continue;
            }
            accumulatedOptimizationSeconds +=
                std::chrono::duration<double>(
                    Clock::now() - optimizationStarted).count();

            const auto validationStarted = Clock::now();
            const auto validation = validateDualTrajectory(
                trajectoryLeft,
                trajectoryRight,
                validationOptions,
                stateValidity);
            accumulatedValidationSeconds +=
                std::chrono::duration<double>(
                    Clock::now() - validationStarted).count();
            if (!validation.valid)
            {
                std::cerr << "Smoothed trajectory validation failed at t="
                          << validation.firstInvalidTime << ": "
                          << validation.failureReason
                          << " (joint violation="
                          << validation.maxJointViolation
                          << ", velocity violation="
                          << validation.maxVelocityViolation << ')'
                          << std::endl;
                continue;
            }

            std::ofstream leftOutput(leftOutputPath);
            std::ofstream rightOutput(rightOutputPath);
            if (!leftOutput || !rightOutput ||
                !optimizer.writeCoefficients(leftOutput, rightOutput))
            {
                std::cerr
                    << "Failed to save complete dual-arm KIRARA coefficients."
                    << std::endl;
                return false;
            }

            std::cout << "DualKIRARA succeeded. cost=" << finalCost
                      << ", shared duration="
                      << trajectoryLeft.getTotalDuration()
                      << ", pieces=" << trajectoryLeft.size()
                      << ", attempt=" << (attempt + 1)
                      << std::endl;
            const double initialCost = optimizer.getInitialCost();
            const double savingRate =
                initialCost > 0.0
                    ? 100.0 * (initialCost - finalCost) / initialCost
                    : 0.0;
            std::cout << std::setprecision(17)
                      << "TCST_COST_METRICS"
                      << " initial_total_cost=" << initialCost
                      << " final_total_cost=" << finalCost
                      << " saving_rate_percent=" << savingRate
                      << " successful_attempt=" << (attempt + 1)
                      << std::endl;
            const double totalSmoothingSeconds =
                std::chrono::duration<double>(
                    Clock::now() - smoothingStageStarted).count();
            std::cout << std::setprecision(17)
                      << "TCST_SMOOTHING_TIME_METRICS"
                      << " optimization_time_s="
                      << accumulatedOptimizationSeconds
                      << " validation_time_s="
                      << accumulatedValidationSeconds
                      << " total_smoothing_stage_time_s="
                      << totalSmoothingSeconds
                      << " successful_attempt=" << (attempt + 1)
                      << std::endl;
            return true;
        }

         
         
         
         
         
        if (waypointsLeft.cols() == waypointsRight.cols() &&
            waypointsLeft.cols() >= 2)
        {
            std::ofstream leftOutput(leftOutputPath);
            std::ofstream rightOutput(rightOutputPath);
            bool fallbackValid = leftOutput.good() && rightOutput.good();
            double fallbackDuration = 0.0;
            if (fallbackValid)
            {
                leftOutput << "# one segment per line\n"
                           << "# duration, then joint1[a0..a5], ..., joint6[a0..a5]\n";
                rightOutput << "# one segment per line\n"
                            << "# duration, then joint1[a0..a5], ..., joint6[a0..a5]\n";
                leftOutput.precision(17);
                rightOutput.precision(17);
            }

            for (int segment = 0;
                 fallbackValid && segment + 1 < waypointsLeft.cols();
                 ++segment)
            {
                const Eigen::Matrix<double, 6, 1> q0Left =
                    waypointsLeft.col(segment);
                const Eigen::Matrix<double, 6, 1> q1Left =
                    waypointsLeft.col(segment + 1);
                const Eigen::Matrix<double, 6, 1> q0Right =
                    waypointsRight.col(segment);
                const Eigen::Matrix<double, 6, 1> q1Right =
                    waypointsRight.col(segment + 1);
                const Eigen::Matrix<double, 6, 1> deltaLeft =
                    q1Left - q0Left;
                const Eigen::Matrix<double, 6, 1> deltaRight =
                    q1Right - q0Right;

                double duration = 0.25;
                for (int joint = 0; joint < 6; ++joint)
                {
                     
                    duration = std::max(
                        duration,
                        1.875 * std::abs(deltaLeft(joint)) /
                            std::max(validationOptions.vMaxLeft(joint), 1.0e-6));
                    duration = std::max(
                        duration,
                        1.875 * std::abs(deltaRight(joint)) /
                            std::max(validationOptions.vMaxRight(joint), 1.0e-6));
                }

                const int samples = std::max(
                    1, static_cast<int>(std::ceil(
                           duration / validationOptions.samplePeriod)));
                for (int sample = 0; sample <= samples; ++sample)
                {
                    const double u = static_cast<double>(sample) /
                                     static_cast<double>(samples);
                    const double u2 = u * u;
                    const double u3 = u2 * u;
                    const double u4 = u3 * u;
                    const double u5 = u4 * u;
                    const double scale = 10.0 * u3 - 15.0 * u4 + 6.0 * u5;
                    const double dscale =
                        (30.0 * u2 - 60.0 * u3 + 30.0 * u4) / duration;
                    const Eigen::Matrix<double, 6, 1> qLeft =
                        q0Left + deltaLeft * scale;
                    const Eigen::Matrix<double, 6, 1> qRight =
                        q0Right + deltaRight * scale;
                    const Eigen::Matrix<double, 6, 1> vLeft =
                        deltaLeft * dscale;
                    const Eigen::Matrix<double, 6, 1> vRight =
                        deltaRight * dscale;
                    for (int joint = 0; joint < 6; ++joint)
                    {
                        fallbackValid = fallbackValid &&
                            qLeft(joint) >= validationOptions.qMinLeft(joint) - 1.0e-9 &&
                            qLeft(joint) <= validationOptions.qMaxLeft(joint) + 1.0e-9 &&
                            qRight(joint) >= validationOptions.qMinRight(joint) - 1.0e-9 &&
                            qRight(joint) <= validationOptions.qMaxRight(joint) + 1.0e-9 &&
                            std::abs(vLeft(joint)) <=
                                validationOptions.vMaxLeft(joint) + 1.0e-9 &&
                            std::abs(vRight(joint)) <=
                                validationOptions.vMaxRight(joint) + 1.0e-9;
                    }
                    if (!fallbackValid || !stateValidity(qLeft, qRight))
                    {
                        fallbackValid = false;
                        std::cerr << "C2 planning fallback validation failed on segment "
                                  << segment << " at u=" << u << std::endl;
                        break;
                    }
                }

                if (!fallbackValid)
                    break;

                const auto writeSegment =
                    [&](std::ostream &output,
                        const Eigen::Matrix<double, 6, 1> &q0,
                        const Eigen::Matrix<double, 6, 1> &delta)
                {
                    output << duration;
                    for (int joint = 0; joint < 6; ++joint)
                    {
                        output << ' ' << q0(joint) << " 0 0 "
                               << 10.0 * delta(joint) / std::pow(duration, 3)
                               << ' ' << -15.0 * delta(joint) / std::pow(duration, 4)
                               << ' ' << 6.0 * delta(joint) / std::pow(duration, 5);
                    }
                    output << '\n';
                };
                writeSegment(leftOutput, q0Left, deltaLeft);
                writeSegment(rightOutput, q0Right, deltaRight);
                fallbackDuration += duration;
            }

            if (fallbackValid && leftOutput.good() && rightOutput.good())
            {
                std::cout << std::setprecision(17)
                          << "TCST_C2_PLANNING_FALLBACK"
                          << " duration_s=" << fallbackDuration
                          << " segments=" << (waypointsLeft.cols() - 1)
                          << " validation_dt_s="
                          << validationOptions.samplePeriod
                          << " successful_attempt=1" << std::endl;
                return true;
            }
        }

        const double totalSmoothingSeconds =
            std::chrono::duration<double>(
                Clock::now() - smoothingStageStarted).count();
        std::cerr << std::setprecision(17)
                  << "TCST_SMOOTHING_TIME_METRICS"
                  << " optimization_time_s="
                  << accumulatedOptimizationSeconds
                  << " validation_time_s="
                  << accumulatedValidationSeconds
                  << " total_smoothing_stage_time_s="
                  << totalSmoothingSeconds
                  << " successful_attempt=0"
                  << std::endl;
        std::cerr << "DualKIRARA smoothing exhausted all "
                  << maximumAttempts << " attempts." << std::endl;
        return false;
    }
}

#endif
