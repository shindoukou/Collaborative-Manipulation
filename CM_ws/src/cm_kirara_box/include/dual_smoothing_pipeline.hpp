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
        constexpr int maximumAttempts = 5;
        const double referenceWeightFactors[maximumAttempts] =
            {1.0, 5.0, 25.0, 125.0, 500.0};
        const int intervalFactors[maximumAttempts] = {1, 1, 2, 4, 8};
        for (int attempt = 0; attempt < maximumAttempts; ++attempt)
        {
            gcopter::DualKIRARAOptions attemptOptions = optimizerOptions;
            Eigen::VectorXi attemptIntervals = intervalCounts;
            attemptOptions.weightReference *=
                referenceWeightFactors[attempt];
            attemptIntervals *= intervalFactors[attempt];

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
