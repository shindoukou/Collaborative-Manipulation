#ifndef TCST_DUAL_TASK_METRIC_EXPORT_HPP
#define TCST_DUAL_TASK_METRIC_EXPORT_HPP

#include "joint_trajectory.hpp"

#include <Eigen/Eigen>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <functional>
#include <iomanip>
#include <limits>
#include <string>

namespace tcst
{
    struct DualTaskMetrics
    {
        double eeDistance = std::numeric_limits<double>::quiet_NaN();
        double cosThetaLeft = std::numeric_limits<double>::quiet_NaN();
        double cosThetaRight = std::numeric_limits<double>::quiet_NaN();
        double cosPsiLeft = std::numeric_limits<double>::quiet_NaN();
        double cosPsiRight = std::numeric_limits<double>::quiet_NaN();
        double collisionDistance = std::numeric_limits<double>::quiet_NaN();
        bool valid = false;
    };

    using DualTaskMetricFunction = std::function<DualTaskMetrics(
        const Eigen::Matrix<double, 6, 1> &qLeft,
        const Eigen::Matrix<double, 6, 1> &qRight)>;

     
     
     
    inline bool writeDualTrajectoryWithTaskMetricsCsv(
        const JointTrajectory6 &leftTrajectory,
        const JointTrajectory6 &rightTrajectory,
        const std::string &outputPath,
        const DualTaskMetricFunction &metricFunction,
        const double samplePeriod = 0.01)
    {
        if (leftTrajectory.empty() || rightTrajectory.empty() ||
            !metricFunction || !(samplePeriod > 0.0))
        {
            return false;
        }

        const double duration = leftTrajectory.getTotalDuration();
        if (std::abs(duration - rightTrajectory.getTotalDuration()) > 1.0e-8)
        {
            return false;
        }

        std::ofstream out(outputPath);
        if (!out)
        {
            return false;
        }
        out << std::setprecision(17);
        out << "t";
        for (const char *prefix : {"q", "qdot", "qddot", "qjerk"})
        {
            for (int joint = 1; joint <= 6; ++joint)
            {
                out << ',' << prefix << "_l" << joint;
            }
            for (int joint = 1; joint <= 6; ++joint)
            {
                out << ',' << prefix << "_r" << joint;
            }
        }
        out << ",ee_distance,cos_theta_l,cos_theta_r,cos_psi_l,cos_psi_r"
               ",collision_distance,valid\n";

        const int sampleCount =
            std::max(1, static_cast<int>(std::ceil(duration / samplePeriod)));
        for (int sample = 0; sample <= sampleCount; ++sample)
        {
            const double t = std::min(
                duration,
                duration * static_cast<double>(sample) /
                    static_cast<double>(sampleCount));
            const auto qL = leftTrajectory.getPos(t);
            const auto qR = rightTrajectory.getPos(t);
            const auto vL = leftTrajectory.getVel(t);
            const auto vR = rightTrajectory.getVel(t);
            const auto aL = leftTrajectory.getAcc(t);
            const auto aR = rightTrajectory.getAcc(t);
            const auto jL = leftTrajectory.getJerk(t);
            const auto jR = rightTrajectory.getJerk(t);
            const DualTaskMetrics metrics = metricFunction(qL, qR);

            out << t;
            for (const auto *vector : {&qL, &qR, &vL, &vR, &aL, &aR, &jL, &jR})
            {
                for (int joint = 0; joint < 6; ++joint)
                {
                    out << ',' << (*vector)(joint);
                }
            }
            out << ',' << metrics.eeDistance
                << ',' << metrics.cosThetaLeft
                << ',' << metrics.cosThetaRight
                << ',' << metrics.cosPsiLeft
                << ',' << metrics.cosPsiRight
                << ',' << metrics.collisionDistance
                << ',' << (metrics.valid ? 1 : 0)
                << '\n';
        }
        return out.good();
    }
}

#endif
