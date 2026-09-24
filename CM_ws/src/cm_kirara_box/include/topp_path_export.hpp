#ifndef TCST_TOPP_PATH_EXPORT_HPP
#define TCST_TOPP_PATH_EXPORT_HPP

#include "joint_trajectory.hpp"

#include <Eigen/Eigen>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

namespace tcst
{
    inline void writeDualPathHeader(std::ostream &out)
    {
        out << "s";
        for (int joint = 1; joint <= 6; ++joint)
        {
            out << ",q_l" << joint;
        }
        for (int joint = 1; joint <= 6; ++joint)
        {
            out << ",q_r" << joint;
        }
        out << '\n';
    }

     
     
     
    inline bool writeDualWaypointPathCsv(
        const Eigen::Matrix<double, 6, Eigen::Dynamic> &leftWaypoints,
        const Eigen::Matrix<double, 6, Eigen::Dynamic> &rightWaypoints,
        const std::string &outputPath,
        const double duplicateTolerance = 1.0e-10)
    {
        if (leftWaypoints.rows() != 6 || rightWaypoints.rows() != 6 ||
            leftWaypoints.cols() != rightWaypoints.cols() ||
            leftWaypoints.cols() < 2)
        {
            std::cerr << "TOPP export failed: expected matching 6xK waypoint matrices."
                      << std::endl;
            return false;
        }

        std::vector<Eigen::Matrix<double, 12, 1>> retained;
        retained.reserve(static_cast<std::size_t>(leftWaypoints.cols()));
        for (Eigen::Index column = 0; column < leftWaypoints.cols(); ++column)
        {
            Eigen::Matrix<double, 12, 1> q;
            q << leftWaypoints.col(column), rightWaypoints.col(column);
            if (retained.empty() ||
                (q - retained.back()).norm() > duplicateTolerance)
            {
                retained.push_back(q);
            }
        }
        if (retained.size() < 2)
        {
            std::cerr << "TOPP export failed: path has fewer than two distinct points."
                      << std::endl;
            return false;
        }

        std::vector<double> cumulative(retained.size(), 0.0);
        for (std::size_t i = 1; i < retained.size(); ++i)
        {
            cumulative[i] = cumulative[i - 1] +
                            (retained[i] - retained[i - 1]).norm();
        }
        const double totalLength = cumulative.back();
        if (!(totalLength > duplicateTolerance))
        {
            std::cerr << "TOPP export failed: zero joint-space path length."
                      << std::endl;
            return false;
        }

        std::ofstream out(outputPath);
        if (!out)
        {
            std::cerr << "TOPP export failed: cannot open " << outputPath
                      << std::endl;
            return false;
        }
        out << std::setprecision(17);
        writeDualPathHeader(out);
        for (std::size_t i = 0; i < retained.size(); ++i)
        {
            out << cumulative[i] / totalLength;
            for (int joint = 0; joint < 12; ++joint)
            {
                out << ',' << retained[i](joint);
            }
            out << '\n';
        }
        return out.good();
    }

    inline void writeDualTimedHeader(std::ostream &out)
    {
        out << "t";
        const char *groups[] = {"q", "qdot", "qddot", "qjerk"};
        for (const char *group : groups)
        {
            for (int joint = 1; joint <= 6; ++joint)
            {
                out << ',' << group << "_l" << joint;
            }
            for (int joint = 1; joint <= 6; ++joint)
            {
                out << ',' << group << "_r" << joint;
            }
        }
        out << '\n';
    }

     
     
     
    inline bool writeDualTimedTrajectoryCsv(
        const JointTrajectory6 &leftTrajectory,
        const JointTrajectory6 &rightTrajectory,
        const std::string &outputPath,
        const double samplePeriod = 0.01)
    {
        if (leftTrajectory.empty() || rightTrajectory.empty() ||
            !(samplePeriod > 0.0))
        {
            return false;
        }
        const double leftDuration = leftTrajectory.getTotalDuration();
        const double rightDuration = rightTrajectory.getTotalDuration();
        if (std::abs(leftDuration - rightDuration) > 1.0e-8)
        {
            std::cerr << "TOPP comparison export failed: left/right durations differ."
                      << std::endl;
            return false;
        }

        std::ofstream out(outputPath);
        if (!out)
        {
            return false;
        }
        out << std::setprecision(17);
        writeDualTimedHeader(out);

        const int sampleCount =
            std::max(1, static_cast<int>(std::ceil(leftDuration / samplePeriod)));
        for (int sample = 0; sample <= sampleCount; ++sample)
        {
            const double t = std::min(
                leftDuration,
                leftDuration * static_cast<double>(sample) /
                    static_cast<double>(sampleCount));

            const auto qL = leftTrajectory.getPos(t);
            const auto qR = rightTrajectory.getPos(t);
            const auto vL = leftTrajectory.getVel(t);
            const auto vR = rightTrajectory.getVel(t);
            const auto aL = leftTrajectory.getAcc(t);
            const auto aR = rightTrajectory.getAcc(t);
            const auto jL = leftTrajectory.getJerk(t);
            const auto jR = rightTrajectory.getJerk(t);

            out << t;
            for (const auto *vector : {&qL, &qR, &vL, &vR, &aL, &aR, &jL, &jR})
            {
                for (int joint = 0; joint < 6; ++joint)
                {
                    out << ',' << (*vector)(joint);
                }
            }
            out << '\n';
        }
        return out.good();
    }
}

#endif
