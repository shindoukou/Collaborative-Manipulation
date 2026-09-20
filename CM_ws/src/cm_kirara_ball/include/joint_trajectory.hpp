#ifndef TCST_JOINT_TRAJECTORY_HPP
#define TCST_JOINT_TRAJECTORY_HPP

#include <Eigen/Eigen>

#include <algorithm>
#include <cmath>
#include <vector>

namespace tcst
{
    struct JointPiece6
    {
        double duration = 0.0;

         
        Eigen::Matrix<double, 6, 6> coeffDesc =
            Eigen::Matrix<double, 6, 6>::Zero();

        inline Eigen::Matrix<double, 6, 1> getPos(const double t) const
        {
            Eigen::Matrix<double, 6, 1> beta;
            beta << std::pow(t, 5), std::pow(t, 4), std::pow(t, 3),
                    std::pow(t, 2), t, 1.0;
            return coeffDesc * beta;
        }

        inline Eigen::Matrix<double, 6, 1> getVel(const double t) const
        {
            Eigen::Matrix<double, 6, 1> beta;
            beta << 5.0 * std::pow(t, 4), 4.0 * std::pow(t, 3),
                    3.0 * std::pow(t, 2), 2.0 * t, 1.0, 0.0;
            return coeffDesc * beta;
        }

        inline Eigen::Matrix<double, 6, 1> getAcc(const double t) const
        {
            Eigen::Matrix<double, 6, 1> beta;
            beta << 20.0 * std::pow(t, 3), 12.0 * std::pow(t, 2),
                    6.0 * t, 2.0, 0.0, 0.0;
            return coeffDesc * beta;
        }

        inline Eigen::Matrix<double, 6, 1> getJerk(const double t) const
        {
            Eigen::Matrix<double, 6, 1> beta;
            beta << 60.0 * std::pow(t, 2), 24.0 * t, 6.0, 0.0, 0.0, 0.0;
            return coeffDesc * beta;
        }
    };

    class JointTrajectory6
    {
    public:
        std::vector<JointPiece6> pieces;

        inline void clear()
        {
            pieces.clear();
        }

        inline bool empty() const
        {
            return pieces.empty();
        }

        inline std::size_t size() const
        {
            return pieces.size();
        }

        inline double getTotalDuration() const
        {
            double total = 0.0;
            for (const auto &piece : pieces)
            {
                total += piece.duration;
            }
            return total;
        }

        inline Eigen::VectorXd getDurations() const
        {
            Eigen::VectorXd durations(static_cast<Eigen::Index>(pieces.size()));
            for (std::size_t i = 0; i < pieces.size(); ++i)
            {
                durations(static_cast<Eigen::Index>(i)) = pieces[i].duration;
            }
            return durations;
        }

        inline Eigen::Matrix<double, 6, 1> getPos(const double t) const
        {
            return evaluate(t, &JointPiece6::getPos);
        }

        inline Eigen::Matrix<double, 6, 1> getVel(const double t) const
        {
            return evaluate(t, &JointPiece6::getVel);
        }

        inline Eigen::Matrix<double, 6, 1> getAcc(const double t) const
        {
            return evaluate(t, &JointPiece6::getAcc);
        }

        inline Eigen::Matrix<double, 6, 1> getJerk(const double t) const
        {
            return evaluate(t, &JointPiece6::getJerk);
        }

    private:
        using Evaluator = Eigen::Matrix<double, 6, 1> (JointPiece6::*)(double) const;

        inline Eigen::Matrix<double, 6, 1> evaluate(
            const double globalTime,
            const Evaluator evaluator) const
        {
            if (pieces.empty())
            {
                return Eigen::Matrix<double, 6, 1>::Zero();
            }

            double remaining = std::max(0.0, globalTime);
            for (std::size_t i = 0; i < pieces.size(); ++i)
            {
                const bool last = (i + 1 == pieces.size());
                if (remaining <= pieces[i].duration || last)
                {
                    const double localTime =
                        std::min(std::max(remaining, 0.0), pieces[i].duration);
                    return (pieces[i].*evaluator)(localTime);
                }
                remaining -= pieces[i].duration;
            }

            return Eigen::Matrix<double, 6, 1>::Zero();
        }
    };
}

#endif
