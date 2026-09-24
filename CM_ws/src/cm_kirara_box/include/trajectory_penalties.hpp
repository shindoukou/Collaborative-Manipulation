#ifndef TCST_TRAJECTORY_PENALTIES_HPP
#define TCST_TRAJECTORY_PENALTIES_HPP

#include <Eigen/Eigen>

#include <algorithm>
#include <cmath>

namespace tcst
{
    struct JointPenaltyOptions
    {
        Eigen::Matrix<double, 6, 1> qMin =
            Eigen::Matrix<double, 6, 1>::Constant(-2.0 * M_PI);
        Eigen::Matrix<double, 6, 1> qMax =
            Eigen::Matrix<double, 6, 1>::Constant(2.0 * M_PI);
        Eigen::Matrix<double, 6, 1> vMax =
            Eigen::Matrix<double, 6, 1>::Constant(0.5 * M_PI);
        double weightPosition = 50.0;
        double weightVelocity = 50.0;
        double smoothingWidth = 1.0e-3;
        double smoothAbsEps = 1.0e-6;
        int integralResolution = 12;
    };

    struct SmoothPositiveResult
    {
        double value;
        double derivative;
    };

    inline SmoothPositiveResult smoothPositive(
        const double x,
        const double width)
    {
        const double mu = std::max(width, 1.0e-9);
        if (x <= 0.0)
        {
            return {0.0, 0.0};
        }
        if (x < mu)
        {
            return {0.5 * x * x / mu, x / mu};
        }
        return {x - 0.5 * mu, 1.0};
    }

    inline double addJointPositionVelocityPenalty(
        const Eigen::VectorXd &times,
        const Eigen::MatrixXd &coeffs,
        const JointPenaltyOptions &options,
        Eigen::Matrix<double, Eigen::Dynamic, 6> &gradCoeffs,
        Eigen::VectorXd &gradTimes)
    {
        const int pieceCount = static_cast<int>(times.size());
        const int resolution = std::max(options.integralResolution, 2);
        double totalCost = 0.0;

        for (int piece = 0; piece < pieceCount; ++piece)
        {
            const double duration = times(piece);
            const double step = duration / static_cast<double>(resolution);
            const Eigen::Matrix<double, 6, 6> C =
                coeffs.block<6, 6>(6 * piece, 0);

            for (int sample = 0; sample <= resolution; ++sample)
            {
                const double alpha =
                    static_cast<double>(sample) / static_cast<double>(resolution);
                const double localTime = alpha * duration;
                const double node =
                    (sample == 0 || sample == resolution) ? 0.5 : 1.0;

                Eigen::Matrix<double, 6, 1> beta0;
                Eigen::Matrix<double, 6, 1> beta1;
                Eigen::Matrix<double, 6, 1> beta2;
                beta0 << 1.0, localTime, std::pow(localTime, 2),
                    std::pow(localTime, 3), std::pow(localTime, 4),
                    std::pow(localTime, 5);
                beta1 << 0.0, 1.0, 2.0 * localTime,
                    3.0 * std::pow(localTime, 2),
                    4.0 * std::pow(localTime, 3),
                    5.0 * std::pow(localTime, 4);
                beta2 << 0.0, 0.0, 2.0, 6.0 * localTime,
                    12.0 * std::pow(localTime, 2),
                    20.0 * std::pow(localTime, 3);

                const Eigen::Matrix<double, 6, 1> q = C.transpose() * beta0;
                const Eigen::Matrix<double, 6, 1> v = C.transpose() * beta1;
                const Eigen::Matrix<double, 6, 1> a = C.transpose() * beta2;

                double sampleCost = 0.0;
                Eigen::Matrix<double, 6, 1> gradQ =
                    Eigen::Matrix<double, 6, 1>::Zero();
                Eigen::Matrix<double, 6, 1> gradV =
                    Eigen::Matrix<double, 6, 1>::Zero();

                for (int joint = 0; joint < 6; ++joint)
                {
                    const auto upper = smoothPositive(
                        q(joint) - options.qMax(joint),
                        options.smoothingWidth);
                    const auto lower = smoothPositive(
                        options.qMin(joint) - q(joint),
                        options.smoothingWidth);
                    sampleCost += options.weightPosition *
                                  (upper.value * upper.value +
                                   lower.value * lower.value);
                    gradQ(joint) +=
                        2.0 * options.weightPosition *
                        (upper.value * upper.derivative -
                         lower.value * lower.derivative);

                    const double smoothAbs =
                        std::sqrt(v(joint) * v(joint) +
                                  options.smoothAbsEps * options.smoothAbsEps);
                    const auto velocity = smoothPositive(
                        smoothAbs - options.vMax(joint),
                        options.smoothingWidth);
                    sampleCost += options.weightVelocity *
                                  velocity.value * velocity.value;
                    gradV(joint) +=
                        2.0 * options.weightVelocity *
                        velocity.value * velocity.derivative *
                        v(joint) / smoothAbs;
                }

                gradCoeffs.block<6, 6>(6 * piece, 0) +=
                    node * step *
                    (beta0 * gradQ.transpose() +
                     beta1 * gradV.transpose());

                const double directTimeDerivative =
                    alpha * (gradQ.dot(v) + gradV.dot(a));
                gradTimes(piece) +=
                    node *
                    (sampleCost / static_cast<double>(resolution) +
                     step * directTimeDerivative);
                totalCost += node * step * sampleCost;
            }
        }

        return totalCost;
    }
}

#endif
