#ifndef TCST_DUAL_KIRARA_HPP
#define TCST_DUAL_KIRARA_HPP

#include "joint_trajectory.hpp"
#include "lbfgs.hpp"
#include "kirara.hpp"
#include "trajectory_penalties.hpp"

#include <Eigen/Eigen>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <ostream>
#include <string>

namespace gcopter
{
    struct DualKIRARAOptions
    {
        double weightTime = 0.1;
        double weightJerkLeft = 1.0;
        double weightJerkRight = 1.0;
        double weightReference = 5.0;
        tcst::JointPenaltyOptions leftPenalty;
        tcst::JointPenaltyOptions rightPenalty;
        double initialMinPieceDuration = 0.25;
        int maxIterations = 1024;
    };

    class DualKIRARA
    {
    public:
        using StateMatrix = Eigen::Matrix<double, 6, 3>;
        using PointMatrix = Eigen::Matrix<double, 6, Eigen::Dynamic>;

        inline bool setup(
            const StateMatrix &headLeft,
            const StateMatrix &tailLeft,
            const StateMatrix &headRight,
            const StateMatrix &tailRight,
            const Eigen::VectorXi &intervalCounts,
            const DualKIRARAOptions &optimizerOptions = DualKIRARAOptions())
        {
            if (intervalCounts.size() == 0 ||
                (intervalCounts.array() <= 0).any())
            {
                lastError_ = "interval counts must all be positive";
                return false;
            }

            headLeft_ = headLeft;
            tailLeft_ = tailLeft;
            headRight_ = headRight;
            tailRight_ = tailRight;
            intervalCounts_ = intervalCounts;
            options_ = optimizerOptions;

            pieceCount_ = intervalCounts_.sum();
            temporalDim_ = pieceCount_;
            spatialDimEach_ = 6 * (pieceCount_ - 1);

            kiraraLeft_.setConditions(headLeft_, tailLeft_, pieceCount_);
            kiraraRight_.setConditions(headRight_, tailRight_, pieceCount_);
            configured_ = true;
            waypointsReady_ = false;
            optimized_ = false;
            lastError_.clear();
            return true;
        }

        inline bool setWaypoints(
            const PointMatrix &leftWaypoints,
            const PointMatrix &rightWaypoints)
        {
            if (!configured_)
            {
                lastError_ = "setup must be called before setWaypoints";
                return false;
            }
            if (leftWaypoints.rows() != 6 || rightWaypoints.rows() != 6 ||
                leftWaypoints.cols() != rightWaypoints.cols() ||
                leftWaypoints.cols() != intervalCounts_.size() + 1)
            {
                lastError_ =
                    "left/right waypoint matrices must be 6x(M+1) and match";
                return false;
            }

            Eigen::VectorXd timesLeft;
            Eigen::VectorXd timesRight;
            setInitial(
                leftWaypoints,
                intervalCounts_,
                options_.leftPenalty.vMax,
                options_.initialMinPieceDuration,
                pointsLeft_,
                timesLeft);
            setInitial(
                rightWaypoints,
                intervalCounts_,
                options_.rightPenalty.vMax,
                options_.initialMinPieceDuration,
                pointsRight_,
                timesRight);

            if (pointsLeft_.cols() != pieceCount_ - 1 ||
                pointsRight_.cols() != pieceCount_ - 1)
            {
                lastError_ = "failed to create N-1 internal points";
                return false;
            }

            times_ = timesLeft.cwiseMax(timesRight);
            referenceLeft_ = pointsLeft_;
            referenceRight_ = pointsRight_;
            waypointsReady_ = true;
            optimized_ = false;
            lastError_.clear();
            return true;
        }

        inline bool optimize(
            tcst::JointTrajectory6 &leftTrajectory,
            tcst::JointTrajectory6 &rightTrajectory,
            double &finalCost)
        {
            leftTrajectory.clear();
            rightTrajectory.clear();
            finalCost = std::numeric_limits<double>::infinity();
            if (!waypointsReady_)
            {
                lastError_ = "waypoints are not ready";
                return false;
            }

            Eigen::VectorXd x = initialVariables();
            Eigen::VectorXd initialGradient(x.size());
            initialCost_ = costFunctional(this, x, initialGradient);
            lbfgs::lbfgs_parameter_t parameters;
            parameters.mem_size = 64;
            parameters.past = 5;
             
             
             
             
             
            parameters.delta = 1.0e-4;
            parameters.g_epsilon = 0.0;
            parameters.max_iterations = options_.maxIterations;
            parameters.min_step = 1.0e-24;
            lastIteration_ = 0;
            lastGradientInfinityNorm_ =
                std::numeric_limits<double>::infinity();

            const int returnCode = lbfgs::lbfgs_optimize(
                x,
                finalCost,
                &DualKIRARA::costFunctional,
                nullptr,
                &DualKIRARA::progress,
                this,
                parameters);

            if (returnCode < 0)
            {
                lastError_ =
                    std::string("L-BFGS failed: ") +
                    lbfgs::lbfgs_strerror(returnCode) +
                    " iteration=" + std::to_string(lastIteration_) +
                    ", cost=" + std::to_string(finalCost) +
                    ", gradient_inf=" +
                    std::to_string(lastGradientInfinityNorm_);
                optimized_ = false;
                return false;
            }

            unpackVariables(x);
            kiraraLeft_.setParameters(pointsLeft_, times_);
            kiraraRight_.setParameters(pointsRight_, times_);
            kiraraLeft_.getTrajectory(leftTrajectory);
            kiraraRight_.getTrajectory(rightTrajectory);
            optimized_ = true;
            lastError_.clear();
            return true;
        }

        inline bool writeCoefficients(
            std::ostream &leftOutput,
            std::ostream &rightOutput) const
        {
            if (!optimized_ || !leftOutput.good() || !rightOutput.good())
            {
                return false;
            }
            writeOneArm(leftOutput, kiraraLeft_.getCoeffs());
            writeOneArm(rightOutput, kiraraRight_.getCoeffs());
            return leftOutput.good() && rightOutput.good();
        }

        inline Eigen::VectorXd initialVariables() const
        {
            Eigen::VectorXd x(temporalDim_ + 2 * spatialDimEach_);
            Eigen::Map<Eigen::VectorXd> tau(x.data(), temporalDim_);
            Eigen::Map<Eigen::VectorXd> xiLeft(
                x.data() + temporalDim_, spatialDimEach_);
            Eigen::Map<Eigen::VectorXd> xiRight(
                x.data() + temporalDim_ + spatialDimEach_, spatialDimEach_);
            backwardT(times_, tau);
            flattenPoints(pointsLeft_, xiLeft);
            flattenPoints(pointsRight_, xiRight);
            return x;
        }

        inline double evaluateForTest(
            const Eigen::VectorXd &x,
            Eigen::VectorXd &gradient)
        {
            gradient.resize(x.size());
            return costFunctional(this, x, gradient);
        }

        inline const Eigen::VectorXd &getSharedTimes() const
        {
            return times_;
        }

        inline double getInitialCost() const
        {
            return initialCost_;
        }

        inline const std::string &lastError() const
        {
            return lastError_;
        }

    private:
        kirara::KIRARA_S3KIRARA kiraraLeft_;
        kirara::KIRARA_S3KIRARA kiraraRight_;
        StateMatrix headLeft_;
        StateMatrix tailLeft_;
        StateMatrix headRight_;
        StateMatrix tailRight_;
        Eigen::VectorXi intervalCounts_;
        DualKIRARAOptions options_;

        int pieceCount_ = 0;
        int temporalDim_ = 0;
        int spatialDimEach_ = 0;
        bool configured_ = false;
        bool waypointsReady_ = false;
        bool optimized_ = false;

        PointMatrix pointsLeft_;
        PointMatrix pointsRight_;
        PointMatrix referenceLeft_;
        PointMatrix referenceRight_;
        Eigen::VectorXd times_;

        Eigen::Matrix<double, Eigen::Dynamic, 6> partialCoeffsLeft_;
        Eigen::Matrix<double, Eigen::Dynamic, 6> partialCoeffsRight_;
        Eigen::VectorXd partialTimesLeft_;
        Eigen::VectorXd partialTimesRight_;
        PointMatrix gradPointsLeft_;
        PointMatrix gradPointsRight_;
        Eigen::VectorXd gradTimesLeft_;
        Eigen::VectorXd gradTimesRight_;
        std::string lastError_;
        int lastIteration_ = 0;
        double lastGradientInfinityNorm_ =
            std::numeric_limits<double>::infinity();
        double initialCost_ = std::numeric_limits<double>::infinity();

        static inline int progress(
            void *context,
            const Eigen::VectorXd &,
            const Eigen::VectorXd &gradient,
            const double cost,
            const double,
            const int iteration,
            const int)
        {
            DualKIRARA &object = *static_cast<DualKIRARA *>(context);
            object.lastIteration_ = iteration;
            object.lastGradientInfinityNorm_ =
                gradient.cwiseAbs().maxCoeff();
            if (iteration == 1 || iteration % 100 == 0)
            {
                std::cout << "DualKIRARA iteration=" << iteration
                          << ", cost=" << cost
                          << ", gradient_inf="
                          << object.lastGradientInfinityNorm_
                          << std::endl;
            }
            return 0;
        }

        static inline void setInitial(
            const PointMatrix &path,
            const Eigen::VectorXi &intervalCounts,
            const Eigen::Matrix<double, 6, 1> &vMax,
            const double minimumDuration,
            PointMatrix &innerPoints,
            Eigen::VectorXd &timeAllocation)
        {
            const int pieceCount = intervalCounts.sum();
            innerPoints.resize(6, pieceCount - 1);
            timeAllocation.resize(pieceCount);

            int pieceId = 0;
            int pointId = 0;
            for (int interval = 0; interval < intervalCounts.size(); ++interval)
            {
                const int subdivisions = intervalCounts(interval);
                const Eigen::Matrix<double, 6, 1> q0 = path.col(interval);
                const Eigen::Matrix<double, 6, 1> q1 = path.col(interval + 1);
                const Eigen::Matrix<double, 6, 1> delta =
                    (q1 - q0) / static_cast<double>(subdivisions);

                double duration = std::max(minimumDuration, 1.0e-3);
                for (int joint = 0; joint < 6; ++joint)
                {
                    duration = std::max(
                        duration,
                        std::abs(delta(joint)) /
                            std::max(0.5 * vMax(joint), 1.0e-3));
                }

                for (int subdivision = 1;
                     subdivision <= subdivisions;
                     ++subdivision)
                {
                    timeAllocation(pieceId) = duration;
                    ++pieceId;
                    if (pieceId < pieceCount)
                    {
                        innerPoints.col(pointId) =
                            q0 + delta * static_cast<double>(subdivision);
                        ++pointId;
                    }
                }
            }
        }

        static inline void forwardT(
            const Eigen::VectorXd &tau,
            Eigen::VectorXd &times)
        {
            times.resize(tau.size());
            for (int i = 0; i < tau.size(); ++i)
            {
                times(i) =
                    tau(i) > 0.0
                        ? ((0.5 * tau(i) + 1.0) * tau(i) + 1.0)
                        : 1.0 /
                              ((0.5 * tau(i) - 1.0) * tau(i) + 1.0);
            }
        }

        static inline void backwardT(
            const Eigen::VectorXd &times,
            Eigen::Ref<Eigen::VectorXd> tau)
        {
            tau.resize(times.size());
            for (int i = 0; i < times.size(); ++i)
            {
                tau(i) =
                    times(i) > 1.0
                        ? std::sqrt(2.0 * times(i) - 1.0) - 1.0
                        : 1.0 - std::sqrt(2.0 / times(i) - 1.0);
            }
        }

        static inline void backwardGradT(
            const Eigen::VectorXd &tau,
            const Eigen::VectorXd &gradTimes,
            Eigen::Ref<Eigen::VectorXd> gradTau)
        {
            for (int i = 0; i < tau.size(); ++i)
            {
                if (tau(i) > 0.0)
                {
                    gradTau(i) = gradTimes(i) * (tau(i) + 1.0);
                }
                else
                {
                    const double denominator =
                        (0.5 * tau(i) - 1.0) * tau(i) + 1.0;
                    gradTau(i) =
                        gradTimes(i) * (1.0 - tau(i)) /
                        (denominator * denominator);
                }
            }
        }

        static inline void flattenPoints(
            const PointMatrix &points,
            Eigen::Ref<Eigen::VectorXd> flat)
        {
            for (int i = 0; i < points.cols(); ++i)
            {
                flat.segment<6>(6 * i) = points.col(i);
            }
        }

        static inline void unflattenPoints(
            const Eigen::Ref<const Eigen::VectorXd> flat,
            PointMatrix &points)
        {
            const int pointCount = static_cast<int>(flat.size()) / 6;
            points.resize(6, pointCount);
            for (int i = 0; i < pointCount; ++i)
            {
                points.col(i) = flat.segment<6>(6 * i);
            }
        }

        inline void unpackVariables(const Eigen::VectorXd &x)
        {
            const Eigen::Map<const Eigen::VectorXd> tau(
                x.data(), temporalDim_);
            const Eigen::Map<const Eigen::VectorXd> xiLeft(
                x.data() + temporalDim_, spatialDimEach_);
            const Eigen::Map<const Eigen::VectorXd> xiRight(
                x.data() + temporalDim_ + spatialDimEach_, spatialDimEach_);
            forwardT(tau, times_);
            unflattenPoints(xiLeft, pointsLeft_);
            unflattenPoints(xiRight, pointsRight_);
        }

        static inline double costFunctional(
            void *context,
            const Eigen::VectorXd &x,
            Eigen::VectorXd &gradient)
        {
            DualKIRARA &object = *static_cast<DualKIRARA *>(context);
            const Eigen::Map<const Eigen::VectorXd> tau(
                x.data(), object.temporalDim_);
            const Eigen::Map<const Eigen::VectorXd> xiLeft(
                x.data() + object.temporalDim_, object.spatialDimEach_);
            const Eigen::Map<const Eigen::VectorXd> xiRight(
                x.data() + object.temporalDim_ + object.spatialDimEach_,
                object.spatialDimEach_);
            Eigen::Map<Eigen::VectorXd> gradTau(
                gradient.data(), object.temporalDim_);
            Eigen::Map<Eigen::VectorXd> gradXiLeft(
                gradient.data() + object.temporalDim_, object.spatialDimEach_);
            Eigen::Map<Eigen::VectorXd> gradXiRight(
                gradient.data() + object.temporalDim_ + object.spatialDimEach_,
                object.spatialDimEach_);

            forwardT(tau, object.times_);
            unflattenPoints(xiLeft, object.pointsLeft_);
            unflattenPoints(xiRight, object.pointsRight_);
            object.kiraraLeft_.setParameters(
                object.pointsLeft_, object.times_);
            object.kiraraRight_.setParameters(
                object.pointsRight_, object.times_);

            double energyLeft = 0.0;
            double energyRight = 0.0;
            object.kiraraLeft_.getEnergy(energyLeft);
            object.kiraraRight_.getEnergy(energyRight);
            object.kiraraLeft_.getEnergyPartialGradByCoeffs(
                object.partialCoeffsLeft_);
            object.kiraraRight_.getEnergyPartialGradByCoeffs(
                object.partialCoeffsRight_);
            object.kiraraLeft_.getEnergyPartialGradByTimes(
                object.partialTimesLeft_);
            object.kiraraRight_.getEnergyPartialGradByTimes(
                object.partialTimesRight_);

            const double scaleLeft = 0.5 * object.options_.weightJerkLeft;
            const double scaleRight = 0.5 * object.options_.weightJerkRight;
            double cost =
                scaleLeft * energyLeft + scaleRight * energyRight;
            object.partialCoeffsLeft_ *= scaleLeft;
            object.partialCoeffsRight_ *= scaleRight;
            object.partialTimesLeft_ *= scaleLeft;
            object.partialTimesRight_ *= scaleRight;

            cost += tcst::addJointPositionVelocityPenalty(
                object.times_,
                object.kiraraLeft_.getCoeffs(),
                object.options_.leftPenalty,
                object.partialCoeffsLeft_,
                object.partialTimesLeft_);
            cost += tcst::addJointPositionVelocityPenalty(
                object.times_,
                object.kiraraRight_.getCoeffs(),
                object.options_.rightPenalty,
                object.partialCoeffsRight_,
                object.partialTimesRight_);

            object.kiraraLeft_.propogateGrad(
                object.partialCoeffsLeft_,
                object.partialTimesLeft_,
                object.gradPointsLeft_,
                object.gradTimesLeft_);
            object.kiraraRight_.propogateGrad(
                object.partialCoeffsRight_,
                object.partialTimesRight_,
                object.gradPointsRight_,
                object.gradTimesRight_);

            const PointMatrix referenceDifferenceLeft =
                object.pointsLeft_ - object.referenceLeft_;
            const PointMatrix referenceDifferenceRight =
                object.pointsRight_ - object.referenceRight_;
            cost +=
                0.5 * object.options_.weightReference *
                (referenceDifferenceLeft.squaredNorm() +
                 referenceDifferenceRight.squaredNorm());
            object.gradPointsLeft_ +=
                object.options_.weightReference * referenceDifferenceLeft;
            object.gradPointsRight_ +=
                object.options_.weightReference * referenceDifferenceRight;

            Eigen::VectorXd sharedTimeGradient =
                object.gradTimesLeft_ + object.gradTimesRight_;
            cost += object.options_.weightTime * object.times_.sum();
            sharedTimeGradient.array() += object.options_.weightTime;

            backwardGradT(tau, sharedTimeGradient, gradTau);
            flattenPoints(object.gradPointsLeft_, gradXiLeft);
            flattenPoints(object.gradPointsRight_, gradXiRight);
            return cost;
        }

        inline void writeOneArm(
            std::ostream &output,
            const Eigen::MatrixXd &coefficients) const
        {
            output << "# one segment per line\n";
            output << "# duration, then joint1[a0..a5], ..., joint6[a0..a5]\n";
            output.precision(17);
            for (int segment = 0; segment < pieceCount_; ++segment)
            {
                output << times_(segment);
                for (int joint = 0; joint < 6; ++joint)
                {
                    for (int power = 0; power < 6; ++power)
                    {
                        output << ' '
                               << coefficients(6 * segment + power, joint);
                    }
                }
                output << '\n';
            }
        }
    };
}

#endif
