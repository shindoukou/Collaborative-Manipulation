#define PINOCCHIO_WITH_HPP_FCL true

#include "pinocchio/config.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/multibody/fcl.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/geometry.hpp"
#include "pinocchio/algorithm/geometry.hxx"
#include "pinocchio/parsers/urdf/geometry.hxx"

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

#include "yaml-cpp/yaml.h"

namespace
{

bool expectedAdjacentPair(const std::string &first, const std::string &second)
{
    const auto matches = [&](const std::string &a, const std::string &b)
    {
        return (first == a && second == b) || (first == b && second == a);
    };
    return
        matches("left_base_link_0", "left_shoulder_link_0") ||
        matches("left_shoulder_link_0", "left_upper_arm_link_0") ||
        matches("left_upper_arm_link_0", "left_forearm_link_0") ||
        matches("left_forearm_link_0", "left_wrist_1_link_0") ||
        matches("left_wrist_1_link_0", "left_wrist_2_link_0") ||
        matches("left_wrist_2_link_0", "left_wrist_3_link_0") ||
        matches("right_base_link_0", "right_shoulder_link_0") ||
        matches("right_shoulder_link_0", "right_upper_arm_link_0") ||
        matches("right_upper_arm_link_0", "right_forearm_link_0") ||
        matches("right_forearm_link_0", "right_wrist_1_link_0") ||
        matches("right_wrist_1_link_0", "right_wrist_2_link_0") ||
        matches("right_wrist_2_link_0", "right_wrist_3_link_0");
}

bool graspContact(const std::string &first, const std::string &second)
{
    const auto isToolLink = [](const std::string &name)
    {
        return name.find("wrist_3_link") != std::string::npos ||
               name.find("ft300_mounting_plate") != std::string::npos ||
               name.find("ft300_sensor") != std::string::npos;
    };
    return (first == "target_ball_0" && isToolLink(second)) ||
           (second == "target_ball_0" && isToolLink(first));
}

struct Limits
{
    double distanceMin;
    double distanceMax;
    double cosineThetaLeft;
    double cosineThetaRight;
    double cosinePsiLeft;
    double cosinePsiRight;
};

double optionalScalar(const YAML::Node &node,
                      const std::string &key,
                      double fallback)
{
    return node && node[key] ? node[key].as<double>() : fallback;
}

class PairValidator
{
public:
    PairValidator(const std::string &phase, const YAML::Node &config)
        : phase_(phase),
          model_(),
          data_(model_),
          geometryModel_(),
          geometryData_(geometryModel_)
    {
        const std::string urdf =
            config["file_path"]["pinocchio_file"].as<std::string>();
        const std::string modelPath =
            config["model_path"]["pinocchio_path"].as<std::string>();
        pinocchio::urdf::buildModel(urdf, model_);
        data_ = pinocchio::Data(model_);
        pinocchio::urdf::buildGeom(
            model_, urdf, pinocchio::COLLISION, geometryModel_, modelPath);
        geometryModel_.addAllCollisionPairs();
        geometryData_ = pinocchio::GeometryData(geometryModel_);

        const YAML::Node goal = config["goal_generator"];
        const YAML::Node planning = config["planning"]
                                        ? config["planning"]
                                        : config["palnning"];
        limits_.distanceMin = optionalScalar(
            goal, "distance_min", optionalScalar(planning, "lowbound", 0.0));
        limits_.distanceMax = optionalScalar(
            goal, "distance_max",
            optionalScalar(planning, "upperbound",
                           optionalScalar(config["cost"], "eff", 0.20)));
        const double thetaLeft = optionalScalar(goal, "theta_left", 0.45);
        const double thetaRight = optionalScalar(goal, "theta_right", 0.45);
        const double psiLeft = optionalScalar(goal, "psi_left", thetaLeft);
        const double psiRight = optionalScalar(goal, "psi_right", thetaRight);
        limits_.cosineThetaLeft = std::cos(thetaLeft);
        limits_.cosineThetaRight = std::cos(thetaRight);
        limits_.cosinePsiLeft = std::cos(psiLeft);
        limits_.cosinePsiRight = std::cos(psiRight);
        const YAML::Node target = goal ? goal["target"] : YAML::Node();
        targetPosition_ = Eigen::Vector3d(
            optionalScalar(target, "x", 0.0),
            optionalScalar(target, "y", 0.0),
            optionalScalar(target, "z", 0.0));
        const std::string leftTip = goal && goal["left_tip"]
                                        ? goal["left_tip"].as<std::string>()
                                        : "left_tool0";
        const std::string rightTip = goal && goal["right_tip"]
                                         ? goal["right_tip"].as<std::string>()
                                         : "right_tool0";
        leftFrame_ = model_.getFrameId(leftTip);
        rightFrame_ = model_.getFrameId(rightTip);
        if (leftFrame_ >= model_.frames.size() ||
            rightFrame_ >= model_.frames.size())
        {
            throw std::runtime_error("configured tool frame not found in URDF");
        }
    }

    std::size_t configurationSize() const
    {
        return model_.nq;
    }

    std::size_t collisionPairCount() const
    {
        return geometryModel_.collisionPairs.size();
    }

    bool valid(const Eigen::Matrix<double, 12, 1> &pair,
               bool terminalSample,
               std::string *reason = nullptr)
    {
        Eigen::VectorXd modelConfiguration = Eigen::VectorXd::Zero(model_.nq);
        modelConfiguration.head<12>() = pair;
        if (phase_ == "move" && model_.nq >= 19)
        {
            modelConfiguration.segment<4>(15) << 0.0, 0.0, 0.0, 1.0;
        }

        pinocchio::framesForwardKinematics(model_, data_, modelConfiguration);
        Eigen::Vector3d leftPosition = data_.oMf[leftFrame_].translation();
        Eigen::Vector3d rightPosition = data_.oMf[rightFrame_].translation();
        Eigen::Vector3d leftAxis = data_.oMf[leftFrame_].rotation().col(2);
        Eigen::Vector3d rightAxis = data_.oMf[rightFrame_].rotation().col(2);
        if (phase_ == "move")
        {
            // During transport the carried ball center is the synchronized
            // midpoint of the two tool frames.  This task definition is
            // independent of whether the validation URDF embeds the spawned
            // Gazebo ball as a floating joint (nq >= 19) or omits it (nq=12).
            targetPosition_ = 0.5 * (leftPosition + rightPosition);
        }
        if (phase_ == "move" && model_.nq >= 19)
        {
            modelConfiguration.segment<3>(12) = targetPosition_;
            pinocchio::framesForwardKinematics(model_, data_, modelConfiguration);
            leftPosition = data_.oMf[leftFrame_].translation();
            rightPosition = data_.oMf[rightFrame_].translation();
            leftAxis = data_.oMf[leftFrame_].rotation().col(2);
            rightAxis = data_.oMf[rightFrame_].rotation().col(2);
        }

        pinocchio::updateGeometryPlacements(
            model_, data_, geometryModel_, geometryData_, modelConfiguration);
        for (std::size_t index = 0;
             index < geometryModel_.collisionPairs.size(); ++index)
        {
            const pinocchio::CollisionPair &pairIndex =
                geometryModel_.collisionPairs[index];
            const std::string &first =
                geometryModel_.geometryObjects[pairIndex.first].name;
            const std::string &second =
                geometryModel_.geometryObjects[pairIndex.second].name;
            if (expectedAdjacentPair(first, second) ||
                (phase_ == "move" && graspContact(first, second)))
            {
                continue;
            }
            geometryData_.collisionResults[index].clear();
            if (pinocchio::computeCollision(
                    geometryModel_, geometryData_, index) &&
                pinocchio::computeDistance(
                    geometryModel_, geometryData_, index).min_distance <= 0.0)
            {
                if (reason)
                {
                    *reason = "collision:" + first + "&&" + second;
                }
                return false;
            }
        }

        // Grasp task constraints apply only at tf.  Intermediate grasp samples
        // still receive the complete collision check above.  Transport task
        // constraints apply at every synchronized sample.
        if (phase_ == "catch" && !terminalSample)
        {
            return true;
        }

        const Eigen::Vector3d separation = rightPosition - leftPosition;
        const double distance = separation.norm();
        if (!(distance >= limits_.distanceMin &&
              distance <= limits_.distanceMax) || distance < 1.0e-12)
        {
            if (reason)
            {
                std::ostringstream message;
                message << "distance=" << distance
                        << ",allowed=[" << limits_.distanceMin
                        << ',' << limits_.distanceMax << ']';
                *reason = message.str();
            }
            return false;
        }
        const double leftCosine = separation.dot(leftAxis) /
                                  (distance * leftAxis.norm());
        const double rightCosine = -separation.dot(rightAxis) /
                                   (distance * rightAxis.norm());
        const Eigen::Vector3d leftToTarget = targetPosition_ - leftPosition;
        const Eigen::Vector3d rightToTarget = targetPosition_ - rightPosition;
        const bool targetDirectionsFinite =
            leftToTarget.norm() >= 1.0e-12 && rightToTarget.norm() >= 1.0e-12;
        const double leftPsi = targetDirectionsFinite
            ? leftToTarget.normalized().dot(separation.normalized()) : -1.0;
        const double rightPsi = targetDirectionsFinite
            ? rightToTarget.normalized().dot(-separation.normalized()) : -1.0;
        const bool orientationValid =
            leftCosine >= limits_.cosineThetaLeft &&
            rightCosine >= limits_.cosineThetaRight &&
            leftPsi >= limits_.cosinePsiLeft &&
            rightPsi >= limits_.cosinePsiRight;
        if (!orientationValid && reason)
        {
            std::ostringstream message;
            message << "orientation_theta=" << leftCosine << ',' << rightCosine
                    << ",minimum=" << limits_.cosineThetaLeft
                    << ',' << limits_.cosineThetaRight
                    << ",psi=" << leftPsi << ',' << rightPsi
                    << ",minimum=" << limits_.cosinePsiLeft
                    << ',' << limits_.cosinePsiRight;
            *reason = message.str();
        }
        return orientationValid;
    }

private:
    std::string phase_;
    pinocchio::Model model_;
    pinocchio::Data data_;
    pinocchio::GeometryModel geometryModel_;
    pinocchio::GeometryData geometryData_;
    Limits limits_;
    Eigen::Vector3d targetPosition_;
    pinocchio::FrameIndex leftFrame_;
    pinocchio::FrameIndex rightFrame_;
};

}  // namespace

int main(int argc, char **argv)
{
    if (argc != 3)
    {
        std::cerr << "usage: CM_SOCKET_BALL_VALIDATOR catch|move CONFIG.yaml"
                  << std::endl;
        return 2;
    }
    const std::string phase(argv[1]);
    if (phase != "catch" && phase != "move")
    {
        std::cerr << "phase must be catch or move" << std::endl;
        return 2;
    }

    const YAML::Node config = YAML::LoadFile(argv[2]);
    PairValidator validator(phase, config);
    std::cout << "READY " << phase << ' ' << validator.configurationSize()
              << ' ' << validator.collisionPairCount() << std::endl;

    std::string line;
    while (std::getline(std::cin, line))
    {
        if (line == "X")
        {
            break;
        }
        std::istringstream input(line);
        char command = 0;
        std::uint64_t sequence = 0;
        input >> command >> sequence;
        Eigen::Matrix<double, 12, 1> pair;
        bool parsed = command == 'Q' || command == 'D' || command == 'F';
        for (int joint = 0; joint < 12 && parsed; ++joint)
        {
            parsed = static_cast<bool>(input >> pair[joint]);
        }
        if (!parsed || !pair.allFinite())
        {
            std::cout << "E " << sequence << " expected_12_finite_joints"
                      << std::endl;
            continue;
        }
        std::string reason = "valid";
        const bool diagnostic = command == 'D' || command == 'F';
        const bool valid = validator.valid(
            pair, command == 'F', diagnostic ? &reason : nullptr);
        std::cout << (diagnostic ? command : 'V') << ' ' << sequence << ' '
                  << (valid ? 1 : 0);
        if (diagnostic)
        {
            std::cout << ' ' << reason;
        }
        std::cout << std::endl;
    }
    return 0;
}
