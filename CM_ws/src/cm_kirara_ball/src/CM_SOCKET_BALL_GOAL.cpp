#define PINOCCHIO_WITH_HPP_FCL true

#include "pinocchio/config.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/multibody/fcl.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/geometry.hpp"
#include "pinocchio/algorithm/geometry.hxx"
#include "pinocchio/parsers/urdf/geometry.hxx"

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "tcst_ik_goal_generator.hpp"
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

class CollisionValidator
{
public:
    CollisionValidator(const std::string &phase, const YAML::Node &config)
        : phase_(phase), model_(), data_(model_), geometryModel_(),
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
        leftFrame_ = model_.getFrameId(
            goal && goal["left_tip"]
                ? goal["left_tip"].as<std::string>() : "left_tool0");
        rightFrame_ = model_.getFrameId(
            goal && goal["right_tip"]
                ? goal["right_tip"].as<std::string>() : "right_tool0");
    }

    bool valid(const tcst::ik_goal::Vec12 &pair)
    {
        Eigen::VectorXd configuration = Eigen::VectorXd::Zero(model_.nq);
        configuration.head<12>() = pair;
        if (phase_ == "move" && model_.nq >= 19)
        {
            configuration.segment<4>(15) << 0.0, 0.0, 0.0, 1.0;
            pinocchio::framesForwardKinematics(model_, data_, configuration);
            configuration.segment<3>(12) = 0.5 * (
                data_.oMf[leftFrame_].translation() +
                data_.oMf[rightFrame_].translation());
        }
        pinocchio::updateGeometryPlacements(
            model_, data_, geometryModel_, geometryData_, configuration);
        for (std::size_t index = 0;
             index < geometryModel_.collisionPairs.size(); ++index)
        {
            const pinocchio::CollisionPair &collisionPair =
                geometryModel_.collisionPairs[index];
            const std::string &first =
                geometryModel_.geometryObjects[collisionPair.first].name;
            const std::string &second =
                geometryModel_.geometryObjects[collisionPair.second].name;
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
                return false;
            }
        }
        return true;
    }

    Eigen::Vector3d center(const tcst::ik_goal::Vec12 &pair)
    {
        Eigen::VectorXd configuration = Eigen::VectorXd::Zero(model_.nq);
        configuration.head<12>() = pair;
        if (model_.nq >= 19)
        {
            configuration.segment<4>(15) << 0.0, 0.0, 0.0, 1.0;
        }
        pinocchio::framesForwardKinematics(model_, data_, configuration);
        return 0.5 * (data_.oMf[leftFrame_].translation() +
                      data_.oMf[rightFrame_].translation());
    }

private:
    std::string phase_;
    pinocchio::Model model_;
    pinocchio::Data data_;
    pinocchio::GeometryModel geometryModel_;
    pinocchio::GeometryData geometryData_;
    pinocchio::FrameIndex leftFrame_;
    pinocchio::FrameIndex rightFrame_;
};

tcst::ik_goal::Vec12 loadInitial(const YAML::Node &config,
                                 const std::string &optionalPath)
{
    tcst::ik_goal::Vec12 result;
    if (!optionalPath.empty())
    {
        std::ifstream input(optionalPath);
        for (int index = 0; index < 12; ++index)
        {
            if (!(input >> result[index]))
            {
                throw std::runtime_error("reference pair must contain 12 values");
            }
        }
        return result;
    }
    for (int index = 0; index < 12; ++index)
    {
        result[index] = config["Joint_init"]
                              ["joint_" + std::to_string(index + 1)]
                                  .as<double>();
    }
    return result;
}

void writeRows(const std::string &path,
               const Eigen::Matrix<double, 6, 1> &start,
               const Eigen::Matrix<double, 6, 1> &goal)
{
    std::ofstream output(path);
    output << std::setprecision(17);
    for (int row = 0; row < 2; ++row)
    {
        const Eigen::Matrix<double, 6, 1> &values = row == 0 ? start : goal;
        for (int index = 0; index < 6; ++index)
        {
            output << (index ? " " : "") << values[index];
        }
        output << '\n';
    }
}

void writePath(const std::string &path,
               const std::vector<tcst::ik_goal::Vec12> &pairs,
               int offset)
{
    std::ofstream output(path);
    output << std::setprecision(17);
    for (const tcst::ik_goal::Vec12 &pair : pairs)
    {
        for (int index = 0; index < 6; ++index)
        {
            output << (index ? " " : "") << pair[offset + index];
        }
        output << '\n';
    }
}

void writePair(const std::string &path, const tcst::ik_goal::Vec12 &pair)
{
    std::ofstream output(path);
    output << std::setprecision(17);
    for (int index = 0; index < 12; ++index)
    {
        output << (index ? " " : "") << pair[index];
    }
    output << '\n';
}

void writeLocalCandidates(
    const std::string &path,
    const std::vector<tcst::ik_goal::LocalGoalCandidate> &candidates)
{
    std::ofstream output(path);
    output << std::setprecision(17);
    output << "sample_index\taxis_index\troll_index\tposition_error"
              "\torientation_error\tq1\tq2\tq3\tq4\tq5\tq6\n";
    for (const tcst::ik_goal::LocalGoalCandidate &candidate : candidates)
    {
        output << candidate.sample_index << '\t'
               << candidate.axis_index << '\t'
               << candidate.roll_index << '\t'
               << candidate.ik.position_error << '\t'
               << candidate.ik.orientation_error;
        for (int joint = 0; joint < 6; ++joint)
            output << '\t' << candidate.ik.q[joint];
        output << '\n';
    }
}

}  // namespace

int main(int argc, char **argv)
{
    if ((argc == 6 || argc == 7) && std::string(argv[1]) == "local")
    {
        const std::string phase(argv[2]);
        const std::string side(argv[3]);
        if ((phase != "catch" && phase != "move") ||
            (side != "left" && side != "right"))
        {
            std::cerr << "local mode requires catch|move and left|right"
                      << std::endl;
            return 2;
        }
        const YAML::Node config = YAML::LoadFile(argv[4]);
        const tcst::ik_goal::Vec12 reference =
            loadInitial(config, argc == 7 ? argv[6] : "");
        tcst::ik_goal::GoalGeneratorOptions options =
            tcst::ik_goal::loadOptions(
                config,
                phase == "catch" ? tcst::ik_goal::TaskMode::CATCH
                                   : tcst::ik_goal::TaskMode::MOVE);
        options.output_file.clear();
        options.debug_file.clear();
        tcst::ik_goal::DualArmGoalGenerator generator(
            config["file_path"]["pinocchio_file"].as<std::string>(),
            options, reference);
        if (!generator.initialized())
        {
            std::cerr << "local candidate generator failed: "
                      << generator.error() << std::endl;
            return 1;
        }
        const std::vector<tcst::ik_goal::LocalGoalCandidate> candidates =
            generator.generateLocalCandidates(side == "left");
        writeLocalCandidates(argv[5], candidates);
        std::cout << "LOCAL_GOAL_CANDIDATES phase=" << phase
                  << " side=" << side
                  << " count=" << candidates.size()
                  << " output=" << argv[5] << std::endl;
        return candidates.empty() ? 1 : 0;
    }
    if (argc == 5 && std::string(argv[1]) == "guide")
    {
        const YAML::Node config = YAML::LoadFile(argv[2]);
        const std::string prefix(argv[3]);
        tcst::ik_goal::Vec12 current = loadInitial(config, argv[4]);
        const YAML::Node planning = config["planning"]
                                        ? config["planning"]
                                        : config["palnning"];
        const YAML::Node waypoints = planning["target_waypoints"];
        if (!waypoints || !waypoints.IsSequence() || waypoints.size() == 0)
        {
            std::cerr << "planning.target_waypoints must be a nonempty sequence"
                      << std::endl;
            return 2;
        }
        CollisionValidator collisionValidator("move", config);
        std::vector<tcst::ik_goal::Vec12> pairs;
        pairs.push_back(current);
        std::vector<Eigen::Vector3d> expandedTargets;
        Eigen::Vector3d previousTarget = collisionValidator.center(current);
        const double maximumTargetStep = 0.0125;
        for (std::size_t waypoint = 0; waypoint < waypoints.size(); ++waypoint)
        {
            Eigen::Vector3d desired;
            desired << waypoints[waypoint]["x"].as<double>(),
                       waypoints[waypoint]["y"].as<double>(),
                       waypoints[waypoint]["z"].as<double>();
            const int pieces = std::max(
                1, static_cast<int>(std::ceil(
                       (desired - previousTarget).norm() / maximumTargetStep)));
            for (int piece = 1; piece <= pieces; ++piece)
            {
                expandedTargets.push_back(
                    previousTarget + (desired - previousTarget) *
                                         (static_cast<double>(piece) / pieces));
            }
            previousTarget = desired;
        }
        std::size_t validPairs = 0;
        double generationTime = 0.0;
        std::vector<int> diversionAttempts(expandedTargets.size(), 0);
        Eigen::Vector3d activeTargetOffset = Eigen::Vector3d::Zero();
        int offsetHold = 0;
        for (std::size_t waypoint = 0;
             waypoint < expandedTargets.size(); ++waypoint)
        {
            if (waypoint + 1 == expandedTargets.size())
            {
                activeTargetOffset.setZero();
                offsetHold = 0;
            }
            tcst::ik_goal::GoalGeneratorOptions options =
                tcst::ik_goal::loadOptions(
                    config, tcst::ik_goal::TaskMode::MOVE);
            options.target =
                expandedTargets[waypoint] + activeTargetOffset;
            options.target_from_reference = false;
            options.preserve_left_reference = false;
            options.preserve_right_world_pose = false;
            options.right_base_translation_delta.setZero();
            options.target_offset.setZero();
            options.stop_at_first_valid = false;
            options.distance_min += 0.005;
            options.distance_max -= 0.005;
            options.theta_left *= 0.85;
            options.theta_right *= 0.85;
            options.psi_left *= 0.85;
            options.psi_right *= 0.85;
            options.axis_samples = std::min(options.axis_samples, 12);
            options.roll_samples = std::min(options.roll_samples, 2);
            options.ik_restarts = std::min(options.ik_restarts, 20);
            options.ik_position_tolerance =
                std::min(options.ik_position_tolerance, 0.002);
            options.local_seed_ratio = 1.0;
            options.random_seed += static_cast<unsigned int>(waypoint);
            options.output_file.clear();
            tcst::ik_goal::DualArmGoalGenerator generator(
                config["file_path"]["pinocchio_file"].as<std::string>(),
                options, current);
            const tcst::ik_goal::GoalGeneratorOptions pathLimits =
                tcst::ik_goal::loadOptions(
                    config, tcst::ik_goal::TaskMode::MOVE);
            std::size_t rejectedDistance = 0;
            std::size_t rejectedAxis = 0;
            std::size_t rejectedCollision = 0;
            std::size_t rejectedEndpointTask = 0;
            const auto connectionValid = [
                &collisionValidator, &current, &rejectedDistance,
                &rejectedAxis, &rejectedCollision](
                const tcst::ik_goal::DualArmGoalGenerator &solver,
                const tcst::ik_goal::Vec12 &candidate,
                const tcst::ik_goal::GoalGeneratorOptions &limits)
            {
                const tcst::ik_goal::Vec12 delta = candidate - current;
                const int samples = std::max(
                    1, static_cast<int>(std::ceil(
                           delta.cwiseAbs().maxCoeff() / 0.01)));
                for (int sample = 1; sample <= samples; ++sample)
                {
                    const tcst::ik_goal::Vec12 state =
                        current + delta *
                                      (static_cast<double>(sample) / samples);
                    const Eigen::Matrix4d left =
                        solver.forwardPose(true, state.head<6>());
                    const Eigen::Matrix4d right =
                        solver.forwardPose(false, state.tail<6>());
                    const Eigen::Vector3d between =
                        right.block<3, 1>(0, 3) - left.block<3, 1>(0, 3);
                    const double distance = between.norm();
                    if (distance < limits.distance_min ||
                        distance > limits.distance_max || distance < 1.0e-12)
                    {
                        ++rejectedDistance;
                        return false;
                    }
                    const Eigen::Vector3d direction = between / distance;
                    const double leftMinimum = std::max(
                        std::cos(limits.theta_left),
                        std::cos(limits.psi_left));
                    const double rightMinimum = std::max(
                        std::cos(limits.theta_right),
                        std::cos(limits.psi_right));
                    const double leftCosine =
                        left.block<3, 1>(0, 2).dot(direction);
                    const double rightCosine =
                        right.block<3, 1>(0, 2).dot(-direction);
                    if (leftCosine < leftMinimum ||
                        rightCosine < rightMinimum)
                    {
                        ++rejectedAxis;
                        return false;
                    }
                    if (!collisionValidator.valid(state))
                    {
                        ++rejectedCollision;
                        return false;
                    }
                }
                return true;
            };
            const auto nearestEquivalent = [&current](
                const tcst::ik_goal::Vec12 &candidate)
            {
                tcst::ik_goal::Vec12 normalized = candidate;
                for (int joint = 0; joint < 12; ++joint)
                {
                    while (normalized[joint] - current[joint] > M_PI)
                        normalized[joint] -= 2.0 * M_PI;
                    while (normalized[joint] - current[joint] < -M_PI)
                        normalized[joint] += 2.0 * M_PI;
                }
                return normalized;
            };
            Eigen::Matrix4d desiredLeft =
                generator.forwardPose(true, current.head<6>());
            Eigen::Matrix4d desiredRight =
                generator.forwardPose(false, current.tail<6>());
            const Eigen::Vector3d presentCenter = 0.5 * (
                desiredLeft.block<3, 1>(0, 3) +
                desiredRight.block<3, 1>(0, 3));
            const Eigen::Vector3d translation =
                options.target - presentCenter;
            desiredLeft.block<3, 1>(0, 3) += translation;
            desiredRight.block<3, 1>(0, 3) += translation;
            const std::vector<tcst::ik_goal::IKCandidate> leftCandidates =
                generator.solveForPose(true, desiredLeft, current.head<6>());
            const std::vector<tcst::ik_goal::IKCandidate> rightCandidates =
                generator.solveForPose(false, desiredRight, current.tail<6>());
            tcst::ik_goal::DualArmGoal result;
            result.score = std::numeric_limits<double>::infinity();
            result.left_candidate_count = leftCandidates.size();
            result.right_candidate_count = rightCandidates.size();
            for (const tcst::ik_goal::IKCandidate &left : leftCandidates)
            {
                for (const tcst::ik_goal::IKCandidate &right : rightCandidates)
                {
                    tcst::ik_goal::Vec12 candidate;
                    candidate << left.q, right.q;
                    candidate = nearestEquivalent(candidate);
                    ++result.tested_pair_count;
                    tcst::ik_goal::TaskMetrics metrics;
                    if (!generator.checkDualGoalConstraint(candidate, metrics))
                    {
                        ++rejectedEndpointTask;
                        continue;
                    }
                    if (!connectionValid(generator, candidate, pathLimits))
                    {
                        continue;
                    }
                    ++result.valid_pair_count;
                    const double score = (candidate - current).squaredNorm();
                    if (score < result.score)
                    {
                        result.success = true;
                        result.q = candidate;
                        result.metrics = metrics;
                        result.score = score;
                        result.message = "continuous pose propagation";
                    }
                }
            }
            if (!result.success)
            {
                result = generator.generate(
                    [&](const tcst::ik_goal::Vec12 &candidate)
                    {
                        const tcst::ik_goal::Vec12 normalized =
                            nearestEquivalent(candidate);
                        return connectionValid(
                            generator, normalized, pathLimits);
                    });
            }
            if (!result.success)
            {
                tcst::ik_goal::GoalGeneratorOptions retryOptions =
                    tcst::ik_goal::loadOptions(
                        config, tcst::ik_goal::TaskMode::MOVE);
                retryOptions.target =
                    expandedTargets[waypoint] + activeTargetOffset;
                retryOptions.target_from_reference = false;
                retryOptions.preserve_left_reference = false;
                retryOptions.preserve_right_world_pose = false;
                retryOptions.right_base_translation_delta.setZero();
                retryOptions.target_offset.setZero();
                retryOptions.stop_at_first_valid = false;
                retryOptions.distance_min += 0.005;
                retryOptions.distance_max -= 0.005;
                retryOptions.theta_left *= 0.85;
                retryOptions.theta_right *= 0.85;
                retryOptions.psi_left *= 0.85;
                retryOptions.psi_right *= 0.85;
                retryOptions.local_seed_ratio = 1.0;
                retryOptions.ik_position_tolerance =
                    std::min(retryOptions.ik_position_tolerance, 0.002);
                retryOptions.random_seed +=
                    static_cast<unsigned int>(waypoint);
                retryOptions.output_file.clear();
                tcst::ik_goal::DualArmGoalGenerator retryGenerator(
                    config["file_path"]["pinocchio_file"].as<std::string>(),
                    retryOptions, current);
                result = retryGenerator.generate(
                    [&](const tcst::ik_goal::Vec12 &candidate)
                    {
                        const tcst::ik_goal::Vec12 normalized =
                            nearestEquivalent(candidate);
                        return connectionValid(
                            retryGenerator, normalized, pathLimits);
                    });
                if (!result.success)
                {
                    bool diversionFound = false;
                    while (diversionAttempts[waypoint] < 8 &&
                           !diversionFound)
                    {
                        const int attempt = diversionAttempts[waypoint]++;
                        tcst::ik_goal::GoalGeneratorOptions sideOptions =
                            options;
                        static const Eigen::Vector3d offsets[] = {
                            Eigen::Vector3d(0.0, 0.0, 0.0),
                            Eigen::Vector3d(0.0, 0.0, 0.01),
                            Eigen::Vector3d(0.01, 0.0, 0.0),
                            Eigen::Vector3d(0.0, -0.01, 0.0),
                            Eigen::Vector3d(0.0, 0.0, 0.01),
                            Eigen::Vector3d(-0.01, 0.0, 0.0),
                            Eigen::Vector3d(0.0, 0.01, 0.0),
                            Eigen::Vector3d(0.0, 0.0, -0.005)};
                        sideOptions.target =
                            attempt == 0
                                ? collisionValidator.center(current)
                                : expandedTargets[waypoint] +
                                      activeTargetOffset + offsets[attempt];
                        sideOptions.random_seed += 5000u +
                            static_cast<unsigned int>(97 * attempt);
                        tcst::ik_goal::DualArmGoalGenerator sideGenerator(
                            config["file_path"]["pinocchio_file"]
                                .as<std::string>(),
                            sideOptions, current);
                        const tcst::ik_goal::DualArmGoal side =
                            sideGenerator.generate(
                                [&](const tcst::ik_goal::Vec12 &candidate)
                                {
                                    const tcst::ik_goal::Vec12 normalized =
                                        nearestEquivalent(candidate);
                                    return (normalized - current)
                                                   .cwiseAbs().maxCoeff() >=
                                               0.03 &&
                                           connectionValid(
                                               sideGenerator, normalized,
                                               pathLimits);
                                });
                        if (side.success)
                        {
                            current = nearestEquivalent(side.q);
                            pairs.push_back(current);
                            validPairs += side.valid_pair_count;
                            generationTime += side.generation_time_ms;
                            std::cout << "GUIDE_DIVERSION target_index="
                                      << waypoint << " attempt=" << attempt + 1
                                      << " valid_pairs="
                                      << side.valid_pair_count << std::endl;
                            if (attempt == 0 ||
                                waypoint + 1 == expandedTargets.size())
                            {
                                --waypoint;
                            }
                            else
                            {
                                activeTargetOffset += offsets[attempt];
                                offsetHold = 6;
                            }
                            diversionFound = true;
                        }
                    }
                    if (diversionFound)
                    {
                        continue;
                    }
                    std::cerr << "guide waypoint " << waypoint
                              << " failed: " << result.message
                              << "; rejected_endpoint_task="
                              << rejectedEndpointTask
                              << " rejected_edge_distance=" << rejectedDistance
                              << " rejected_edge_axis=" << rejectedAxis
                              << " rejected_edge_collision="
                              << rejectedCollision << std::endl;
                    return 1;
                }
            }
            const tcst::ik_goal::Vec12 continuous =
                nearestEquivalent(result.q);
            if (!collisionValidator.valid(continuous))
            {
                std::cerr << "equivalent-angle normalization failed at waypoint "
                          << waypoint << std::endl;
                return 1;
            }
            current = continuous;
            pairs.push_back(current);
            validPairs += result.valid_pair_count;
            generationTime += result.generation_time_ms;
            if (offsetHold > 0)
            {
                --offsetHold;
            }
            else if (activeTargetOffset.norm() > 1.0e-6)
            {
                activeTargetOffset *= 0.75;
                if (activeTargetOffset.norm() < 1.0e-4)
                {
                    activeTargetOffset.setZero();
                }
            }
            std::cout << "GUIDE_WAYPOINT index=" << waypoint
                      << " valid_pairs=" << result.valid_pair_count
                      << " generation_time_ms=" << result.generation_time_ms
                      << std::endl;
        }
        writePath(prefix + "_left.txt", pairs, 0);
        writePath(prefix + "_right.txt", pairs, 6);
        writePair(prefix + "_pair.txt", pairs.back());
        std::cout << "GUIDE_SUCCESS states=" << pairs.size()
                  << " valid_pairs=" << validPairs
                  << " generation_time_ms=" << generationTime
                  << " prefix=" << prefix << std::endl;
        return 0;
    }
    if (argc != 4 && argc != 5)
    {
        std::cerr << "usage: CM_SOCKET_BALL_GOAL catch|move CONFIG.yaml "
                     "OUTPUT_PREFIX [REFERENCE_PAIR.txt]\n"
                     "   or: CM_SOCKET_BALL_GOAL local catch|move left|right "
                     "CONFIG.yaml OUTPUT.tsv [REFERENCE_PAIR.txt]"
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
    const tcst::ik_goal::Vec12 initial =
        loadInitial(config, argc == 5 ? argv[4] : "");
    tcst::ik_goal::GoalGeneratorOptions options = tcst::ik_goal::loadOptions(
        config, phase == "catch" ? tcst::ik_goal::TaskMode::CATCH
                                  : tcst::ik_goal::TaskMode::MOVE);
    options.output_file.clear();
    CollisionValidator collisionValidator(phase, config);
    tcst::ik_goal::DualArmGoalGenerator generator(
        config["file_path"]["pinocchio_file"].as<std::string>(),
        options, initial);
    const tcst::ik_goal::DualArmGoal result = generator.generate(
        [&](const tcst::ik_goal::Vec12 &candidate)
        {
            return collisionValidator.valid(candidate);
        });
    if (!result.success)
    {
        std::cerr << "goal generation failed: " << result.message << std::endl;
        return 1;
    }

    const std::string prefix(argv[3]);
    writeRows(prefix + "_left.txt", initial.head<6>(), result.q.head<6>());
    writeRows(prefix + "_right.txt", initial.tail<6>(), result.q.tail<6>());
    writePair(prefix + "_pair.txt", result.q);
    std::cout << "GOAL_SUCCESS phase=" << phase
              << " valid_pairs=" << result.valid_pair_count
              << " generation_time_ms=" << result.generation_time_ms
              << " prefix=" << prefix << std::endl;
    return 0;
}
