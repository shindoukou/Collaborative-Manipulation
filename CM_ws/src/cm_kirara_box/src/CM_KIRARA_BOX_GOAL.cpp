#define PINOCCHIO_WITH_HPP_FCL true

#include <pinocchio/config.hpp>
#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/multibody/fcl.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/parsers/urdf/geometry.hxx>

#include "tcst_box_ik_goal_generator.hpp"
#include "cm_box_attachment.hpp"

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <random>
#include <set>
#include <string>

namespace
{

class BoxCollisionChecker
{
public:
    BoxCollisionChecker(
        const std::string &urdf_file,
        const std::string &model_path,
        const tcst::ik_goal::Vec12 &initial,
        const tcst::ik_goal::Vec12 &reference,
        const Eigen::Vector3d &box_center,
        const Eigen::Matrix3d &box_rotation,
        bool allow_reference_collisions,
        bool allow_target_contact,
        const YAML::Node &config)
        : box_center_(box_center), box_rotation_(box_rotation),
          allow_target_contact_(allow_target_contact)
    {
        pinocchio::urdf::buildModel(urdf_file, model_);
        data_.reset(new pinocchio::Data(model_));
        if (model_.nq >= 19)
            attachment_.load(config, model_);
        pinocchio::urdf::buildGeom(
            model_, urdf_file, pinocchio::COLLISION, geometry_model_, model_path);
        geometry_model_.addAllCollisionPairs();
        geometry_data_.reset(new pinocchio::GeometryData(geometry_model_));
        baseline_ = collisions(initial);
        if (allow_reference_collisions)
        {
            const std::set<std::size_t> reference_pairs = collisions(reference);
            for (const std::size_t index : reference_pairs)
            {
                if (baseline_.count(index) == 0 &&
                    !isIntendedTargetContact(index))
                    allowed_reference_pairs_.insert(index);
                baseline_.insert(index);
            }
        }
    }

    bool operator()(const tcst::ik_goal::Vec12 &q)
    {
        const std::set<std::size_t> active = collisions(q);
        bool valid = true;
        for (const std::size_t index : active)
        {
            if (baseline_.count(index) == 0 &&
                !isIntendedTargetContact(index))
            {
                ++rejected_pair_counts_[index];
                valid = false;
            }
        }
        return valid;
    }

    std::string describe(std::size_t index) const
    {
        const pinocchio::CollisionPair &pair =
            geometry_model_.collisionPairs[index];
        return geometry_model_.geometryObjects[pair.first].name + "<->" +
               geometry_model_.geometryObjects[pair.second].name;
    }

    const std::set<std::size_t> &allowedReferencePairs() const
    {
        return allowed_reference_pairs_;
    }

    const std::map<std::size_t, std::size_t> &rejectedPairCounts() const
    {
        return rejected_pair_counts_;
    }

    void printCollisions(
        const std::string &label, const tcst::ik_goal::Vec12 &q)
    {
        const std::set<std::size_t> active = collisions(q);
        std::cout << "[box collisions] " << label
                  << " model_nq=" << model_.nq << " active=";
        for (const std::size_t index : active)
            std::cout << index << "(" << describe(index) << ")"
                      << (baseline_.count(index) ? "[baseline]" : "")
                      << ",";
        std::cout << std::endl;
    }

private:
    Eigen::VectorXd configuration(const tcst::ik_goal::Vec12 &q) const
    {
        Eigen::VectorXd result = pinocchio::neutral(model_);
        result.head(std::min<int>(12, result.size())) =
            q.head(std::min<int>(12, result.size()));
        if (result.size() >= 19)
        {
            attachment_.apply(model_, *data_, result);
        }
        return result;
    }

    std::set<std::size_t> collisions(const tcst::ik_goal::Vec12 &q)
    {
        const Eigen::VectorXd model_q = configuration(q);
        pinocchio::computeCollisions(
            model_, *data_, geometry_model_, *geometry_data_, model_q);
        std::set<std::size_t> result;
        for (std::size_t i = 0; i < geometry_model_.collisionPairs.size(); ++i)
            if (geometry_data_->collisionResults[i].isCollision())
                result.insert(i);
        return result;
    }

    bool isIntendedTargetContact(std::size_t index) const
    {
        if (!allow_target_contact_)
            return false;
        const pinocchio::CollisionPair &pair =
            geometry_model_.collisionPairs[index];
        const std::string &first =
            geometry_model_.geometryObjects[pair.first].name;
        const std::string &second =
            geometry_model_.geometryObjects[pair.second].name;
        const bool first_target =
            first.find("target_box") != std::string::npos;
        const bool second_target =
            second.find("target_box") != std::string::npos;
        const bool first_ee =
            first.find("ft300") != std::string::npos ||
            first.find("tool0") != std::string::npos;
        const bool second_ee =
            second.find("ft300") != std::string::npos ||
            second.find("tool0") != std::string::npos;
        return (first_target && second_ee) || (second_target && first_ee);
    }

    pinocchio::Model model_;
    cm_box::Attachment attachment_;
    std::unique_ptr<pinocchio::Data> data_;
    pinocchio::GeometryModel geometry_model_;
    std::unique_ptr<pinocchio::GeometryData> geometry_data_;
    std::set<std::size_t> baseline_;
    std::set<std::size_t> allowed_reference_pairs_;
    std::map<std::size_t, std::size_t> rejected_pair_counts_;
    Eigen::Vector3d box_center_;
    Eigen::Matrix3d box_rotation_;
    bool allow_target_contact_ = false;
};

bool verifyRoundTrips(
    tcst::ik_goal::DualArmGoalGenerator &ik,
    const tcst::ik_goal::Vec12 &reference,
    int trials)
{
    std::mt19937 rng(20260728u);
    std::uniform_real_distribution<double> perturbation(-0.10, 0.10);
    int passed = 1;
    for (int trial = 0; trial < trials; ++trial)
    {
        tcst::ik_goal::Vec12 target = reference;
        for (int i = 0; i < 12; ++i)
            target(i) += perturbation(rng);
        const Eigen::Matrix4d left_pose =
            ik.forwardPose(true, target.head<6>());
        const Eigen::Matrix4d right_pose =
            ik.forwardPose(false, target.tail<6>());
        if (!ik.solveForPose(
                    true, left_pose, reference.head<6>())
                 .empty() &&
            !ik.solveForPose(
                    false, right_pose, reference.tail<6>())
                 .empty())
            ++passed;
        else
            std::cerr << "[box round-trip] failed trial " << trial << "\n";
    }
    std::cout << "[box round-trip] passed " << passed << "/"
              << trials + 1
              << " (includes Joint_final FK->IK->FK)" << std::endl;
    return passed == trials + 1;
}

}  

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        std::cerr << "usage: tcst_box_ik_goal_test CONFIG_YAML "
                     "[goal_trials=100] [roundtrip_trials=100]"
                  << std::endl;
        return 64;
    }
    const int goal_trials =
        argc > 2 ? std::max(1, std::atoi(argv[2])) : 100;
    const int roundtrip_trials =
        argc > 3 ? std::max(0, std::atoi(argv[3])) : 100;
    const YAML::Node config = YAML::LoadFile(argv[1]);
    const tcst::ik_goal::Vec12 initial =
        tcst::ik_goal::loadJointVector(config["Joint_init"]);
    const tcst::ik_goal::Vec12 reference =
        tcst::ik_goal::loadJointVector(config["Joint_final"]);
    const std::string urdf_file =
        config["file_path"]["pinocchio_file"].as<std::string>();
    const std::string model_path =
        config["model_path"]["pinocchio_path"].as<std::string>();

    tcst::ik_goal::GoalGeneratorOptions ik_options =
        tcst::ik_goal::loadOptions(
            config, tcst::ik_goal::TaskMode::CATCH);
    ik_options.debug_file.clear();
    ik_options.output_file.clear();
    tcst::ik_goal::DualArmGoalGenerator ik(
        urdf_file, ik_options, reference);
    if (!ik.initialized())
    {
        std::cerr << "[box IK init] " << ik.error() << std::endl;
        return 2;
    }

    tcst::box_goal::BoxGoalOptions box_options =
        tcst::box_goal::loadBoxOptions(config);
    tcst::box_goal::BoxDualArmGoalGenerator box_generator(
        ik, box_options, reference, initial);
    const Eigen::Matrix4d left_reference =
        ik.forwardPose(true, reference.head<6>());
    const Eigen::Matrix4d right_reference =
        ik.forwardPose(false, reference.tail<6>());
    const Eigen::Vector3d reference_direction =
        (right_reference.block<3, 1>(0, 3) -
         left_reference.block<3, 1>(0, 3))
            .normalized();
    std::cout << "[box geometry] center="
              << box_generator.center().transpose()
              << " grasp_center=" << box_generator.graspCenter().transpose()
              << " normal=" << box_generator.normal().transpose()
              << " long_axis=" << box_generator.longAxis().transpose()
              << " reference_direction=" << reference_direction.transpose()
              << " left x/y parallel="
              << std::abs(left_reference.block<3, 1>(0, 0).dot(
                     box_generator.longAxis()))
              << "/"
              << std::abs(left_reference.block<3, 1>(0, 1).dot(
                     box_generator.longAxis()))
              << " reference_left="
              << left_reference.block<3, 1>(0, 3).transpose()
              << " reference_right="
              << right_reference.block<3, 1>(0, 3).transpose()
              << std::endl;

    const bool allow_reference_collisions =
        config["box_goal_generator"]["allow_reference_collisions"]
            ? config["box_goal_generator"]["allow_reference_collisions"]
                  .as<bool>()
            : false;
    const bool allow_target_contact =
        config["box_goal_generator"]["allow_target_contact"]
            ? config["box_goal_generator"]["allow_target_contact"].as<bool>()
            : box_options.mode == tcst::ik_goal::TaskMode::MOVE;
    BoxCollisionChecker collision_checker(
        urdf_file, model_path, initial, reference,
        box_generator.center(), box_generator.boxRotation(),
        allow_reference_collisions, allow_target_contact, config);
    std::cout << "[box collision-policy] allowed_reference_pairs=";
    for (const std::size_t index :
         collision_checker.allowedReferencePairs())
        std::cout << index << "(" << collision_checker.describe(index)
                  << "),";
    std::cout << std::endl;

    const bool roundtrips_ok =
        verifyRoundTrips(ik, reference, roundtrip_trials);
    if (!box_options.output_file.empty())
        std::ofstream(
            box_options.output_file,
            std::ios::out | std::ios::trunc);

    int passed = 0;
    double total_ms = 0.0;
    tcst::box_goal::BoxGoal last_goal;
    for (int trial = 0; trial < goal_trials; ++trial)
    {
        last_goal = box_generator.generate(
            [&](const tcst::ik_goal::Vec12 &q)
            { return collision_checker(q); });
        total_ms += last_goal.time_ms;
        if (last_goal.success)
            ++passed;
        else
            std::cerr << "[box goal-only] failed trial " << trial
                      << " pairs=" << last_goal.tested_pairs << "\n";
    }

    std::cout << std::fixed << std::setprecision(6)
              << "[box goal-only] mode="
              << (box_options.mode == tcst::ik_goal::TaskMode::CATCH
                      ? "CATCH"
                      : "MOVE")
              << " passed=" << passed << "/" << goal_trials
              << " success_rate="
              << 100.0 * static_cast<double>(passed) / goal_trials
              << "% mean_time_ms=" << total_ms / goal_trials;
    if (last_goal.success)
        std::cout << " distance=" << last_goal.metrics.distance
                  << " normal_cos="
                  << last_goal.metrics.normal_alignment
                  << " parallel_cos="
                  << last_goal.metrics.left_parallel_cos << "/"
                  << last_goal.metrics.right_parallel_cos;
    std::cout << std::endl;

    if (last_goal.success)
        collision_checker.printCollisions("selected_goal", last_goal.q);

    if (passed != goal_trials)
    {
        std::cout << "[box collision-rejections] ";
        for (const auto &entry : collision_checker.rejectedPairCounts())
            std::cout << entry.first << "("
                      << collision_checker.describe(entry.first) << ")="
                      << entry.second << " ";
        std::cout << std::endl;
    }
    return roundtrips_ok && passed == goal_trials ? 0 : 1;
}
