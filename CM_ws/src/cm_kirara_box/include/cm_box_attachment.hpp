#pragma once

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/multibody/model.hpp>
#include <yaml-cpp/yaml.h>
#include <Eigen/Geometry>
#include <stdexcept>
#include <string>

namespace cm_box
{
// Geometry only: the task's DH endpoint, objective and controller are unchanged.
// GazeboGraspFix carries the object at its captured pose relative to palm_link.
class Attachment
{
public:
    void load(const YAML::Node &config, const pinocchio::Model &model)
    {
        const YAML::Node node = config["carried_box"];
        if (!node || !node["anchor_frame"] || !node["translation_m"] ||
            !node["quaternion_xyzw"])
            throw std::runtime_error("Box Move requires a calibrated carried_box mapping");
        const std::string name = node["anchor_frame"].as<std::string>();
        frame_ = model.getFrameId(name, pinocchio::BODY);
        joint_ = model.getJointId("target_box_world");
        if (frame_ >= model.frames.size() || joint_ >= model.joints.size() ||
            joint_ == 0 || model.joints[joint_].nq() != 7)
            throw std::runtime_error("Invalid Box anchor frame or target_box_world free joint");
        const YAML::Node t = node["translation_m"], r = node["quaternion_xyzw"];
        if (!t.IsSequence() || t.size() != 3 || !r.IsSequence() || r.size() != 4)
            throw std::runtime_error("carried_box requires 3 translation and 4 quaternion values");
        Eigen::Vector3d translation(t[0].as<double>(), t[1].as<double>(), t[2].as<double>());
        Eigen::Quaterniond rotation(r[3].as<double>(), r[0].as<double>(),
                                    r[1].as<double>(), r[2].as<double>());
        if (!translation.allFinite() || !rotation.coeffs().allFinite() ||
            std::abs(rotation.norm() - 1.0) > 1e-6)
            throw std::runtime_error("carried_box must contain finite values and a unit quaternion");
        rotation.normalize();
        relative_ = pinocchio::SE3(rotation.toRotationMatrix(), translation);
        loaded_ = true;
    }

    template<class Derived>
    pinocchio::SE3 apply(const pinocchio::Model &model, pinocchio::Data &data,
                         Eigen::MatrixBase<Derived> &q) const
    {
        if (!loaded_)
            throw std::runtime_error("Box attachment mapping was not loaded");
        pinocchio::framesForwardKinematics(model, data, q);
        const pinocchio::SE3 world = data.oMf[frame_] * relative_;
        // Convert world pose to this free joint's parent coordinates.
        const pinocchio::SE3 parent = data.oMi[model.parents[joint_]] *
                                      model.jointPlacements[joint_];
        const pinocchio::SE3 local = parent.inverse() * world;
        const int index = model.joints[joint_].idx_q();
        q.template segment<3>(index) = local.translation();
        q.template segment<4>(index + 3) = Eigen::Quaterniond(local.rotation()).coeffs();
        return world;
    }

private:
    bool loaded_ = false;
    pinocchio::FrameIndex frame_ = 0;
    pinocchio::JointIndex joint_ = 0;
    pinocchio::SE3 relative_ = pinocchio::SE3::Identity();
};
}  // namespace cm_box
