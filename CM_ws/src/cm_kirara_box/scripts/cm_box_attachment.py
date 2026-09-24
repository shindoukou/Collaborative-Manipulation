#!/usr/bin/env python3
"""Box collision geometry, using the same captured palm transform as Gazebo."""
import numpy as np
import pinocchio as pin


class BoxAttachment:
    def __init__(self, config, model):
        node = config.get('carried_box')
        if not node:
            raise ValueError('Box Move requires a calibrated carried_box mapping')
        self.frame = model.getFrameId(node['anchor_frame'], pin.FrameType.BODY)
        self.joint = model.getJointId('target_box_world')
        if (self.frame >= model.nframes or self.joint == 0 or
                self.joint >= model.njoints or model.nqs[self.joint] != 7):
            raise ValueError('Invalid Box anchor or target_box_world free joint')
        t = np.asarray(node['translation_m'], dtype=float)
        r = np.asarray(node['quaternion_xyzw'], dtype=float)
        if (t.shape != (3,) or r.shape != (4,) or
                not np.isfinite(t).all() or not np.isfinite(r).all() or
                abs(np.linalg.norm(r) - 1.0) > 1e-6):
            raise ValueError('carried_box requires finite translation and a unit quaternion')
        self.relative = pin.SE3(pin.Quaternion(r / np.linalg.norm(r)).matrix(), t)

    def apply(self, model, data, q):
        pin.framesForwardKinematics(model, data, q)
        world = data.oMf[self.frame] * self.relative
        parent = data.oMi[model.parents[self.joint]] * model.jointPlacements[self.joint]
        local = parent.inverse() * world
        index = model.idx_qs[self.joint]
        q[index:index + 3] = local.translation
        q[index + 3:index + 7] = pin.Quaternion(local.rotation).coeffs()
        return world
