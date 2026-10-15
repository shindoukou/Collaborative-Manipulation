Unified Ball / Box simulation runtime

The cm_ball and cm_box simulation entry points load this directory before
cm_dnmpc/scripts. Physical-robot entry points continue to use the original
controller_node.py and controller_core.py in scripts.

Stage 1 tracks the FK position of the local grasping reference. Stages 2-4
use the communicated peer prediction to construct the desired separation.
This prevents a post-contact clamping objective from pulling the arms
together before the approach trajectory is complete.

Shared protocol, transport, object-profile and impedance modules are reused
from scripts; this directory contains no historical backup copies.
The verified simulation configuration retains N=5 and Ts=0.02 s.
Ball and Box Stage 2/4 force-completion tolerances are 2 N. These are
completion criteria, not a guarantee of a 2 N error throughout transport.

The corresponding remote runtime passed three complete four-stage Gazebo
runs per object on 2026-10-15. This repository keeps its public module and
package names; physical-object profiles and recorded trajectories were not
changed by this synchronization. Full-path collision safety was not audited
by those completion tests.
