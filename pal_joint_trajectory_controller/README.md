This package was forked from `joint_trajectory_controller` to provide some
additional flexibility, which is necessary to implement whole body dynamic
control. In particular, original JointTrajectoryController class choses a
hardware interface adapter based on the hardware interface type, but we need to
have different adapters working with the same hardware interface type. This
package leaves the choice of an adapter to user.
