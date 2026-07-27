# Archived ZeroErr Joint Jog GUI

`zeroerr_joint_jog_gui.py` is a standalone PyQt6 test utility for sending
small single-joint `FollowJointTrajectory` moves to
`/manipulator_controller/follow_joint_trajectory`.

It subscribes to `/joint_states`, displays current joint positions, and lets an
operator send relative degree steps to one selected joint.

The script was not referenced by active launch files, runtime code, shell
scripts, or configs. It was only installed as a manually runnable executable, so
it has been archived to keep the active package surface smaller.
