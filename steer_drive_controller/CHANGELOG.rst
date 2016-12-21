^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package steer_drive_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.0 (2016-12-28)
------------------
* steer_drive_controller: New controller for steering mechanism drive systems.
* This controller was built up from an existing controller, diff_drive_controller.
* Control is in the form of a velocity command, that is split then sent on the single wheel joint and single steer joint.
* Odometry is published to tf and to a dedicated nav__msgs/Odometry topic.
* Realtime-safe implementation.
* Implements task-space velocity, acceleration and jerk limits.
* Automatic stop after command time-out.
* Contributors: Masaru Morita.
