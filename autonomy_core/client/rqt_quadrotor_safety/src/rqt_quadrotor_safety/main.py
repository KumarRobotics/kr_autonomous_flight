"""Standalone launcher for the rqt_quadrotor_safety plugin.

This provides an entry-point-compatible alternative to the ROS1
``scripts/rqt_quadrotor_safety`` file so the plugin can be run as::

    ros2 run rqt_quadrotor_safety rqt_quadrotor_safety

It still delegates to ``rqt_gui.main.Main`` which sets up an rclpy node
internally and injects it into the plugin via ``context.node``.
"""

import sys

from rqt_gui.main import Main


def main(argv=None):
    if argv is None:
        argv = sys.argv
    app = Main()
    return app.main(
        argv,
        standalone='rqt_quadrotor_safety.quadrotor_safety.QuadrotorSafety',
    )


if __name__ == '__main__':
    sys.exit(main())
