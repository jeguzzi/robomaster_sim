# type: ignore

import time
import patch_ftp
import robomaster.config  # noqa
from robomaster import robot, logger, logging  # noqa


def main():
    logger.setLevel(logging.ERROR)
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="sta")
    ep_robot.set_robot_mode('chassis_lead')
    print('Drive in circle for 5 second')
    ep_robot.chassis.drive_speed(0.3, 0, 45)
    time.sleep(5)
    print('Move forward 1 meter and turn by 90 degrees')
    time.sleep(1)
    ep_robot.chassis.move(x=1, y=0, z=90, xy_speed=1.0, z_speed=90).wait_for_completed()
    print('Arrived')
    ep_robot.chassis.stop()
    ep_robot.close()


if __name__ == '__main__':
    main()
