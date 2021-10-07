# type: ignore

import time
import patch_ftp
import robomaster.config  # noqa
from robomaster import robot, logger, logging  # noqa


def main():
    logger.setLevel(logging.ERROR)
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="sta")
    ep_robot.chassis.drive_speed(x=0.1, y=0.1, z=20)
    rate = 5

    def cb(msg, name=''):
        print(f'msg {name}:', msg)

    for name in ('position', 'attitude', 'velocity', 'status', 'imu', 'mode', 'esc')[:]:
        print(f'\n[Test] sub_{name}\n')
        getattr(ep_robot.chassis, f'sub_{name}')(freq=rate, callback=cb, name=name)
        time.sleep(2)
        getattr(ep_robot.chassis, f'unsub_{name}')()

    ep_robot.chassis.drive_speed(x=0.0, y=0.0, z=0.0)
    ep_robot.chassis.stop()
    ep_robot.close()


if __name__ == '__main__':
    main()
