# type: ignore

import time
from robomaster import robot, logger, logging, sensor  # noqa

import patch_ftp # noqa


def data_info(self):
    return self._cmd_id, self._direct, self._flag, self._distance


sensor.TofSubject.data_info = data_info


def cb(msg):
    print(msg)


def main():
    logger.setLevel(logging.ERROR)
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="sta")
    ep_robot.chassis.drive_speed(z=30)
    ep_robot.sensor.sub_distance(freq=5, callback=cb)
    time.sleep(10)
    ep_robot.sensor.unsub_distance()
    ep_robot.chassis.drive_speed(z=0)
    ep_robot.close()


if __name__ == '__main__':
    main()
