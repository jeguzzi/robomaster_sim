# type: ignore

import patch_ftp
from robomaster import robot, logger, logging  # noqa


def main():
    logger.setLevel(logging.WARN)
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="sta")
    print('Connected')
    ep_robot.close()
    print('Unconnected')


if __name__ == '__main__':
    main()
