# type: ignore

import time
import patch_ftp
import robomaster.config  # noqa
from robomaster import robot, logger, logging, protocol, util   # noqa
import robomaster.conn
import robomaster.led
from robomaster.led import EFFECT_ON, EFFECT_OFF


def main():
    logger.setLevel(logging.WARN)
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="sta")
    ep_robot.led.set_led(r=120, g=0, b=230, effect=robomaster.led.EFFECT_BREATH)
    time.sleep(3)
    ep_robot.led.set_led(r=0, g=0, b=230, effect=EFFECT_ON)
    time.sleep(3)
    ep_robot.led.set_led(effect=EFFECT_OFF)
    ep_robot.close()


if __name__ == '__main__':
    main()
