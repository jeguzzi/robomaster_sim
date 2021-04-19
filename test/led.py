# type: ignore

import time
import robomaster.config  # noqa
from robomaster import robot, logger, logging  # noqa
import robomaster.conn
import robomaster.led

import ftpless


def main():
    logger.setLevel(logging.WARN)
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="sta")
    time.sleep(1)
    print('\n[Test] set_led\n')
    # ep_robot.led.set_led(r=120, g=0, b=230, effect=robomaster.led.EFFECT_BREATH, freq=5)
    ep_robot.led.set_led(r=120, g=0, b=230, effect=robomaster.led.EFFECT_SCROLLING, freq=1)
    time.sleep(3)

    # print('\n[Test] set_gimbal_led\n')
    # for i in range(8):
    #     ep_robot.led.set_gimbal_led(g=255 - (i + 1) * 255 // 8, b=0, r=(i + 1) * 255 // 8, led_list=[i])
    #     time.sleep(0.2)
    #
    # ep_robot.led.set_led(r=120, g=0, b=230, effect=robomaster.led.EFFECT_BREATH, freq=5)
    # time.sleep(3)

    ep_robot.close()


if __name__ == '__main__':
    main()
