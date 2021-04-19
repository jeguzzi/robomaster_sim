# type: ignore

import time
# import sys
# sys.path.insert(0, '/Users/Jerome/Dev/My/robomaster/src')

import robomaster.config  # noqa
from robomaster import robot, logger, logging  # noqa
import robomaster.conn

IP = "127.0.0.1"


class FakeFtpConnection:
    def connect(self, ip):
        logger.info("Fake FtpConnection: connect ip: {0}".format(ip))

    def upload(self, src_file, target_file):
        ...

    def stop(self):
        ...


robomaster.conn.FtpConnection = FakeFtpConnection


def main():
    logger.setLevel(logging.ERROR)
    # robomaster.config.LOCAL_IP_STR = IP
    # robomaster.config.ROBOT_IP_STR = IP
    ep_robot = robot.Robot()
    # ep_robot.initialize(conn_type="sta", ftp=False)
    ep_robot.initialize(conn_type="sta")
    # time.sleep(1)
    # x = 0.5  # m/s, float
    # y = 1.0  # m/s, float
    x= 0
    y = 0
    z = 90  # int, degrees
    # print('\n[Test] drive_speed\n')
    ep_robot.chassis.drive_speed(x=x, y=y, z=z)
    # time.sleep(1)
    #
    # print('\n[Test] drive_wheels\n')
    # ep_robot.chassis.drive_wheels(w2=1, w3=3.3, w4=-6.6)
    # time.sleep(1)

    # print('\n[Test] set_pwm_value\n')
    # ep_robot.chassis.set_pwm_value(pwm1=10, pwm2=51, pwm6=100)
    # time.sleep(1)
    #
    # print('\n[Test] set_pwm_freq\n')
    # ep_robot.chassis.set_pwm_freq(pwm1=10000, pwm2=11111, pwm6=12345)
    # time.sleep(1)

    # print('\n[Test] move\n')
    # ep_robot.chassis.move(x=0, y=0, z=720, xy_speed=0.25, z_speed=180).wait_for_completed()
    time.sleep(1)

    rate = 5

    def cb(msg, name=''):
        print(f'msg {name}:', msg)

    for name in ('position', 'attitude', 'velocity', 'status', 'imu', 'mode', 'esc')[1:2]:
        print(f'\n[Test] sub_{name}\n')
        getattr(ep_robot.chassis, f'sub_{name}')(freq=rate, callback=cb, name=name)
        time.sleep(2)
        getattr(ep_robot.chassis, f'unsub_{name}')()

    # print('\n[Test] _sub_sbus\n')
    # ep_robot.chassis._sub_sbus(freq=rate, callback=cb, name='sbus')
    # time.sleep(2)
    # ep_robot.chassis._unsub_sbus()

    print('\n[Test] stop\n')
    ep_robot.chassis.stop()
    ep_robot.close()


if __name__ == '__main__':
    main()
