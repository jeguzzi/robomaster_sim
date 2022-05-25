# type: ignore
import time

import robomaster.config
import robomaster.config  # noqa
from robomaster import robot, logger, logging, config  # noqa
import robomaster.conn
import robomaster.led
import socket

import patch_ftp # noqa

from typing import Dict


def discover_robots(timeout: float = 3.0) -> Dict[str, str]:
    # ip -> sn
    robots: Dict[str, str] = {}
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("0.0.0.0", config.ROBOT_BROADCAST_PORT))
    s.settimeout(timeout)
    start = time.time()
    while time.time() - start < timeout:
        data, (ip, port) = s.recvfrom(1024)
        if ip not in robots:
            sn = str(data[:-1].decode(encoding='utf-8'))
            robots[ip] = sn
    return robots


def main():
    logger.setLevel(logging.CRITICAL)
    robots = discover_robots()
    print(f'Discovered {len(robots)} robots')
    for ip, sn in robots.items():
        ep_robot = robot.Robot()
        ep_robot.initialize(conn_type="sta", sn=sn)
        version = ep_robot.get_version()
        play = ep_robot.play_sound(7, times=1)
        ep_robot.led.set_led(r=255, effect=robomaster.led.EFFECT_FLASH, freq=5)
        print(f'Found {sn} at {ip} with version {version}')
        for name, module in sorted(ep_robot._modules.items()):
            version = module.get_version()
            if version:
                print(f'\t{name} {version}')
        play.wait_for_completed()
        ep_robot.led.set_led(effect=robomaster.led.EFFECT_OFF)
        ep_robot.play_sound(8, times=1).wait_for_completed()
        ep_robot.close()


if __name__ == '__main__':
    main()
