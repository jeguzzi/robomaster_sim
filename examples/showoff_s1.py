# type: ignore

import time
import patch_ftp
import robomaster.config  # noqa
from robomaster import robot, logger, logging  # noqa
import robomaster.conn
import robomaster.camera
import cv2
import sys
# import threading
import patch_ftp  # noqa


# IP = "127.0.0.1" # Leave to None to scan for IPs on (0.0.0.0)
IP = ""


run_video = True


def _video(ep_robot):
    print('start video stream')
    ep_robot.camera.start_video_stream(display=False, resolution="720p")
    global run_video
    run_video = True
    while run_video:
        try:
            frame = ep_robot.camera.read_cv2_image()
        except:
            frame = None
        if frame is not None:
            cv2.imshow('image', frame)
            cv2.waitKey(1)
        else:
            time.sleep(0.01)
    ep_robot.camera.stop_video_stream()


def _action(ep_robot):
    global run_video
    print('start action')
    for i in range(12):
        if i % 2:
            r, g, b = 200, 0, 0
            ep_robot.set_robot_mode('free')
            ep_robot.blaster.set_led(effect='off')
        else:
            r, g, b = 0, 0, 200
            ep_robot.set_robot_mode('chassis_lead')
            ep_robot.blaster.set_led(brightness=100, effect='on')
        ep_robot.led.set_led(r=r, g=g, b=b, effect=robomaster.led.EFFECT_BREATH, freq=10)
        ep_robot.chassis.move(x=1, y=0, z=0, xy_speed=1.0, z_speed=90).wait_for_completed()
        ep_robot.chassis.move(x=0, y=0, z=90, xy_speed=1.0, z_speed=90).wait_for_completed()
        ep_robot.gimbal.moveto(yaw=-90, pitch=-20, yaw_speed=90, pitch_speed=30).wait_for_completed()
        ep_robot.gimbal.moveto(yaw=-45, pitch=0, yaw_speed=90, pitch_speed=30).wait_for_completed()
    run_video = False


def main():
    logger.setLevel(logging.WARN)

    if IP:
        robomaster.config.LOCAL_IP_STR = IP
        robomaster.config.ROBOT_IP_STR = IP

    if(len(sys.argv) > 1):
        protocol = sys.argv[-1]
        robomaster.config.ep_conf.video_stream_proto = protocol
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="sta")
    # action = threading.Thread(target=_action, args=(ep_robot,))
    # action.start()
    # _video(ep_robot)
    _action(ep_robot)
    ep_robot.close()


if __name__ == '__main__':
    main()
