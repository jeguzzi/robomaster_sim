# type: ignore

import time
from robomaster import robot, logger, logging, sensor  # noqa



def cb(msg):
    print(msg)


def main():
    logger.setLevel(logging.ERROR)
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="sta")
    # ep_robot.robotic_arm.moveto(71, 56).wait_for_completed()
    ep_robot.robotic_arm.moveto(180, -80).wait_for_completed()
    # ep_robot.robotic_arm.recenter().wait_for_completed()
    ep_robot.servo.sub_servo_info(freq=10, callback=cb)
    ep_robot.robotic_arm.sub_position(freq=10, callback=cb)
    # ep_robot.servo.moveto(index=1, angle=0).wait_for_completed()
    # ep_robot.servo.moveto(index=2, angle=0).wait_for_completed()
    while True:
        a0 = ep_robot.servo.get_angle(index=1)
        a1 = ep_robot.servo.get_angle(index=2)
        time.sleep(1)
        print(f'angles: {a0} {a1}')
    ep_robot.close()


if __name__ == '__main__':
    main()


# ISSUES
# - SIM get angles ritorna garbage 574810.0 537330.0
# - SIM ep_robot.robotic_arm.recenter().wait_for_completed() non fa quello che mi aspetto:
#   adesso cede lentamente i motori dopo essere andato a ...
# - SIM la posizione (z) dell'end effector non va sotto 0
#


# LOG SIM moveto(180, -80)
# ([1, 1, 0, 0], [0, 0, 0, 0], [1174, 484, 0, 0])
# (178, 0)
# ([1, 1, 0, 0], [0, 0, 0, 0], [1174, 484, 0, 0])
# (178, 0)
# ([1, 1, 0, 0], [0, 0, 0, 0], [1174, 484, 0, 0])
# (178, 0)
# ([1, 1, 0, 0], [0, 0, 0, 0], [1174, 484, 0, 0])
# (178, 0)
# ([1, 1, 0, 0], [0, 0, 0, 0], [1174, 484, 0, 0])
# (178, 0)
# ([1, 1, 0, 0], [0, 0, 0, 0], [1174, 484, 0, 0])
# (178, 0)
# ([1, 1, 0, 0], [0, 0, 0, 0], [1174, 484, 0, 0])
# (178, 0)
# ([1, 1, 0, 0], [0, 0, 0, 0], [1174, 484, 0, 0])
# (178, 0)
# ([1, 1, 0, 0], [0, 0, 0, 0], [1174, 484, 0, 0])
# (178, 0)
# ([1, 1, 0, 0], [0, 0, 0, 0], [1174, 484, 0, 0])
# (178, 0)
# angles: 575120.0 536540.0
#

# LOG REAL moveto(180, -80)
