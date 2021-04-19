# type: ignore
import time

import robomaster.config
import robomaster.config  # noqa
from robomaster import robot, logger, logging  # noqa
import robomaster.conn


class FakeFtpConnection:
    def connect(self, ip):
        logger.info("Fake FtpConnection: connect ip: {0}".format(ip))

    def upload(self, src_file, target_file):
        ...

    def stop(self):
        ...


robomaster.conn.FtpConnection = FakeFtpConnection


def main():
    logger.setLevel(logging.INFO)
    # connection internally uses scan_robot_ip(sn=None)
    # which listens to broadcast messages on <port>:
    #   sn != None
    #       -> port = 40927
    #       -> check sn == sender_sn
    #   sn == None
    #       -> port = 45678
    #       -> no check
    # sn is propagate down from `ep_robot.initialize(..., sn=None)`
    # remote id than is the broadcast message sender
    # def get_sn_form_data(data):
    #   data = data.split(b'\x00')
    #   recv_sn = data[0]
    #   recv_sn = recv_sn.decode(encoding='utf-8')
    #   return recv_sn
    #
    #
    # The helper is just to communicate ssid and password to the robot.
    # We don't need it for the simulator (unless we want to communicate with the app)
    #
    # helper = robomaster.conn.ConnectionHelper()
    # helper._appid = "e009555"
    # helper.wait_for_connection()
    print('[Scan for ip]', robomaster.conn.scan_robot_ip())
    # print(robomaster.conn.scan_robot_ip_list())
    print('[Connect]')
    ep_robot = robot.Robot()
    # ep_robot.initialize(conn_type="ap")
    ep_robot.initialize(conn_type="sta", sn=None)
    time.sleep(5)
    print('[Unconnect]')
    ep_robot.close()


if __name__ == '__main__':
    main()
