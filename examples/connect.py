# Adapted SDK examples/01_robot/05_sta_conn_helper.py
import argparse
import qrcode
import robomaster.conn


def main() -> None:
    parser = argparse.ArgumentParser(description='')
    parser.add_argument('ssid', type=str, help='The network SSID')
    parser.add_argument('password', type=str, help='The network password')
    parser.add_argument('--app_id', default='', type=str, help='The app id of at most 7 letters')
    args = parser.parse_args()
    helper = robomaster.conn.ConnectionHelper()
    if args.app_id:
        helper._appid = args.app_id[:8]
    info = helper.build_qrcode_string(ssid=args.ssid, password=args.password)
    img = qrcode.make(info)
    img.show()
    if helper.wait_for_connection():
        print("Connected")
    else:
        print("Connection failed")


if __name__ == '__main__':
    main()
