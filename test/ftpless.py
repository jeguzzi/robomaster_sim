# type: ignore

import robomaster.conn


class FakeFtpConnection:
    def connect(self, ip):
        ...

    def upload(self, src_file, target_file):
        ...

    def stop(self):
        ...


robomaster.conn.FtpConnection = FakeFtpConnection
