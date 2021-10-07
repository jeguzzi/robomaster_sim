# type: ignore
#

# Patches the FtpConnection to accept failures
# and therefore robots that do not offer FTP (like in simulation)


from ftplib import FTP

import robomaster.conn


class FtpConnection:

    def __init__(self) -> None:
        self._ftp = FTP()
        self._ftp.set_debuglevel(0)
        self._connected = False
        self._bufsize = 1024

    @property
    def connected(self) -> bool:
        return self._connected

    def connect(self, ip: str) -> None:
        robomaster.logger.info(f"FtpConnection: connect ip: {ip}")
        try:
            self._ftp.connect(ip, 21, timeout=1.0)
            self._connected = True
        except:
            robomaster.logger.warning(f"FtpConnection: could not connect to {ip}")
            self._connected = False

    def upload(self, src_file: str, target_file: str) -> None:
        if self._connected:
            try:
                with open(src_file, 'rb') as fp:
                    self._ftp.storbinary("STOR " + target_file, fp, self._bufsize)
            except Exception as e:
                robomaster.logger.warning("FtpConnection: upload e {0}".format(e))
        else:
            robomaster.logger.warning("FtpConnection: connection is not open, cannot upload e")

    def stop(self) -> None:
        if self._connected:
            self._ftp.close()


robomaster.conn.FtpConnection = FtpConnection
