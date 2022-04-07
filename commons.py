from fastcrc import crc16


def get_crc(frame):
    return crc16.xmodem(bytes(frame))
