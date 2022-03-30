import fcntl
import os
import struct
from serial import Serial, PARITY_NONE

from umodbus.client.serial import rtu

import params


def get_serial_port():
    port = Serial(port=os.path.expanduser('~') + '/dev/vcommaster', baudrate=9600, parity=PARITY_NONE,
                  stopbits=1, bytesize=8, timeout=1)

    fh = port.fileno()

    # A struct with configuration for serial port.
    # serial_rs485 = struct.pack('hhhhhhhh', 1, 0, 0, 0, 0, 0, 0, 0)
    # fcntl.ioctl(fh, 0x542F, serial_rs485)

    return port


serial_port = get_serial_port()

# Returns a message or Application Data Unit (ADU) specific for doing
# Modbus RTU.
# message = rtu.write_multiple_coils(slave_id=1, starting_address=1, values=[1, 0, 1, 1])
# message = rtu.write_single_coil(1,5,1)
# message=rtu.read_discrete_inputs(1,1,1)
message = rtu.read_holding_registers(slave_id=1,starting_address=0,quantity=10)
# Response depends on Modbus function code. This particular returns the
# amount of coils written, in this case it is.
# while(1):
response = rtu.send_message(message, serial_port)

print(response)

serial_port.close()
