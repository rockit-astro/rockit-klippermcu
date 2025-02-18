#
# This file is part of the Robotic Observatory Control Kit (rockit)
#
# rockit is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# rockit is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with rockit.  If not, see <http://www.gnu.org/licenses/>.

import math
import threading

# Helper functions copied from Klipper tmc_uart.py
# Copyright (C) 2018-2021  Kevin O'Connor <kevin@koconnor.net>

def _calc_crc8(data):
    # Generate a CRC8-ATM value for a bytearray
    crc = 0
    for b in data:
        for _ in range(8):
            if (crc >> 7) ^ (b & 0x01):
                crc = (crc << 1) ^ 0x07
            else:
                crc = (crc << 1)
            crc &= 0xff
            b >>= 1
    return crc


def _add_serial_bits(data):
    # Add serial start and stop bits to a message in a bytearray
    out = 0
    pos = 0
    for d in data:
        b = (d << 1) | 0x200
        out |= (b << pos)
        pos += 10
    res = bytearray()
    for i in range((pos + 7) // 8):
        res.append((out >> (i * 8)) & 0xff)
    return res


def _encode_read(sync, addr, reg):
    # Generate a uart read register message
    msg = bytearray([sync, addr, reg])
    msg.append(_calc_crc8(msg))
    return _add_serial_bits(msg)


def _encode_write(sync, addr, reg, val):
    # Generate a uart write register message
    msg = bytearray([sync, addr, reg, (val >> 24) & 0xff,
                     (val >> 16) & 0xff, (val >> 8) & 0xff, val & 0xff])
    msg.append(_calc_crc8(msg))
    return _add_serial_bits(msg)


def _decode_read(reg, data):
    # Extract a uart read response message
    if len(data) != 10:
        return None

    # Convert data into a long integer for easy manipulation
    mval = pos = 0
    for d in bytearray(data):
        mval |= d << pos
        pos += 8

    # Extract register value
    val = ((((mval >> 31) & 0xff) << 24) | (((mval >> 41) & 0xff) << 16)
           | (((mval >> 51) & 0xff) << 8) | ((mval >> 61) & 0xff))

    # Verify start/stop bits and crc
    encoded_data = _encode_write(0x05, 0xff, reg, val)
    if data != encoded_data:
        return None
    return val

class TMCUART:
    def __init__(self, mcu, uart_type, tx_pin, rx_pin):
        self._mcu = mcu
        self._type = uart_type

        self._oid = mcu.reserve_oid()
        self._lock = threading.RLock()

        if uart_type == 'tmc2209':
            self._register_map = {
                'GCONF': 0x00,
                'IFCNT': 0x02,
                'SLAVECONF': 0x03,
                'IHOLD_IRUN': 0x10,
                'TPOWERDOWN': 0x11,
                'VACTUAL': 0x22,
                'MSCNT': 0x6A,
                'CHOPCONF': 0x6C,
                'PWMCONF': 0x70
            }
        else:
            raise NotImplementedError(type)

        def configure():
            self._mcu.send_command('config_tmcuart', oid=self._oid, rx_pin=rx_pin,
                                   pull_up=0, tx_pin=tx_pin, bit_time=300)
        mcu.register_init_callback(configure)

    def configure_stepper(self, address, microsteps, run_current):
        if self._type == 'tmc2209':
            # enable pdn_disable, mstep_reg_select
            self.write_register(address, 'GCONF', 0xC0)

            # senddelay=2
            self.write_register(address, 'SLAVECONF', 0x200)

            # enable dedge, intpol, vsense, tbl=2, hstrt=5, toff=3
            chopconf = 0x10030053
            if self._mcu.msgparser.get_constant_int('STEPPER_BOTH_EDGE', 0):
                chopconf |= 0x20000000

            # set microsteps
            chopconf |= int(8 - math.log2(microsteps)) << 24
            self.write_register(address, 'CHOPCONF', chopconf)

            # tpowerdown=20
            self.write_register(address, 'TPOWERDOWN', 0x14)

            # enable pasive breaking (HS drivers), pwm_autoscale, pwm_autograd
            # pwm_ofs=36 pwm_grad=14 pwm_freq=1 pwm_reg=8 pwm_lim=12
            self.write_register(address, 'PWMCONF', 0xC83D0E24)

            # iholddelay=8, ihold=0
            iholdirun = 0x80000

            # from TMCCurrentHelper, assuming a 0.11 ohm sense resistor
            irun = max(0, min(31, int(32. * 0.130 * run_current * math.sqrt(2.) / 0.18 + .5) - 1))
            iholdirun |= irun << 8
            self.write_register(address, 'IHOLD_IRUN', iholdirun)

    def write_register(self, addr, register_name, value):
        reg = self._register_map[register_name]
        data = _encode_write(0xf5, addr, reg | 0x80, value)

        with self._lock:
            ifcnt = self.read_register(addr, 'IFCNT')
            for _ in range(5):
                self._mcu.send_query('tmcuart_send', oid=self._oid, write=data, read=0)
                after = self.read_register(addr, 'IFCNT')
                if after == (ifcnt + 1) & 0xff:
                    return

        raise Exception(f'Unable to write tmc uart register {reg}')

    def read_register(self, addr, register_name):
        with self._lock:
            reg = self._register_map[register_name]
            data = _encode_read(0xf5, addr, reg)
            read = self._mcu.send_query('tmcuart_send', oid=self._oid, write=data, read=10)
            return _decode_read(reg, read)
