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

from ctypes import c_int16
import threading
import time

class DS2484:
    def __init__(self, mcu, i2c_bus):
        self._mcu = mcu
        self._oid = mcu.reserve_oid()
        self._lock = threading.Lock()

        def configure():
            self._mcu.send_command('config_i2c', oid=self._oid)
            self._mcu.send_command('i2c_set_bus', oid=self._oid, i2c_bus=i2c_bus, rate=400000, address=0x18)

        mcu.register_init_callback(configure)

    def _i2c_write(self, data):
        if not isinstance(data, list):
            data = [data]
        self._mcu.send_command('i2c_write', oid=self._oid, data=bytearray(data))

    def _i2c_read(self):
        # Read pointer is assumed to always be at the status register
        # except when _onewire_read explicitly changes it to read data (and then changes it back)
        return self._mcu.send_query('i2c_read', oid=self._oid, reg=[], read_len=1)[0]

    def _onewire_wait_while_busy(self):
        start = self._mcu.host_clock()
        while (self._mcu.host_clock() - start) < 1:
            status = self._i2c_read()
            if not status & 0x01:
                return
            time.sleep(0.001)
        raise TimeoutError('timed out waiting for 1WB to clear')

    def _onewire_reset(self) -> bool:
        self._onewire_wait_while_busy()
        self._i2c_write(0xB4)
        status = self._i2c_read()
        return not (status & 0x04) and (status & 0x02)

    def _onewire_read(self):
        self._onewire_wait_while_busy()
        self._i2c_write(0x96)
        self._onewire_wait_while_busy()
        self._i2c_write([0xE1, 0xE1])
        data = self._i2c_read()
        self._i2c_write([0xE1, 0xF0])
        return data

    def _onewire_write(self, data):
        self._onewire_wait_while_busy()
        self._i2c_write([0xA5, data])

    def _onewire_select(self, address):
        self._onewire_reset()
        if address is not None:
            self._onewire_write(0x55)
            for i in range(8):
                b = int(address[2*i:2*(i+1)], 16)
                self._onewire_write(b)
        else:
            self._onewire_write(0xCC)

    def ds18b20_read(self, address=None):
        with self._lock:
            # Start a temperature conversion
            self._onewire_select(address)
            self._onewire_write(0x44)
            time.sleep(0.8)

            # Read scratchpad
            data = bytearray(9)
            self._onewire_select(address)
            self._onewire_write(0xBE)
            for i in range(9):
                data[i] = self._onewire_read()

            # Convert to deg C
            # TODO: check CRC and retry on error(?)
            return c_int16((data[1] << 8) | data[0]).value / 16.0
