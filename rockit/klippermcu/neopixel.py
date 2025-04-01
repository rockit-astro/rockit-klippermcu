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

from .stepper import parse_pin

OUTPUT_DELAY = 0.1
BIT_MAX_TIME = .000004
RESET_MIN_TIME = .000050

class NeoPixel:
    def __init__(self, mcu, pin, count):
        self.color = '000000'
        self._mcu = mcu
        self._oid = mcu.reserve_oid()
        self._pin = pin
        self._count = count

        def configure():
            bit_max_ticks = int(BIT_MAX_TIME * self._mcu.mcu_freq)
            reset_min_ticks = int(RESET_MIN_TIME * self._mcu.mcu_freq)
            self._mcu.send_command('config_neopixel', oid=self._oid,
                                   pin=self._pin, data_size=3 * self._count,
                                   bit_max_ticks=bit_max_ticks,
                                   reset_min_ticks=reset_min_ticks)

        def start():
            self.set_color('000000')

        mcu.register_init_callback(configure)
        mcu.register_post_init_callback(start)

    def set_color(self, value):
        # Convert RGB string to GRB bytes
        color = bytearray([int(value[2:4], 16), int(value[0:2], 16), int(value[4:6], 16)])
        self.color = value

        # Batch updates but need to stay below 64 byte message limit
        for i in range((self._count + 15) // 16):
            count = min(16, self._count - 16 * i)
            self._mcu.send_command('neopixel_update', oid=self._oid, pos=48 * i, data=bytearray(color * count))
        return self._mcu.send_query('neopixel_send', oid=self._oid)
