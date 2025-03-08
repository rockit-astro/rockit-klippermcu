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

class OutputPin:
    def __init__(self, mcu, pin):
        self._mcu = mcu
        self._oid = mcu.reserve_oid()
        self._pin_invert, self._pin = parse_pin(pin, parse_invert=True)
        self.status = self._pin_invert

        mcu.register_init_callback(self._configure)

    def _configure(self):
        self._mcu.send_command('config_digital_out', oid=self._oid,
                       pin=self._pin, value=int(self._pin_invert),
                       default_value=int(self._pin_invert), max_duration=0)

    def set(self, value):
        time = self._mcu.host_clock() + OUTPUT_DELAY
        clock = self._mcu.host_clock_to_mcu_clock(time)
        self._mcu.send_command('queue_digital_out', oid=self._oid, clock=clock, on_ticks=int(self._pin_invert ^ value))
