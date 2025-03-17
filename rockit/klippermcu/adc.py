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

SAMPLE_TIME = 0.001
SAMPLE_COUNT = 8
RANGE_CHECK_COUNT = 4

class ADC:
    def __init__(self, mcu, pin, sample_cadence):
        self._mcu = mcu
        self._oid = mcu.reserve_oid()
        self._adc_max = 0
        self.last_measurement = None

        def configure():
            self.last_measurement = None
            self._mcu.send_command('config_analog_in', oid=self._oid, pin=pin)
            self._adc_max = SAMPLE_COUNT * self._mcu.msgparser.get_constant_float('ADC_MAX')

        def update(params):
            self.last_measurement = params['value'] / self._adc_max
            self._mcu.register_response_callback('analog_in_state', self._oid, update)

        def start():
            sample_ticks = int(SAMPLE_TIME * self._mcu.mcu_freq)
            rest_ticks = int(sample_cadence * self._mcu.mcu_freq)

            self._mcu.register_response_callback('analog_in_state', self._oid, update)
            clock = self._mcu.host_clock_to_mcu_clock(self._mcu.host_clock() + sample_cadence)
            self._mcu.send_command('query_analog_in', oid=self._oid, clock=clock,
                                   sample_ticks=sample_ticks, sample_count=SAMPLE_COUNT, rest_ticks=rest_ticks,
                                   min_value=0, max_value=int(self._adc_max),
                                   range_check_count=RANGE_CHECK_COUNT)

        mcu.register_init_callback(configure)
        mcu.register_post_init_callback(start)
