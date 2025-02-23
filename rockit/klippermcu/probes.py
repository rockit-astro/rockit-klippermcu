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
from .adc import ADC

class ThermistorProbe:
    def __init__(self, mcu, coeffs, pin, sample_cadence):
        self._adc = ADC(mcu, pin, sample_cadence)
        self._coeffs = coeffs

    @property
    def temperature(self):
        v = self._adc.last_measurement
        if v is None:
            return None

        adc = max(.00001, min(.99999, v))
        r = 4700 * adc / (1.0 - adc)
        ln_r = math.log(r)
        inv_t = self._coeffs[0] + self._coeffs[1] * ln_r + self._coeffs[2] * ln_r**3
        return 1.0 / inv_t - 273.15


class RP2040Probe:
    def __init__(self, mcu, sample_cadence):
        self._adc = ADC(mcu, 'ADC_TEMPERATURE', sample_cadence)

    @property
    def temperature(self):
        v = self._adc.last_measurement
        if v is None:
            return None
        return 27 - (3.3 * v - 0.706) / 0.001721
