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

"""Logic to control a stepper motor using the Klipper MCU API"""

import threading
import numpy as np

HOMING_START_DELAY = 0.1
ENDSTOP_SAMPLE_TIME = .000015
ENDSTOP_SAMPLE_COUNT = 4

TRIGGER_ACTIVE = 0
TRIGGER_AT_LIMIT = 1
TRIGGER_MANUAL = 2
TRIGGER_TIMEOUT = 4

class StepperStatus:
    NotHomed, Homing, Idle, Moving, Tracking = range(5)

    _labels = {
        0: 'NOT HOMED',
        1: 'HOMING',
        2: 'READY',
        3: 'MOVING',
        4: 'TRACKING'
    }

    _colors = {
        0: 'red',
        1: 'yellow',
        2: 'default',
        3: 'yellow',
        4: 'green'
    }

    @classmethod
    def label(cls, status, formatting=False):
        """Returns a human readable string describing a status
           Set formatting=true to enable terminal formatting characters
        """
        if formatting:
            if status in cls._labels and status in cls._colors:
                return f'[b][{cls._colors[status]}]{cls._labels[status]}[/{cls._colors[status]}][/b]'
            return '[b][red]UNKNOWN[/red][/b]'

        if status in cls._labels:
            return cls._labels[status]
        return 'UNKNOWN'

def parse_pin(value):
    pullup = invert = 0
    if value[0] in ['^', '~']:
        pullup = -1 if value[0] == '~' else 1
        value = value[1:]

    if value[0] == '!':
        invert = 1
        value = value[1:]
    return pullup, invert, value


class Stepper:
    def __init__(self, config_json, mcu, uarts):
        self._status = StepperStatus.NotHomed
        self._config = config_json
        self._steps_per_distance = self._config['rotation_microsteps'] / self._config['rotation_distance']
        self._mcu = mcu
        self._mcu_freq = 0
        self._stepper_oid = mcu.reserve_oid()
        self._enable_oid = mcu.reserve_oid()
        self._trigger_oid = mcu.reserve_oid()
        self._endstop_oid = mcu.reserve_oid() if self.has_endstop else -1
        self._pos_steps_at_origin = 0
        self._pos_steps = 0
        self._stopped = False

        self._uart = None
        if 'tmc_uart' in self._config:
            self._uart = uarts[self._config['tmc_uart']['uart']]

        mcu.register_init_callback(self._configure)
        mcu.register_post_init_callback(self._post_configure)

    @property
    def status(self):
        return self._status

    @property
    def position(self):
        return (self._pos_steps - self._pos_steps_at_origin) / self._steps_per_distance

    @property
    def has_endstop(self):
        return 'endstop_pin' in self._config

    def _configure(self):
        _, _, step_pin = parse_pin(self._config['step_pin'])
        _, _, dir_pin = parse_pin(self._config['dir_pin'])
        _, enable_invert, enable_pin = parse_pin(self._config['enable_pin'])

        step_pulse_ticks = int(.000002 * self._mcu_freq)
        invert_step = 0
        sbe = int(self._mcu.msgparser.get_constant_int('STEPPER_BOTH_EDGE', '0'))
        if self._uart is not None and sbe:
            invert_step = -1
            step_pulse_ticks = 0

        self._mcu.send_command('config_stepper', oid=self._stepper_oid,
                       step_pin=step_pin, dir_pin=dir_pin,
                       invert_step=invert_step, step_pulse_ticks=step_pulse_ticks)

        # Set idle powerdown via the uart if available
        # Otherwise manually enable stepper only while moving
        disable_on_idle = int(self._uart is None)
        self._mcu.send_command('config_digital_out', oid=self._enable_oid,
                       pin=enable_pin, value=int(not enable_invert ^ disable_on_idle),
                       default_value=enable_invert, max_duration=0)
        self._mcu.send_command('config_trsync', oid=self._trigger_oid)

        if self.has_endstop:
            endstop_pullup, _, endstop_pin = parse_pin(self._config['endstop_pin'])
            self._mcu.send_command('config_endstop', oid=self._endstop_oid,
                                   pin=endstop_pin, pull_up=endstop_pullup)

    def _post_configure(self):
        self._status = StepperStatus.NotHomed if self.has_endstop else StepperStatus.Idle
        self._mcu_freq = self._mcu.msgparser.get_constant_float('CLOCK_FREQ')
        if self._uart is not None:
            c = self._config['tmc_uart']
            self._uart.configure_stepper(c['address'], c['microsteps'], c['run_current'])

    def _build_trapezoidal_move(self, start_time, start_pos, start_vel, end_pos, end_vel, speed=None):
        distance = abs(end_pos - start_pos)
        if distance == 0:
            return lambda t: start_pos, 0, 0, 0, 0

        if speed is None:
            speed = self._config['speed']

        sign = 1 if end_pos >= start_pos else -1
        start_vel *= sign
        end_vel *= sign

        coast_speed = min(speed, np.sqrt(0.5 * distance * self._config['acceleration']))
        accel_time = abs(coast_speed - start_vel) / self._config['acceleration']
        accel_distance = start_vel * accel_time + 0.5 * self._config['acceleration'] * accel_time ** 2
        decel_time = abs(end_vel - coast_speed) / self._config['acceleration']
        decel_distance = end_vel * decel_time + 0.5 * self._config['acceleration'] * decel_time ** 2

        coast_distance = distance - accel_distance - decel_distance
        coast_time = coast_distance / coast_speed
        total_time = accel_time + coast_time + decel_time

        def inner(t):
            if np.isscalar(t):
                return inner(np.array(t)).item()

            dt = t - start_time
            accel_filt = (dt > 0) & (dt <= accel_time)
            coast_filt = (dt > accel_time) & (dt <= accel_time + coast_time)
            decel_filt = (dt > accel_time + coast_time) & (dt <= total_time)
            after_filt = dt > total_time

            tmdt = total_time - dt
            offset = np.zeros_like(t)
            offset[accel_filt] = start_vel * dt[accel_filt] + 0.5 * self._config['acceleration'] * dt[accel_filt] ** 2
            offset[coast_filt] = (dt[coast_filt] - accel_time) * coast_speed + accel_distance
            offset[decel_filt] = (distance - end_vel * tmdt[decel_filt]
                                  - 0.5 * self._config['acceleration'] * tmdt[decel_filt] ** 2)
            offset[after_filt] = distance
            return start_pos + sign * offset

        return inner, total_time, accel_time, coast_time, decel_time

    def _move(self, distance, speed, check_endstop=False, check_limits=True):
        if check_limits:
            self._pos_steps = self._mcu.send_query('stepper_get_position', oid=self._stepper_oid)
            distance = min(max(self.position + distance, self._config['position_min']),
                           self._config['position_max']) - self.position
            if distance == 0:
                return TRIGGER_TIMEOUT

        start_time = self._mcu.host_clock() + HOMING_START_DELAY
        start_clock = self._mcu.host_clock_to_mcu_clock(start_time)
        step_dir = 1 if distance * speed > 0 else 0

        _, enable_invert, _ = parse_pin(self._config['enable_pin'])
        if self._uart is None:
            self._mcu.send_command('queue_digital_out', oid=self._enable_oid, clock=start_clock, on_ticks=enable_invert)

        self._mcu.send_command('set_next_step_dir', oid=self._stepper_oid, dir=step_dir)
        self._mcu.send_command('reset_step_clock', oid=self._stepper_oid, clock=start_clock)

        move_fn, total_time, accel_time, coast_time, _ = self._build_trapezoidal_move(
            0, 0, 0, abs(distance), 0, speed)

        # Queue the coast phase as a single segment to avoid overloading the move queue
        step = self._config['tracking_cadence']
        times = np.concatenate([
            np.arange(0, accel_time, step),
            np.arange(accel_time + coast_time, total_time + step, step)
        ])

        clocks = (times * self._mcu_freq).astype(int)
        steps = (move_fn(times) * self._steps_per_distance + 0.5).astype(int)
        for i in range(len(times) - 1):
            count = steps[i+1] - steps[i]
            if count == 0:
                continue

            interval = int((clocks[i + 1] - clocks[i]) / count)
            while count > 0:
                # queue_step is limited to 16 bit counts
                c = min(count, 65535)
                self._mcu.send_command('queue_step', oid=self._stepper_oid, interval=interval, count=c, add=0)
                count -= c

        trigger_status = TRIGGER_ACTIVE
        def on_trigger(params):
            nonlocal trigger_status
            trigger_status = params['trigger_reason']

        self._mcu.register_response_callback('trsync_state', self._trigger_oid, on_trigger)
        self._mcu.send_command('trsync_start', oid=self._trigger_oid,
                               report_clock=0, report_ticks=0,
                               expire_reason=TRIGGER_TIMEOUT)
        self._mcu.send_command('trsync_set_timeout', oid=self._trigger_oid, clock=start_clock + clocks[-1])
        self._mcu.send_command('stepper_stop_on_trigger', oid=self._stepper_oid, trsync_oid=self._trigger_oid)

        if check_endstop:
            sample_clocks = int(self._mcu_freq * ENDSTOP_SAMPLE_TIME)
            rest_clocks = int(self._mcu_freq / (5 * speed * self._steps_per_distance) + 0.5)

            _, endstop_inverted, _ = parse_pin(self._config['endstop_pin'])
            endstop_value = 0 if endstop_inverted else 1
            self._mcu.send_command('endstop_home', oid=self._endstop_oid, clock=start_clock, sample_ticks=sample_clocks,
                                   sample_count=ENDSTOP_SAMPLE_COUNT, rest_ticks=rest_clocks, pin_value=endstop_value,
                                   trsync_oid=self._trigger_oid, trigger_reason=TRIGGER_AT_LIMIT)

        while trigger_status == TRIGGER_ACTIVE:
            self._pos_steps = self._mcu.send_query('stepper_get_position', oid=self._stepper_oid)

        # Ensure position is synchronised
        self._pos_steps = self._mcu.send_query('stepper_get_position', oid=self._stepper_oid)

        if self._uart is None:
            disable_clock = self._mcu.host_clock_to_mcu_clock(self._mcu.host_clock() + 0.1)
            self._mcu.send_command('queue_digital_out', oid=self._enable_oid,
                                   clock=disable_clock, on_ticks=not enable_invert)
        return trigger_status

    def home(self, blocking=True):
        if not self.has_endstop:
            raise Exception('Cannot home stepper without endstop')

        self._stopped = False
        if self._status not in [StepperStatus.Idle, StepperStatus.NotHomed]:
            raise Exception('Cannot home while moving')

        status = self._status
        self._status = StepperStatus.Homing

        def inner():
            # Move quickly to endstop
            distance_rough = self._config['position_min'] - self._config['position_max']
            self._move(distance_rough, self._config['speed'], check_endstop=True, check_limits=False)

            if self._stopped:
                self._status = status
                return

            # back off a bit
            self._move(self._config['homing_backoff'], self._config['speed'] / 2, check_limits=False)

            if self._stopped:
                self._status = status
                return

            trigger_status = self._move(-2 * self._config['homing_backoff'], self._config['speed'] / 10,
                                        check_endstop=True, check_limits=False)
            if trigger_status == TRIGGER_AT_LIMIT:
                self._pos_steps_at_origin = self._pos_steps
                self._status = StepperStatus.Idle
            else:
                self._status = status

        if blocking:
            inner()
        else:
            threading.Thread(target=inner).start()

    def move(self, distance, blocking=True):
        if self._status != StepperStatus.Idle:
            raise Exception('already moving')

        self._stopped = False
        self._status = StepperStatus.Moving
        def inner():
            self._move(distance, self._config['speed'])
            self._status = StepperStatus.Idle

        if blocking:
            inner()
        else:
            threading.Thread(target=inner).start()

    def stop(self):
        self._stopped = True
        self._mcu.send_command('trsync_trigger', oid=self._trigger_oid, reason=TRIGGER_MANUAL)

