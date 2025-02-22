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

import collections
import sys
import threading
import traceback
import numpy as np

START_DELAY = 0.1
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

def parse_pin(value, parse_pullup=False, parse_invert=False):
    pullup = invert = 0
    if value is not None and len(value) > 0:
        if value[0] in ['^', '~']:
            pullup = -1 if value[0] == '~' else 1
            value = value[1:]

        if value[0] == '!':
            invert = 1
            value = value[1:]

    ret = []
    if parse_pullup:
        ret.append(pullup)
    if parse_invert:
        ret.append(invert)
    ret.append(value)

    return ret[0] if len(ret) == 1 else ret


class Stepper:
    def __init__(self, mcu, uart, rotation_microsteps, rotation_distance,
                 step_pin, dir_pin, enable_pin, endstop_pin,
                 uart_address, uart_microsteps, uart_run_current,
                 speed, acceleration, tracking_cadence, tracking_commit_buffer,
                 position_min, position_max, homing_backoff):

        self._mcu = mcu
        self._uart = uart
        self._uart_address = uart_address
        self._uart_microsteps = uart_microsteps
        self._uart_run_current = uart_run_current

        self._step_pin = parse_pin(step_pin)
        self._dir_pin = parse_pin(dir_pin)
        self._enable_pin_invert, self._enable_pin = parse_pin(enable_pin, parse_invert=True)
        self._endstop_pin_pullup, self._endstop_pin_invert, self._endstop_pin = parse_pin(
            endstop_pin, parse_pullup=True, parse_invert=True)

        self._stepper_oid = mcu.reserve_oid()
        self._enable_oid = mcu.reserve_oid()
        self._trigger_oid = mcu.reserve_oid()
        self._endstop_oid = mcu.reserve_oid() if endstop_pin is not None else -1

        self._speed = speed
        self._acceleration = acceleration
        self._steps_per_distance = rotation_microsteps / rotation_distance
        self._tracking_cadence = tracking_cadence
        self._tracking_commit_buffer = tracking_commit_buffer
        self._position_min = position_min
        self._position_max = position_max
        self._homing_backoff = homing_backoff

        self._status = StepperStatus.NotHomed
        self._pos_steps_at_origin = 0
        self._pos_steps = 0
        self._stopped = False

        self._track_lock = threading.Lock()
        self._track_thread = None
        self._track_status = collections.deque()

        mcu.register_init_callback(self._configure)
        mcu.register_post_init_callback(self._post_configure)

    @property
    def status(self):
        if self._track_thread is not None and self._track_thread.is_alive() and len(self._track_status):
            return self._track_status[0][1]

        return self._status

    @property
    def position(self):
        return (self._pos_steps - self._pos_steps_at_origin) / self._steps_per_distance

    @property
    def has_endstop(self):
        return self._endstop_pin is not None

    def _configure(self):
        step_pulse_ticks = int(.000002 * self._mcu.mcu_freq)
        invert_step = 0
        sbe = int(self._mcu.msgparser.get_constant_int('STEPPER_BOTH_EDGE', '0'))
        if self._uart is not None and sbe:
            invert_step = -1
            step_pulse_ticks = 0

        self._mcu.send_command('config_stepper', oid=self._stepper_oid,
                               step_pin=self._step_pin, dir_pin=self._dir_pin,
                               invert_step=invert_step, step_pulse_ticks=step_pulse_ticks)

        # Set idle powerdown via the uart if available
        # Otherwise manually enable stepper only while moving
        disable_on_idle = int(self._uart is None)
        self._mcu.send_command('config_digital_out', oid=self._enable_oid,
                       pin=self._enable_pin, value=int(not self._enable_pin_invert ^ disable_on_idle),
                       default_value=self._enable_pin_invert, max_duration=0)
        self._mcu.send_command('config_trsync', oid=self._trigger_oid)

        if self.has_endstop:
            self._mcu.send_command('config_endstop', oid=self._endstop_oid,
                                   pin=self._endstop_pin, pull_up=self._endstop_pin_pullup)

    def _post_configure(self):
        self._status = StepperStatus.NotHomed if self.has_endstop else StepperStatus.Idle
        if self._uart is not None:
            self._uart.configure_stepper(self._uart_address, self._uart_microsteps, self._uart_run_current)

    def _build_trapezoidal_move(self, start_time, start_pos, start_vel, end_pos, end_vel, speed=None):
        distance = abs(end_pos - start_pos)
        if distance == 0:
            return lambda t: start_pos, 0, 0, 0, 0

        if speed is None:
            speed = self._speed

        sign = 1 if end_pos >= start_pos else -1
        start_vel *= sign
        end_vel *= sign

        coast_speed = min(speed, np.sqrt(0.5 * distance * self._acceleration))
        accel_time = abs(coast_speed - start_vel) / self._acceleration
        accel_distance = start_vel * accel_time + 0.5 * self._acceleration * accel_time ** 2
        decel_time = abs(end_vel - coast_speed) / self._acceleration
        decel_distance = end_vel * decel_time + 0.5 * self._acceleration * decel_time ** 2

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
            offset[accel_filt] = start_vel * dt[accel_filt] + 0.5 * self._acceleration * dt[accel_filt] ** 2
            offset[coast_filt] = (dt[coast_filt] - accel_time) * coast_speed + accel_distance
            offset[decel_filt] = (distance - end_vel * tmdt[decel_filt]
                                  - 0.5 * self._acceleration * tmdt[decel_filt] ** 2)
            offset[after_filt] = distance
            return start_pos + sign * offset

        return inner, total_time, accel_time, coast_time, decel_time

    def _update_track_thread(self, track_func):
        start_time = self._mcu.host_clock() + START_DELAY
        start_clock = self._mcu.host_clock_to_mcu_clock(start_time)

        if self._uart is None:
            self._mcu.send_command('queue_digital_out', oid=self._enable_oid, clock=start_clock,
                                   on_ticks=self._enable_pin_invert)
        self._mcu.send_command('reset_step_clock', oid=self._stepper_oid, clock=start_clock)
        self._mcu.send_command('trsync_start', oid=self._trigger_oid,
                               report_clock=0, report_ticks=0,
                               expire_reason=TRIGGER_TIMEOUT)
        self._mcu.send_command('stepper_stop_on_trigger', oid=self._stepper_oid, trsync_oid=self._trigger_oid)

        wait_condition = threading.Condition()
        self._status = StepperStatus.Tracking

        # Calculated track state
        track_segments = collections.deque()
        start_pos = self.position
        start_vel = 0

        # Committed track state
        last_committed = (start_clock, self._pos_steps, None)

        try:
            while not self._stopped:
                loop_start_time = self._mcu.host_clock()

                # Expire stale tracking status
                while len(self._track_status) and self._track_status[0][0] < loop_start_time:
                    self._track_status.popleft()

                # Calculate more track if needed
                with self._track_lock:
                    next_committed = self._mcu.host_clock() + self._tracking_commit_buffer
                    while start_time < next_committed:
                        end_time = start_time + self._tracking_cadence
                        end_pos = np.clip(track_func(end_time), self._position_min, self._position_max)

                        tracking_vel = (end_pos - start_pos) / self._tracking_cadence
                        if abs(tracking_vel - start_vel) < self._acceleration * self._tracking_cadence:
                            # We are tracking the target
                            track_segments.append((start_time, end_time, end_pos, StepperStatus.Tracking))
                            start_time = end_time
                            start_pos = end_pos
                            start_vel = tracking_vel
                            continue

                        converge_pos, converge_vel = end_pos, start_vel
                        for _ in range(25):
                            move_fn, move_time, *_ = self._build_trapezoidal_move(
                                start_time, start_pos, start_vel, converge_pos, converge_vel)

                            move_steps = int(np.ceil(move_time / self._tracking_cadence))
                            while True:
                                after_move_start_pos = np.clip(
                                    track_func(start_time + move_steps * self._tracking_cadence),
                                    self._position_min, self._position_max)
                                after_move_end_pos = np.clip(
                                    track_func(start_time + (move_steps + 1) * self._tracking_cadence),
                                    self._position_min, self._position_max)
                                after_move_vel = (after_move_end_pos - after_move_start_pos) / self._tracking_cadence
                                if abs(after_move_vel) < self._speed:
                                    break

                                move_steps += 1

                            if abs(after_move_start_pos - converge_pos) < 1 / self._steps_per_distance:
                                break

                            converge_pos, converge_vel = after_move_start_pos, after_move_vel

                        for i in range(move_steps):
                            move_start_time = start_time + i * self._tracking_cadence
                            move_end_time = move_start_time + self._tracking_cadence
                            move_start_pos = move_fn(move_start_time)
                            move_end_pos = move_fn(move_end_time)
                            track_segments.append((move_start_time, move_end_time, move_end_pos, StepperStatus.Moving))

                        start_time = move_end_time
                        start_pos = move_end_pos
                        start_vel = (move_end_pos - move_start_pos) / self._tracking_cadence

                # Commit track segments to the MCU
                while len(track_segments) and track_segments[0][0] < next_committed:
                    last_clock, last_steps, last_dir = last_committed
                    seg_start_time, seg_end_time, seg_end_pos, seg_status = track_segments.popleft()
                    self._track_status.append((seg_end_time, seg_status))

                    seg_end_steps = int(seg_end_pos * self._steps_per_distance + self._pos_steps_at_origin + 0.5)
                    count = abs(seg_end_steps - last_steps)
                    if count == 0:
                        continue

                    start_clock = self._mcu.host_clock_to_mcu_clock(seg_start_time)
                    end_clock = self._mcu.host_clock_to_mcu_clock(seg_end_time)
                    interval = 0

                    step_dir = 1 if seg_end_steps >= last_steps else 0
                    step_sign = 1 if seg_end_steps >= last_steps else -1
                    if step_dir != last_dir:
                        self._mcu.send_command('set_next_step_dir', oid=self._stepper_oid, dir=step_dir)

                    # Reset step clock if we have been paused, allowing
                    # for small rounding errors from the last segment
                    if start_clock - last_clock > (end_clock - start_clock) / count:
                        if count == 1:
                            self._mcu.send_command('queue_step', oid=self._stepper_oid,
                                                   interval=end_clock - last_clock, count=1, add=0)
                            last_clock = end_clock
                        else:
                            self._mcu.send_command('queue_step', oid=self._stepper_oid,
                                                   interval=start_clock - last_clock, count=1, add=0)
                            last_clock = start_clock

                        last_steps += step_sign
                        count -= 1

                    if count > 0:
                        interval = int((end_clock - last_clock) / count)
                        self._mcu.send_command('queue_step', oid=self._stepper_oid,
                                               interval=interval, count=count, add=0)

                    last_committed = (last_clock + count * interval, last_steps + step_sign * count, step_dir)

                self._pos_steps = self._mcu.send_query('stepper_get_position', oid=self._stepper_oid)
                with wait_condition:
                    delay = 0.25 * self._tracking_cadence - (self._mcu.host_clock() - loop_start_time)
                    if delay > 0:
                        wait_condition.wait(delay)
        except Exception:
            print('exception in track thread')
            traceback.print_exc(file=sys.stdout)

        self._pos_steps = self._mcu.send_query('stepper_get_position', oid=self._stepper_oid)
        self._status = StepperStatus.Idle
        if self._uart is None:
            disable_clock = self._mcu.host_clock_to_mcu_clock(self._mcu.host_clock() + 0.1)
            self._mcu.send_command('queue_digital_out', oid=self._enable_oid, clock=disable_clock,
                                   on_ticks=not self._enable_pin_invert)

    def _move(self, distance, speed, check_endstop=False, check_limits=True):
        if self._track_thread is not None:
            raise Exception('Cannot issue move commands while tracking is active')

        if check_limits:
            self._pos_steps = self._mcu.send_query('stepper_get_position', oid=self._stepper_oid)
            distance = np.clip(self.position + distance, self._position_min, self._position_max) - self.position
            if distance == 0:
                return TRIGGER_TIMEOUT

        start_time = self._mcu.host_clock() + START_DELAY
        start_clock = self._mcu.host_clock_to_mcu_clock(start_time)
        step_dir = 1 if distance * speed > 0 else 0

        if self._uart is None:
            self._mcu.send_command('queue_digital_out', oid=self._enable_oid, clock=start_clock,
                                   on_ticks=self._enable_pin_invert)

        self._mcu.send_command('set_next_step_dir', oid=self._stepper_oid, dir=step_dir)
        self._mcu.send_command('reset_step_clock', oid=self._stepper_oid, clock=start_clock)

        move_fn, total_time, accel_time, coast_time, _ = self._build_trapezoidal_move(
            0, 0, 0, abs(distance), 0, speed)

        # Queue the coast phase as a single segment to avoid overloading the move queue
        times = np.concatenate([
            np.arange(0, accel_time, self._tracking_cadence),
            np.arange(accel_time + coast_time, total_time + self._tracking_cadence, self._tracking_cadence)
        ])

        clocks = (times * self._mcu.mcu_freq).astype(int)
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
            sample_clocks = int(self._mcu.mcu_freq * ENDSTOP_SAMPLE_TIME)
            rest_clocks = int(self._mcu.mcu_freq / (5 * speed * self._steps_per_distance) + 0.5)

            endstop_value = 0 if self._endstop_pin_invert else 1
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
                                   clock=disable_clock, on_ticks=not self._enable_pin_invert)
        return trigger_status

    def sync(self, position):
        if self.has_endstop:
            raise Exception('Cannot sync stepper that has an endstop')

        if self._status not in [StepperStatus.Idle, StepperStatus.NotHomed]:
            raise Exception('Cannot sync while moving')

        # Ensure position is synchronised
        self._pos_steps = self._mcu.send_query('stepper_get_position', oid=self._stepper_oid)
        self._pos_steps_at_origin = self._pos_steps - position * self._steps_per_distance

    def home(self, blocking=True):
        if not self.has_endstop:
            raise Exception('Cannot home stepper that does not have an endstop')

        self._stopped = False
        if self._status not in [StepperStatus.Idle, StepperStatus.NotHomed]:
            raise Exception('Cannot home while moving')

        status = self._status
        self._status = StepperStatus.Homing

        def inner():
            # Move quickly to endstop
            distance_rough = self._position_min - self._position_max
            trigger = self._move(distance_rough, self._speed, check_endstop=True, check_limits=False)

            if trigger == TRIGGER_MANUAL:
                self._status = status
                return

            if trigger == TRIGGER_TIMEOUT:
                self._status = StepperStatus.NotHomed
                return

            # back off a bit
            trigger = self._move(self._homing_backoff, self._speed / 2, check_limits=False)
            if trigger == TRIGGER_MANUAL:
                self._status = status
                return

            trigger = self._move(-2 * self._homing_backoff, self._speed / 10,
                                        check_endstop=True, check_limits=False)
            if trigger == TRIGGER_MANUAL:
                self._status = status
                return

            if trigger == TRIGGER_AT_LIMIT:
                self._pos_steps_at_origin = self._pos_steps
                self._status = StepperStatus.Idle
            else:
                self._status = StepperStatus.NotHomed

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
            self._move(distance, self._speed)
            self._status = StepperStatus.Idle

        if blocking:
            inner()
        else:
            threading.Thread(target=inner).start()

    def track(self, track_func):
        with self._track_lock:
            if self._track_thread is not None:
                self.stop()
                self._track_thread.join()

            if track_func is not None:
                self._stopped = False
                self._track_thread = threading.Thread(target=self._update_track_thread, args=(track_func,))
                self._track_thread.start()
            else:
                self._track_thread = None

    def stop(self):
        self._stopped = True
        self._mcu.send_command('trsync_trigger', oid=self._trigger_oid, reason=TRIGGER_MANUAL)
