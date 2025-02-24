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

"""Daemon for controlling a multi-channel focus controller via klipper and Pyro"""

from ctypes import CDLL, Structure, POINTER, pointer
from ctypes import c_char, c_char_p, c_double, c_int, c_uint8, c_uint64, c_void_p
import math
import threading
import serial
from .msgproto import MessageParser

COMMAND_RESPONSES = {
    'get_clock': 'clock',
    'get_config': 'config',
    'get_uptime': 'uptime',
    'identify': 'identify_response',
    'stepper_get_position': 'stepper_position',
    'tmcuart_send': 'tmcuart_response',
    'i2c_read': 'i2c_read_response'
}

COMMAND_PROCESS_RESPONSE = {
    'stepper_position': lambda p: p['pos'],
    'tmcuart_response': lambda p: p['read'],
    'i2c_read_response': lambda p: p['response']
}

RTT_AGE = .000010 / (60. * 60.)
DECAY = 1. / 30.
TRANSMIT_EXTRA = .001

class PullQueueMessage(Structure):
    _fields_ = (('msg', c_uint8 * 64),
                ('len', c_int),
                ('sent_time', c_double),
                ('receive_time', c_double),
                ('notify_id', c_uint64))

class NativeLib:
    def __init__(self):
        lib = CDLL('/usr/lib64/libklippermcu.so')
        self.serialqueue_pull = lib.serialqueue_pull
        self.serialqueue_pull.argtypes = [c_void_p, c_void_p]
        self.serialqueue_alloc = lib.serialqueue_alloc
        self.serialqueue_alloc.argtypes = [c_void_p, c_char, c_int]
        self.serialqueue_alloc.restype = c_void_p
        self.serialqueue_send = lib.serialqueue_send
        self.serialqueue_send.argtypes = [c_void_p, c_void_p, POINTER(c_uint8), c_int, c_uint64, c_uint64, c_uint64]
        self.serialqueue_set_clock_est = lib.serialqueue_set_clock_est
        self.serialqueue_set_clock_est.argtypes = [c_void_p, c_double, c_double, c_uint64, c_uint64]
        self.serialqueue_get_stats = lib.serialqueue_get_stats
        self.serialqueue_get_stats.argtypes = [c_void_p, c_char_p, c_int]
        self.serialqueue_set_wire_frequency = lib.serialqueue_set_wire_frequency
        self.serialqueue_set_wire_frequency.argtypes = [c_void_p, c_double]
        self.serialqueue_set_receive_window = lib.serialqueue_set_receive_window
        self.serialqueue_set_receive_window.argtypes = [c_void_p, c_int]
        self.serialqueue_alloc_commandqueue = lib.serialqueue_alloc_commandqueue
        self.serialqueue_alloc_commandqueue.restype = c_void_p
        self.serialqueue_exit = lib.serialqueue_exit
        self.serialqueue_exit.argtypes = [c_void_p]
        self.serialqueue_free = lib.serialqueue_free
        self.serialqueue_free.argtypes = [c_void_p]
        self.get_monotonic = lib.get_monotonic
        self.get_monotonic.restype = c_double


class MCU:
    """Daemon interface for multi-channel focuser"""
    def __init__(self, serial_port, serial_baud):
        self._serial_port = serial_port
        self._serial_baud = serial_baud
        self._oid_count = 0
        self._init_callbacks = []
        self._post_init_callbacks = []

        self._lib = NativeLib()
        self.msgparser = MessageParser()
        self._serial_queue = None
        self._serial_queue_lock = threading.Lock()

        self.cmd_lock = threading.Lock()
        self.cmd_queue = self._lib.serialqueue_alloc_commandqueue()
        self.cmd_response_callbacks = {}

        # Clock syncing state
        self.mcu_freq = 1.
        self.last_clock = 0
        self.clock_est = (0., 0., 0.)

        # Minimum round-trip-time tracking
        self.min_half_rtt = 999999999.9
        self.min_rtt_time = 0.

        # Linear regression of mcu clock and system sent_time
        self.time_avg = self.time_variance = 0.
        self.clock_avg = self.clock_covariance = 0.
        self.prediction_variance = 0.
        self.last_prediction_time = 0.

        self._serial_thread = None
        threading.Thread(target=self._clock_sync_thread, daemon=True).start()
        self._configured = False

    def _connection_thread(self):
        """Background thread managing communication over the serial connection"""
        with self._serial_queue_lock:
            try:
                serial_dev = serial.Serial(baudrate=self._serial_baud, timeout=0,
                                           exclusive=True)
                serial_dev.port = self._serial_port
                serial_dev.rts = True
                serial_dev.open()
                self._serial_queue = self._lib.serialqueue_alloc(serial_dev.fileno(), b'u', 0)
            except (OSError, IOError, serial.SerialException) as e:
                print(f'Unable to open serial port: {e}')
                init_cb = self.cmd_response_callbacks.pop(None, None)
                if init_cb is not None:
                    init_cb({})
                return

        response = PullQueueMessage()
        default_process = lambda x: x
        while True:
            self._lib.serialqueue_pull(self._serial_queue, pointer(response))
            count = response.len
            if count < 0:
                break

            params = self.msgparser.parse(response.msg[0:count])
            params['#sent_time'] = response.sent_time
            params['#receive_time'] = response.receive_time
            key = (params['#name'], params.get('oid', None))
            try:
                with self.cmd_lock:
                    cb = self.cmd_response_callbacks.pop(key, None)
                    if cb is None:
                        cb = self.cmd_response_callbacks.pop(None, None)

                    if cb is not None:
                        process = COMMAND_PROCESS_RESPONSE.get(params['#name'], default_process)
                        cb(process(params))
                    else:
                        print('Unhandled response', params)
            except Exception:
                print("Exception in serial callback")

        with self._serial_queue_lock:
            self._lib.serialqueue_free(self._serial_queue)
            self._serial_queue = None
            self._serial_thread = None
            serial_dev.close()

    def _sync_clock(self):
        # Clock sync logic copied from Klipper clocksync.py
        # Copyright (C) 2016-2018  Kevin O'Connor <kevin@koconnor.net>
        params = self.send_query('get_clock')

        # Extend clock to 64bit
        last_clock = self.last_clock
        clock_delta = (params['clock'] - last_clock) & 0xffffffff
        self.last_clock = clock = last_clock + clock_delta

        # Check if this is the best round-trip-time seen so far
        sent_time = params['#sent_time']
        if not sent_time:
            return

        receive_time = params['#receive_time']
        half_rtt = .5 * (receive_time - sent_time)
        aged_rtt = (sent_time - self.min_rtt_time) * RTT_AGE
        if half_rtt < self.min_half_rtt + aged_rtt:
            self.min_half_rtt = half_rtt
            self.min_rtt_time = sent_time
            print(f'new minimum rtt {sent_time:.3f}: hrtt={half_rtt:.6f} freq={self.clock_est[2]}')

        # Filter out samples that are extreme outliers
        exp_clock = ((sent_time - self.time_avg) * self.clock_est[2] + self.clock_avg)
        clock_diff2 = (clock - exp_clock) ** 2
        if clock_diff2 > 25. * self.prediction_variance and clock_diff2 > (.000500 * self.mcu_freq) ** 2:
            freq = self.clock_est[2]
            diff = clock - exp_clock
            stddev = math.sqrt(self.prediction_variance)
            if clock > exp_clock and sent_time < self.last_prediction_time + 10.:
                print(f'Ignoring clock sample {sent_time:.3f}: freq={freq} diff={diff} stddev={stddev:.3f}')
                return

            print(f'Resetting prediction variance {sent_time:.3f}: freq={freq} diff={diff} stddev={stddev:.3f}')
            self.prediction_variance = (.001 * self.mcu_freq) ** 2
        else:
            self.last_prediction_time = sent_time
            self.prediction_variance = ((1. - DECAY) * (self.prediction_variance + clock_diff2 * DECAY))

        # Add clock and sent_time to linear regression
        diff_sent_time = sent_time - self.time_avg
        self.time_avg += DECAY * diff_sent_time
        self.time_variance = (1. - DECAY) * (self.time_variance + diff_sent_time ** 2 * DECAY)

        diff_clock = clock - self.clock_avg
        self.clock_avg += DECAY * diff_clock
        self.clock_covariance = (1. - DECAY) * (self.clock_covariance + diff_sent_time * diff_clock * DECAY)

        # Update prediction from linear regression
        new_freq = self.clock_covariance / self.time_variance
        pred_stddev = math.sqrt(self.prediction_variance)
        conv_time = self.time_avg + TRANSMIT_EXTRA
        conv_clock = int(self.clock_avg - 3. * pred_stddev)

        self.clock_est = (self.time_avg + self.min_half_rtt, self.clock_avg, new_freq)
        with self._serial_queue_lock:
            if self._serial_queue is not None:
                self._lib.serialqueue_set_clock_est(self._serial_queue, new_freq, conv_time, conv_clock, clock)

    def _clock_sync_thread(self):
        wait_condition = threading.Condition()
        while True:
            start = self._lib.get_monotonic()
            try:
                if self.connected:
                    self._sync_clock()
            except Exception:
                print("Exception in clock sync thread")

            with wait_condition:
                delay = .9839 - (self._lib.get_monotonic() - start)
                if delay > 0:
                    wait_condition.wait(delay)

    @property
    def connected(self):
        t = self._serial_thread
        return t is not None and t.is_alive() and self._configured

    def reserve_oid(self):
        ret = self._oid_count
        self._oid_count += 1
        return ret

    def register_init_callback(self, cb):
        self._init_callbacks.append(cb)

    def register_post_init_callback(self, cb):
        self._post_init_callbacks.append(cb)

    def send_command(self, command, **kwargs):
        mp = self.msgparser.messages_by_name.get(command, None)
        if mp is None:
            raise Exception(f'Unknown command type: {command}')

        cmd = mp.encode_by_name(**kwargs)
        with self._serial_queue_lock:
            if self._serial_queue is None:
                raise Exception('Serial queue not available')
            self._lib.serialqueue_send(self._serial_queue, self.cmd_queue,
                                       (c_uint8 * len(cmd))(*cmd), len(cmd), 0, 0, 0)

    def send_query(self, command, timeout=5, **kwargs) -> dict:
        response = {}
        cond = threading.Condition()
        def cb(r):
            nonlocal response
            response = r
            with cond:
                cond.notify()

        key = (COMMAND_RESPONSES[command], kwargs.get('oid', None))
        if key in self.cmd_response_callbacks:
            raise Exception(f'cb already registered for {key}')

        self.cmd_response_callbacks[key] = cb

        with cond:
            self.send_command(command, **kwargs)
            if not cond.wait(timeout):
                self.cmd_response_callbacks.pop(key, None)
                raise TimeoutError('Send timed out waiting for response')

        return response

    def register_response_callback(self, response, oid, cb):
        self.cmd_response_callbacks[(response, oid)] = cb

    def host_clock(self):
        return self._lib.get_monotonic()

    def host_clock_to_mcu_clock(self, host_time):
        sample_time, clock, freq = self.clock_est
        return int(clock + (host_time - sample_time) * freq)

    def mcu_clock_to_host_clock(self, mcu_clock):
        sample_time, clock, freq = self.clock_est
        return float(mcu_clock - clock) / freq + sample_time

    def initialize(self):
        self._configured = False
        cond = threading.Condition()
        def mcu_ready(_):
            with cond:
                cond.notify()

        # Hook the first status message from the MCU to know the connection is alive
        with cond:
            self.cmd_response_callbacks[None] = mcu_ready
            with self._serial_queue_lock:
                self._serial_thread = threading.Thread(target=self._connection_thread)
                self._serial_thread.start()
            if not cond.wait(5):
                raise TimeoutError('Connection timed out')

        with self._serial_queue_lock:
            if self._serial_queue is None:
                self._serial_thread = None
                return False

        # Query MessageParser state dictionary from the MCU in chunks
        msgparser_identify_data = b''
        while True:
            params = self.send_query('identify', offset=len(msgparser_identify_data), count=40)
            if params.get('offset', None) == len(msgparser_identify_data):
                if not params['data']:
                    self.msgparser.process_identify(msgparser_identify_data)
                    break

                msgparser_identify_data += params['data']

        wire_freq = self.msgparser.get_constant_float('SERIAL_BAUD', None)
        if wire_freq is not None:
            self._lib.serialqueue_set_wire_frequency(self._serial_queue, wire_freq)

        receive_window = self.msgparser.get_constant_int('RECEIVE_WINDOW', None)
        if receive_window is not None:
            self._lib.serialqueue_set_receive_window(self._serial_queue, receive_window)

        self.mcu_freq = self.msgparser.get_constant_float('CLOCK_FREQ')

        while True:
            params = self.send_query('get_config')
            if not params['is_shutdown'] and not params['is_config']:
                break

            # Reboot MCU if needed
            with cond:
                self.cmd_response_callbacks[None] = mcu_ready
                self.send_command('reset')
                if not cond.wait(5):
                    raise TimeoutError('Connection timed out')

        self.send_command('allocate_oids', count=self._oid_count)

        for cb in self._init_callbacks:
            cb()

        self.send_command('finalize_config', crc=0)

        params = self.send_query('get_uptime')
        self.last_clock = (params['high'] << 32) | params['clock']
        self.clock_avg = self.last_clock
        self.time_avg = params['#sent_time']
        self.clock_est = (self.time_avg, self.clock_avg, self.mcu_freq)
        self.prediction_variance = (.001 * self.mcu_freq)**2

        for cb in self._post_init_callbacks:
            cb()

        self._configured = True
        return True

    def shutdown(self):
        """Disconnects from the device"""
        with self._serial_queue_lock:
            if self._serial_queue is None:
                return False
            self._lib.serialqueue_exit(self._serial_queue)

        self._serial_thread.join()
        return True
