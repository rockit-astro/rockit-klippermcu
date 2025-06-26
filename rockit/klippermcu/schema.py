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

def stepper_schema(require_uart=False, require_endstop=False, require_tracking=False):
    schema = {
        'type': 'object',
        'additionalProperties': False,
        # optional keys are appended below
        'required': ['step_pin', 'dir_pin', 'enable_pin', 'rotation_microsteps', 'rotation_distance',
                     'position_min', 'position_max', 'speed', 'acceleration'],
        'properties': {
            'step_pin': {'type': 'string'},
            'dir_pin': {'type': 'string'},
            'enable_pin': {'type': 'string'},
            'endstop_pin': {'type': 'string'},
            'endstop_pos': {'type': 'string', 'enum': ['max', 'min']},
            'rotation_microsteps': {'type': 'integer'},
            'rotation_distance': {'type': 'number'},
            'position_min': {'type': 'number'},
            'position_max': {'type': 'number'},
            'speed': {'type': 'number'},
            'acceleration': {'type': 'number'},
            'homing_backoff': {'type': 'number'},
            'tracking_cadence': {'type': 'number'},
            'tracking_commit_buffer': {'type': 'number'},
            'interface': {'type': 'string'},
            'uart_address': {'type': 'integer', 'enum': [0, 1, 2, 3]},
            'uart_microsteps': {'type': 'integer', 'enum': [1, 2, 4, 8, 16, 32, 64, 128, 256]},
            'uart_run_current': {'type': 'number', 'minimum': 0}
        },
        'dependencies': {
            'uart_address': ['interface'],
            'uart_microsteps': ['interface'],
            'uart_run_current': ['interface'],
            'interface': ['uart_address', 'uart_microsteps', 'uart_run_current'],
            'tracking_cadence': ['tracking_commit_buffer'],
            'tracking_commit_buffer': ['tracking_cadence'],
        }
    }

    if require_uart:
        schema['required'] += ['uart_address', 'uart_microsteps', 'uart_run_current', 'interface']

    if require_endstop:
        schema['required'] += ['endstop_pin', 'endstop_pos', 'homing_backoff']

    if require_tracking:
        schema['required'] += ['tracking_cadence', 'tracking_commit_buffer']

    return schema

def gpio_schema():
    return {
        'type': 'object',
        'required': ['pin', 'idle_timeout'],
        'properties': {
            'pin': {'type': 'string'},
            'idle_timeout': {
                'type': 'number',
                'minimum': 1
            }
        },
        'additionalProperties': False
    }

def neopixel_schema():
    return {
        'type': 'object',
        'required': ['pin', 'count'],
        'properties': {
            'pin': {'type': 'string'},
            'count': {
                'type': 'integer',
                'minimum': 1
            }
        },
        'additionalProperties': False
    }

def interfaces_schema(tmc2209=False, ds2484=False):
    interfaces = []
    if tmc2209:
        interfaces.append({
            'properties': {
                'type': {'type': 'string', 'enum': ['tmc2209']},
                'uart_pin': {'type': 'string'},
                'tx_pin': {'type': 'string'},
            },
            'required': ['uart_pin'],
            'additionalProperties': False
        })

    if ds2484:
        interfaces.append({
            'properties': {
                'type': {'enum': ['DS2484']},
                'i2c_bus': {'type': 'string'},
            },
            'additionalProperties': False
        })

    return {
        'type': 'object',
        'additionalProperties': {
            'type': 'object',
            'oneOf': interfaces
        }
    }
