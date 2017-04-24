# Copyright (C) 2016 OpenMotics BVBA
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Affero General Public License as
# published by the Free Software Foundation, either version 3 of the
# License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Affero General Public License for more details.
#
# You should have received a copy of the GNU Affero General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
"""
The GatewayApi defines high level functions, these are used by the interface
and call the master_api to complete the actions.

@author: fryckbos
"""

import os
import threading
import time as pytime
import datetime
import traceback
import math
import sqlite3
import constants
import logging
from threading import Timer
from serial_utils import CommunicationTimedOutException
import master.master_api as master_api
from master.outputs import OutputStatus
from master.inputs import InputStatus
from master.thermostats import ThermostatStatus
from master.shutters import ShutterStatus
from master.master_communicator import BackgroundConsumer
from master.eeprom_controller import EepromController, EepromFile
from master.eeprom_extension import EepromExtension
from master.eeprom_models import OutputConfiguration, InputConfiguration, ThermostatConfiguration, \
    SensorConfiguration, PumpGroupConfiguration, GroupActionConfiguration, \
    ScheduledActionConfiguration, PulseCounterConfiguration, StartupActionConfiguration, \
    ShutterConfiguration, ShutterGroupConfiguration, DimmerConfiguration, \
    GlobalThermostatConfiguration, CoolingConfiguration, CoolingPumpGroupConfiguration, \
    GlobalRTD10Configuration, RTD10HeatingConfiguration, RTD10CoolingConfiguration, \
    CanLedConfiguration, RoomConfiguration, ThermostatSetpointConfiguration
import power.power_api as power_api

LOGGER = logging.getLogger("openmotics")


def convert_nan(number):
    """ Convert nan to 0. """
    return 0.0 if math.isnan(number) else number


def check_basic_action(ret_dict):
    """ Checks if the response is 'OK', throws a ValueError otherwise. """
    if ret_dict['resp'] != 'OK':
        raise ValueError("Basic action did not return OK.")


class GatewayApi(object):
    """ The GatewayApi combines master_api functions into high level functions. """

    def __init__(self, master_communicator, power_communicator, power_controller):
        self.__master_communicator = master_communicator
        self.__eeprom_controller = EepromController(
            EepromFile(self.__master_communicator),
            EepromExtension(constants.get_eeprom_extension_database_file())
        )
        self.__power_communicator = power_communicator
        self.__power_controller = power_controller
        self.__plugin_controller = None

        self.__last_maintenance_send_time = 0
        self.__maintenance_timeout_timer = None

        self.__discover_mode_timer = None

        self.__output_status = None
        self.__input_status = InputStatus()
        self.__module_log = []
        self.__thermostat_status = None
        self.__shutter_status = ShutterStatus()

        self.__master_communicator.register_consumer(
                BackgroundConsumer(master_api.output_list(), 0, self.__update_outputs, True)
        )
        self.__master_communicator.register_consumer(
                BackgroundConsumer(master_api.input_list(), 0, self.__update_inputs)
        )
        self.__master_communicator.register_consumer(
                BackgroundConsumer(master_api.module_initialize(), 0, self.__update_modules)
        )
        self.__master_communicator.register_consumer(
            BackgroundConsumer(master_api.event_triggered(), 0, self.__event_triggered, True)
        )

        self.__init_shutter_status()
        self.__master_communicator.register_consumer(
                BackgroundConsumer(master_api.shutter_status(), 0,
                                   self.__shutter_status.handle_shutter_update)
        )

        self.__extend_method("set_shutter_configuration", self.__init_shutter_status)
        self.__extend_method("set_shutter_configurations", self.__init_shutter_status)

        self.__init_master()
        self.__load_thermostat_setpoints()
        self.__run_master_timer()

    def __extend_method(self, method_name, extension):
        """ Extend a method of the object to call the extension function after method execution.
        This is used to add an event to the auto-generated code. This way, we don't have to modify
        the auto-generated code.
        """
        old = getattr(self, method_name)

        def override(*args, **kwargs):
            ret = old(*args, **kwargs)
            extension()
            return ret

        setattr(self, method_name, override)

    def set_plugin_controller(self, plugin_controller):
        """ Set the plugin controller. """
        self.__plugin_controller = plugin_controller

    def __init_master(self):
        """ Initialize the master: disable the async RO messages, enable async OL, IL and SO
        messages, enables multi-tenant thermostats. """
        try:
            eeprom_data = self.__master_communicator.do_command(master_api.eeprom_list(),
                                                                {"bank": 0})['data']

            write = False

            if eeprom_data[11] != chr(255):
                LOGGER.info("Disabling async RO messages.")
                self.__master_communicator.do_command(
                    master_api.write_eeprom(),
                    {"bank": 0, "address": 11, "data": chr(255)}
                )
                write = True

            if eeprom_data[18] != chr(0):
                LOGGER.info("Enabling async OL messages.")
                self.__master_communicator.do_command(
                    master_api.write_eeprom(),
                    {"bank": 0, "address": 18, "data": chr(0)}
                )
                write = True

            if eeprom_data[20] != chr(0):
                LOGGER.info("Enabling async IL messages.")
                self.__master_communicator.do_command(
                    master_api.write_eeprom(),
                    {"bank": 0, "address": 20, "data": chr(0)}
                )
                write = True

            if eeprom_data[28] != chr(0):
                LOGGER.info("Enabling async SO messages.")
                self.__master_communicator.do_command(
                    master_api.write_eeprom(),
                    {"bank": 0, "address": 28, "data": chr(0)}
                )
                write = True

            thermostat_mode = ord(eeprom_data[14])
            if thermostat_mode & 64 == 0:
                LOGGER.info("Enabling multi-tenant thermostats.")
                self.__master_communicator.do_command(
                    master_api.write_eeprom(),
                    {"bank": 0, "address": 14, "data": chr(thermostat_mode | 64)}
                )
                write = True

            if eeprom_data[59] != chr(32):
                LOGGER.info("Enabling 32 thermostats.")
                self.__master_communicator.do_command(
                    master_api.write_eeprom(),
                    {"bank": 0, "address": 59, "data": chr(32)}
                )
                write = True

            if write:
                self.__master_communicator.do_command(master_api.activate_eeprom(), {'eep': 0})

        except CommunicationTimedOutException:
            LOGGER.error("Got CommunicationTimedOutException during gateway_api initialization.")

    def __load_thermostat_setpoints(self):
        """ Load the thermostat setpoints from the EepromController into the master. """
        for config in self.__eeprom_controller.read_all(ThermostatSetpointConfiguration):
            if config.setpoint != 255:  # Skip not initialised ThermostatSetpointConfiguration
                self.set_per_thermostat_mode(config.id, config.automatic, config.setpoint)

    def __run_master_timer(self):
        """ Run the master timer, this sets the masters clock if it differs more than 3 minutes
        from the gateway clock. """

        try:
            status = self.__master_communicator.do_command(master_api.status())

            master_time = datetime.datetime(1, 1, 1, status['hours'], status['minutes'], status['seconds'])

            now = datetime.datetime.now()
            expected_weekday = now.weekday() + 1
            expected_time = now.replace(year=1, month=1, day=1, microsecond=0)

            sync = False
            if abs((master_time - expected_time).total_seconds()) > 180:  # Allow 3 minutes difference
                sync = True
            if status['weekday'] != expected_weekday:
                sync = True

            if sync is True:
                LOGGER.info('Time - master: {0} ({1}) - gateway: {2} ({3})'.format(
                    master_time, status['weekday'], expected_time, expected_weekday)
                )
                self.sync_master_time()

        except:
            LOGGER.error("Got error while setting the time on the master.")
            traceback.print_exc()
        finally:
            Timer(120, self.__run_master_timer).start()

    def sync_master_time(self):
        """ Set the time on the master. """
        LOGGER.info("Setting the time on the master.")
        now = datetime.datetime.now()
        self.__master_communicator.do_command(
            master_api.set_time(),
            {'sec': now.second, 'min': now.minute, 'hours': now.hour,
             'weekday': now.isoweekday(), 'day': now.day, 'month': now.month,
             'year': now.year % 100}
        )

    def __init_shutter_status(self):
        """ Initialize the shutter status. """
        ret = self.__master_communicator.do_command(master_api.number_of_io_modules())
        num_shutter_modules = ret['shutter']

        configs = []
        for i in range(num_shutter_modules):
            configs.append([self.get_shutter_configuration(i * 4 + j) for j in range(4)])

        status = []
        for i in range(num_shutter_modules):
            status.append(self.__master_communicator.do_command(master_api.shutter_status(),
                                                                {'module_nr': i})['status'])

        self.__shutter_status.init(configs, status)

    def __event_triggered(self, ev_output):
        """ Handle an event triggered by the master. """
        code = ev_output['code']

        if self.__plugin_controller is not None:
            self.__plugin_controller.process_event(code)

    # Maintenance functions

    def start_maintenance_mode(self, timeout=600):
        """ Start maintenance mode, if the time between send_maintenance_data calls exceeds the
        timeout, the maintenance mode will be closed automatically. """
        try:
            self.set_master_status_leds(True)
        except Exception as exception:
            msg = "Exception while setting status leds before maintenance mode:" + str(exception)
            LOGGER.warning(msg)

        self.__eeprom_controller.invalidate_cache()  # Eeprom can be changed in maintenance mode.
        self.__master_communicator.start_maintenance_mode()

        def check_maintenance_timeout():
            """ Checks if the maintenance if the timeout is exceeded, and closes maintenance mode
            if required. """
            if self.__master_communicator.in_maintenance_mode():
                current_time = pytime.time()
                if self.__last_maintenance_send_time + timeout < current_time:
                    LOGGER.info("Stopping maintenance mode because of timeout.")
                    self.stop_maintenance_mode()
                else:
                    wait_time = self.__last_maintenance_send_time + timeout - current_time
                    self.__maintenance_timeout_timer = Timer(wait_time, check_maintenance_timeout)
                    self.__maintenance_timeout_timer.start()

        self.__maintenance_timeout_timer = Timer(timeout, check_maintenance_timeout)
        self.__maintenance_timeout_timer.start()

    def send_maintenance_data(self, data):
        """ Send data to the master in maintenance mode.

        :param data: data to send to the master
        :type data: string
        """
        self.__last_maintenance_send_time = pytime.time()
        self.__master_communicator.send_maintenance_data(data)

    def get_maintenance_data(self):
        """ Get data from the master in maintenance mode.

        :returns: string containing unprocessed output
        """
        return self.__master_communicator.get_maintenance_data()

    def stop_maintenance_mode(self):
        """ Stop maintenance mode. """
        self.__master_communicator.stop_maintenance_mode()
        if self.__output_status is not None:
            self.__output_status.force_refresh()

        if self.__thermostat_status is not None:
            self.__thermostat_status.force_refresh()

        if self.__maintenance_timeout_timer is not None:
            self.__maintenance_timeout_timer.cancel()
            self.__maintenance_timeout_timer = None

        self.__init_shutter_status()

        try:
            self.set_master_status_leds(False)
        except Exception as exception:
            msg = "Exception while setting status leds after maintenance mode:" + str(exception)
            LOGGER.warning(msg)

    def get_status(self):
        """ Get the status of the Master.

        :returns: dict with 'time' (HH:MM), 'date' (DD:MM:YYYY), 'mode', 'version' (a.b.c)
                  and 'hw_version' (hardware version)
        """
        out_dict = self.__master_communicator.do_command(master_api.status())
        return {'time': '%02d:%02d' % (out_dict['hours'], out_dict['minutes']),
                'date': '%02d/%02d/%d' % (out_dict['day'], out_dict['month'], out_dict['year']),
                'mode': out_dict['mode'],
                'version': "%d.%d.%d" % (out_dict['f1'], out_dict['f2'], out_dict['f3']),
                'hw_version': out_dict['h']}

    def reset_master(self):
        """ Perform a cold reset on the master. Turns the power off, waits 5 seconds and
        turns the power back on.

        :returns: 'status': 'OK'.
        """
        _ = self  # Must be an instance method
        gpio_direction = open('/sys/class/gpio/gpio44/direction', 'w')
        gpio_direction.write('out')
        gpio_direction.close()

        def power(master_on):
            """ Set the power on the master. """
            gpio_file = open('/sys/class/gpio/gpio44/value', 'w')
            gpio_file.write('1' if master_on else '0')
            gpio_file.close()

        power(False)
        pytime.sleep(5)
        power(True)

        return {'status': 'OK'}

    # Master module functions

    def __update_modules(self, api_data):
        """ Create a log entry when the MI message is received. """
        module_map = {'O': 'output', 'I': 'input', 'T': 'temperature', 'D': 'dimmer'}
        message_map = {'N': 'New %s module found.',
                       'E': 'Existing %s module found.',
                       'D': 'The %s module tried to register but the registration failed, '
                            'please presse the init button again.'}
        log_level_map = {'N': 'INFO', 'E': 'WARN', 'D': 'ERROR'}

        module_type = module_map.get(api_data['id'][0])
        message = message_map.get(api_data['instr']) % module_type
        log_level = log_level_map.get(api_data['instr'])

        self.__module_log.append((log_level, message))

    def module_discover_start(self, timeout=900):
        """ Start the module discover mode on the master.

        :returns: dict with 'status' ('OK').
        """
        ret = self.__master_communicator.do_command(master_api.module_discover_start())

        if self.__discover_mode_timer is not None:
            self.__discover_mode_timer.cancel()

        self.__discover_mode_timer = Timer(timeout, self.module_discover_stop)
        self.__discover_mode_timer.start()

        self.__module_log = []

        return {'status': ret['resp']}

    def module_discover_stop(self):
        """ Stop the module discover mode on the master.

        :returns: dict with 'status' ('OK').
        """
        ret = self.__master_communicator.do_command(master_api.module_discover_stop())

        if self.__discover_mode_timer is not None:
            self.__discover_mode_timer.cancel()
            self.__discover_mode_timer = None

        self.__module_log = []

        return {'status': ret['resp']}

    def module_discover_status(self):
        """ Gets the status of the module discover mode on the master.

        :returns dict with 'running': True|False
        """
        return {'running': self.__discover_mode_timer is not None}

    def get_module_log(self):
        """ Get the log messages from the module discovery mode. This returns the current log
        messages and clear the log messages.

        :returns: dict with 'log' (list of tuples (log_level, message)).
        """
        (module_log, self.__module_log) = (self.__module_log, [])
        return {'log': module_log}

    def get_modules(self):
        """ Get a list of all modules attached and registered with the master.

        :returns: dict with 'outputs' (list of module types: O,R,D), 'inputs' \
        (list of input module types: I,T,L) and 'shutters' (List of modules types: S).
        """
        mods = self.__master_communicator.do_command(master_api.number_of_io_modules())

        inputs = []
        outputs = []
        shutters = []

        for i in range(mods['in']):
            ret = self.__master_communicator.do_command(
                            master_api.read_eeprom(),
                            {'bank': 2 + i, 'addr': 0, 'num': 1})

            inputs.append(ret['data'][0])

        for i in range(mods['out']):
            ret = self.__master_communicator.do_command(
                            master_api.read_eeprom(),
                            {'bank': 33 + i, 'addr': 0, 'num': 1})

            outputs.append(ret['data'][0])

        for shutter in range(mods['shutter']):
            shutters.append('S')

        return {'outputs': outputs, 'inputs': inputs, 'shutters': shutters}

    def flash_leds(self, led_type, led_id):
        """ Flash the leds on the module for an output/input/sensor.

        :type led_type: byte
        :param led_type: The module type: output/dimmer (0), input (1), sensor/temperatur (2).
        :type led_id: bytes
        :param led_id: The id of the output/input/sensor.
        :returns: dict with 'status' ('OK').
        """
        ret = self.__master_communicator.do_command(master_api.indicate(),
                                                    {'type': led_type, 'id': led_id})
        return {'status': ret['resp']}

    # Output functions

    def __read_outputs(self):
        """ Read all output information from the MasterApi.

        :returns: a list of dicts with all fields from master_api.read_output.
        """
        ret = self.__master_communicator.do_command(master_api.number_of_io_modules())
        num_outputs = ret['out'] * 8

        outputs = []
        for i in range(0, num_outputs):
            outputs.append(self.__master_communicator.do_command(master_api.read_output(),
                                                                 {'id': i}))
        return outputs

    def __update_outputs(self, ol_output):
        """ Update the OutputStatus when an OL is received. """
        on_outputs = ol_output['outputs']

        if self.__output_status is not None:
            self.__output_status.partial_update(on_outputs)

        if self.__plugin_controller is not None:
            self.__plugin_controller.process_output_status(on_outputs)

    def get_output_status(self):
        """ Get a list containing the status of the Outputs.

        :returns: A list is a dicts containing the following keys: id, status, ctimer
        and dimmer.
        """
        if self.__output_status is None:
            self.__output_status = OutputStatus(self.__read_outputs())

        if self.__output_status.should_refresh():
            self.__output_status.full_update(self.__read_outputs())

        outputs = self.__output_status.get_outputs()
        return [{'id': output['id'], 'status': output['status'],
                 'ctimer': output['ctimer'], 'dimmer': output['dimmer']}
                for output in outputs]

    def set_output(self, output_id, is_on, dimmer=None, timer=None):
        """ Set the status, dimmer and timer of an output.

        :param output_id: The id of the output to set
        :type output_id: Integer [0, 240]
        :param is_on: Whether the output should be on
        :type is_on: Boolean
        :param dimmer: The dimmer value to set, None if unchanged
        :type dimmer: Integer [0, 100] or None
        :param timer: The timer value to set, None if unchanged
        :type timer: Integer in [150, 450, 900, 1500, 2220, 3120]
        :returns: emtpy dict.
        """
        if not is_on:
            if dimmer is not None or timer is not None:
                raise ValueError("Cannot set timer and dimmer when setting output to off")
            else:
                self.set_output_status(output_id, False)
        else:
            if dimmer is not None:
                self.set_output_dimmer(output_id, dimmer)

            self.set_output_status(output_id, True)

            if timer is not None:
                self.set_output_timer(output_id, timer)

        return dict()

    def set_output_status(self, output_id, is_on):
        """ Set the status of an output.

        :param output_id: The id of the output to set
        :type output_id: Integer [0, 240]
        :param is_on: Whether the output should be on
        :type is_on: Boolean
        :returns: empty dict.
        """
        if output_id < 0 or output_id > 240:
            raise ValueError("id not in [0, 240]: %d" % output_id)

        if is_on:
            self.__master_communicator.do_command(
                master_api.basic_action(),
                {"action_type": master_api.BA_LIGHT_ON, "action_number": output_id}
            )
        else:
            self.__master_communicator.do_command(
                master_api.basic_action(),
                {"action_type": master_api.BA_LIGHT_OFF, "action_number": output_id}
            )

        return dict()

    def set_output_dimmer(self, output_id, dimmer):
        """ Set the dimmer of an output.

        :param output_id: The id of the output to set
        :type output_id: Integer [0, 240]
        :param dimmer: The dimmer value to set, None if unchanged
        :type dimmer: Integer [0, 100] or None
        :returns: empty dict.
        """
        if output_id < 0 or output_id > 240:
            raise ValueError("id not in [0, 240]: %d" % output_id)

        if dimmer < 0 or dimmer > 100:
            raise ValueError("Dimmer value not in [0, 100]: %d" % dimmer)

        dimmer = int(dimmer) / 10 * 10

        if dimmer == 0:
            dimmer_action = master_api.BA_DIMMER_MIN
        elif dimmer == 100:
            dimmer_action = master_api.BA_DIMMER_MAX
        else:
            dimmer_action = master_api.__dict__['BA_LIGHT_ON_DIMMER_' + str(dimmer)]

        self.__master_communicator.do_command(
            master_api.basic_action(),
            {"action_type": dimmer_action, "action_number": output_id}
        )

        return dict()

    def set_output_timer(self, output_id, timer):
        """ Set the timer of an output.

        :param output_id: The id of the output to set
        :type output_id: Integer [0, 240]
        :param timer: The timer value to set, None if unchanged
        :type timer: Integer in [150, 450, 900, 1500, 2220, 3120]
        :returns: empty dict.
        """
        if output_id < 0 or output_id > 240:
            raise ValueError("id not in [0, 240]: %d" % output_id)

        if timer not in [150, 450, 900, 1500, 2220, 3120]:
            raise ValueError("Timer value not in [150, 450, 900, 1500, 2220, 3120]: %d" % timer)

        timer_action = master_api.__dict__['BA_LIGHT_ON_TIMER_' + str(timer) + '_OVERRULE']

        self.__master_communicator.do_command(
            master_api.basic_action(),
            {"action_type": timer_action, "action_number": output_id}
        )

        return dict()

    def set_all_lights_off(self):
        """ Turn all lights off.

        :returns: empty dict.
        """
        self.__master_communicator.do_command(
            master_api.basic_action(),
            {"action_type": master_api.BA_ALL_LIGHTS_OFF, "action_number": 0}
        )

        return dict()

    def set_all_lights_floor_off(self, floor):
        """ Turn all lights on a given floor off.

        :returns: empty dict.
        """
        self.__master_communicator.do_command(
            master_api.basic_action(),
            {"action_type": master_api.BA_LIGHTS_OFF_FLOOR, "action_number": floor}
        )

        return dict()

    def set_all_lights_floor_on(self, floor):
        """ Turn all lights on a given floor on.

        :returns: empty dict.
        """
        self.__master_communicator.do_command(
            master_api.basic_action(),
            {"action_type": master_api.BA_LIGHTS_ON_FLOOR, "action_number": floor}
        )

        return dict()

    # Shutter functions

    def get_shutter_status(self):
        """ Get a list containing the status of the Shutters.

        :returns: A list is a dicts containing the following keys: id, status.
        """
        return self.__shutter_status.get_status()

    def do_shutter_down(self, shutter_id):
        """ Make a shutter go down. The shutter stops automatically when the down position is
        reached (after the predefined number of seconds).

        :param shutter_id: The id of the shutter.
        :type shutter_id: Byte
        :returns:'status': 'OK'.
        """
        if shutter_id < 0 or shutter_id > 120:
            raise ValueError("id not in [0, 120]: %d" % shutter_id)

        self.__master_communicator.do_command(
            master_api.basic_action(),
            {"action_type": master_api.BA_SHUTTER_DOWN, "action_number": shutter_id}
        )

        return {'status': 'OK'}

    def do_shutter_up(self, shutter_id):
        """ Make a shutter go up. The shutter stops automatically when the up position is
        reached (after the predefined number of seconds).

        :param shutter_id: The id of the shutter.
        :type shutter_id: Byte
        :returns:'status': 'OK'.
        """
        if shutter_id < 0 or shutter_id > 120:
            raise ValueError("id not in [0, 120]: %d" % shutter_id)

        self.__master_communicator.do_command(
            master_api.basic_action(),
            {"action_type": master_api.BA_SHUTTER_UP, "action_number": shutter_id}
        )

        return {'status': 'OK'}

    def do_shutter_stop(self, shutter_id):
        """ Make a shutter stop.

        :param shutter_id: The id of the shutter.
        :type shutter_id: Byte
        :returns:'status': 'OK'.
        """
        if shutter_id < 0 or shutter_id > 120:
            raise ValueError("id not in [0, 120]: %d" % shutter_id)

        self.__master_communicator.do_command(
            master_api.basic_action(),
            {"action_type": master_api.BA_SHUTTER_STOP, "action_number": shutter_id}
        )

        return {'status': 'OK'}

    def do_shutter_group_down(self, group_id):
        """ Make a shutter group go down. The shutters stop automatically when the down position is
        reached (after the predefined number of seconds).

        :param group_id: The id of the shutter group.
        :type group_id: Byte
        :returns:'status': 'OK'.
        """
        if group_id < 0 or group_id > 30:
            raise ValueError("id not in [0, 30]: %d" % group_id)

        self.__master_communicator.do_command(
            master_api.basic_action(),
            {"action_type": master_api.BA_SHUTTER_GROUP_DOWN, "action_number": group_id}
        )

        return {'status': 'OK'}

    def do_shutter_group_up(self, group_id):
        """ Make a shutter group go up. The shutters stop automatically when the up position is
        reached (after the predefined number of seconds).

        :param group_id: The id of the shutter group.
        :type group_id: Byte
        :returns:'status': 'OK'.
        """
        if group_id < 0 or group_id > 30:
            raise ValueError("id not in [0, 30]: %d" % group_id)

        self.__master_communicator.do_command(
            master_api.basic_action(),
            {"action_type": master_api.BA_SHUTTER_GROUP_UP, "action_number": id}
        )

        return {'status': 'OK'}

    def do_shutter_group_stop(self, group_id):
        """ Make a shutter group stop.

        :param group_id: The id of the shutter group.
        :type group_id: Byte
        :returns:'status': 'OK'.
        """
        if group_id < 0 or group_id > 30:
            raise ValueError("id not in [0, 30]: %d" % group_id)

        self.__master_communicator.do_command(
            master_api.basic_action(),
            {"action_type": master_api.BA_SHUTTER_GROUP_STOP, "action_number": group_id}
        )

        return {'status': 'OK'}

    # Input functions

    def __update_inputs(self, api_data):
        """ Update the InputStatus with data from an IL message. """
        data_set = (api_data['input'], api_data['output'])
        self.__input_status.add_data(data_set)
        if self.__plugin_controller is not None:
            self.__plugin_controller.process_input_status(data_set)

    def get_last_inputs(self):
        """ Get the 5 last pressed inputs during the last 5 minutes.

        :returns: a list of tuples (input, output).
        """
        return self.__input_status.get_status()

    # Thermostat functions

    def __get_all_thermostats(self):
        """ Get basic information about all thermostats.

        :returns: array containing 32 dicts (one for each thermostats) with the following keys: \
        'active', 'sensor_nr', 'output0_nr', 'output1_nr', 'name'.
        """
        thermostats = {'heating': [], 'cooling': []}

        fields = ['sensor', 'output0', 'output1', 'name']
        heating_config = self.get_thermostat_configurations(fields=fields)
        cooling_config = self.get_cooling_configurations(fields=fields)

        for (key, config) in [('heating', heating_config), ('cooling', cooling_config)]:
            for thermostat in config:
                info = {'active': (thermostat['sensor'] < 30 or thermostat['sensor'] == 240) and thermostat['output0'] <= 240,
                        'sensor_nr': thermostat['sensor'],
                        'output0_nr': thermostat['output0'],
                        'output1_nr': thermostat['output1'],
                        'name': thermostat['name']}

                thermostats[key].append(info)

        return thermostats

    def get_thermostat_status(self):
        """ Get the status of the thermostats. Note that the automatic and setpoint field returned
        in the main dict are deprecated and reflect the state of the first thermostat.

        :returns: dict with global status information about the thermostats: 'thermostats_on',
        'automatic' (deprecated) and 'setpoint' (deprecated) and a list ('status') with status
        information for all thermostats, each element in the list is a dict with the following keys:
        'id', 'act', 'csetp', 'output0', 'output1', 'outside', 'mode', 'name', 'sensor_nr',
        'automatic', 'setpoint'.
        """
        if self.__thermostat_status is None:
            self.__thermostat_status = ThermostatStatus(self.__get_all_thermostats(), 1800)
        elif self.__thermostat_status.should_refresh():
            self.__thermostat_status.update(self.__get_all_thermostats())

        thermostat_info = self.__master_communicator.do_command(master_api.thermostat_list())
        thermostat_mode = self.__master_communicator.do_command(master_api.thermostat_mode_list())

        mode = thermostat_info['mode']
        thermostats_on = bool(mode & 1 << 7)
        cooling = bool(mode & 1 << 4)

        def get_automatic_setpoint(_mode):
            _automatic = bool(_mode & 1 << 3)
            return _automatic, 0 if _automatic else (_mode & 0b00000111)

        (automatic, setpoint) = get_automatic_setpoint(thermostat_mode['mode0'])

        thermostats = []
        outputs = self.get_output_status()

        cached_thermostats = self.__thermostat_status.get_thermostats()['cooling' if cooling else 'heating']

        aircos = self.__master_communicator.do_command(master_api.read_airco_status_bits())

        for thermostat_id in range(0, 32):
            if cached_thermostats[thermostat_id]['active'] is True:
                thermostat = {'id': thermostat_id,
                              'act': thermostat_info['tmp%d' % thermostat_id].get_temperature(),
                              'csetp': thermostat_info['setp%d' % thermostat_id].get_temperature(),
                              'outside': thermostat_info['outside'].get_temperature(),
                              'mode': thermostat_mode['mode%d' % thermostat_id]}
                (thermostat['automatic'], thermostat['setpoint']) = get_automatic_setpoint(thermostat['mode'])

                output0_nr = cached_thermostats[thermostat_id]['output0_nr']
                if output0_nr < len(outputs) and outputs[output0_nr]['status'] == 1:
                    thermostat['output0'] = outputs[output0_nr]['dimmer']
                else:
                    thermostat['output0'] = 0

                output1_nr = cached_thermostats[thermostat_id]['output1_nr']
                if output1_nr < len(outputs) and outputs[output1_nr]['status'] == 1:
                    thermostat['output1'] = outputs[output1_nr]['dimmer']
                else:
                    thermostat['output1'] = 0

                thermostat['name'] = cached_thermostats[thermostat_id]['name']
                thermostat['sensor_nr'] = cached_thermostats[thermostat_id]['sensor_nr']

                thermostat['airco'] = aircos["ASB%d" % thermostat_id]

                thermostats.append(thermostat)

        return {'thermostats_on': thermostats_on,
                'automatic': automatic,
                'setpoint': setpoint,
                'cooling': cooling,
                'status': thermostats}

    @staticmethod
    def __check_thermostat(thermostat):
        """ :raises ValueError if thermostat not in range [0, 32]. """
        if thermostat not in range(0, 32):
            raise ValueError("Thermostat not in [0,32]: %d" % thermostat)

    def set_current_setpoint(self, thermostat, temperature):
        """ Set the current setpoint of a thermostat.

        :param thermostat: The id of the thermostat to set
        :type thermostat: Integer [0, 32]
        :param temperature: The temperature to set in degrees Celcius
        :type temperature: float
        :returns: dict with 'thermostat', 'config' and 'temp'
        """
        GatewayApi.__check_thermostat(thermostat)

        self.__master_communicator.do_command(master_api.write_setpoint(),
                                              {'thermostat': thermostat,
                                               'config': 0,
                                               'temp': master_api.Svt.temp(temperature)})

        return {'status': 'OK'}

    def set_thermostat_mode(self, thermostat_on, cooling_mode=False, cooling_on=False):
        """ Set the mode of the thermostats.

        :param thermostat_on: Whether the thermostats are on
        :type thermostat_on: boolean
        :param cooling_mode: Cooling mode (True) of Heating mode (False)
        :type cooling_mode: boolean (optional)
        :param cooling_on: Turns cooling ON when set to true.
        :param cooling_on: boolean (optional)
        :returns: dict with 'status'
        """
        mode = 0
        mode |= 1 << 7 if thermostat_on else 0
        mode |= 1 << 6  # multi-tenant mode
        mode |= 1 << 4 if cooling_mode else 0

        check_basic_action(self.__master_communicator.do_command(master_api.basic_action(),
                           {'action_type': master_api.BA_THERMOSTAT_MODE,
                            'action_number': mode}))

        cooling_mode = 0 if not cooling_mode else (1 if not cooling_on else 2)

        check_basic_action(self.__master_communicator.do_command(
            master_api.basic_action(),
            {'action_type': master_api.BA_THERMOSTAT_COOLING_HEATING,
             'action_number': cooling_mode})
        )

        return {'status': 'OK'}

    def set_per_thermostat_mode(self, thermostat_id, automatic, setpoint):
        """ Set the setpoint/mode for a certain thermostat.

        :param thermostat_id: The id of the thermostat.
        :type thermostat_id: Integer [0, 31]
        :param automatic: Automatic mode (True) or Manual mode (False)
        :type automatic: boolean
        :param setpoint: The current setpoint
        :type setpoint: Integer [0, 5]
        :returns: dict with 'status'
        """
        if thermostat_id < 0 or thermostat_id > 31:
            raise ValueError("thermostat_id not in [0, 31]: %d" % thermostat_id)

        if setpoint < 0 or setpoint > 5:
            raise ValueError("setpoint not in [0, 5]: %d" % setpoint)

        if automatic:
            check_basic_action(self.__master_communicator.do_command(master_api.basic_action(),
                               {'action_type': master_api.BA_THERMOSTAT_TENANT_AUTO,
                                'action_number': thermostat_id}))
        else:
            check_basic_action(self.__master_communicator.do_command(master_api.basic_action(),
                               {'action_type': master_api.BA_THERMOSTAT_TENANT_MANUAL,
                                'action_number': thermostat_id}))

            check_basic_action(self.__master_communicator.do_command(master_api.basic_action(),
                               {'action_type': master_api.__dict__['BA_ONE_SETPOINT_%d' % setpoint],
                                'action_number': thermostat_id}))

        self.__eeprom_controller.write(
            ThermostatSetpointConfiguration.from_dict(
                {'id': thermostat_id, 'automatic': automatic, 'setpoint': setpoint}
            )
        )

        return {'status': 'OK'}

    def get_airco_status(self):
        """ Get the mode of the airco attached to a all thermostats.

        :returns: dict with ASB0-ASB31.
        """
        return self.__master_communicator.do_command(master_api.read_airco_status_bits())

    def set_airco_status(self, thermostat_id, airco_on):
        """ Set the mode of the airco attached to a given thermostat.

        :param thermostat_id: The thermostat id.
        :type thermostat_id: Integer [0, 31]
        :param airco_on: Turns the airco on if True.
        :type airco_on: boolean.

        :returns: dict with 'status'.
        """
        if thermostat_id < 0 or thermostat_id > 31:
            raise ValueError("thermostat_id not in [0, 31]: %d" % thermostat_id)

        modifier = 0 if airco_on else 100

        check_basic_action(self.__master_communicator.do_command(
            master_api.basic_action(),
            {'action_type': master_api.BA_THERMOSTAT_AIRCO_STATUS,
             'action_number': modifier + thermostat_id}
        ))

        return {'status': 'OK'}

    # Sensor status

    def get_sensor_temperature_status(self):
        """ Get the current temperature of all sensors.

        :returns: list with 32 temperatures, 1 for each sensor. None/null if not connected
        """
        output = []

        sensor_list = self.__master_communicator.do_command(master_api.sensor_temperature_list())
        for i in range(32):
            output.append(sensor_list['tmp%d' % i].get_temperature())

        return output

    def get_sensor_humidity_status(self):
        """ Get the current humidity of all sensors.

        :returns: list with 32 percentages, 1 for each sensor. None/null if not connected
        """
        output = []

        sensor_list = self.__master_communicator.do_command(master_api.sensor_humidity_list())
        for i in range(32):
            output.append(sensor_list['hum%d' % i].get_humidity())

        return output

    def get_sensor_brightness_status(self):
        """ Get the current brightness of all sensors.

        :returns: list with 32 percentages, 1 for each sensor. None/null if not connected
        """
        output = []

        sensor_list = self.__master_communicator.do_command(master_api.sensor_brightness_list())
        for i in range(32):
            output.append(sensor_list['bri%d' % i].get_brightness())

        return output

    def set_virtual_sensor(self, sensor_id, temperature, humidity, brightness):
        """ Set the temperature, humidity and brightness value of a virtual sensor.

        :param sensor_id: The id of the sensor.
        :type sensor_id: Integer [0, 31]
        :param temperature: The temperature to set in degrees Celcius
        :type temperature: float
        :param humidity: The humidity to set in percentage
        :type humidity: float
        :param brightness: The brightness to set in percentage
        :type brightness: float
        :returns: dict with 'status'.
        """
        if 0 > sensor_id > 31:
            raise ValueError("sensor_id not in [0, 31]: %d" % sensor_id)

        self.__master_communicator.do_command(master_api.set_virtual_sensor(),
                                              {'sensor': sensor_id,
                                               'tmp': master_api.Svt.temp(temperature),
                                               'hum': master_api.Svt.humidity(humidity),
                                               'bri': master_api.Svt.brightness(brightness)})

        return {'status': 'OK'}

    # Basic and group actions

    def do_basic_action(self, action_type, action_number):
        """ Execute a basic action.

        :param action_type: The type of the action as defined by the master api.
        :type action_type: Integer [0, 254]
        :param action_number: The number provided to the basic action, its meaning depends on the \
        action_type.
        :type action_number: Integer [0, 254]
        """
        if action_type < 0 or action_type > 254:
            raise ValueError("action_type not in [0, 254]: %d" % action_type)

        if action_number < 0 or action_number > 254:
            raise ValueError("action_number not in [0, 254]: %d" % action_number)

        self.__master_communicator.do_command(
            master_api.basic_action(),
            {"action_type": action_type,
             "action_number": action_number}
        )

        return dict()

    def do_group_action(self, group_action_id):
        """ Execute a group action.

        :param group_action_id: The id of the group action
        :type group_action_id: Integer (0 - 159)
        :returns: empty dict.
        """
        if group_action_id < 0 or group_action_id > 159:
            raise ValueError("group_action_id not in [0, 160]: %d" % group_action_id)

        self.__master_communicator.do_command(
            master_api.basic_action(),
            {"action_type": master_api.BA_GROUP_ACTION,
             "action_number": group_action_id}
        )

        return dict()

    # Backup and restore functions

    def get_full_backup(self):
        """ Get a backup (tar) of the master eeprom and the sqlite databases.

        :returns: Tar containing 4 files: master.eep, users.db, scheduled.db, power.db and
        eeprom_extensions.db as a string of bytes.
        """
        import shutil
        import tempfile
        import subprocess

        def backup_sqlite_db(input_db_path, backup_db_path):
            """ Backup an sqlite db provided the path to the db to backup and the backup db. """
            # Connect to database
            connection = sqlite3.connect(input_db_path)
            cursor = connection.cursor()

            # Lock database before making a backup
            cursor.execute('begin immediate')

            # Make new backup file
            shutil.copyfile(input_db_path, backup_db_path)

            # Unlock database
            connection.rollback()

        tmp_dir = tempfile.mkdtemp()
        try:
            with open("%s/master.eep" % tmp_dir, "w") as eeprom_file:
                eeprom_file.write(self.get_master_backup())

            backup_sqlite_db(constants.get_user_database_file(),
                             "%s/users.db" % tmp_dir)
            backup_sqlite_db(constants.get_scheduling_database_file(),
                             "%s/scheduled.db" % tmp_dir)
            backup_sqlite_db(constants.get_power_database_file(),
                             "%s/power.db" % tmp_dir)
            backup_sqlite_db(constants.get_eeprom_extension_database_file(),
                             "%s/eeprom_extensions.db" % tmp_dir)

            retcode = subprocess.call("cd %s; tar cf backup.tar *" % tmp_dir, shell=True)
            if retcode != 0:
                raise Exception("The backup tar could not be created.")

            with open("%s/backup.tar" % tmp_dir, "r") as backup_file:
                return backup_file.read()

        finally:
            shutil.rmtree(tmp_dir)

    def restore_full_backup(self, data):
        """ Restore a full backup containing the master eeprom and the sqlite databases.

        :param data: The eeprom backup to restore.
        :type data: tar containing 4 files: master.eep, users.db, scheduled.db, power.db and\
        eeprom_extensions.db as a string of bytes.
        :returns: dict with 'output' key.
        """
        import shutil
        import tempfile
        import subprocess

        tmp_dir = tempfile.mkdtemp()
        try:
            with open("%s/backup.tar" % tmp_dir, "wb") as backup_file:
                backup_file.write(data)

            retcode = subprocess.call("cd %s; tar xf backup.tar" % tmp_dir, shell=True)
            if retcode != 0:
                raise Exception("The backup tar could not be extracted.")

            with open("%s/master.eep" % tmp_dir, "r") as eeprom_file:
                eeprom_content = eeprom_file.read()
                self.master_restore(eeprom_content)

            shutil.copyfile("%s/users.db" % tmp_dir,
                            constants.get_user_database_file())
            shutil.copyfile("%s/scheduled.db" % tmp_dir,
                            constants.get_scheduling_database_file())
            shutil.copyfile("%s/power.db" % tmp_dir,
                            constants.get_power_database_file())
            shutil.copyfile("%s/eeprom_extensions.db" % tmp_dir,
                            constants.get_eeprom_extension_database_file())

            return {'output': 'Restore complete'}

        finally:
            shutil.rmtree(tmp_dir)

            # Restart the Cherrypy server after 1 second. Lets the current request terminate.
            threading.Timer(1, lambda: os._exit(0)).start()

    def get_master_backup(self):
        """ Get a backup of the eeprom of the master.

        :returns: String of bytes (size = 64kb).
        """
        output = ""
        for bank in range(0, 256):
            output += self.__master_communicator.do_command(
                master_api.eeprom_list(),
                {'bank': bank}
            )['data']
        return output

    def master_restore(self, data):
        """ Restore a backup of the eeprom of the master.

        :param data: The eeprom backup to restore.
        :type data: string of bytes (size = 64 kb).
        :returns: dict with 'output' key (contains an array with the addresses that were written).
        """
        ret = []
        (num_banks, bank_size, write_size) = (256, 256, 10)

        for bank in range(0, num_banks):
            read = self.__master_communicator.do_command(master_api.eeprom_list(),
                                                         {'bank': bank})['data']
            for addr in range(0, bank_size, write_size):
                orig = read[addr:addr + write_size]
                new = data[bank * bank_size + addr: bank * bank_size + addr + len(orig)]
                if new != orig:
                    ret.append("B" + str(bank) + "A" + str(addr))

                    self.__master_communicator.do_command(
                        master_api.write_eeprom(),
                        {'bank': bank, 'address': addr, 'data': new}
                    )

        self.__master_communicator.do_command(master_api.activate_eeprom(), {'eep': 0})
        ret.append("Activated eeprom")

        return {'output': ret}

    def master_reset(self):
        """ Reset the master.

        :returns: emtpy dict.
        """
        self.__master_communicator.do_command(master_api.reset())
        return dict()

    # Error functions

    def master_error_list(self):
        """ Get the error list per module (input and output modules). The modules are identified by
        O1, O2, I1, I2, ...

        :returns: dict with 'errors' key, it contains list of tuples (module, nr_errors).
        """
        error_list = self.__master_communicator.do_command(master_api.error_list())
        return error_list["errors"]

    def master_last_success(self):
        """ Get the number of seconds since the last successful communication with the master.
        """
        return self.__master_communicator.get_seconds_since_last_success()

    def power_last_success(self):
        """ Get the number of seconds since the last successful communication with the power
        modules.
        """
        return self.__power_communicator.get_seconds_since_last_success()

    def master_clear_error_list(self):
        """ Clear the number of errors.

        :returns: empty dict.
        """
        self.__master_communicator.do_command(master_api.clear_error_list())
        return dict()

    # Status led functions

    def set_master_status_leds(self, status):
        """ Set the status of the leds on the master.

        :param status: whether the leds should be on or off.
        :type status: boolean.
        :returns: empty dict.
        """
        on = 1 if status is True else 0
        self.__master_communicator.do_command(
            master_api.basic_action(),
            {"action_type": master_api.BA_STATUS_LEDS, "action_number": on}
        )
        return dict()

    # Pulse counter functions

    def get_pulse_counter_status(self):
        """ Get the pulse counter values.

        :returns: array with the 24 pulse counter values.
        """
        out_dict = self.__master_communicator.do_command(master_api.pulse_list())
        return [out_dict['pv0'], out_dict['pv1'], out_dict['pv2'], out_dict['pv3'],
                out_dict['pv4'], out_dict['pv5'], out_dict['pv6'], out_dict['pv7'],
                out_dict['pv8'], out_dict['pv9'], out_dict['pv10'], out_dict['pv11'],
                out_dict['pv12'], out_dict['pv13'], out_dict['pv14'], out_dict['pv15'],
                out_dict['pv16'], out_dict['pv17'], out_dict['pv18'], out_dict['pv19'],
                out_dict['pv20'], out_dict['pv21'], out_dict['pv22'], out_dict['pv23']]

    # Below are the auto generated master configuration functions

    def get_output_configuration(self, output_id, fields=None):
        """
        Get a specific output_configuration defined by its id.

        :param output_id: The id of the output_configuration
        :type output_id: Id
        :param fields: The field of the output_configuration to get. (None gets all fields)
        :type fields: List of strings
        :returns: output_configuration dict: contains 'id' (Id), 'can_led_1_function' (Enum), 'can_led_1_id' (Byte), 'can_led_2_function' (Enum), 'can_led_2_id' (Byte), 'can_led_3_function' (Enum), 'can_led_3_id' (Byte), 'can_led_4_function' (Enum), 'can_led_4_id' (Byte), 'floor' (Byte), 'module_type' (String[1]), 'name' (String[16]), 'room' (Byte), 'timer' (Word), 'type' (Byte)
        """
        return self.__eeprom_controller.read(OutputConfiguration, output_id, fields).to_dict()

    def get_output_configurations(self, fields=None):
        """
        Get all output_configurations.

        :param fields: The field of the output_configuration to get. (None gets all fields)
        :type fields: List of strings
        :returns: list of output_configuration dict: contains 'id' (Id), 'can_led_1_function' (Enum), 'can_led_1_id' (Byte), 'can_led_2_function' (Enum), 'can_led_2_id' (Byte), 'can_led_3_function' (Enum), 'can_led_3_id' (Byte), 'can_led_4_function' (Enum), 'can_led_4_id' (Byte), 'floor' (Byte), 'module_type' (String[1]), 'name' (String[16]), 'room' (Byte), 'timer' (Word), 'type' (Byte)
        """
        return [o.to_dict() for o in self.__eeprom_controller.read_all(OutputConfiguration, fields)]

    def set_output_configuration(self, config):
        """
        Set one output_configuration.

        :param config: The output_configuration to set
        :type config: output_configuration dict: contains 'id' (Id), 'can_led_1_function' (Enum), 'can_led_1_id' (Byte), 'can_led_2_function' (Enum), 'can_led_2_id' (Byte), 'can_led_3_function' (Enum), 'can_led_3_id' (Byte), 'can_led_4_function' (Enum), 'can_led_4_id' (Byte), 'floor' (Byte), 'name' (String[16]), 'room' (Byte), 'timer' (Word), 'type' (Byte)
        """
        self.__eeprom_controller.write(OutputConfiguration.from_dict(config))

    def set_output_configurations(self, config):
        """
        Set multiple output_configurations.

        :param config: The list of output_configurations to set
        :type config: list of output_configuration dict: contains 'id' (Id), 'can_led_1_function' (Enum), 'can_led_1_id' (Byte), 'can_led_2_function' (Enum), 'can_led_2_id' (Byte), 'can_led_3_function' (Enum), 'can_led_3_id' (Byte), 'can_led_4_function' (Enum), 'can_led_4_id' (Byte), 'floor' (Byte), 'name' (String[16]), 'room' (Byte), 'timer' (Word), 'type' (Byte)
        """
        self.__eeprom_controller.write_batch([OutputConfiguration.from_dict(o) for o in config])

    def get_shutter_configuration(self, shutter_id, fields=None):
        """
        Get a specific shutter_configuration defined by its id.

        :param shutter_id: The id of the shutter_configuration
        :type shutter_id: Id
        :param fields: The field of the shutter_configuration to get. (None gets all fields)
        :type fields: List of strings
        :returns: shutter_configuration dict: contains 'id' (Id), 'group_1' (Byte), 'group_2' (Byte), 'name' (String[16]), 'room' (Byte), 'timer_down' (Byte), 'timer_up' (Byte), 'up_down_config' (Byte)
        """
        return self.__eeprom_controller.read(ShutterConfiguration, shutter_id, fields).to_dict()

    def get_shutter_configurations(self, fields=None):
        """
        Get all shutter_configurations.

        :param fields: The field of the shutter_configuration to get. (None gets all fields)
        :type fields: List of strings
        :returns: list of shutter_configuration dict: contains 'id' (Id), 'group_1' (Byte), 'group_2' (Byte), 'name' (String[16]), 'room' (Byte), 'timer_down' (Byte), 'timer_up' (Byte), 'up_down_config' (Byte)
        """
        return [o.to_dict() for o in self.__eeprom_controller.read_all(ShutterConfiguration, fields)]

    def set_shutter_configuration(self, config):
        """
        Set one shutter_configuration.

        :param config: The shutter_configuration to set
        :type config: shutter_configuration dict: contains 'id' (Id), 'group_1' (Byte), 'group_2' (Byte), 'name' (String[16]), 'room' (Byte), 'timer_down' (Byte), 'timer_up' (Byte), 'up_down_config' (Byte)
        """
        self.__eeprom_controller.write(ShutterConfiguration.from_dict(config))

    def set_shutter_configurations(self, config):
        """
        Set multiple shutter_configurations.

        :param config: The list of shutter_configurations to set
        :type config: list of shutter_configuration dict: contains 'id' (Id), 'group_1' (Byte), 'group_2' (Byte), 'name' (String[16]), 'room' (Byte), 'timer_down' (Byte), 'timer_up' (Byte), 'up_down_config' (Byte)
        """
        self.__eeprom_controller.write_batch([ShutterConfiguration.from_dict(o) for o in config])

    def get_shutter_group_configuration(self, group_id, fields=None):
        """
        Get a specific shutter_group_configuration defined by its id.

        :param group_id: The id of the shutter_group_configuration
        :type group_id: Id
        :param fields: The field of the shutter_group_configuration to get. (None gets all fields)
        :type fields: List of strings
        :returns: shutter_group_configuration dict: contains 'id' (Id), 'room' (Byte), 'timer_down' (Byte), 'timer_up' (Byte)
        """
        return self.__eeprom_controller.read(ShutterGroupConfiguration, group_id, fields).to_dict()

    def get_shutter_group_configurations(self, fields=None):
        """
        Get all shutter_group_configurations.

        :param fields: The field of the shutter_group_configuration to get. (None gets all fields)
        :type fields: List of strings
        :returns: list of shutter_group_configuration dict: contains 'id' (Id), 'room' (Byte), 'timer_down' (Byte), 'timer_up' (Byte)
        """
        return [o.to_dict() for o in self.__eeprom_controller.read_all(ShutterGroupConfiguration, fields)]

    def set_shutter_group_configuration(self, config):
        """
        Set one shutter_group_configuration.

        :param config: The shutter_group_configuration to set
        :type config: shutter_group_configuration dict: contains 'id' (Id), 'room' (Byte), 'timer_down' (Byte), 'timer_up' (Byte)
        """
        self.__eeprom_controller.write(ShutterGroupConfiguration.from_dict(config))

    def set_shutter_group_configurations(self, config):
        """
        Set multiple shutter_group_configurations.

        :param config: The list of shutter_group_configurations to set
        :type config: list of shutter_group_configuration dict: contains 'id' (Id), 'room' (Byte), 'timer_down' (Byte), 'timer_up' (Byte)
        """
        self.__eeprom_controller.write_batch([ShutterGroupConfiguration.from_dict(o) for o in config])

    def get_input_configuration(self, input_id, fields=None):
        """
        Get a specific input_configuration defined by its id.

        :param input_id: The id of the input_configuration
        :type input_id: Id
        :param fields: The field of the input_configuration to get. (None gets all fields)
        :type fields: List of strings
        :returns: input_configuration dict: contains 'id' (Id), 'action' (Byte), 'basic_actions' (Actions[15]), 'invert' (Byte), 'module_type' (String[1]), 'name' (String[8]), 'room' (Byte)
        """
        return self.__eeprom_controller.read(InputConfiguration, input_id, fields).to_dict()

    def get_input_configurations(self, fields=None):
        """
        Get all input_configurations.

        :param fields: The field of the input_configuration to get. (None gets all fields)
        :type fields: List of strings
        :returns: list of input_configuration dict: contains 'id' (Id), 'action' (Byte), 'basic_actions' (Actions[15]), 'invert' (Byte), 'module_type' (String[1]), 'name' (String[8]), 'room' (Byte)
        """
        return [o.to_dict() for o in self.__eeprom_controller.read_all(InputConfiguration, fields)]

    def set_input_configuration(self, config):
        """
        Set one input_configuration.

        :param config: The input_configuration to set
        :type config: input_configuration dict: contains 'id' (Id), 'action' (Byte), 'basic_actions' (Actions[15]), 'invert' (Byte), 'name' (String[8]), 'room' (Byte)
        """
        self.__eeprom_controller.write(InputConfiguration.from_dict(config))

    def set_input_configurations(self, config):
        """
        Set multiple input_configurations.

        :param config: The list of input_configurations to set
        :type config: list of input_configuration dict: contains 'id' (Id), 'action' (Byte), 'basic_actions' (Actions[15]), 'invert' (Byte), 'name' (String[8]), 'room' (Byte)
        """
        self.__eeprom_controller.write_batch([InputConfiguration.from_dict(o) for o in config])

    def get_thermostat_configuration(self, thermostat_id, fields=None):
        """
        Get a specific thermostat_configuration defined by its id.

        :param thermostat_id: The id of the thermostat_configuration
        :type thermostat_id: Id
        :param fields: The field of the thermostat_configuration to get. (None gets all fields)
        :type fields: List of strings
        :returns: thermostat_configuration dict: contains 'id' (Id), 'auto_fri' ([temp_n(Temp),start_d1(Time),stop_d1(Time),temp_d1(Temp),start_d2(Time),stop_d2(Time),temp_d2(Temp)]), 'auto_mon' ([temp_n(Temp),start_d1(Time),stop_d1(Time),temp_d1(Temp),start_d2(Time),stop_d2(Time),temp_d2(Temp)]), 'auto_sat' ([temp_n(Temp),start_d1(Time),stop_d1(Time),temp_d1(Temp),start_d2(Time),stop_d2(Time),temp_d2(Temp)]), 'auto_sun' ([temp_n(Temp),start_d1(Time),stop_d1(Time),temp_d1(Temp),start_d2(Time),stop_d2(Time),temp_d2(Temp)]), 'auto_thu' ([temp_n(Temp),start_d1(Time),stop_d1(Time),temp_d1(Temp),start_d2(Time),stop_d2(Time),temp_d2(Temp)]), 'auto_tue' ([temp_n(Temp),start_d1(Time),stop_d1(Time),temp_d1(Temp),start_d2(Time),stop_d2(Time),temp_d2(Temp)]), 'auto_wed' ([temp_n(Temp),start_d1(Time),stop_d1(Time),temp_d1(Temp),start_d2(Time),stop_d2(Time),temp_d2(Temp)]), 'name' (String[16]), 'output0' (Byte), 'output1' (Byte), 'permanent_manual' (Boolean), 'pid_d' (Byte), 'pid_i' (Byte), 'pid_int' (Byte), 'pid_p' (Byte), 'room' (Byte), 'sensor' (Byte), 'setp0' (Temp), 'setp1' (Temp), 'setp2' (Temp), 'setp3' (Temp), 'setp4' (Temp), 'setp5' (Temp)
        """
        return self.__eeprom_controller.read(ThermostatConfiguration, thermostat_id, fields).to_dict()

    def get_thermostat_configurations(self, fields=None):
        """
        Get all thermostat_configurations.

        :param fields: The field of the thermostat_configuration to get. (None gets all fields)
        :type fields: List of strings
        :returns: list of thermostat_configuration dict: contains 'id' (Id), 'auto_fri' ([temp_n(Temp),start_d1(Time),stop_d1(Time),temp_d1(Temp),start_d2(Time),stop_d2(Time),temp_d2(Temp)]), 'auto_mon' ([temp_n(Temp),start_d1(Time),stop_d1(Time),temp_d1(Temp),start_d2(Time),stop_d2(Time),temp_d2(Temp)]), 'auto_sat' ([temp_n(Temp),start_d1(Time),stop_d1(Time),temp_d1(Temp),start_d2(Time),stop_d2(Time),temp_d2(Temp)]), 'auto_sun' ([temp_n(Temp),start_d1(Time),stop_d1(Time),temp_d1(Temp),start_d2(Time),stop_d2(Time),temp_d2(Temp)]), 'auto_thu' ([temp_n(Temp),start_d1(Time),stop_d1(Time),temp_d1(Temp),start_d2(Time),stop_d2(Time),temp_d2(Temp)]), 'auto_tue' ([temp_n(Temp),start_d1(Time),stop_d1(Time),temp_d1(Temp),start_d2(Time),stop_d2(Time),temp_d2(Temp)]), 'auto_wed' ([temp_n(Temp),start_d1(Time),stop_d1(Time),temp_d1(Temp),start_d2(Time),stop_d2(Time),temp_d2(Temp)]), 'name' (String[16]), 'output0' (Byte), 'output1' (Byte), 'permanent_manual' (Boolean), 'pid_d' (Byte), 'pid_i' (Byte), 'pid_int' (Byte), 'pid_p' (Byte), 'room' (Byte), 'sensor' (Byte), 'setp0' (Temp), 'setp1' (Temp), 'setp2' (Temp), 'setp3' (Temp), 'setp4' (Temp), 'setp5' (Temp)
        """
        return [o.to_dict() for o in self.__eeprom_controller.read_all(ThermostatConfiguration, fields)]

    def set_thermostat_configuration(self, config):
        """
        Set one thermostat_configuration.

        :param config: The thermostat_configuration to set
        :type config: thermostat_configuration dict: contains 'id' (Id), 'auto_fri' ([temp_n(Temp),start_d1(Time),stop_d1(Time),temp_d1(Temp),start_d2(Time),stop_d2(Time),temp_d2(Temp)]), 'auto_mon' ([temp_n(Temp),start_d1(Time),stop_d1(Time),temp_d1(Temp),start_d2(Time),stop_d2(Time),temp_d2(Temp)]), 'auto_sat' ([temp_n(Temp),start_d1(Time),stop_d1(Time),temp_d1(Temp),start_d2(Time),stop_d2(Time),temp_d2(Temp)]), 'auto_sun' ([temp_n(Temp),start_d1(Time),stop_d1(Time),temp_d1(Temp),start_d2(Time),stop_d2(Time),temp_d2(Temp)]), 'auto_thu' ([temp_n(Temp),start_d1(Time),stop_d1(Time),temp_d1(Temp),start_d2(Time),stop_d2(Time),temp_d2(Temp)]), 'auto_tue' ([temp_n(Temp),start_d1(Time),stop_d1(Time),temp_d1(Temp),start_d2(Time),stop_d2(Time),temp_d2(Temp)]), 'auto_wed' ([temp_n(Temp),start_d1(Time),stop_d1(Time),temp_d1(Temp),start_d2(Time),stop_d2(Time),temp_d2(Temp)]), 'name' (String[16]), 'output0' (Byte), 'output1' (Byte), 'permanent_manual' (Boolean), 'pid_d' (Byte), 'pid_i' (Byte), 'pid_int' (Byte), 'pid_p' (Byte), 'room' (Byte), 'sensor' (Byte), 'setp0' (Temp), 'setp1' (Temp), 'setp2' (Temp), 'setp3' (Temp), 'setp4' (Temp), 'setp5' (Temp)
        """
        self.__eeprom_controller.write(ThermostatConfiguration.from_dict(config))

    def set_thermostat_configurations(self, config):
        """
        Set multiple thermostat_configurations.

        :param config: The list of thermostat_configurations to set
        :type config: list of thermostat_configuration dict: contains 'id' (Id), 'auto_fri' ([temp_n(Temp),start_d1(Time),stop_d1(Time),temp_d1(Temp),start_d2(Time),stop_d2(Time),temp_d2(Temp)]), 'auto_mon' ([temp_n(Temp),start_d1(Time),stop_d1(Time),temp_d1(Temp),start_d2(Time),stop_d2(Time),temp_d2(Temp)]), 'auto_sat' ([temp_n(Temp),start_d1(Time),stop_d1(Time),temp_d1(Temp),start_d2(Time),stop_d2(Time),temp_d2(Temp)]), 'auto_sun' ([temp_n(Temp),start_d1(Time),stop_d1(Time),temp_d1(Temp),start_d2(Time),stop_d2(Time),temp_d2(Temp)]), 'auto_thu' ([temp_n(Temp),start_d1(Time),stop_d1(Time),temp_d1(Temp),start_d2(Time),stop_d2(Time),temp_d2(Temp)]), 'auto_tue' ([temp_n(Temp),start_d1(Time),stop_d1(Time),temp_d1(Temp),start_d2(Time),stop_d2(Time),temp_d2(Temp)]), 'auto_wed' ([temp_n(Temp),start_d1(Time),stop_d1(Time),temp_d1(Temp),start_d2(Time),stop_d2(Time),temp_d2(Temp)]), 'name' (String[16]), 'output0' (Byte), 'output1' (Byte), 'permanent_manual' (Boolean), 'pid_d' (Byte), 'pid_i' (Byte), 'pid_int' (Byte), 'pid_p' (Byte), 'room' (Byte), 'sensor' (Byte), 'setp0' (Temp), 'setp1' (Temp), 'setp2' (Temp), 'setp3' (Temp), 'setp4' (Temp), 'setp5' (Temp)
        """
        self.__eeprom_controller.write_batch([ThermostatConfiguration.from_dict(o) for o in config])

    def get_sensor_configuration(self, sensor_id, fields=None):
        """
        Get a specific sensor_configuration defined by its id.

        :param sensor_id: The id of the sensor_configuration
        :type sensor_id: Id
        :param fields: The field of the sensor_configuration to get. (None gets all fields)
        :type fields: List of strings
        :returns: sensor_configuration dict: contains 'id' (Id), 'name' (String[16]), 'offset' (SignedTemp(-7.5 to 7.5 degrees)), 'room' (Byte), 'virtual' (Boolean)
        """
        return self.__eeprom_controller.read(SensorConfiguration, sensor_id, fields).to_dict()

    def get_sensor_configurations(self, fields=None):
        """
        Get all sensor_configurations.

        :param fields: The field of the sensor_configuration to get. (None gets all fields)
        :type fields: List of strings
        :returns: list of sensor_configuration dict: contains 'id' (Id), 'name' (String[16]), 'offset' (SignedTemp(-7.5 to 7.5 degrees)), 'room' (Byte), 'virtual' (Boolean)
        """
        return [o.to_dict() for o in self.__eeprom_controller.read_all(SensorConfiguration, fields)]

    def set_sensor_configuration(self, config):
        """
        Set one sensor_configuration.

        :param config: The sensor_configuration to set
        :type config: sensor_configuration dict: contains 'id' (Id), 'name' (String[16]), 'offset' (SignedTemp(-7.5 to 7.5 degrees)), 'room' (Byte), 'virtual' (Boolean)
        """
        self.__eeprom_controller.write(SensorConfiguration.from_dict(config))

    def set_sensor_configurations(self, config):
        """
        Set multiple sensor_configurations.

        :param config: The list of sensor_configurations to set
        :type config: list of sensor_configuration dict: contains 'id' (Id), 'name' (String[16]), 'offset' (SignedTemp(-7.5 to 7.5 degrees)), 'room' (Byte), 'virtual' (Boolean)
        """
        self.__eeprom_controller.write_batch([SensorConfiguration.from_dict(o) for o in config])

    def get_pump_group_configuration(self, pump_group_id, fields=None):
        """
        Get a specific pump_group_configuration defined by its id.

        :param pump_group_id: The id of the pump_group_configuration
        :type pump_group_id: Id
        :param fields: The field of the pump_group_configuration to get. (None gets all fields)
        :type fields: List of strings
        :returns: pump_group_configuration dict: contains 'id' (Id), 'outputs' (CSV[32]), 'room' (Byte)
        """
        return self.__eeprom_controller.read(PumpGroupConfiguration, pump_group_id, fields).to_dict()

    def get_pump_group_configurations(self, fields=None):
        """
        Get all pump_group_configurations.

        :param fields: The field of the pump_group_configuration to get. (None gets all fields)
        :type fields: List of strings
        :returns: list of pump_group_configuration dict: contains 'id' (Id), 'outputs' (CSV[32]), 'room' (Byte)
        """
        return [o.to_dict() for o in self.__eeprom_controller.read_all(PumpGroupConfiguration, fields)]

    def set_pump_group_configuration(self, config):
        """
        Set one pump_group_configuration.

        :param config: The pump_group_configuration to set
        :type config: pump_group_configuration dict: contains 'id' (Id), 'outputs' (CSV[32]), 'room' (Byte)
        """
        self.__eeprom_controller.write(PumpGroupConfiguration.from_dict(config))

    def set_pump_group_configurations(self, config):
        """
        Set multiple pump_group_configurations.

        :param config: The list of pump_group_configurations to set
        :type config: list of pump_group_configuration dict: contains 'id' (Id), 'outputs' (CSV[32]), 'room' (Byte)
        """
        self.__eeprom_controller.write_batch([PumpGroupConfiguration.from_dict(o) for o in config])

    def get_cooling_configuration(self, cooling_id, fields=None):
        """
        Get a specific cooling_configuration defined by its id.

        :param cooling_id: The id of the cooling_configuration
        :type cooling_id: Id
        :param fields: The field of the cooling_configuration to get. (None gets all fields)
        :type fields: List of strings
        :returns: cooling_configuration dict: contains 'id' (Id), 'auto_fri' ([temp_n(Temp),start_d1(Time),stop_d1(Time),temp_d1(Temp),start_d2(Time),stop_d2(Time),temp_d2(Temp)]), 'auto_mon' ([temp_n(Temp),start_d1(Time),stop_d1(Time),temp_d1(Temp),start_d2(Time),stop_d2(Time),temp_d2(Temp)]), 'auto_sat' ([temp_n(Temp),start_d1(Time),stop_d1(Time),temp_d1(Temp),start_d2(Time),stop_d2(Time),temp_d2(Temp)]), 'auto_sun' ([temp_n(Temp),start_d1(Time),stop_d1(Time),temp_d1(Temp),start_d2(Time),stop_d2(Time),temp_d2(Temp)]), 'auto_thu' ([temp_n(Temp),start_d1(Time),stop_d1(Time),temp_d1(Temp),start_d2(Time),stop_d2(Time),temp_d2(Temp)]), 'auto_tue' ([temp_n(Temp),start_d1(Time),stop_d1(Time),temp_d1(Temp),start_d2(Time),stop_d2(Time),temp_d2(Temp)]), 'auto_wed' ([temp_n(Temp),start_d1(Time),stop_d1(Time),temp_d1(Temp),start_d2(Time),stop_d2(Time),temp_d2(Temp)]), 'name' (String[16]), 'output0' (Byte), 'output1' (Byte), 'permanent_manual' (Boolean), 'pid_d' (Byte), 'pid_i' (Byte), 'pid_int' (Byte), 'pid_p' (Byte), 'room' (Byte), 'sensor' (Byte), 'setp0' (Temp), 'setp1' (Temp), 'setp2' (Temp), 'setp3' (Temp), 'setp4' (Temp), 'setp5' (Temp)
        """
        return self.__eeprom_controller.read(CoolingConfiguration, cooling_id, fields).to_dict()

    def get_cooling_configurations(self, fields=None):
        """
        Get all cooling_configurations.

        :param fields: The field of the cooling_configuration to get. (None gets all fields)
        :type fields: List of strings
        :returns: list of cooling_configuration dict: contains 'id' (Id), 'auto_fri' ([temp_n(Temp),start_d1(Time),stop_d1(Time),temp_d1(Temp),start_d2(Time),stop_d2(Time),temp_d2(Temp)]), 'auto_mon' ([temp_n(Temp),start_d1(Time),stop_d1(Time),temp_d1(Temp),start_d2(Time),stop_d2(Time),temp_d2(Temp)]), 'auto_sat' ([temp_n(Temp),start_d1(Time),stop_d1(Time),temp_d1(Temp),start_d2(Time),stop_d2(Time),temp_d2(Temp)]), 'auto_sun' ([temp_n(Temp),start_d1(Time),stop_d1(Time),temp_d1(Temp),start_d2(Time),stop_d2(Time),temp_d2(Temp)]), 'auto_thu' ([temp_n(Temp),start_d1(Time),stop_d1(Time),temp_d1(Temp),start_d2(Time),stop_d2(Time),temp_d2(Temp)]), 'auto_tue' ([temp_n(Temp),start_d1(Time),stop_d1(Time),temp_d1(Temp),start_d2(Time),stop_d2(Time),temp_d2(Temp)]), 'auto_wed' ([temp_n(Temp),start_d1(Time),stop_d1(Time),temp_d1(Temp),start_d2(Time),stop_d2(Time),temp_d2(Temp)]), 'name' (String[16]), 'output0' (Byte), 'output1' (Byte), 'permanent_manual' (Boolean), 'pid_d' (Byte), 'pid_i' (Byte), 'pid_int' (Byte), 'pid_p' (Byte), 'room' (Byte), 'sensor' (Byte), 'setp0' (Temp), 'setp1' (Temp), 'setp2' (Temp), 'setp3' (Temp), 'setp4' (Temp), 'setp5' (Temp)
        """
        return [o.to_dict() for o in self.__eeprom_controller.read_all(CoolingConfiguration, fields)]

    def set_cooling_configuration(self, config):
        """
        Set one cooling_configuration.

        :param config: The cooling_configuration to set
        :type config: cooling_configuration dict: contains 'id' (Id), 'auto_fri' ([temp_n(Temp),start_d1(Time),stop_d1(Time),temp_d1(Temp),start_d2(Time),stop_d2(Time),temp_d2(Temp)]), 'auto_mon' ([temp_n(Temp),start_d1(Time),stop_d1(Time),temp_d1(Temp),start_d2(Time),stop_d2(Time),temp_d2(Temp)]), 'auto_sat' ([temp_n(Temp),start_d1(Time),stop_d1(Time),temp_d1(Temp),start_d2(Time),stop_d2(Time),temp_d2(Temp)]), 'auto_sun' ([temp_n(Temp),start_d1(Time),stop_d1(Time),temp_d1(Temp),start_d2(Time),stop_d2(Time),temp_d2(Temp)]), 'auto_thu' ([temp_n(Temp),start_d1(Time),stop_d1(Time),temp_d1(Temp),start_d2(Time),stop_d2(Time),temp_d2(Temp)]), 'auto_tue' ([temp_n(Temp),start_d1(Time),stop_d1(Time),temp_d1(Temp),start_d2(Time),stop_d2(Time),temp_d2(Temp)]), 'auto_wed' ([temp_n(Temp),start_d1(Time),stop_d1(Time),temp_d1(Temp),start_d2(Time),stop_d2(Time),temp_d2(Temp)]), 'name' (String[16]), 'output0' (Byte), 'output1' (Byte), 'permanent_manual' (Boolean), 'pid_d' (Byte), 'pid_i' (Byte), 'pid_int' (Byte), 'pid_p' (Byte), 'room' (Byte), 'sensor' (Byte), 'setp0' (Temp), 'setp1' (Temp), 'setp2' (Temp), 'setp3' (Temp), 'setp4' (Temp), 'setp5' (Temp)
        """
        self.__eeprom_controller.write(CoolingConfiguration.from_dict(config))

    def set_cooling_configurations(self, config):
        """
        Set multiple cooling_configurations.

        :param config: The list of cooling_configurations to set
        :type config: list of cooling_configuration dict: contains 'id' (Id), 'auto_fri' ([temp_n(Temp),start_d1(Time),stop_d1(Time),temp_d1(Temp),start_d2(Time),stop_d2(Time),temp_d2(Temp)]), 'auto_mon' ([temp_n(Temp),start_d1(Time),stop_d1(Time),temp_d1(Temp),start_d2(Time),stop_d2(Time),temp_d2(Temp)]), 'auto_sat' ([temp_n(Temp),start_d1(Time),stop_d1(Time),temp_d1(Temp),start_d2(Time),stop_d2(Time),temp_d2(Temp)]), 'auto_sun' ([temp_n(Temp),start_d1(Time),stop_d1(Time),temp_d1(Temp),start_d2(Time),stop_d2(Time),temp_d2(Temp)]), 'auto_thu' ([temp_n(Temp),start_d1(Time),stop_d1(Time),temp_d1(Temp),start_d2(Time),stop_d2(Time),temp_d2(Temp)]), 'auto_tue' ([temp_n(Temp),start_d1(Time),stop_d1(Time),temp_d1(Temp),start_d2(Time),stop_d2(Time),temp_d2(Temp)]), 'auto_wed' ([temp_n(Temp),start_d1(Time),stop_d1(Time),temp_d1(Temp),start_d2(Time),stop_d2(Time),temp_d2(Temp)]), 'name' (String[16]), 'output0' (Byte), 'output1' (Byte), 'permanent_manual' (Boolean), 'pid_d' (Byte), 'pid_i' (Byte), 'pid_int' (Byte), 'pid_p' (Byte), 'room' (Byte), 'sensor' (Byte), 'setp0' (Temp), 'setp1' (Temp), 'setp2' (Temp), 'setp3' (Temp), 'setp4' (Temp), 'setp5' (Temp)
        """
        self.__eeprom_controller.write_batch([CoolingConfiguration.from_dict(o) for o in config])

    def get_cooling_pump_group_configuration(self, pump_group_id, fields=None):
        """
        Get a specific cooling_pump_group_configuration defined by its id.

        :param pump_group_id: The id of the cooling_pump_group_configuration
        :type pump_group_id: Id
        :param fields: The field of the cooling_pump_group_configuration to get. (None gets all fields)
        :type fields: List of strings
        :returns: cooling_pump_group_configuration dict: contains 'id' (Id), 'outputs' (CSV[32]), 'room' (Byte)
        """
        return self.__eeprom_controller.read(CoolingPumpGroupConfiguration, pump_group_id, fields).to_dict()

    def get_cooling_pump_group_configurations(self, fields=None):
        """
        Get all cooling_pump_group_configurations.

        :param fields: The field of the cooling_pump_group_configuration to get. (None gets all fields)
        :type fields: List of strings
        :returns: list of cooling_pump_group_configuration dict: contains 'id' (Id), 'outputs' (CSV[32]), 'room' (Byte)
        """
        return [o.to_dict() for o in self.__eeprom_controller.read_all(CoolingPumpGroupConfiguration, fields)]

    def set_cooling_pump_group_configuration(self, config):
        """
        Set one cooling_pump_group_configuration.

        :param config: The cooling_pump_group_configuration to set
        :type config: cooling_pump_group_configuration dict: contains 'id' (Id), 'outputs' (CSV[32]), 'room' (Byte)
        """
        self.__eeprom_controller.write(CoolingPumpGroupConfiguration.from_dict(config))

    def set_cooling_pump_group_configurations(self, config):
        """
        Set multiple cooling_pump_group_configurations.

        :param config: The list of cooling_pump_group_configurations to set
        :type config: list of cooling_pump_group_configuration dict: contains 'id' (Id), 'outputs' (CSV[32]), 'room' (Byte)
        """
        self.__eeprom_controller.write_batch([CoolingPumpGroupConfiguration.from_dict(o) for o in config])

    def get_global_rtd10_configuration(self, fields=None):
        """
        Get the global_rtd10_configuration.

        :param fields: The field of the global_rtd10_configuration to get. (None gets all fields)
        :type fields: List of strings
        :returns: global_rtd10_configuration dict: contains 'output_value_cooling_16' (Byte), 'output_value_cooling_16_5' (Byte), 'output_value_cooling_17' (Byte), 'output_value_cooling_17_5' (Byte), 'output_value_cooling_18' (Byte), 'output_value_cooling_18_5' (Byte), 'output_value_cooling_19' (Byte), 'output_value_cooling_19_5' (Byte), 'output_value_cooling_20' (Byte), 'output_value_cooling_20_5' (Byte), 'output_value_cooling_21' (Byte), 'output_value_cooling_21_5' (Byte), 'output_value_cooling_22' (Byte), 'output_value_cooling_22_5' (Byte), 'output_value_cooling_23' (Byte), 'output_value_cooling_23_5' (Byte), 'output_value_cooling_24' (Byte), 'output_value_heating_16' (Byte), 'output_value_heating_16_5' (Byte), 'output_value_heating_17' (Byte), 'output_value_heating_17_5' (Byte), 'output_value_heating_18' (Byte), 'output_value_heating_18_5' (Byte), 'output_value_heating_19' (Byte), 'output_value_heating_19_5' (Byte), 'output_value_heating_20' (Byte), 'output_value_heating_20_5' (Byte), 'output_value_heating_21' (Byte), 'output_value_heating_21_5' (Byte), 'output_value_heating_22' (Byte), 'output_value_heating_22_5' (Byte), 'output_value_heating_23' (Byte), 'output_value_heating_23_5' (Byte), 'output_value_heating_24' (Byte)
        """
        return self.__eeprom_controller.read(GlobalRTD10Configuration, fields).to_dict()

    def set_global_rtd10_configuration(self, config):
        """
        Set the global_rtd10_configuration.

        :param config: The global_rtd10_configuration to set
        :type config: global_rtd10_configuration dict: contains 'output_value_cooling_16' (Byte), 'output_value_cooling_16_5' (Byte), 'output_value_cooling_17' (Byte), 'output_value_cooling_17_5' (Byte), 'output_value_cooling_18' (Byte), 'output_value_cooling_18_5' (Byte), 'output_value_cooling_19' (Byte), 'output_value_cooling_19_5' (Byte), 'output_value_cooling_20' (Byte), 'output_value_cooling_20_5' (Byte), 'output_value_cooling_21' (Byte), 'output_value_cooling_21_5' (Byte), 'output_value_cooling_22' (Byte), 'output_value_cooling_22_5' (Byte), 'output_value_cooling_23' (Byte), 'output_value_cooling_23_5' (Byte), 'output_value_cooling_24' (Byte), 'output_value_heating_16' (Byte), 'output_value_heating_16_5' (Byte), 'output_value_heating_17' (Byte), 'output_value_heating_17_5' (Byte), 'output_value_heating_18' (Byte), 'output_value_heating_18_5' (Byte), 'output_value_heating_19' (Byte), 'output_value_heating_19_5' (Byte), 'output_value_heating_20' (Byte), 'output_value_heating_20_5' (Byte), 'output_value_heating_21' (Byte), 'output_value_heating_21_5' (Byte), 'output_value_heating_22' (Byte), 'output_value_heating_22_5' (Byte), 'output_value_heating_23' (Byte), 'output_value_heating_23_5' (Byte), 'output_value_heating_24' (Byte)
        """
        self.__eeprom_controller.write(GlobalRTD10Configuration.from_dict(config))

    def get_rtd10_heating_configuration(self, heating_id, fields=None):
        """
        Get a specific rtd10_heating_configuration defined by its id.

        :param heating_id: The id of the rtd10_heating_configuration
        :type heating_id: Id
        :param fields: The field of the rtd10_heating_configuration to get. (None gets all fields)
        :type fields: List of strings
        :returns: rtd10_heating_configuration dict: contains 'id' (Id), 'mode_output' (Byte), 'mode_value' (Byte), 'on_off_output' (Byte), 'poke_angle_output' (Byte), 'poke_angle_value' (Byte), 'room' (Byte), 'temp_setpoint_output' (Byte), 'ventilation_speed_output' (Byte), 'ventilation_speed_value' (Byte)
        """
        return self.__eeprom_controller.read(RTD10HeatingConfiguration, heating_id, fields).to_dict()

    def get_rtd10_heating_configurations(self, fields=None):
        """
        Get all rtd10_heating_configurations.

        :param fields: The field of the rtd10_heating_configuration to get. (None gets all fields)
        :type fields: List of strings
        :returns: list of rtd10_heating_configuration dict: contains 'id' (Id), 'mode_output' (Byte), 'mode_value' (Byte), 'on_off_output' (Byte), 'poke_angle_output' (Byte), 'poke_angle_value' (Byte), 'room' (Byte), 'temp_setpoint_output' (Byte), 'ventilation_speed_output' (Byte), 'ventilation_speed_value' (Byte)
        """
        return [o.to_dict() for o in self.__eeprom_controller.read_all(RTD10HeatingConfiguration, fields)]

    def set_rtd10_heating_configuration(self, config):
        """
        Set one rtd10_heating_configuration.

        :param config: The rtd10_heating_configuration to set
        :type config: rtd10_heating_configuration dict: contains 'id' (Id), 'mode_output' (Byte), 'mode_value' (Byte), 'on_off_output' (Byte), 'poke_angle_output' (Byte), 'poke_angle_value' (Byte), 'room' (Byte), 'temp_setpoint_output' (Byte), 'ventilation_speed_output' (Byte), 'ventilation_speed_value' (Byte)
        """
        self.__eeprom_controller.write(RTD10HeatingConfiguration.from_dict(config))

    def set_rtd10_heating_configurations(self, config):
        """
        Set multiple rtd10_heating_configurations.

        :param config: The list of rtd10_heating_configurations to set
        :type config: list of rtd10_heating_configuration dict: contains 'id' (Id), 'mode_output' (Byte), 'mode_value' (Byte), 'on_off_output' (Byte), 'poke_angle_output' (Byte), 'poke_angle_value' (Byte), 'room' (Byte), 'temp_setpoint_output' (Byte), 'ventilation_speed_output' (Byte), 'ventilation_speed_value' (Byte)
        """
        self.__eeprom_controller.write_batch([RTD10HeatingConfiguration.from_dict(o) for o in config])

    def get_rtd10_cooling_configuration(self, cooling_id, fields=None):
        """
        Get a specific rtd10_cooling_configuration defined by its id.

        :param cooling_id: The id of the rtd10_cooling_configuration
        :type cooling_id: Id
        :param fields: The field of the rtd10_cooling_configuration to get. (None gets all fields)
        :type fields: List of strings
        :returns: rtd10_cooling_configuration dict: contains 'id' (Id), 'mode_output' (Byte), 'mode_value' (Byte), 'on_off_output' (Byte), 'poke_angle_output' (Byte), 'poke_angle_value' (Byte), 'room' (Byte), 'temp_setpoint_output' (Byte), 'ventilation_speed_output' (Byte), 'ventilation_speed_value' (Byte)
        """
        return self.__eeprom_controller.read(RTD10CoolingConfiguration, cooling_id, fields).to_dict()

    def get_rtd10_cooling_configurations(self, fields=None):
        """
        Get all rtd10_cooling_configurations.

        :param fields: The field of the rtd10_cooling_configuration to get. (None gets all fields)
        :type fields: List of strings
        :returns: list of rtd10_cooling_configuration dict: contains 'id' (Id), 'mode_output' (Byte), 'mode_value' (Byte), 'on_off_output' (Byte), 'poke_angle_output' (Byte), 'poke_angle_value' (Byte), 'room' (Byte), 'temp_setpoint_output' (Byte), 'ventilation_speed_output' (Byte), 'ventilation_speed_value' (Byte)
        """
        return [o.to_dict() for o in self.__eeprom_controller.read_all(RTD10CoolingConfiguration, fields)]

    def set_rtd10_cooling_configuration(self, config):
        """
        Set one rtd10_cooling_configuration.

        :param config: The rtd10_cooling_configuration to set
        :type config: rtd10_cooling_configuration dict: contains 'id' (Id), 'mode_output' (Byte), 'mode_value' (Byte), 'on_off_output' (Byte), 'poke_angle_output' (Byte), 'poke_angle_value' (Byte), 'room' (Byte), 'temp_setpoint_output' (Byte), 'ventilation_speed_output' (Byte), 'ventilation_speed_value' (Byte)
        """
        self.__eeprom_controller.write(RTD10CoolingConfiguration.from_dict(config))

    def set_rtd10_cooling_configurations(self, config):
        """
        Set multiple rtd10_cooling_configurations.

        :param config: The list of rtd10_cooling_configurations to set
        :type config: list of rtd10_cooling_configuration dict: contains 'id' (Id), 'mode_output' (Byte), 'mode_value' (Byte), 'on_off_output' (Byte), 'poke_angle_output' (Byte), 'poke_angle_value' (Byte), 'room' (Byte), 'temp_setpoint_output' (Byte), 'ventilation_speed_output' (Byte), 'ventilation_speed_value' (Byte)
        """
        self.__eeprom_controller.write_batch([RTD10CoolingConfiguration.from_dict(o) for o in config])

    def get_group_action_configuration(self, group_action_id, fields=None):
        """
        Get a specific group_action_configuration defined by its id.

        :param group_action_id: The id of the group_action_configuration
        :type group_action_id: Id
        :param fields: The field of the group_action_configuration to get. (None gets all fields)
        :type fields: List of strings
        :returns: group_action_configuration dict: contains 'id' (Id), 'actions' (Actions[16]), 'name' (String[16])
        """
        return self.__eeprom_controller.read(GroupActionConfiguration, group_action_id, fields).to_dict()

    def get_group_action_configurations(self, fields=None):
        """
        Get all group_action_configurations.

        :param fields: The field of the group_action_configuration to get. (None gets all fields)
        :type fields: List of strings
        :returns: list of group_action_configuration dict: contains 'id' (Id), 'actions' (Actions[16]), 'name' (String[16])
        """
        return [o.to_dict() for o in self.__eeprom_controller.read_all(GroupActionConfiguration, fields)]

    def set_group_action_configuration(self, config):
        """
        Set one group_action_configuration.

        :param config: The group_action_configuration to set
        :type config: group_action_configuration dict: contains 'id' (Id), 'actions' (Actions[16]), 'name' (String[16])
        """
        self.__eeprom_controller.write(GroupActionConfiguration.from_dict(config))

    def set_group_action_configurations(self, config):
        """
        Set multiple group_action_configurations.

        :param config: The list of group_action_configurations to set
        :type config: list of group_action_configuration dict: contains 'id' (Id), 'actions' (Actions[16]), 'name' (String[16])
        """
        self.__eeprom_controller.write_batch([GroupActionConfiguration.from_dict(o) for o in config])

    def get_scheduled_action_configuration(self, scheduled_action_id, fields=None):
        """
        Get a specific scheduled_action_configuration defined by its id.

        :param scheduled_action_id: The id of the scheduled_action_configuration
        :type scheduled_action_id: Id
        :param fields: The field of the scheduled_action_configuration to get. (None gets all fields)
        :type fields: List of strings
        :returns: scheduled_action_configuration dict: contains 'id' (Id), 'action' (Actions[1]), 'day' (Byte), 'hour' (Byte), 'minute' (Byte)
        """
        return self.__eeprom_controller.read(ScheduledActionConfiguration, scheduled_action_id, fields).to_dict()

    def get_scheduled_action_configurations(self, fields=None):
        """
        Get all scheduled_action_configurations.

        :param fields: The field of the scheduled_action_configuration to get. (None gets all fields)
        :type fields: List of strings
        :returns: list of scheduled_action_configuration dict: contains 'id' (Id), 'action' (Actions[1]), 'day' (Byte), 'hour' (Byte), 'minute' (Byte)
        """
        return [o.to_dict() for o in self.__eeprom_controller.read_all(ScheduledActionConfiguration, fields)]

    def set_scheduled_action_configuration(self, config):
        """
        Set one scheduled_action_configuration.

        :param config: The scheduled_action_configuration to set
        :type config: scheduled_action_configuration dict: contains 'id' (Id), 'action' (Actions[1]), 'day' (Byte), 'hour' (Byte), 'minute' (Byte)
        """
        self.__eeprom_controller.write(ScheduledActionConfiguration.from_dict(config))

    def set_scheduled_action_configurations(self, config):
        """
        Set multiple scheduled_action_configurations.

        :param config: The list of scheduled_action_configurations to set
        :type config: list of scheduled_action_configuration dict: contains 'id' (Id), 'action' (Actions[1]), 'day' (Byte), 'hour' (Byte), 'minute' (Byte)
        """
        self.__eeprom_controller.write_batch([ScheduledActionConfiguration.from_dict(o) for o in config])

    def get_pulse_counter_configuration(self, pulse_counter_id, fields=None):
        """
        Get a specific pulse_counter_configuration defined by its id.

        :param pulse_counter_id: The id of the pulse_counter_configuration
        :type pulse_counter_id: Id
        :param fields: The field of the pulse_counter_configuration to get. (None gets all fields)
        :type fields: List of strings
        :returns: pulse_counter_configuration dict: contains 'id' (Id), 'input' (Byte), 'name' (String[16]), 'room' (Byte)
        """
        return self.__eeprom_controller.read(PulseCounterConfiguration, pulse_counter_id, fields).to_dict()

    def get_pulse_counter_configurations(self, fields=None):
        """
        Get all pulse_counter_configurations.

        :param fields: The field of the pulse_counter_configuration to get. (None gets all fields)
        :type fields: List of strings
        :returns: list of pulse_counter_configuration dict: contains 'id' (Id), 'input' (Byte), 'name' (String[16]), 'room' (Byte)
        """
        return [o.to_dict() for o in self.__eeprom_controller.read_all(PulseCounterConfiguration, fields)]

    def set_pulse_counter_configuration(self, config):
        """
        Set one pulse_counter_configuration.

        :param config: The pulse_counter_configuration to set
        :type config: pulse_counter_configuration dict: contains 'id' (Id), 'input' (Byte), 'name' (String[16]), 'room' (Byte)
        """
        self.__eeprom_controller.write(PulseCounterConfiguration.from_dict(config))

    def set_pulse_counter_configurations(self, config):
        """
        Set multiple pulse_counter_configurations.

        :param config: The list of pulse_counter_configurations to set
        :type config: list of pulse_counter_configuration dict: contains 'id' (Id), 'input' (Byte), 'name' (String[16]), 'room' (Byte)
        """
        self.__eeprom_controller.write_batch([PulseCounterConfiguration.from_dict(o) for o in config])

    def get_startup_action_configuration(self, fields=None):
        """
        Get the startup_action_configuration.

        :param fields: The field of the startup_action_configuration to get. (None gets all fields)
        :type fields: List of strings
        :returns: startup_action_configuration dict: contains 'actions' (Actions[100])
        """
        return self.__eeprom_controller.read(StartupActionConfiguration, fields).to_dict()

    def set_startup_action_configuration(self, config):
        """
        Set the startup_action_configuration.

        :param config: The startup_action_configuration to set
        :type config: startup_action_configuration dict: contains 'actions' (Actions[100])
        """
        self.__eeprom_controller.write(StartupActionConfiguration.from_dict(config))

    def get_dimmer_configuration(self, fields=None):
        """
        Get the dimmer_configuration.

        :param fields: The field of the dimmer_configuration to get. (None gets all fields)
        :type fields: List of strings
        :returns: dimmer_configuration dict: contains 'dim_memory' (Byte), 'dim_step' (Byte), 'dim_wait_cycle' (Byte), 'min_dim_level' (Byte)
        """
        return self.__eeprom_controller.read(DimmerConfiguration, fields).to_dict()

    def set_dimmer_configuration(self, config):
        """
        Set the dimmer_configuration.

        :param config: The dimmer_configuration to set
        :type config: dimmer_configuration dict: contains 'dim_memory' (Byte), 'dim_step' (Byte), 'dim_wait_cycle' (Byte), 'min_dim_level' (Byte)
        """
        self.__eeprom_controller.write(DimmerConfiguration.from_dict(config))

    def get_global_thermostat_configuration(self, fields=None):
        """
        Get the global_thermostat_configuration.

        :param fields: The field of the global_thermostat_configuration to get. (None gets all fields)
        :type fields: List of strings
        :returns: global_thermostat_configuration dict: contains 'outside_sensor' (Byte), 'pump_delay' (Byte), 'switch_to_cooling_output_0' (Byte), 'switch_to_cooling_output_1' (Byte), 'switch_to_cooling_output_2' (Byte), 'switch_to_cooling_output_3' (Byte), 'switch_to_cooling_value_0' (Byte), 'switch_to_cooling_value_1' (Byte), 'switch_to_cooling_value_2' (Byte), 'switch_to_cooling_value_3' (Byte), 'switch_to_heating_output_0' (Byte), 'switch_to_heating_output_1' (Byte), 'switch_to_heating_output_2' (Byte), 'switch_to_heating_output_3' (Byte), 'switch_to_heating_value_0' (Byte), 'switch_to_heating_value_1' (Byte), 'switch_to_heating_value_2' (Byte), 'switch_to_heating_value_3' (Byte), 'threshold_temp' (Temp)
        """
        return self.__eeprom_controller.read(GlobalThermostatConfiguration, fields).to_dict()

    def set_global_thermostat_configuration(self, config):
        """
        Set the global_thermostat_configuration.

        :param config: The global_thermostat_configuration to set
        :type config: global_thermostat_configuration dict: contains 'outside_sensor' (Byte), 'pump_delay' (Byte), 'switch_to_cooling_output_0' (Byte), 'switch_to_cooling_output_1' (Byte), 'switch_to_cooling_output_2' (Byte), 'switch_to_cooling_output_3' (Byte), 'switch_to_cooling_value_0' (Byte), 'switch_to_cooling_value_1' (Byte), 'switch_to_cooling_value_2' (Byte), 'switch_to_cooling_value_3' (Byte), 'switch_to_heating_output_0' (Byte), 'switch_to_heating_output_1' (Byte), 'switch_to_heating_output_2' (Byte), 'switch_to_heating_output_3' (Byte), 'switch_to_heating_value_0' (Byte), 'switch_to_heating_value_1' (Byte), 'switch_to_heating_value_2' (Byte), 'switch_to_heating_value_3' (Byte), 'threshold_temp' (Temp)
        """
        self.__eeprom_controller.write(GlobalThermostatConfiguration.from_dict(config))

    def get_can_led_configuration(self, can_led_id, fields=None):
        """
        Get a specific can_led_configuration defined by its id.

        :param can_led_id: The id of the can_led_configuration
        :type can_led_id: Id
        :param fields: The field of the can_led_configuration to get. (None gets all fields)
        :type fields: List of strings
        :returns: can_led_configuration dict: contains 'id' (Id), 'can_led_1_function' (Enum), 'can_led_1_id' (Byte), 'can_led_2_function' (Enum), 'can_led_2_id' (Byte), 'can_led_3_function' (Enum), 'can_led_3_id' (Byte), 'can_led_4_function' (Enum), 'can_led_4_id' (Byte), 'room' (Byte)
        """
        return self.__eeprom_controller.read(CanLedConfiguration, can_led_id, fields).to_dict()

    def get_can_led_configurations(self, fields=None):
        """
        Get all can_led_configurations.

        :param fields: The field of the can_led_configuration to get. (None gets all fields)
        :type fields: List of strings
        :returns: list of can_led_configuration dict: contains 'id' (Id), 'can_led_1_function' (Enum), 'can_led_1_id' (Byte), 'can_led_2_function' (Enum), 'can_led_2_id' (Byte), 'can_led_3_function' (Enum), 'can_led_3_id' (Byte), 'can_led_4_function' (Enum), 'can_led_4_id' (Byte), 'room' (Byte)
        """
        return [o.to_dict() for o in self.__eeprom_controller.read_all(CanLedConfiguration, fields)]

    def set_can_led_configuration(self, config):
        """
        Set one can_led_configuration.

        :param config: The can_led_configuration to set
        :type config: can_led_configuration dict: contains 'id' (Id), 'can_led_1_function' (Enum), 'can_led_1_id' (Byte), 'can_led_2_function' (Enum), 'can_led_2_id' (Byte), 'can_led_3_function' (Enum), 'can_led_3_id' (Byte), 'can_led_4_function' (Enum), 'can_led_4_id' (Byte), 'room' (Byte)
        """
        self.__eeprom_controller.write(CanLedConfiguration.from_dict(config))

    def set_can_led_configurations(self, config):
        """
        Set multiple can_led_configurations.

        :param config: The list of can_led_configurations to set
        :type config: list of can_led_configuration dict: contains 'id' (Id), 'can_led_1_function' (Enum), 'can_led_1_id' (Byte), 'can_led_2_function' (Enum), 'can_led_2_id' (Byte), 'can_led_3_function' (Enum), 'can_led_3_id' (Byte), 'can_led_4_function' (Enum), 'can_led_4_id' (Byte), 'room' (Byte)
        """
        self.__eeprom_controller.write_batch([CanLedConfiguration.from_dict(o) for o in config])

    def get_room_configuration(self, room_id, fields=None):
        """
        Get a specific room_configuration defined by its id.

        :param room_id: The id of the room_configuration
        :type room_id: Id
        :param fields: The field of the room_configuration to get. (None gets all fields)
        :type fields: List of strings
        :returns: room_configuration dict: contains 'id' (Id), 'floor' (Byte), 'name' (String)
        """
        return self.__eeprom_controller.read(RoomConfiguration, room_id, fields).to_dict()

    def get_room_configurations(self, fields=None):
        """
        Get all room_configurations.

        :param fields: The field of the room_configuration to get. (None gets all fields)
        :type fields: List of strings
        :returns: list of room_configuration dict: contains 'id' (Id), 'floor' (Byte), 'name' (String)
        """
        return [o.to_dict() for o in self.__eeprom_controller.read_all(RoomConfiguration, fields)]

    def set_room_configuration(self, config):
        """
        Set one room_configuration.

        :param config: The room_configuration to set
        :type config: room_configuration dict: contains 'id' (Id), 'floor' (Byte), 'name' (String)
        """
        self.__eeprom_controller.write(RoomConfiguration.from_dict(config))

    def set_room_configurations(self, config):
        """
        Set multiple room_configurations.

        :param config: The list of room_configurations to set
        :type config: list of room_configuration dict: contains 'id' (Id), 'floor' (Byte), 'name' (String)
        """
        self.__eeprom_controller.write_batch([RoomConfiguration.from_dict(o) for o in config])

    # End of auto generated functions

    # Power functions

    def get_power_modules(self):
        """ Get information on the power modules.

        :returns: List of dict depending on the version of the power module. All versions \
        contain 'id', 'name', 'input0', 'input1', 'input2', 'input3', 'input4', 'input5', \
        'input6', 'input7', 'times0', 'times1', 'times2', 'times3', 'times4', 'times5', 'times6', \
        'times7'. For the 8-port power it also contains 'sensor0', 'sensor1', 'sensor2', \
        'sensor3', 'sensor4', 'sensor5', 'sensor6', 'sensor7'. For the 12-port power module also \
        contains 'input8', 'input9', 'input10', 'input11', 'times8', 'times9', 'times10', \
        'times11'.
        """
        modules = self.__power_controller.get_power_modules().values()

        def translate_address(_module):
            """ Translate the address from an integer to the external address format (eg. E1). """
            _module['address'] = "E" + str(_module['address'])
            return _module

        return [translate_address(module) for module in modules]

    def set_power_modules(self, modules):
        """ Set information for the power modules.

        :param modules: list of dict depending on the version of the power module. All versions \
        contain 'id', 'name', 'input0', 'input1', 'input2', 'input3', 'input4', 'input5', \
        'input6', 'input7', 'times0', 'times1', 'times2', 'times3', 'times4', 'times5', 'times6', \
        'times7'. For the 8-port power it also contains 'sensor0', 'sensor1', 'sensor2', \
        'sensor3', 'sensor4', 'sensor5', 'sensor6', 'sensor7'. For the 12-port power module also \
        contains 'input8', 'input9', 'input10', 'input11', 'times8', 'times9', 'times10', \
        'times11'.
        :returns: empty dict.
        """
        for module in modules:
            self.__power_controller.update_power_module(module)

            version = self.__power_controller.get_version(module['id'])
            addr = self.__power_controller.get_address(module['id'])
            if version == power_api.POWER_API_8_PORTS:
                # 2 = 25A, 3 = 50A
                self.__power_communicator.do_command(
                    addr, power_api.set_sensor_types(version),
                    *[module['sensor%d' % i] for i in xrange(power_api.NUM_PORTS[version])]
                )
            elif version == power_api.POWER_API_12_PORTS:
                def _convert_ccf(key):
                    if module[key] == 2:  # 12.5A
                        return 0.5
                    if module[key] == 3:  # 25A
                        return 1
                    if module[key] == 4:  # 50A
                        return 2
                    if module[key] == 5:  # 100A
                        return 4
                    return 0.5  # 12.5A is default
                self.__power_communicator.do_command(
                    addr, power_api.set_current_clamp_factor(version),
                    *[_convert_ccf('sensor%d' % i) for i in xrange(power_api.NUM_PORTS[version])]
                )

                def _convert_sci(key):
                    if key not in module:
                        return 0
                    return 1 if module[key] in [True, 1] else 0
                self.__power_communicator.do_command(
                    addr, power_api.set_current_inverse(version),
                    *[_convert_sci('inverted%d' % i) for i in xrange(power_api.NUM_PORTS[version])]
                )
            else:
                raise ValueError('Unknown power api version')

        return dict()

    def get_realtime_power(self):
        """ Get the realtime power measurement values.

        :returns: dict with the module id as key and the following array as value: \
        [voltage, frequency, current, power].
        """
        output = dict()

        modules = self.__power_controller.get_power_modules()
        for module_id in sorted(modules.keys()):
            try:
                addr = modules[module_id]['address']
                version = modules[module_id]['version']
                num_ports = power_api.NUM_PORTS[version]

                if version == power_api.POWER_API_8_PORTS:
                    raw_volt = self.__power_communicator.do_command(addr,
                                                                    power_api.get_voltage(version))
                    raw_freq = self.__power_communicator.do_command(addr,
                                                                    power_api.get_frequency(version))

                    volt = [raw_volt[0] for _ in range(num_ports)]
                    freq = [raw_freq[0] for _ in range(num_ports)]

                elif version == power_api.POWER_API_12_PORTS:
                    volt = self.__power_communicator.do_command(addr,
                                                                power_api.get_voltage(version))
                    freq = self.__power_communicator.do_command(addr,
                                                                power_api.get_frequency(version))
                else:
                    raise ValueError("Unknown power api version")

                current = self.__power_communicator.do_command(addr,
                                                               power_api.get_current(version))
                power = self.__power_communicator.do_command(addr,
                                                             power_api.get_power(version))

                out = []
                for i in range(num_ports):
                    out.append([convert_nan(volt[i]), convert_nan(freq[i]),
                                convert_nan(current[i]), convert_nan(power[i])])

                output[str(module_id)] = out
            except Exception as ex:
                LOGGER.exception("Got Exception for power module %s: %s", module_id, ex)

        return output

    def get_total_energy(self):
        """ Get the total energy (kWh) consumed by the power modules.

        :returns: dict with the module id as key and the following array as value: [day, night].
        """
        output = dict()

        modules = self.__power_controller.get_power_modules()
        for module_id in sorted(modules.keys()):
            try:
                addr = modules[module_id]['address']
                version = modules[module_id]['version']

                day = self.__power_communicator.do_command(addr,
                                                           power_api.get_day_energy(version))
                night = self.__power_communicator.do_command(addr,
                                                             power_api.get_night_energy(version))

                out = []
                for i in range(power_api.NUM_PORTS[version]):
                    out.append([convert_nan(day[i]), convert_nan(night[i])])

                output[str(module_id)] = out
            except Exception as ex:
                LOGGER.exception("Got Exception for power module %s: %s", module_id, ex)

        return output

    def start_power_address_mode(self):
        """ Start the address mode on the power modules.

        :returns: empty dict.
        """
        self.__power_communicator.start_address_mode()
        return dict()

    def stop_power_address_mode(self):
        """ Stop the address mode on the power modules.

        :returns: empty dict
        """
        self.__power_communicator.stop_address_mode()
        return dict()

    def in_power_address_mode(self):
        """ Check if the power modules are in address mode

        :returns: dict with key 'address_mode' and value True or False.
        """
        return {'address_mode': self.__power_communicator.in_address_mode()}

    def set_power_voltage(self, module_id, voltage):
        """ Set the voltage for a given module.

        :param module_id: The id of the power module.
        :param voltage: The voltage to set for the power module.
        :returns: empty dict
        """
        addr = self.__power_controller.get_address(module_id)
        version = self.__power_controller.get_version(module_id)
        if version != power_api.POWER_API_12_PORTS:
            raise ValueError('Unknown power api version')
        self.__power_communicator.do_command(addr, power_api.set_voltage(), voltage)
        return dict()

    def get_energy_time(self, module_id, input_id=None):
        """ Get a 'time' sample of voltage and current

        :returns: dict with input_id and the voltage and cucrrent time samples
        """
        addr = self.__power_controller.get_address(module_id)
        version = self.__power_controller.get_version(module_id)
        if version != power_api.POWER_API_12_PORTS:
            raise ValueError('Unknown power api version')
        if input_id is None:
            input_ids = range(12)
        else:
            input_id = int(input_id)
            if input_id < 0 or input_id > 11:
                raise ValueError('Invalid input_id (should be 0-11)')
            input_ids = [input_id]
        data = {}
        for input_id in input_ids:
            voltage = list(self.__power_communicator.do_command(addr, power_api.get_voltage_sample_time(version), input_id, 0))
            current = list(self.__power_communicator.do_command(addr, power_api.get_current_sample_time(version), input_id, 0))
            for entry in self.__power_communicator.do_command(addr, power_api.get_voltage_sample_time(version), input_id, 1):
                if entry == float('inf'):
                    break
                voltage.append(entry)
            for entry in self.__power_communicator.do_command(addr, power_api.get_current_sample_time(version), input_id, 1):
                if entry == float('inf'):
                    break
                current.append(entry)
            data[str(input_id)] = {'voltage': voltage,
                                   'current': current}
        return data

    def get_energy_frequency(self, module_id, input_id=None):
        """ Get a 'frequency' sample of voltage and current

        :returns: dict with input_id and the voltage and cucrrent frequency samples
        """
        addr = self.__power_controller.get_address(module_id)
        version = self.__power_controller.get_version(module_id)
        if version != power_api.POWER_API_12_PORTS:
            raise ValueError('Unknown power api version')
        if input_id is None:
            input_ids = range(12)
        else:
            input_id = int(input_id)
            if input_id < 0 or input_id > 11:
                raise ValueError('Invalid input_id (should be 0-11)')
            input_ids = [input_id]
        data = {}
        for input_id in input_ids:
            voltage = self.__power_communicator.do_command(addr, power_api.get_voltage_sample_frequency(version), input_id, 20)
            current = self.__power_communicator.do_command(addr, power_api.get_current_sample_frequency(version), input_id, 20)
            # The received data has a length of 40; 20 harmonics entries, and 20 phase entries. For easier usage, the
            # API calls splits them into two parts so the customers doesn't have to do the splitting.
            data[str(input_id)] = {'voltage': [voltage[:20], voltage[20:]],
                                   'current': [current[:20], current[20:]]}
        return data

    def do_raw_energy_command(self, address, mode, command, data):
        """ Perform a raw energy module command, for debugging purposes.

        :param address: The address of the energy module
        :param mode: 1 char: S or G
        :param command: 3 char power command
        :param data: list of bytes
        :returns: list of bytes
        """
        return self.__power_communicator.do_command(address,
                                                    power_api.raw_command(mode, command, len(data)),
                                                    *data)
