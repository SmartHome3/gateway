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
The vpn_service asks the OpenMotics cloud it a vpn tunnel should be opened. It start openvpn
if required. On each check the vpn_service sends some status information about the outputs and
thermostats to the cloud, to keep the status information in the cloud in sync.
"""

import requests
import time
import subprocess
import os
import traceback
from threading import Thread
from ConfigParser import ConfigParser
from datetime import datetime
try:
    import json
except ImportError:
    import simplejson as json

import constants

from bus.led_service import LedService

REBOOT_TIMEOUT = 900


def reboot_gateway():
    """ Reboot the gateway. """
    subprocess.call('sync && reboot', shell=True)


class VpnController(object):
    """ Contains methods to check the vpn status, start and stop the vpn. """

    vpnService = "openvpn.service"
    startCmd = "systemctl start " + vpnService
    stopCmd = "systemctl stop " + vpnService
    checkCmd = "systemctl is-active " + vpnService

    def __init__(self):
        pass

    @staticmethod
    def start_vpn():
        """ Start openvpn """
        return subprocess.call(VpnController.startCmd, shell=True) == 0

    @staticmethod
    def stop_vpn():
        """ Stop openvpn """
        return subprocess.call(VpnController.stopCmd, shell=True) == 0

    @staticmethod
    def check_vpn():
        """ Check if openvpn is running """
        return subprocess.call(VpnController.checkCmd, shell=True) == 0


class Cloud(object):
    """ Connects to the OpenMotics cloud to check if the vpn should be opened. """

    DEFAULT_SLEEP_TIME = 30

    def __init__(self, url, led_service, action_executor, sleep_time=DEFAULT_SLEEP_TIME):
        self.__url = url
        self.__led_service = led_service
        self.__action_executor = action_executor
        self.__last_connect = time.time()
        self.__sleep_time = sleep_time
        self.__modes = None

    def should_open_vpn(self, extra_data):
        """ Check with the OpenMotics could if we should open a VPN """
        try:
            request = requests.post(self.__url, data={'extra_data': json.dumps(extra_data)},
                                    timeout=10.0, verify=True)
            data = json.loads(request.text)

            if 'sleep_time' in data:
                self.__sleep_time = data['sleep_time']
            else:
                self.__sleep_time = Cloud.DEFAULT_SLEEP_TIME

            if 'actions' in data:
                self.__action_executor.execute_actions_in_background(data['actions'])

            if 'modes' in data:
                self.__modes = data['modes']
            else:
                self.__modes = None

            self.__led_service.set_led('cloud', True)
            self.__led_service.toggle_led('alive')
            self.__last_connect = time.time()

            return True, data['open_vpn']
        except Exception as exception:
            print "Exception occured during check: ", exception
            self.__led_service.set_led('cloud', False)
            self.__led_service.set_led('alive', False)

            return False, True

    def get_sleep_time(self):
        """ Get the time to sleep between two cloud checks. """
        return self.__sleep_time

    def get_current_modes(self):
        """ Get the current modes of the cloud. """
        return self.__modes

    def get_last_connect(self):
        """ Get the timestamp of the last connection with the cloud. """
        return self.__last_connect


class Gateway(object):
    """ Class to get the current status of the gateway. """

    def __init__(self, host="127.0.0.1"):
        self.__host = host
        self.__last_pulse_counters = None

    def do_call(self, uri):
        """ Do a call to the webservice, returns a dict parsed from the json returned by the
        webserver. """
        try:
            request = requests.get("http://" + self.__host + "/" + uri, timeout=15.0)
            return json.loads(request.text)
        except Exception as exception:
            print "Exception during Gateway call: ", exception
            return None

    def get_enabled_outputs(self):
        """ Get the enabled outputs.

        :returns: a list of tuples containing the output number and dimmer value. None on error.
        """
        data = self.do_call("get_output_status?token=None")
        if data is None or data['success'] is False:
            return None
        else:
            ret = []
            for output in data['status']:
                if output["status"] == 1:
                    ret.append((output["id"], output["dimmer"]))
            return ret

    def get_thermostats(self):
        """ Fetch the setpoints for the enabled thermostats from the webservice.

        :returns: a dict with 'thermostats_on', 'automatic' and an array of dicts in 'status'
        with the following fields: 'id', 'act', 'csetp', 'output0', 'output1' and 'mode'.
        None on error.
        """
        data = self.do_call("get_thermostat_status?token=None")
        if data is None or data['success'] is False:
            return None
        else:
            ret = {'thermostats_on': data['thermostats_on'],
                   'automatic': data['automatic'],
                   'cooling': data['cooling']}
            thermostats = []
            for thermostat in data['status']:
                to_add = {}
                for field in ['id', 'act', 'csetp', 'mode', 'output0', 'output1', 'outside', 'airco']:
                    to_add[field] = thermostat[field]
                thermostats.append(to_add)
            ret['status'] = thermostats
            return ret

    def get_update_status(self):
        """ Get the status of an executing update. """
        _ = self  # Needs to be an instance method
        filename = '/opt/openmotics/update_status'
        if os.path.exists(filename):
            update_status_file = open(filename, 'r')
            status = update_status_file.read()
            update_status_file.close()
            if status.endswith('DONE\n'):
                os.remove(filename)
            return status
        else:
            return None

    def get_real_time_power(self):
        """ Get the real time power measurements. """
        data = self.do_call("get_realtime_power?token=None")
        if data is None or data['success'] is False:
            return None
        else:
            del data['success']
            return data

    def get_total_energy(self):
        """ Get the total energy. """
        data = self.do_call("get_total_energy?token=None")
        if data is None or data['success'] is False:
            return None
        else:
            del data['success']
            return data

    def get_pulse_counter_status(self):
        """ Get the total pulse counter values. """
        data = self.do_call("get_pulse_counter_status?token=None")
        if data is None or data['success'] is False:
            return None
        else:
            return data['counters']

    def get_pulse_counter_diff(self):
        """ Get the pulse counter differences. """
        data = self.do_call("get_pulse_counter_status?token=None")
        if data is None or data['success'] is False:
            return None
        else:
            counters = data['counters']

            if self.__last_pulse_counters is None:
                ret = [0 for _ in range(0, 24)]
            else:
                ret = [Gateway.__counter_diff(counters[i], self.__last_pulse_counters[i])
                       for i in range(0, 24)]

            self.__last_pulse_counters = counters
            return ret

    @staticmethod
    def __counter_diff(current, previous):
        """ Calculate the diff between two counter values. """
        diff = current - previous
        return diff if diff >= 0 else 65536 - previous + current

    def get_errors(self):
        """ Get the errors on the gateway. """
        data = self.do_call("get_errors?token=None")
        if data is None:
            return None
        else:
            if data['errors'] is not None:
                master_errors = sum([error[1] for error in data['errors']])
            else:
                master_errors = 0

            return {'master_errors': master_errors,
                    'master_last_success': data['master_last_success'],
                    'power_last_success': data['power_last_success']}

    def get_local_ip_address(self):
        """ Get the local ip address. """
        _ = self  # Needs to be an instance method
        try:
            lines = subprocess.check_output("ifconfig eth0", shell=True)
            return lines.split("\n")[1].strip().split(" ")[1].split(":")[1]
        except:
            return None

    def get_modules(self):
        """ Get the modules known by the master.
        :returns: a list of characters. The master module (M), the output modules (O, D or R),
        the input modules (I or T), the shutter modules (S), followed by the power modules (P).
        """
        data = self.do_call("get_modules?token=None")
        power_data = self.do_call("get_power_modules?token=None")

        data_failed = data is None or data['success'] is False
        power_failed = power_data is None or power_data['success'] is False

        if data_failed and power_failed:
            return None
        else:
            ret = []

            if not data_failed:
                ret.append('M')
                for mod in data['outputs']:
                    ret.append(str(mod))
                for mod in data['inputs']:
                    ret.append(str(mod))
                for mod in data['shutters']:
                    ret.append(str(mod))

            if not power_failed:
                for _ in range(len(power_data['modules'])):
                    ret.append('P')

            return ret

    def get_module_log(self):
        """ Get the module log.
        :returns: a list of tuples (log_level, message).
        """
        data = self.do_call("get_module_log?token=None")
        if data is None or data['success'] is False:
            return None
        else:
            return data['log']

    def get_last_inputs(self):
        """ Get the last pressed inputs.
        :returns: a list of input ids.
        """
        data = self.do_call("get_last_inputs?token=None")
        if data is None or data['success'] is False:
            return None
        else:
            return [t[0] for t in data['inputs']]

    def get_sensor_temperature_status(self):
        """ Get the temperature measured of the sensors.
        :returns: a list of temperatures.
        """
        data = self.do_call("get_sensor_temperature_status?token=None")
        if data is None or data['success'] is False:
            return None
        else:
            return data['status']

    def get_sensor_humidity_status(self):
        """ Get the humidity measured by the sensors.
        :returns: a list of humidity values.
        """
        data = self.do_call("get_sensor_humidity_status?token=None")
        if data is None or data['success'] is False:
            return None
        else:
            return data['status']

    def get_sensor_brightness_status(self):
        """ Get the brightness measured by the sensors.
        :returns: a list of brightness values.
        """
        data = self.do_call("get_sensor_brightness_status?token=None")
        if data is None or data['success'] is False:
            return None
        else:
            return data['status']


class DataCollector(object):
    """ Defines a function to retrieve data, the period between two collections. If a mode is
    specified, the collector will only collect data if the mode is enabled.
    """

    def __init__(self, function, period=0, mode=None):
        """
        Create a collector with a function to call and a period.
        If a mode is provided the collector will only run if that mode is enabled.

        If the period is 0, the collector will be executed on each call.
        """
        self.__function = function
        self.__period = period
        self.__last_collect = 0
        self.__mode = mode

    def __should_collect(self, current_modes):
        """ Should we execute the collect ? """
        if self.__mode is not None and (current_modes is None or self.__mode not in current_modes):
            return False

        return self.__period == 0 or time.time() >= self.__last_collect + self.__period

    def collect(self, current_modes):
        """ Execute the collect if required, return None otherwise. """
        try:
            if self.__should_collect(current_modes):
                if self.__period != 0:
                    self.__last_collect = time.time()
                return self.__function()
            else:
                return None
        except Exception as exception:
            print "Exception while collecting data: ", exception
            traceback.print_exc()
            return None


class BufferingDataCollector(DataCollector):
    """ Defines a Collector that buffers data when it cannot be sent to the cloud. """

    BUFFER_SIZE = 2500  # Elements in the in-memory buffer
    FILE_SIZE = 50 * 1024 * 1024  # Max size of the on-disk buffer

    def __init__(self, function, period=0, mode=None):
        DataCollector.__init__(self, function, period, mode)
        self.__name = function.__name__
        self.__buffer_path = constants.get_buffer_file(self.__name)
        self.__buffer = []
        self.__read_buffer()
        self.__last_point = None

    def __read_buffer(self):
        if os.path.exists(self.__buffer_path):
            try:
                f = open(self.__buffer_path, "r")
                for line in f:
                    self.__append_to_buffer(json.loads(line))
                f.close()
            except Exception as e:
                print "Exception while reading buffer %s : %s" % (self.__buffer_path, e)

    def collect(self, current_modes):
        """ Execute the collect if required, return None otherwise. """
        point = DataCollector.collect(self, current_modes)
        if point is not None:
            self.__last_point = [time.time(), point]
            self.__append_to_buffer(self.__last_point)
            return {'timestamp': time.time(), 'values': self.__buffer}
        else:
            return None

    def data_sent_callback(self, success):
        """ A function that should be called after each collection, to notify the collector that
        the data was sent succesfully or failed. The BufferingDataCollector buffers the data when
        sending the data failed. If the callback is not called, the data will not be buffered. """
        if success:
            self.__buffer = []
            if os.path.exists(self.__buffer_path):
                os.remove(self.__buffer_path)

        elif self.__last_point is not None:
            self.__append_to_file(self.__last_point)
            self.__last_point = None

    def __append_to_buffer(self, element):
        """ Append an element to the buffer, limits the size of the in-memory buffer to BUFFER_SIZE
        elements. """
        self.__buffer.append(element)

        if len(self.__buffer) > BufferingDataCollector.BUFFER_SIZE:
                self.__buffer = \
                    self.__buffer[len(self.__buffer) - BufferingDataCollector.BUFFER_SIZE:]

    def __append_to_file(self, element):
        """ Append an element to the file buffer, limits the size of the buffer to FILE_SIZE. """
        f = open(self.__buffer_path, "a")
        f.write("%s\n" % json.dumps(element))
        f.close()

        # Keep the size of the file limited. When the maximum file size is reached, a new
        # file of half the size if created. The limits the amount of writes on the flash disk.
        if os.stat(self.__buffer_path).st_size > BufferingDataCollector.FILE_SIZE:
            old = open(self.__buffer_path, "r")
            new = open(self.__buffer_path + ".new", "w")

            skipped = 0
            for line in old:
                if skipped > BufferingDataCollector.FILE_SIZE / 2:
                    new.write(line)
                else:
                    skipped += len(line)

            new.close()
            old.close()

            os.remove(self.__buffer_path)
            os.rename(self.__buffer_path + ".new", self.__buffer_path)


class ActionExecutor(object):
    """ Executes actions received from the cloud. """

    def __init__(self, gateway):
        """ Use a Gateway instance to communicate with the gateway. """
        self.__gateway = gateway

    def execute_actions_in_background(self, actions):
        """ Execute a list of actions in the background. """
        def run():
            """ Function that executes a list of actions. """
            for action in actions:
                try:
                    self.execute(action)
                except Exception as exception:
                    print "Error wile executing action '" + str(action) + "': " + str(exception)

        thread = Thread(name="Action Executor", target=run)
        thread.daemon = True
        thread.start()

    def execute(self, action):
        """ Execute an action. """
        name = action.get('name', None)
        args = action.get('args', None)

        if name == 'set_output':
            self.__gateway.do_call("set_output?id=%s&on=%s&dimmer=%s&timer=%s&token=None" %
                                   (args['id'], args['on'], args['dimmer'], args['timer']))

        elif name == 'set_all_lights_off':
            self.__gateway.do_call("set_all_lights_off?token=None")

        elif name == 'set_all_lights_floor_off':
            self.__gateway.do_call("set_all_lights_floor_off?floor=%s&token=None" % args['floor'])

        elif name == 'set_all_lights_floor_on':
            self.__gateway.do_call("set_all_lights_floor_on?floor=%s&token=None" % args['floor'])

        elif name == 'set_current_setpoint':
            self.__gateway.do_call(
                    "set_current_setpoint?thermostat=%s&temperature=%s&token=None" %
                    (args['thermostat'], args['temperature']))

        elif name == 'set_mode':
            self.__gateway.do_call("set_mode?on=%s&automatic=%s&setpoint=%s&token=None" %
                                   (args['on'], args['automatic'], args['setpoint']))

        elif name == 'do_group_action':
            self.__gateway.do_call("do_group_action?group_action_id=%s&token=None" %
                                   args['group_action_id'])

        else:
            raise Exception("Could not find action '%s'" % name)


def main():
    """ The main function contains the loop that check if the vpn should be opened every 2 seconds.
    Status data is sent when the vpn is checked. """

    led_service = LedService()

    # Get the configuration
    config = ConfigParser()
    config.read(constants.get_config_file())

    check_url = config.get('OpenMotics', 'vpn_check_url') % config.get('OpenMotics', 'uuid')

    gateway = Gateway()
    cloud = Cloud(check_url, led_service, ActionExecutor(gateway))

    collectors = {'energy': BufferingDataCollector(gateway.get_total_energy, 300),
                  'thermostats': DataCollector(gateway.get_thermostats, 60),
                  'pulse_totals': BufferingDataCollector(gateway.get_pulse_counter_status, 300),
                  'pulses': DataCollector(gateway.get_pulse_counter_diff, 60),
                  'outputs': DataCollector(gateway.get_enabled_outputs, mode='rt'),
                  'power': DataCollector(gateway.get_real_time_power, mode='rt'),
                  'update': DataCollector(gateway.get_update_status),
                  'errors': DataCollector(gateway.get_errors, 600),
                  'local_ip': DataCollector(gateway.get_local_ip_address, 1800),
                  'modules': DataCollector(gateway.get_modules, mode='init'),
                  'module_log': DataCollector(gateway.get_module_log, mode='init'),
                  'last_inputs': DataCollector(gateway.get_last_inputs, mode='init'),
                  'sensor_tmp': DataCollector(gateway.get_sensor_temperature_status, 10,
                                              mode='init'),
                  'sensor_hum': DataCollector(gateway.get_sensor_humidity_status, 10,
                                              mode='init'),
                  'sensor_bri': DataCollector(gateway.get_sensor_brightness_status, 10,
                                              mode='init')}

    iterations = 0

    # Loop: check vpn and open/close if needed
    while True:
        vpn_data = {}
        for collector_name in collectors:
            collector = collectors[collector_name]
            data = collector.collect(cloud.get_current_modes())
            if data is not None:
                vpn_data[collector_name] = data

        (success, should_open) = cloud.should_open_vpn(vpn_data)

        for collector_name in vpn_data.keys():
            collector = collectors[collector_name]
            if type(collector) == BufferingDataCollector:
                collector.data_sent_callback(success)

        if iterations > 20 and cloud.get_last_connect() < time.time() - REBOOT_TIMEOUT:
            # The cloud is not responding for a while, perhaps the BeagleBone network stack is
            # hanging, reboot the gateway to reset the BeagleBone.
            reboot_gateway()

        is_open = VpnController.check_vpn()
        led_service.set_led('vpn', is_open)

        if should_open and not is_open:
            print str(datetime.now()) + ": opening vpn"
            VpnController.start_vpn()
        elif not should_open and is_open:
            print str(datetime.now()) + ": closing vpn"
            VpnController.stop_vpn()

        print "Sleeping for %ds" % cloud.get_sleep_time()
        time.sleep(cloud.get_sleep_time())

        iterations += 1


if __name__ == '__main__':
    print "\nStarting VPN service\n"
    main()
