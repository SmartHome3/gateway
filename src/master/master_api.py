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
Contains the definition of the Master Api.
Requires Firmware 3.137.17 -- Caution on update: status api changed (used in update mechanism !)

@author: fryckbos
"""

from master_command import MasterCommandSpec, Field, OutputFieldType, DimmerFieldType, \
                           ErrorListFieldType

BA_GROUP_ACTION = 2

BA_TRIGGER_EVENT = 60
BA_STATUS_LEDS = 64

BA_THERMOSTAT_COOLING_HEATING = 80
BA_THERMOSTAT_AIRCO_STATUS = 81
BA_SET_PERMANENT_MANUAL_MODE = 82
BA_CLEAR_PERMANENT_MANUAL_MODE = 83

BA_THERMOSTAT_TENANT_AUTO = 90
BA_THERMOSTAT_TENANT_MANUAL = 91

BA_SHUTTER_UP = 100
BA_SHUTTER_DOWN = 101
BA_SHUTTER_STOP = 102
BA_SHUTTER_GROUP_UP = 104
BA_SHUTTER_GROUP_DOWN = 105
BA_SHUTTER_GROUP_STOP = 106

BA_ONE_SETPOINT_0 = 128
BA_ONE_SETPOINT_1 = 129
BA_ONE_SETPOINT_2 = 130
BA_ONE_SETPOINT_3 = 131
BA_ONE_SETPOINT_4 = 132
BA_ONE_SETPOINT_5 = 133

BA_ALL_SETPOINT_0 = 134
BA_ALL_SETPOINT_1 = 135
BA_ALL_SETPOINT_2 = 136
BA_ALL_SETPOINT_3 = 137
BA_ALL_SETPOINT_4 = 138
BA_ALL_SETPOINT_5 = 139

BA_THERMOSTAT_MODE = 140
BA_THERMOSTAT_AUTOMATIC = 141

BA_LIGHT_OFF = 160
BA_LIGHT_ON = 161
BA_LIGHT_TOGGLE = 162
BA_ALL_LIGHTS_OFF = 163
BA_ALL_OUTPUTS_OFF = 164
BA_LIGHT_ON_DIMMER_MIN = 165
BA_LIGHT_ON_DIMMER_MAX = 166
BA_LIGHT_ON_DIMMER_PLUS_5 = 167
BA_LIGHTS_ON_DIMMER_MIN_5 = 168
BA_DIMMER_MIN = 169
BA_DIMMER_MAX = 170
BA_LIGHTS_OFF_FLOOR = 171
BA_LIGHTS_ON_FLOOR = 172
BA_LIGHTS_TOGGLE_FLOOR = 173
BA_LIGHT_ON_DIMMER_10 = 176
BA_LIGHT_ON_DIMMER_20 = 177
BA_LIGHT_ON_DIMMER_30 = 178
BA_LIGHT_ON_DIMMER_40 = 179
BA_LIGHT_ON_DIMMER_50 = 180
BA_LIGHT_ON_DIMMER_60 = 181
BA_LIGHT_ON_DIMMER_70 = 182
BA_LIGHT_ON_DIMMER_80 = 183
BA_LIGHT_ON_DIMMER_90 = 184
BA_LIGHT_TOGGLE_DIMMER_10 = 185
BA_LIGHT_TOGGLE_DIMMER_20 = 186
BA_LIGHT_TOGGLE_DIMMER_30 = 187
BA_LIGHT_TOGGLE_DIMMER_40 = 188
BA_LIGHT_TOGGLE_DIMMER_50 = 189
BA_LIGHT_TOGGLE_DIMMER_60 = 190
BA_LIGHT_TOGGLE_DIMMER_70 = 191
BA_LIGHT_TOGGLE_DIMMER_80 = 192
BA_LIGHT_TOGGLE_DIMMER_90 = 193
BA_LIGHT_TOGGLE_DIMMER_100 = 194
BA_LIGHT_ON_TIMER_150_OVERRULE = 195
BA_LIGHT_ON_TIMER_450_OVERRULE = 196
BA_LIGHT_ON_TIMER_900_OVERRULE = 197
BA_LIGHT_ON_TIMER_1500_OVERRULE = 198
BA_LIGHT_ON_TIMER_2220_OVERRULE = 199
BA_LIGHT_ON_TIMER_3120_OVERRULE = 200
BA_LIGHT_ON_TIMER_150_NO_OVERRULE = 201
BA_LIGHT_ON_TIMER_450_NO_OVERRULE = 202
BA_LIGHT_ON_TIMER_900_NO_OVERRULE = 203
BA_LIGHT_ON_TIMER_1500_NO_OVERRULE = 204
BA_LIGHT_ON_TIMER_2220_NO_OVERRULE = 205
BA_LIGHT_ON_TIMER_3120_NO_OVERRULE = 206


def basic_action():
    """ Basic actions. """
    return MasterCommandSpec("BA",
        [Field.byte("action_type"), Field.byte("action_number"), Field.padding(11)],
        [Field.str("resp", 2), Field.padding(11), Field.lit("\r\n")])

def reset():
    """ Reset the gateway, used for firmware updates. """
    return MasterCommandSpec("re",
        [Field.padding(13)],
        [Field.str("resp", 2), Field.padding(11), Field.lit("\r\n")])

def status():
    """ Get the status of the master. """
    return MasterCommandSpec("ST",
        [Field.padding(13)],
        [Field.byte('seconds'), Field.byte('minutes'), Field.byte('hours'), Field.byte('weekday'),
         Field.byte('day'), Field.byte('month'), Field.byte('year'), Field.lit('\x00'),
         Field.byte('mode'), Field.byte('f1'), Field.byte('f2'), Field.byte('f3'),
         Field.byte('h'), Field.lit('\r\n')])

def set_time():
    """ Set the time on the master. """
    return MasterCommandSpec("st",
        [Field.byte('sec'), Field.byte('min'), Field.byte('hours'), Field.byte('weekday'),
         Field.byte('day'), Field.byte('month'), Field.byte('year'), Field.padding(6)],
        [Field.byte('sec'), Field.byte('min'), Field.byte('hours'), Field.byte('weekday'),
         Field.byte('day'), Field.byte('month'), Field.byte('year'), Field.padding(6),
         Field.lit("\r\n")])

def eeprom_list():
    """ List all bytes from a certain eeprom bank """
    return MasterCommandSpec("EL",
        [Field.byte("bank"), Field.padding(12)],
        [Field.byte("bank"), Field.str("data", 256), Field.lit("\r\n")])

def read_eeprom():
    """ Read a number (1-10) of bytes from a certain eeprom bank and address. """
    return MasterCommandSpec("RE",
        [Field.byte('bank'), Field.byte('addr'), Field.byte('num'), Field.padding(10)],
        [Field.byte('bank'), Field.byte('addr'), Field.varstr('data', 10), Field.lit('\r\n')])

def write_eeprom():
    """ Write data bytes to the addr in the specified eeprom bank """
    return MasterCommandSpec("WE",
        [Field.byte("bank"), Field.byte("address"), Field.varstr("data", 10)],
        [Field.byte("bank"), Field.byte("address"), Field.varstr("data", 10), Field.lit('\r\n')])

def activate_eeprom():
    """ Activate eeprom after write """
    return MasterCommandSpec("AE",
        [Field.byte("eep"), Field.padding(12)],
        [Field.byte("eep"), Field.str("resp", 2), Field.padding(10), Field.lit('\r\n')])

def number_of_io_modules():
    """ Read the number of input and output modules """
    return MasterCommandSpec("rn",
        [Field.padding(13)],
        [Field.byte("in"), Field.byte("out"), Field.byte("shutter"), Field.padding(10),
         Field.lit('\r\n')])

def read_output():
    """ Read the information about an output """
    return MasterCommandSpec("ro",
        [Field.byte("id"), Field.padding(12)],
        [Field.byte('id'), Field.str('type', 1), Field.byte('light'), Field.int('timer'),
         Field.int('ctimer'), Field.byte('status'), Field.dimmer('dimmer'),
         Field.byte('controller_out'), Field.byte('max_power'), Field.byte('floor_level'),
         Field.bytes('menu_position', 3), Field.str('name', 16), Field.crc(),
         Field.lit('\r\n')])

def read_input():
    """ Read the information about an input """
    return MasterCommandSpec("ri",
        [Field.byte("input_nr"), Field.padding(12)],
        [Field.byte('input_nr'), Field.byte('output_action'), Field.bytes('output_list', 30),
         Field.str('input_name', 8), Field.crc(), Field.lit('\r\n')])

def shutter_status():
    """ Read the status of a shutter module. """
    return MasterCommandSpec("SO",
        [Field.byte("module_nr"), Field.padding(12)],
        [Field.byte("module_nr"), Field.padding(3), Field.byte("status"), Field.lit('\r\n')])

def temperature_list():
    """ Read the temperature thermostat sensor list for a series of 12 sensors """
    return MasterCommandSpec("TL",
        [Field.byte("series"), Field.padding(12)],
        [Field.byte("series"), Field.svt('tmp0'), Field.svt('tmp1'), Field.svt('tmp2'),
         Field.svt('tmp3'), Field.svt('tmp4'), Field.svt('tmp5'), Field.svt('tmp6'),
         Field.svt('tmp7'), Field.svt('tmp8'), Field.svt('tmp9'), Field.svt('tmp10'),
         Field.svt('tmp11'), Field.lit('\r\n')])

def setpoint_list():
    """ Read the current setpoint of the thermostats in series of 12 """
    return MasterCommandSpec("SL",
        [Field.byte("series"), Field.padding(12)],
        [Field.byte("series"), Field.svt('tmp0'), Field.svt('tmp1'), Field.svt('tmp2'),
         Field.svt('tmp3'), Field.svt('tmp4'), Field.svt('tmp5'), Field.svt('tmp6'),
         Field.svt('tmp7'), Field.svt('tmp8'), Field.svt('tmp9'), Field.svt('tmp10'),
         Field.svt('tmp11'), Field.lit('\r\n')])

def thermostat_mode():
    """ Read the current thermostat mode """
    return MasterCommandSpec("TM",
        [Field.padding(13)],
        [Field.byte('mode'), Field.padding(12), Field.lit('\r\n')])

def read_setpoint():
    """ Read the programmed setpoint of a thermostat """
    return MasterCommandSpec("rs",
        [Field.byte('thermostat'), Field.padding(12)],
        [Field.byte('thermostat'), Field.svt('act'), Field.svt('csetp'), Field.svt('psetp0'),
         Field.svt('psetp1'), Field.svt('psetp2'), Field.svt('psetp3'), Field.svt('psetp4'),
         Field.svt('psetp5'), Field.byte('sensor_nr'), Field.byte('output0_nr'),
         Field.byte('output1_nr'), Field.byte('output0'), Field.byte('output1'),
         Field.svt('outside'), Field.byte('mode'), Field.str('name', 16), Field.byte('pid_p'),
         Field.byte('pid_i'), Field.byte('pid_d'), Field.byte('pid_ithresh'),
         Field.svt('threshold_temp'), Field.byte('days'), Field.byte('hours'),
         Field.byte('minutes'), Field.svt('mon_start_d1'), Field.svt('mon_stop_d1'),
         Field.svt('mon_start_d2'), Field.svt('mon_stop_d2'), Field.svt('tue_start_d1'),
         Field.svt('tue_stop_d1'), Field.svt('tue_start_d2'), Field.svt('tue_stop_d2'),
         Field.svt('wed_start_d1'), Field.svt('wed_stop_d1'), Field.svt('wed_start_d2'),
         Field.svt('wed_stop_d2'), Field.svt('thu_start_d1'), Field.svt('thu_stop_d1'),
         Field.svt('thu_start_d2'), Field.svt('thu_stop_d2'), Field.svt('fri_start_d1'),
         Field.svt('fri_stop_d1'), Field.svt('fri_start_d2'), Field.svt('fri_stop_d2'),
         Field.svt('sat_start_d1'), Field.svt('sat_stop_d1'), Field.svt('sat_start_d2'),
         Field.svt('sat_stop_d2'), Field.svt('sun_start_d1'), Field.svt('sun_stop_d1'),
         Field.svt('sun_start_d2'), Field.svt('sun_stop_d2'), Field.lit('T'),
         Field.svt('mon_temp_d1'), Field.svt('tue_temp_d1'), Field.svt('wed_temp_d1'),
         Field.svt('thu_temp_d1'), Field.svt('fri_temp_d1'), Field.svt('sat_temp_d1'),
         Field.svt('sun_temp_d1'), Field.svt('mon_temp_d2'), Field.svt('tue_temp_d2'),
         Field.svt('wed_temp_d2'), Field.svt('thu_temp_d2'), Field.svt('fri_temp_d2'),
         Field.svt('sat_temp_d2'), Field.svt('sun_temp_d2'), Field.svt('mon_temp_n'),
         Field.svt('tue_temp_n'), Field.svt('wed_temp_n'), Field.svt('thu_temp_n'),
         Field.svt('fri_temp_n'), Field.svt('sat_temp_n'), Field.svt('sun_temp_n'),
         Field.crc(), Field.lit('\r\n')])

def write_setpoint():
    """ Write a setpoints of a thermostats """
    return MasterCommandSpec("ws",
        [Field.byte("thermostat"), Field.byte("config"), Field.svt("temp"), Field.padding(10)],
        [Field.byte("thermostat"), Field.byte("config"), Field.svt("temp"), Field.padding(10),
         Field.lit('\r\n')])

def permanent_manual_thermostat_list():
    """ Read the permanent manual bytes, 1 per thermostat. """
    return MasterCommandSpec("pL",
        [Field.padding(13)],
        [Field.byte('tm'),
         Field.byte('pmt0'), Field.byte('pmt1'), Field.byte('pmt2'), Field.byte('pmt3'),
         Field.byte('pmt4'), Field.byte('pmt5'), Field.byte('pmt6'), Field.byte('pmt7'),
         Field.byte('pmt8'), Field.byte('pmt9'), Field.byte('pmt10'), Field.byte('pmt11'),
         Field.byte('pmt12'), Field.byte('pmt13'), Field.byte('pmt14'), Field.byte('pmt15'),
         Field.byte('pmt16'), Field.byte('pmt17'), Field.byte('pmt18'), Field.byte('pmt19'),
         Field.byte('pmt20'), Field.byte('pmt21'), Field.byte('pmt22'), Field.byte('pmt23'),
         Field.byte('pmt24'), Field.byte('pmt25'), Field.byte('pmt26'), Field.byte('pmt27'),
         Field.byte('pmt28'), Field.byte('pmt29'), Field.byte('pmt30'), Field.byte('pmt31'),
         Field.crc(), Field.lit('\r\n')])

def thermostat_list():
    """ Read the thermostat mode, the outside temperature, the temperature of each thermostat,
    as well as the setpoint.
    """
    return MasterCommandSpec("tl",
        [Field.padding(13)],
        [Field.byte('mode'), Field.svt('outside'),
         Field.svt('tmp0'), Field.svt('tmp1'), Field.svt('tmp2'), Field.svt('tmp3'),
         Field.svt('tmp4'), Field.svt('tmp5'), Field.svt('tmp6'), Field.svt('tmp7'),
         Field.svt('tmp8'), Field.svt('tmp9'), Field.svt('tmp10'), Field.svt('tmp11'),
         Field.svt('tmp12'), Field.svt('tmp13'), Field.svt('tmp14'), Field.svt('tmp15'),
         Field.svt('tmp16'), Field.svt('tmp17'), Field.svt('tmp18'), Field.svt('tmp19'),
         Field.svt('tmp20'), Field.svt('tmp21'), Field.svt('tmp22'), Field.svt('tmp23'),
         Field.svt('tmp24'), Field.svt('tmp25'), Field.svt('tmp26'), Field.svt('tmp27'),
         Field.svt('tmp28'), Field.svt('tmp29'), Field.svt('tmp30'), Field.svt('tmp31'),
         Field.svt('setp0'), Field.svt('setp1'), Field.svt('setp2'), Field.svt('setp3'),
         Field.svt('setp4'), Field.svt('setp5'), Field.svt('setp6'), Field.svt('setp7'),
         Field.svt('setp8'), Field.svt('setp9'), Field.svt('setp10'), Field.svt('setp11'),
         Field.svt('setp12'), Field.svt('setp13'), Field.svt('setp14'), Field.svt('setp15'),
         Field.svt('setp16'), Field.svt('setp17'), Field.svt('setp18'), Field.svt('setp19'),
         Field.svt('setp20'), Field.svt('setp21'), Field.svt('setp22'), Field.svt('setp23'),
         Field.svt('setp24'), Field.svt('setp25'), Field.svt('setp26'), Field.svt('setp27'),
         Field.svt('setp28'), Field.svt('setp29'), Field.svt('setp30'), Field.svt('setp31'),
         Field.crc(), Field.lit('\r\n')])

def thermostat_mode_list():
    """ Read the thermostat mode for each thermostat. """
    return MasterCommandSpec("ml",
        [Field.padding(13)],
        [Field.byte('mode0'), Field.byte('mode1'), Field.byte('mode2'), Field.byte('mode3'),
         Field.byte('mode4'), Field.byte('mode5'), Field.byte('mode6'), Field.byte('mode7'),
         Field.byte('mode8'), Field.byte('mode9'), Field.byte('mode10'), Field.byte('mode11'),
         Field.byte('mode12'), Field.byte('mode13'), Field.byte('mode14'), Field.byte('mode15'),
         Field.byte('mode16'), Field.byte('mode17'), Field.byte('mode18'), Field.byte('mode19'),
         Field.byte('mode20'), Field.byte('mode21'), Field.byte('mode22'), Field.byte('mode23'),
         Field.byte('mode24'), Field.byte('mode25'), Field.byte('mode26'), Field.byte('mode27'),
         Field.byte('mode28'), Field.byte('mode29'), Field.byte('mode30'), Field.byte('mode31'),
         Field.crc(), Field.lit('\r\n')])

def sensor_humidity_list():
    """ Reads the list humidity values of the 32 (0-31) sensors. """
    return MasterCommandSpec("hl",
        [Field.padding(13)],
        [Field.svt('hum0'), Field.svt('hum1'), Field.svt('hum2'), Field.svt('hum3'),
         Field.svt('hum4'), Field.svt('hum5'), Field.svt('hum6'), Field.svt('hum7'),
         Field.svt('hum8'), Field.svt('hum9'), Field.svt('hum10'), Field.svt('hum11'),
         Field.svt('hum12'), Field.svt('hum13'), Field.svt('hum14'), Field.svt('hum15'),
         Field.svt('hum16'), Field.svt('hum17'), Field.svt('hum18'), Field.svt('hum19'),
         Field.svt('hum20'), Field.svt('hum21'), Field.svt('hum22'), Field.svt('hum23'),
         Field.svt('hum24'), Field.svt('hum25'), Field.svt('hum26'), Field.svt('hum27'),
         Field.svt('hum28'), Field.svt('hum29'), Field.svt('hum30'), Field.svt('hum31'),
         Field.crc(), Field.lit('\r\n')])

def sensor_temperature_list():
    """ Reads the list temperature values of the 32 (0-31) sensors. """
    return MasterCommandSpec("cl",
        [Field.padding(13)],
        [Field.svt('tmp0'), Field.svt('tmp1'), Field.svt('tmp2'), Field.svt('tmp3'),
         Field.svt('tmp4'), Field.svt('tmp5'), Field.svt('tmp6'), Field.svt('tmp7'),
         Field.svt('tmp8'), Field.svt('tmp9'), Field.svt('tmp10'), Field.svt('tmp11'),
         Field.svt('tmp12'), Field.svt('tmp13'), Field.svt('tmp14'), Field.svt('tmp15'),
         Field.svt('tmp16'), Field.svt('tmp17'), Field.svt('tmp18'), Field.svt('tmp19'),
         Field.svt('tmp20'), Field.svt('tmp21'), Field.svt('tmp22'), Field.svt('tmp23'),
         Field.svt('tmp24'), Field.svt('tmp25'), Field.svt('tmp26'), Field.svt('tmp27'),
         Field.svt('tmp28'), Field.svt('tmp29'), Field.svt('tmp30'), Field.svt('tmp31'),
         Field.crc(), Field.lit('\r\n')])

def sensor_brightness_list():
    """ Reads the list brightness values of the 32 (0-31) sensors. """
    return MasterCommandSpec("bl",
        [Field.padding(13)],
        [Field.svt('bri0'), Field.svt('bri1'), Field.svt('bri2'), Field.svt('bri3'),
         Field.svt('bri4'), Field.svt('bri5'), Field.svt('bri6'), Field.svt('bri7'),
         Field.svt('bri8'), Field.svt('bri9'), Field.svt('bri10'), Field.svt('bri11'),
         Field.svt('bri12'), Field.svt('bri13'), Field.svt('bri14'), Field.svt('bri15'),
         Field.svt('bri16'), Field.svt('bri17'), Field.svt('bri18'), Field.svt('bri19'),
         Field.svt('bri20'), Field.svt('bri21'), Field.svt('bri22'), Field.svt('bri23'),
         Field.svt('bri24'), Field.svt('bri25'), Field.svt('bri26'), Field.svt('bri27'),
         Field.svt('bri28'), Field.svt('bri29'), Field.svt('bri30'), Field.svt('bri31'),
         Field.crc(), Field.lit('\r\n')])

def virtual_sensor_list():
    """ Read the list with virtual settings of the 32 (0-31) sensors. """
    return MasterCommandSpec("VL",
        [Field.padding(13)],
        [Field.byte('vir0'), Field.byte('vir1'), Field.byte('vir2'), Field.byte('vir3'),
         Field.byte('vir4'), Field.byte('vir5'), Field.byte('vir6'), Field.byte('vir7'),
         Field.byte('vir8'), Field.byte('vir9'), Field.byte('vir10'), Field.byte('vir11'),
         Field.byte('vir12'), Field.byte('vir13'), Field.byte('vir14'), Field.byte('vir15'),
         Field.byte('vir16'), Field.byte('vir17'), Field.byte('vir18'), Field.byte('vir19'),
         Field.byte('vir20'), Field.byte('vir21'), Field.byte('vir22'), Field.byte('vir23'),
         Field.byte('vir24'), Field.byte('vir25'), Field.byte('vir26'), Field.byte('vir27'),
         Field.byte('vir28'), Field.byte('vir29'), Field.byte('vir30'), Field.byte('vir31'),
         Field.crc(), Field.lit('\r\n')])

def set_virtual_sensor():
    """ Set the values (temperature, humidity, brightness) of a virtual sensor. """
    return MasterCommandSpec("VS",
        [Field.byte('sensor'), Field.svt('tmp'), Field.svt('hum'), Field.svt('bri'),
         Field.padding(9)],
        [Field.byte('sensor'), Field.svt('tmp'), Field.svt('hum'), Field.svt('bri'),
         Field.padding(9), Field.lit('\r\n')])

def pulse_list():
    """ List the pulse counter values. """
    return MasterCommandSpec("PL",
        [Field.padding(13)],
        [Field.int('pv0'), Field.int('pv1'), Field.int('pv2'), Field.int('pv3'),
         Field.int('pv4'), Field.int('pv5'), Field.int('pv6'), Field.int('pv7'),
         Field.int('pv8'), Field.int('pv9'), Field.int('pv10'), Field.int('pv11'),
         Field.int('pv12'), Field.int('pv13'), Field.int('pv14'), Field.int('pv15'),
         Field.int('pv16'), Field.int('pv17'), Field.int('pv18'), Field.int('pv19'),
         Field.int('pv20'), Field.int('pv21'), Field.int('pv22'), Field.int('pv23'),
         Field.crc(), Field.lit('\r\n')])

def error_list():
    """ Get the number of errors for each input and output module. """
    return MasterCommandSpec("el",
        [Field.padding(13)],
        [Field("errors", ErrorListFieldType()), Field.crc(), Field.lit("\r\n")])

def clear_error_list():
    """ Clear the number of errors. """
    return MasterCommandSpec("ec",
        [Field.padding(13)],
        [Field.str("resp", 2), Field.padding(11), Field.lit("\r\n")])

def write_airco_status_bit():
    """ Write the airco status bit. """
    return MasterCommandSpec("AW",
        [Field.byte("thermostat"), Field.byte("ASB"), Field.padding(11)],
        [Field.byte("ASB0"), Field.byte("ASB1"), Field.byte("ASB2"), Field.byte("ASB3"),
         Field.byte("ASB4"), Field.byte("ASB5"), Field.byte("ASB6"), Field.byte("ASB7"),
         Field.byte("ASB8"), Field.byte("ASB9"), Field.byte("ASB10"), Field.byte("ASB11"),
         Field.byte("ASB12"), Field.byte("ASB13"), Field.byte("ASB14"), Field.byte("ASB15"),
         Field.byte("ASB16"), Field.byte("ASB17"), Field.byte("ASB18"), Field.byte("ASB19"),
         Field.byte("ASB20"), Field.byte("ASB21"), Field.byte("ASB22"), Field.byte("ASB23"),
         Field.byte("ASB24"), Field.byte("ASB25"), Field.byte("ASB26"), Field.byte("ASB27"),
         Field.byte("ASB28"), Field.byte("ASB29"), Field.byte("ASB30"), Field.byte("ASB31"),
         Field.lit("\r\n")])

def read_airco_status_bits():
    """ Read the airco status bits. """
    return MasterCommandSpec("AR",
        [Field.padding(13)],
        [Field.byte("ASB0"), Field.byte("ASB1"), Field.byte("ASB2"), Field.byte("ASB3"),
         Field.byte("ASB4"), Field.byte("ASB5"), Field.byte("ASB6"), Field.byte("ASB7"),
         Field.byte("ASB8"), Field.byte("ASB9"), Field.byte("ASB10"), Field.byte("ASB11"),
         Field.byte("ASB12"), Field.byte("ASB13"), Field.byte("ASB14"), Field.byte("ASB15"),
         Field.byte("ASB16"), Field.byte("ASB17"), Field.byte("ASB18"), Field.byte("ASB19"),
         Field.byte("ASB20"), Field.byte("ASB21"), Field.byte("ASB22"), Field.byte("ASB23"),
         Field.byte("ASB24"), Field.byte("ASB25"), Field.byte("ASB26"), Field.byte("ASB27"),
         Field.byte("ASB28"), Field.byte("ASB29"), Field.byte("ASB30"), Field.byte("ASB31"),
         Field.lit("\r\n")])

def to_cli_mode():
    """ Go to CLI mode """
    return MasterCommandSpec("CM",
        [Field.padding(13)],
        None)

def module_discover_start():
    """ Put the master in module discovery mode. """
    return MasterCommandSpec("DA",
        [Field.padding(13)],
        [Field.str("resp", 2), Field.padding(11), Field.lit("\r\n")])

def module_discover_stop():
    """ Put the master into the normal working state. """
    return MasterCommandSpec("DO",
        [Field.padding(13)],
        [Field.str("resp", 2), Field.padding(11), Field.lit("\r\n")])

def indicate():
    """ Flash the led for a given output/input/sensor. """
    return MasterCommandSpec("IN",
        [Field.byte('type'), Field.byte('id'), Field.padding(11)],
        [Field.str("resp", 2), Field.padding(11), Field.lit("\r\n")])

### Below are the asynchronous messages, sent by the master to the gateway

def output_list():
    """ The message sent by the master whenever the outputs change. """
    return MasterCommandSpec("OL",
        [],
        [Field("outputs", OutputFieldType()), Field.lit("\r\n")])

def input_list():
    """ The message sent by the master whenever an input is enabled. """
    return MasterCommandSpec("IL",
        [],
        [Field.byte('input'), Field.byte('output'), Field.lit("\r\n")])

def module_initialize():
    """ The message sent by the master whenever a module is initialized in module discovery mode.
    """
    return MasterCommandSpec("MI",
        [],
        [Field.str('id', 4), Field.str('instr', 1), Field.byte('module_nr'), Field.byte('data'),
         Field.byte('io_type'), Field.padding(5), Field.lit('\r\n')])

def event_triggered():
    """ The message sent by the master to trigger an event. This event is triggered by basic
    action 60. """
    return MasterCommandSpec("EV",
        [],
        [Field.byte('code'), Field.padding(12), Field.lit('\r\n')])

### Below are the function to update the firmware of the modules (input/output/dimmer/thermostat)

def modules_goto_bootloader():
    """ Reset the module to go to the bootloader. """
    return MasterCommandSpec("FR",
        [Field.str('addr', 4), Field.byte('sec'), Field.lit('C'), Field.byte('crc0'),
         Field.byte('crc1'), Field.padding(5)],
        [Field.str('addr', 4), Field.byte("error_code"), Field.lit('C'), Field.byte('crc0'),
         Field.byte('crc1'), Field.padding(5), Field.lit("\r\n")])

def modules_new_firmware_version():
    """ Preprare the slave module for a new version. """
    return MasterCommandSpec("FN",
        [Field.str('addr', 4), Field.byte("f1n"), Field.byte("f2n"), Field.byte("f3n"),
         Field.lit('C'), Field.byte('crc0'), Field.byte('crc1'), Field.padding(3)],
        [Field.str('addr', 4), Field.byte("error_code"), Field.lit('C'), Field.byte('crc0'),
         Field.byte('crc1'), Field.padding(5), Field.lit("\r\n")])

def modules_new_crc():
    """ Write the new crc code to the bootloaded module. """
    return MasterCommandSpec("FC",
        [Field.str('addr', 4), Field.byte("ccrc0"), Field.byte("ccrc1"), Field.byte("ccrc2"),
         Field.byte("ccrc3"), Field.lit('C'), Field.byte('crc0'), Field.byte('crc1'),
         Field.padding(2)],
        [Field.str('addr', 4), Field.byte("error_code"), Field.lit('C'), Field.byte('crc0'),
         Field.byte('crc1'), Field.padding(5), Field.lit("\r\n")])

def change_communication_mode_to_long():
    """ Change the number of bytes used to communicate with the master to 75. """
    return MasterCommandSpec("cm",
        [Field.lit('\x4d'), Field.lit('\x01'), Field.padding(11)],
        [Field.lit('\x4d'), Field.lit('\x01'), Field.padding(11), Field.lit("\r\n")])

def change_communication_mode_to_short():
    """ Change the number of bytes used to communicate with the master to 18. """
    return MasterCommandSpec("cm",
        [Field.lit('\x12'), Field.lit('\x01'), Field.padding(71)],
        [Field.lit('\x12'), Field.lit('\x01'), Field.padding(11), Field.lit("\r\n")])

def modules_update_firmware_block():
    """ Upload 1 block of 64 bytes to the module. """
    return MasterCommandSpec("FD",
        [Field.str('addr', 4), Field.int("block"), Field.str("bytes", 64),
         Field.lit('C'), Field.byte('crc0'), Field.byte('crc1')],
        [Field.str('addr', 4), Field.byte("error_code"), Field.lit('C'), Field.byte('crc0'),
         Field.byte('crc1'), Field.lit("\r\n")])

def modules_get_version():
    """ Get the version of the module. """
    return MasterCommandSpec("FV",
        [Field.str('addr', 4), Field.lit('C'), Field.byte('crc0'), Field.byte('crc1'),
         Field.padding(6)],
        [Field.str('addr', 4), Field.byte("error_code"), Field.byte("hw_version"),
         Field.byte("f1"), Field.byte("f2"), Field.byte("f3"), Field.byte("status"),
         Field.lit('C'), Field.byte('crc0'), Field.byte('crc1'), Field.lit("\r\n")])

def modules_integrity_check():
    """ Check the integrity of the new code. """
    return MasterCommandSpec("FE",
        [Field.str('addr', 4), Field.lit('C'), Field.byte('crc0'), Field.byte('crc1'),
         Field.padding(6)],
        [Field.str('addr', 4), Field.byte("error_code"), Field.lit('C'), Field.byte('crc0'),
         Field.byte('crc1'), Field.padding(5), Field.lit("\r\n")])

def modules_goto_application():
    """ Let the module go to application. """
    return MasterCommandSpec("FG",
        [Field.str('addr', 4), Field.lit('C'), Field.byte('crc0'), Field.byte('crc1'),
         Field.padding(6)],
        [Field.str('addr', 4), Field.byte("error_code"), Field.lit('C'), Field.byte('crc0'),
         Field.byte('crc1'), Field.padding(5), Field.lit("\r\n")])


### Below are helpers for the Svt (System value type).

class Svt(object):
    """ Class for the System Value Type, this can be either a time, temperature, humidity or brightness. """
    TIME = 1
    TEMPERATURE = 2
    HUMIDITY = 3
    BRIGHTNESS = 4
    RAW = 5

    def __init__(self, type, value):
        """ Default constructor.
        :param type: Type of the Svt (can be Svt.TIME, Svt.TEMPERATURE, Svt.HUMIDITY, Svt.BRIGHTNESS or Svt.RAW).
        :param value: The human-friendly value
        """
        if type == Svt.TIME:
            split = [int(x) for x in value.split(":")]
            if len(split) != 2:
                raise ValueError("Time is not in HH:MM format: %s" % value)
            self.__value = (split[0] * 6) + (split[1] / 10)
        elif type in [Svt.TEMPERATURE, Svt.HUMIDITY, Svt.BRIGHTNESS]:
            if value is None:
                self.__value = 255
            elif type == Svt.TEMPERATURE:
                self.__value = int((value + 32) * 2)
            elif type == Svt.HUMIDITY:
                self.__value = int(value * 2)
            elif type == Svt.BRIGHTNESS:
                self.__value = 254 - int(value * 2.54)
        elif type == Svt.RAW:
            self.__value = value
        else:
            raise ValueError("Unknown type for Svt: " + str(type))
        self.__value = min(max(self.__value, 0), 255)

    def get_time(self):
        """ Convert an Svt to time.
        :returns: String with form HH:MM
        """
        hours = (self.__value / 6)
        minutes = (self.__value % 6) * 10
        return "%02d:%02d" % (hours, minutes)

    def get_temperature(self):
        """ Convert an Svt to temperature.
        :returns: degrees celcius (float).
        """
        if self.__value == 255:
            return None
        return (float(self.__value) / 2) - 32

    def get_humidity(self):
        """ Convert an Svt to humidity.
        :returns: humidity in percent.
        """
        if self.__value > 200:
            return None
        return self.__value / 2.0

    def get_brightness(self):
        """ Convert an Svt to brightness.
        :returns: brightness in percent.
        """
        if self.__value == 255:
            return None
        return round((254 - self.__value) / 2.54, 2)

    def get_byte(self):
        """ Get the Svt value as a byte.
        :returns: one byte
        """
        return chr(self.__value)

    @staticmethod
    def from_byte(byte_value):
        """ Create an Svt instance from a byte.
        :returns: instance of the Svt class.
        """
        return Svt(Svt.RAW, ord(byte_value))

    @staticmethod
    def temp(temperature):
        """ Create an Svt instance from a temperature.
        :param temperature: in degrees celcius (float)
        """
        return Svt(Svt.TEMPERATURE, temperature)

    @staticmethod
    def humidity(humidity):
        """ Create an Svt instance from a humidity.
        :param humidity: in percent (float)"""
        return Svt(Svt.HUMIDITY, humidity)

    @staticmethod
    def brightness(brightness):
        """ Create an Svt instance from a brightness.
        :param brightness: in percent (float)"""
        return Svt(Svt.BRIGHTNESS, brightness)

    @staticmethod
    def time(time_value):
        """ Create an Svt instance from a time value.
        :param time_value: String in format HH:MM
        """
        return Svt(Svt.TIME, time_value)

def dimmer_to_percentage(dimmer_value):
    """ Convert a dimmer value to an integer in [0, 100].

    :param dimmer_value: integer in [0, 63].
    :returns: dimmer percentage in [0, 100].
    """
    return DimmerFieldType().decode(chr(dimmer_value))
