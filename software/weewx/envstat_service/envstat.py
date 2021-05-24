#
#    Copyright 2021 Manuel BOUYER
#
"""Service that run NetBSD's envstat command and inserts values in the data stream

Put this file (envstat.py) in your WeeWX user subdirectory.
Add to your weewx.conf file

[ENVSTAT]
    # Map from weewx names to sensor names. Only these types will be processed.
    # units will be extracted from envstat's output
    mustHave = outTemp
    [[sensor_map]]
        CPUtemp = temperature
        CPUvoltage = AC input voltage
        CPUcurrent = AC input current

Add the ENVSTAT service to the list of data_services to be run:

[Engine]
  [[Services]]
    ...
    data_services = user.envstat.ENVSTAT

"""

import subprocess
import weewx.engine
import weewx.units
from weeutil.weeutil import to_int

import weeutil.logger
import logging

import weewx.units
weewx.units.obs_group_dict['CPUtemp'] = 'group_temperature'
weewx.units.obs_group_dict['CPUvoltage'] = 'group_volt'
weewx.units.obs_group_dict['CPUcurrent'] = 'group_amp'

log = logging.getLogger(__name__)


def logdbg(msg):
    log.debug(msg)


def loginf(msg):
    log.info(msg)


def logerr(msg):
    log.error(msg)

def surely_a_list(innie):
    if isinstance(innie, list):
        return innie
    if innie is None or innie is "":
        return []
    return [innie] # cross fingers

VERSION = "0.1"

class ENVSTAT(weewx.engine.StdService):
    """WeeWX service for augmenting a record with data parsed from NetBSD's envstat command."""

    def __init__(self, engine, config_dict):
        # Initialize my superclass:
        super(ENVSTAT, self).__init__(engine, config_dict)

        self.envstat_dict = config_dict.get('ENVSTAT', {})

        self.must_have = surely_a_list(self.envstat_dict.get('mustHave', []))

        self.sensor_map = self.envstat_dict.get('sensor_map', {})
        loginf("Sensor map is %s" % self.sensor_map)

        self.bind(weewx.NEW_LOOP_PACKET, self.new_loop_packet)

    def new_loop_packet(self, event):
        """Add envstat output to the LOOP data stream"""

        packet = event.packet

        if all(have in packet for have in self.must_have):
            if 'usUnits' in packet:
                converter = weewx.units.StdUnitConverters[packet['usUnits']]
            else:
                converter = weewx.units.StdUnitConverters[self.default_units]

            envstat = subprocess.Popen(['/usr/sbin/envstat', '-I'], stdout = subprocess.PIPE)
            for line in envstat.stdout:
                line = line.decode().lstrip()
                line = line.split(':')
                if len(line) < 2:
                    continue;
                sensor = line[0]
                val = line[1].split()
                if len(val) < 2: 
                    continue; 
                value, unit = val
                for s in self.sensor_map:
                    if self.sensor_map[s] == sensor:
                        try:
                            f_val = float(value)
                        except ValueError:
                            # If we can't convert it to a float, ignore it.
                            continue
                        if unit == 'degC':
                            unit = 'degree_C'
                            group = 'group_temperature'
                        elif unit == 'V':
                            unit = 'volt'
                            group = 'group_volt'
                        elif unit == 'A':
                            unit = 'amp'
                            group = 'group_amp'
                        else:
                            if weewx.debug >= 0:
                                logdbg("Rejected: %s, %f, %s"
                                       % (sensor, f_val, unit))
                            continue

                        # Form a ValueTuple using the unit and unit group
                        val_t = weewx.units.ValueTuple(f_val, unit, group)
                        # Convert it to the same unit system as the incoming packet
                        target_t = weewx.units.convertStd(val_t, event.packet['usUnits'])
                        # Now update the value in the packet
                        event.packet[s] = target_t.value
                        if weewx.debug >= 0:
                            logdbg("Set type '%s' to %.3f from sensor '%s'"
                                   % (s, event.packet[s], sensor))
