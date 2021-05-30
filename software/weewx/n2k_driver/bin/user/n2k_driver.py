#!/usr/bin/python
#
# Copyright 2014 Matthew Wall
# Copyright 2021 Manuel BOUYER
#
# weewx driver that reads data from a CAN bus
#
# This program is free software: you can redistribute it and/or modify it under
# the terms of the GNU General Public License as published by the Free Software
# Foundation, either version 3 of the License, or any later version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
# FOR A PARTICULAR PURPOSE.
#
# See http://www.gnu.org/licenses/


from __future__ import with_statement
import logging
import time
import socket
import struct
import sys
from math import pi, cos, sin, atan2

import weewx.drivers

DRIVER_NAME = 'N2kDriver'
DRIVER_VERSION = "0.1"

log = logging.getLogger(__name__)

# CAN frame packing/unpacking (see `struct can_frame` in <linux/can.h>)
can_frame_fmt = "=IB3x8s"
can_id_fmt = "=BBBB"

def dissect_can_id(can_id):
    saddr, daddr, iso_pg, pprio = struct.unpack(can_id_fmt, struct.pack("=I", can_id))
    page = (pprio & 0x1)
    pgn = (page << 16) | (iso_pg << 8);
    if iso_pg > 239:
        pgn |= daddr;
    return (saddr, daddr, pgn)

def dissect_can_frame(frame):
    can_id, can_dlc, data = struct.unpack(can_frame_fmt, frame)     
    return (can_id, can_dlc, data[:can_dlc])

def loader(config_dict, engine):
    return N2kDriver(**config_dict[DRIVER_NAME])

class N2kDriver(weewx.drivers.AbstractDevice):
    """weewx driver that reads data from a CAN bus"""

    def __init__(self, **stn_dict):
        # CAN interface name
        self.intf = stn_dict.get('interface', 'canlo0')

        self.s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.s.bind((self.intf,))

        self.last_rain_sid = 0xff
        self.last_rain_value = 0

        self.last_wind_time = 0
        self.last_wind_sid = 0xff
        self.wind_speed_av = 0
        self.wind_dir_av_n = 0
        self.wind_dir_av_e = 0
        self.wind_count = 0

        log.info("listening on interface %s" % self.intf)

    def genLoopPackets(self):
        while True:
            new_data = 0
            cf, addr = self.s.recvfrom(16)
            can_id, can_dlc, data = dissect_can_frame(cf)
            saddr, daddr, pgn = dissect_can_id(can_id)

            # map the data into a weewx loop packet
            current_time = int(time.time() + 0.5)
            _packet = {'dateTime': current_time,
                       'usUnits': weewx.METRICWX}

            if pgn == 130311: #NMEA2000_ENV_PARAM
                sid, source, temp, hum, press = struct.unpack("=BBHHH", data)
                tsource = source & 0x3f
                hsource = (source & 0xc0) >> 6
                if tsource == 1:
                    _packet['outTemp'] = temp / 100 - 273.15
                    new_data += 1
                elif tsource == 2:
                    _packet['inTemp'] = temp / 100 - 273.15
                    new_data += 1
                if hsource == 0:
                    _packet['inHumidity'] = hum / 250
                    new_data += 1
                elif hsource == 1:
                    _packet['outHumidity'] = hum / 250
                    new_data += 1
                if press < 0xffff:
                    _packet['pressure'] = press / 100
                else:
                    press = 0
                #print('SID %x s %x temp %d s %x hum %d press %d' % (sid , tsource, temp, hsource, hum, press))
            elif pgn == 61847: #PRIVATE_RAIN_COUNTER
                sid, rain = struct.unpack("=BH", data)
                if sid != self.last_rain_sid:
                    if self.last_rain_sid != 0xff:
                        if self.last_rain_count > rain:
                            drain = 0xffff - self.last_rain_count + rain
                        else:
                            drain = rain - self.last_rain_count
                        if drain < 255: # avoid silly values
                            _packet['rain'] = drain * 0.2794;
                        #print('SID %x rain %d' % (sid, drain))
                        new_data += 1
                    self.last_rain_count = rain
                    self.last_rain_sid = sid
            elif pgn == 130306: #NMEA2000_WIND_DATA
                sid, speed, dir, ref = struct.unpack("=BHHB", data)

                if self.last_wind_sid == sid:
                    continue
                speed = speed / 100.0
                dir = dir / 62831.853 * 360
                self.last_wind_sid = sid
                self.wind_speed_av += speed
                self.wind_dir_av_n += cos(dir * pi/180)
                self.wind_dir_av_e += sin(dir * pi/180)
                self.wind_count += 1
                if self.last_wind_time != current_time:
                    _packet['windSpeed'] = self.wind_speed_av / self.wind_count
                    dir = atan2(self.wind_dir_av_e, self.wind_dir_av_n) * 180/pi
                    dir = (dir + 360) % 360
                    _packet['windDir'] = dir
                    new_data += 1
                    self.last_wind_time = current_time
                    self.wind_speed_av = 0
                    self.wind_dir_av_n = 0
                    self.wind_dir_av_e = 0
                    self.wind_count = 0
            elif pgn == 127508: #NMEA2000_BATTERY_STATUS
                instance, volt, amp, temp, sid = struct.unpack("=BhhHB", data)

                _packet['consBatteryVoltage'] = volt / 100
                _packet['consBatteryCurrent'] = amp / 10000 
                _packet['consBatteryTemp'] = temp / 100 - 273.15
                new_data += 1
            else:
                continue
                #_packet['supplyVoltage']
 
            if new_data > 0:
                yield _packet

    @property
    def hardware_name(self):
        return "N2kDriver"

# To test this driver, run it directly as follows:
#   PYTHONPATH=/home/weewx/bin python /home/weewx/bin/user/n2k_driver.py
if __name__ == "__main__":
    import weeutil.weeutil
    import weeutil.logger
    import weewx
    weewx.debug = 1
    weeutil.logger.setup('n2k_driver', {})

    driver = N2kDriver()
    for packet in driver.genLoopPackets():
        print(weeutil.weeutil.timestamp_to_string(packet['dateTime']), packet)
