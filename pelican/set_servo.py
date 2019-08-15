#!/usr/bin/python

# This file is part of the MAVLink Router project
#
# Copyright (C) 2017  Intel Corporation. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from __future__ import print_function

import pymavlink.mavutil as mavutil
import sys

#### RUN:
#### mavlink-routerd -e 127.0.0.1:3000 /dev/ttyUSB0:57600
#### connect QGC to TCP endpoint

mav = mavutil.mavlink_connection('udpin:127.0.0.1:3000')
mav.wait_heartbeat()
setpoint = 0.0
if len(sys.argv) > 1:
    setpoint = float(sys.argv[1])

mav.mav.command_long_send(mav.target_system, mav.target_component,
                          mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, 0,
                          setpoint, 0, 0, 0, 0, 0)
