#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# This file is part of ttn-fan.
# Cpoyright (C) 2018 Philippe Vanhaesendonck
#
# Ttn-fan is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# Ttn-fan is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with ttn-fan.  If not, see <http://www.gnu.org/licenses/>.

"""
ttn-fan is a simple temperature controller for an IC880A DIY TTN Gateway using
dbrgn's backplane.
"""

from time import sleep, time
from socket import gethostname
from signal import signal, SIGTERM
from fcntl import flock, LOCK_EX, LOCK_UN
import RPi.GPIO as GPIO
import click
from PID import PID
from sht21 import SHT21
from influxdb import InfluxDBClient
from influxdb import SeriesHelper

# InfluxDB connections settings
influxdb_host = '127.0.0.1'
influxdb_port = 8089
influxdb_dbname = 'ttn'

influxdb_client = InfluxDBClient(host=influxdb_host,
                                 use_udp=True,
                                 udp_port=influxdb_port,
                                 database=influxdb_dbname)


class InfuxdbSeries(SeriesHelper):
    """Instantiate SeriesHelper to write points to the backend."""
    hostname = gethostname()

    class Meta:
        client = influxdb_client
        series_name = 'fan_value'
        fields = ['value']
        tags = ['host', 'type']
        autocommit = False


class SigTerm(Exception):
    """ Exception for SIGTERM """
    pass


def signal_handler(sig, frame):
    """ Signal handler for SIGTERM """
    raise SigTerm


class Fan(object):
    """ Handles fan..."""

    def __init__(self,
                 pin,
                 gpio_mode=GPIO.BOARD,
                 pwm_freq=50,
                 min_pct=0.0,
                 max_pct=100.0,
                 kick_start=0.0,
                 trigger_range=0.0):
        """ Constructor -- Initialize the port
            pin, gpio_mode: fan pin number and gpio mode (BCM / BOARD)
            pwm_freq: PWM frequency (Hz) -- no need for high frequency
            min_pct, max_pct: window of operation for the fan
                (fans tend to stall at low PWM and can be noisy at full power)
            kick_start: give a short pulse at higher RPM to get the fan
            starting """
        GPIO.setmode(gpio_mode)
        GPIO.setup(pin, GPIO.OUT)
        self._fan = GPIO.PWM(pin, pwm_freq)
        self._fan.start(0)
        self._speed = 0.0
        self._min_pct = min_pct
        self._max_pct = max_pct
        self._kick_start = kick_start
        self._trigger_range = trigger_range

    @property
    def speed(self):
        return self._speed

    @speed.setter
    def speed(self, new_speed):
        """ Set the fan speed within operating range """
        if new_speed < self._min_pct:
            new_speed = self._min_pct
        if new_speed > self._max_pct:
            new_speed = self._max_pct

        if self._kick_start > 0.0 and self._speed == 0.0 and new_speed > 0.0:
            self._fan.ChangeDutyCycle(50)
            sleep(self._kick_start)

        self._fan.ChangeDutyCycle(new_speed)
        self._speed = new_speed

    @property
    def trigger_range(self):
        return self._trigger_range

    def stop(self):
        """ Fully stop fan """
        self._fan.ChangeDutyCycle(0)
        self._speed = 0.0

    def cleanup(self):
        """ Release resources """
        self.stop()
        self._fan.stop()
        GPIO.cleanup()


class Temperature(object):
    """ Simple class for temp sensor -- kernel driver """
    def __init__(self, temp_sensor_path):
        self._temp_sensor_path = temp_sensor_path

    @property
    def temperature(self):
        # Read raw value
        with open(self._temp_sensor_path, 'rb') as f:
            temp = f.read().strip()

        # Convert to degrees celsius
        temp = float(int(temp)) / 1000

        return temp


class TemperatureUserMode(SHT21):
    """ Simple class for temp sensor -- user mode driver """
    def __init__(self, device_number=0, lock_file=None):
        if lock_file:
            self._handle = open(lock_file, 'w')
        else:
            self._handle = None

        super(TemperatureUserMode, self).__init__(device_number)
        self._last_temp = 0.0

    @property
    def temperature(self):
        try:
            # If other processes are using the i2c bus concurrently the
            # reading can fail. We just ignore the error and return the last
            # successful read
            # We implment a simple lock mechanism to allow process
            # collaboration
            if self._handle:
                flock(self._handle, LOCK_EX)
            temp = self.read_temperature()
            if self._handle:
                flock(self._handle, LOCK_UN)
            self._last_temp = temp
        except IOError, e:
            print('User-mode sht21: Could not read sensor data: %s' % e)

        return self._last_temp


class TTN_PID(PID):
    """ PID controller for the fan """
    def __init__(self, kp, ki, kd, sample, windup, target):
        self._target = target
        self._windup = windup
        self._sample = sample
        PID.__init__(self, kp, ki, kd)

    def clear(self):
        PID.clear(self)
        PID.setWindup(self, self._windup)
        self.SetPoint = self._target
        self.current_time = time()
        self.last_time = self.current_time

    @property
    def target(self):
        return self._target

    @property
    def windup(self):
        return self._windup

    @property
    def sample(self):
        return self._sample


class ControlLoop(object):
    """ Runs the control loop """
    def __init__(self, fan, sensor, pid, verbose=True):
        self._fan = fan
        self._sensor = sensor
        self._pid = pid
        self._verbose = verbose

    def control_loop(self):
        """ Idle loop until temperature overshoots, then starts the PID loop"""
        print('Entering control loop')
        fan_trigger = self._pid.target + self._fan.trigger_range / 2.0
        while True:
            temp = self._sensor.temperature
            if temp > fan_trigger:
                self._pid_loop()
            self._log('Ctl', temp)
            sleep(self._pid.sample)

    def _pid_loop(self):
        """ Run PID loop until temperature drops beyond treshold """
        print('Entering PID loop')
        fan_trigger = self._pid.target - self._fan.trigger_range / 2.0
        self._pid.clear()

        while True:
            temp = self._sensor.temperature
            self._pid.update(temp)
            fan_speed = -self._pid.output

            # Loop exit condition:
            if (temp < fan_trigger and fan_speed < 0 and
                    self._pid.ITerm >= self._pid.windup):
                self._fan.stop()
                break

            self._fan.speed = fan_speed
            self._log('PID', temp)
            sleep(self._pid.sample)

        print('Exiting PID loop')

    def _log(self, loop, temp):
        """ Log loop status """
        if self._verbose:
            if loop == 'PID':
                print(('{0}: Temp {1:6.2f} | Fan {2:6.2f} | '
                       'PTerm {3:7.2f} | ITerm {4:7.2f} | DTerm {5:6.2f} | '
                       'Output {6:7.2f}').format(
                           loop, temp, self._fan.speed, self._pid.PTerm,
                           self._pid.ITerm, self._pid.DTerm, self._pid.output))
            else:
                print("{0}: Temp {1:6.2f} | Fan {2:6.2f}".format(
                    loop, temp, self._fan.speed))

        InfuxdbSeries(host=InfuxdbSeries.hostname, type='temperature',
                      value=temp)
        InfuxdbSeries(host=InfuxdbSeries.hostname, type='fanSpeed',
                      value=self._fan.speed)
        InfuxdbSeries.commit()


@click.command()
@click.option('-v', '--verbose', is_flag=True,
              help='Print temperature at each iteration')
@click.option('--target-temp', type=float, default=45.0, show_default=True,
              help="Target temperature")
@click.option('--user-mode', is_flag=True,
              help='Use user-mode driver to get temperature')
@click.option('--lock-file', type=click.Path(), default=None,
              help='Optional lock file for user-mode')
def ttn_fan(verbose, target_temp, user_mode, lock_file):
    # Fan configuration
    fan_pin = 12
    fan_gpio_mode = GPIO.BOARD
    fan_pwm_freq = 50
    fan_min_pct = 8.0
    fan_max_pct = 100.0
    fan_kick_start = 0.5
    # We need to account for the range we can't control
    # Typically the temperature drop at minmum fan setting (°C)
    fan_trigger_range = 2.0

    # Temperature sensor -- sht21 temp sensor (provide much better accuracy
    # than CPU temperature)
    temp_sensor_path = '/sys/class/hwmon/hwmon0/temp1_input'
    i2c_device_number = 1

    # PID settings
    pid_kp = 15.0
    pid_ki = 0.5
    pid_kd = 0.0
    pid_sample = 5.0

    if user_mode:
        sensor = TemperatureUserMode(i2c_device_number, lock_file)
    else:
        sensor = Temperature(temp_sensor_path)

    fan = Fan(pin=fan_pin,
              gpio_mode=fan_gpio_mode,
              pwm_freq=fan_pwm_freq,
              min_pct=fan_min_pct,
              max_pct=fan_max_pct,
              kick_start=fan_kick_start,
              trigger_range=fan_trigger_range)

    pid = TTN_PID(kp=pid_kp,
                  ki=pid_ki,
                  kd=pid_kd,
                  sample=pid_sample,
                  windup=fan_max_pct / pid_ki,
                  target=target_temp)

    signal(SIGTERM, signal_handler)

    loop = ControlLoop(fan, sensor, pid, verbose)
    try:
        loop.control_loop()
    except (KeyboardInterrupt, SigTerm):
        print("Interrupted -- Stop fan and cleanup GPIO")
        fan.cleanup()


if __name__ == '__main__':
    ttn_fan()
