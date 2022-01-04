from __future__ import division, print_function

import os
import time
from configparser import ConfigParser
from math import modf

from serial import Serial
from serial.tools.list_ports import comports

from .paths import path_length
from .planner import Planner
from .progress import Bar

DEFAULT_CONFIGS = '''[DEFAULT]
timeslice_ms = 10
microstepping_mode = 1
pen_up_position = 60
pen_up_speed = 150
pen_up_delay = 0
pen_down_position=40
pen_down_speed = 150
pen_down_delay = 0
acceleration = 16
max_velocity = 4
corner_factor = 0.001
jog_acceleration = 16
jog_max_velocity = 8
vid_pid = 04d8:fd92'''


def find_port(vid_pid: str):
    vid_pid = vid_pid.upper()
    for port in comports():
        if vid_pid in port[2]:
            return port[0]
    return None


class Device(object):
    def __init__(self):
        here = os.path.dirname(os.path.abspath(__file__))
        filename = os.path.join(here, 'axidraw.ini')
        if not os.path.exists(filename):
            with open(filename, 'w') as f:
                f.write(DEFAULT_CONFIGS)
        config = ConfigParser()
        config.read(filename)
        print("YO")
        print(list(config['DEFAULT'].items()))
        self.timeslice_ms = int(config['DEFAULT']['timeslice_ms'])
        self.microstepping_mode = int(config['DEFAULT']['microstepping_mode'])
        self.step_divider = 2 ** (self.microstepping_mode - 1)
        self.steps_per_unit = 2032 / self.step_divider
        self.steps_per_mm = 80 / self.step_divider
        self.pen_up_position = float(config['DEFAULT']['pen_up_position'])
        self.pen_up_speed = float(config['DEFAULT']['pen_up_speed'])
        self.pen_up_delay = int(config['DEFAULT']['pen_up_delay'])
        self.pen_down_position = float(config['DEFAULT']['pen_down_position'])
        self.pen_down_speed = float(config['DEFAULT']['pen_down_speed'])
        self.pen_down_delay = int(config['DEFAULT']['pen_down_delay'])
        self.acceleration = float(config['DEFAULT']['acceleration'])
        self.max_velocity = float(config['DEFAULT']['max_velocity'])
        self.corner_factor = float(config['DEFAULT']['corner_factor'])
        self.jog_acceleration = float(config['DEFAULT']['jog_acceleration'])
        self.jog_max_velocity = float(config['DEFAULT']['jog_max_velocity'])
        self.vid_pid = str(config['DEFAULT']['vid_pid'])

        self.error = (0, 0)  # accumulated step error

        port = find_port(self.vid_pid)
        if port is None:
            raise Exception('cannot find axidraw device')
        self.serial = Serial(port, timeout=1)
        self.configure()

    def write_settings(self):
        here = os.path.dirname(os.path.abspath(__file__))
        filename = os.path.join(here, 'axidraw.ini')
        config = ConfigParser()
        config['DEFAULT']['timeslice_ms'] = str(self.timeslice_ms)
        config['DEFAULT']['microstepping_mode'] = str(self.microstepping_mode)
        config['DEFAULT']['pen_up_position'] = str(self.pen_up_position)
        config['DEFAULT']['pen_up_speed'] = str(self.pen_up_speed)
        config['DEFAULT']['pen_up_delay'] = str(self.pen_up_delay)
        config['DEFAULT']['pen_down_position'] = str(self.pen_down_position)
        config['DEFAULT']['pen_down_speed'] = str(self.pen_down_speed)
        config['DEFAULT']['pen_down_delay'] = str(self.pen_down_delay)
        config['DEFAULT']['acceleration'] = str(self.acceleration)
        config['DEFAULT']['max_velocity'] = str(self.max_velocity)
        config['DEFAULT']['corner_factor'] = str(self.corner_factor)
        config['DEFAULT']['jog_acceleration'] = str(self.jog_acceleration)
        config['DEFAULT']['jog_max_velocity'] = str(self.jog_max_velocity)
        config['DEFAULT']['vid_pid'] = str(self.vid_pid)
        with open(filename, 'w') as configfile:
            config.write(configfile)

    def configure(self):
        servo_min = 7500
        servo_max = 28000
        pen_up_position = self.pen_up_position / 100
        pen_up_position = int(
            servo_min + (servo_max - servo_min) * pen_up_position)
        pen_down_position = self.pen_down_position / 100
        pen_down_position = int(
            servo_min + (servo_max - servo_min) * pen_down_position)
        self.command('SC', 4, pen_up_position)
        self.command('SC', 5, pen_down_position)
        self.command('SC', 11, int(self.pen_up_speed * 5))
        self.command('SC', 12, int(self.pen_down_speed * 5))

    def close(self):
        self.serial.close()

    def make_planner(self, jog=False):
        a = self.acceleration
        vmax = self.max_velocity
        cf = self.corner_factor
        if jog:
            a = self.jog_acceleration
            vmax = self.jog_max_velocity
        return Planner(a, vmax, cf)

    def readline(self):
        return self.serial.readline().decode('utf-8').strip()

    def command(self, *args):
        line = ','.join(map(str, args))
        self.serial.write((line + '\r').encode('utf-8'))
        return self.readline()

    # higher level functions
    def move(self, dx, dy):
        self.run_path([(0, 0), (dx, dy)])

    def goto(self, x, y, jog=True):
        # TODO: jog if pen up
        px, py = self.read_position()
        self.run_path([(px, py), (x, y)], jog)

    def home(self):
        self.goto(0, 0, True)

    # misc commands
    def version(self):
        return self.command('V')

    # motor functions
    def enable_motors(self):
        m = self.microstepping_mode
        return self.command('EM', m, m)

    def disable_motors(self):
        return self.command('EM', 0, 0)

    def motor_status(self):
        return self.command('QM')

    def zero_position(self):
        return self.command('CS')

    def read_position(self):
        response = self.command('QS')
        self.readline()
        a, b = map(int, response.split(','))
        a /= self.steps_per_unit
        b /= self.steps_per_unit
        y = (a - b) / 2
        x = y + b
        return x, y

    def stepper_move(self, duration, a, b):
        return self.command('XM', duration, a, b)

    def wait(self):
        while '1' in self.motor_status():
            time.sleep(0.01)

    def run_plan(self, plan):
        step_s = self.timeslice_ms / 1000
        t = 0
        while t < plan.t:
            i1 = plan.instant(t)
            i2 = plan.instant(t + step_s)
            d = i2.p.sub(i1.p)
            ex, ey = self.error
            ex, sx = modf(d.x * self.steps_per_unit + ex)
            ey, sy = modf(d.y * self.steps_per_unit + ey)
            self.error = ex, ey
            self.stepper_move(self.timeslice_ms, int(sx), int(sy))
            t += step_s
        # self.wait()

    def run_path(self, path, jog=False):
        planner = self.make_planner(jog)
        plan = planner.plan(path)
        self.run_plan(plan)

    def run_drawing(self, drawing, progress=True):
        print('number of paths : %d' % len(drawing.paths))
        print('pen down length : %g' % drawing.down_length)
        print('pen up length   : %g' % drawing.up_length)
        print('total length    : %g' % drawing.length)
        print('drawing bounds  : %s' % str(drawing.bounds))
        self.pen_up()
        position = (0, 0)
        bar = Bar(drawing.length, enabled=progress)
        for path in drawing.paths:
            jog = [position, path[0]]
            self.run_path(jog, jog=True)
            bar.increment(path_length(jog))
            self.pen_down()
            self.run_path(path)
            self.pen_up()
            position = path[-1]
            bar.increment(path_length(path))
        bar.done()
        self.run_path([position, (0, 0)], jog=True)

    def plan_drawing(self, drawing):
        result = []
        planner = self.make_planner()
        for path in drawing.all_paths:
            result.append(planner.plan(path))
        return result

    # pen functions
    def pen_up(self):
        delta = abs(self.pen_up_position - self.pen_down_position)
        duration = int(1000 * delta / self.pen_up_speed)
        delay = max(0, duration + self.pen_up_delay)
        return self.command('SP', 1, delay)

    def pen_down(self):
        delta = abs(self.pen_up_position - self.pen_down_position)
        duration = int(1000 * delta / self.pen_down_speed)
        delay = max(0, duration + self.pen_down_delay)
        return self.command('SP', 0, delay)
