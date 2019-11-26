#! /usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright (c) 2013 PAL Robotics SL.
# Released under the BSD License.
#
# Authors:
#   * Siegfried-A. Gevatter

import curses
import math

import rospy
from geometry_msgs.msg import Twist


class TextWindow():

    _screen = None
    _window = None
    _num_lines = None

    def __init__(self, stdscr, lines=10):
        self._screen = stdscr
        self._screen.nodelay(True)
        curses.curs_set(0)

        self._num_lines = lines

    def read_key(self):
        keycode = self._screen.getch()
        return keycode if keycode != -1 else None

    def clear(self):
        self._screen.clear()

    def write_line(self, lineno, message):
        if lineno < 0 or lineno >= self._num_lines:
            raise ValueError, 'lineno out of bounds'
        height, width = self._screen.getmaxyx()
        y = (height / self._num_lines) * lineno
        x = 10
        for text in message.split('\n'):
            text = text.ljust(width)
            self._screen.addstr(y, x, text)
            y += 1

    def refresh(self):
        self._screen.refresh()

    def beep(self):
        curses.flash()
        

class SimpleKeyTeleop():
    def __init__(self, interface):
        self._interface = interface
        self._pub_cmd = rospy.Publisher('key_vel', Twist)

        self._hz = rospy.get_param('~hz', 10)

        self._linear_rate = rospy.get_param('~linear_rate', 10.0)
        self._forward_max = rospy.get_param('~forward_max', 300)
        self._backward_max = rospy.get_param('~backward_max', -300)
        self._angular_rate = rospy.get_param('~angular_rate', 5.0)
        self._rotation_max = rospy.get_param('~rotation_max', 50)
        self._last_pressed = {}
        self._angular = 0
        self._linear = 0
        
        self.linearSpeed = 0
        self.angularSpeed = 0

    movement_bindings = {
        curses.KEY_UP:    ( 1,  0),
        curses.KEY_DOWN:  (-1,  0),
        curses.KEY_LEFT:  ( 0,  1),
        curses.KEY_RIGHT: ( 0, -1),
    }

    def run(self):
        rate = rospy.Rate(self._hz)
        self._running = True
        while self._running:
            while True:
                keycode = self._interface.read_key()
                if keycode is None:
                    break
                self._key_pressed(keycode)
            self._set_velocity()
            self._publish()
            rate.sleep()

    def _get_twist(self, linear, angular):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        return twist

    def _set_velocity(self):
        now = rospy.get_time()
        keys = []
        for a in self._last_pressed:
            if now - self._last_pressed[a] < 0.4:
                keys.append(a)
        linear = 0.0
        angular = 0.0
        for k in keys:
            l, a = self.movement_bindings[k]
            linear += l
            angular += a
        if linear > 0:
            if self.linearSpeed < self._forward_max:
                self.linearSpeed += self._linear_rate
            else:
                self.linearSpeed = self._forward_max
        elif linear < 0:
            if self.linearSpeed > self._backward_max:
                self.linearSpeed -= self._linear_rate
            else:
                self.linearSpeed = self._backward_max
        else: # 0 request
            if abs(self.linearSpeed) < 2 * self._linear_rate:
                self.linearSpeed = 0
            elif self.linearSpeed > 0:
                self.linearSpeed -= self._linear_rate
            elif self.linearSpeed < 0:
                self.linearSpeed += self._linear_rate
            else:
                raise("Invalid path")
            
            
        if angular != 0:
            if abs(self.angularSpeed) < self._rotation_max:
                self.angularSpeed += angular*self._angular_rate
            else:
                self.angularSpeed = angular*self._rotation_max

        else: # 0 request
            if abs(self.angularSpeed) < 2 * self._angular_rate:
                self.angularSpeed = 0
            elif self.angularSpeed > 0:
                self.angularSpeed -= self._angular_rate
            else:
                self.angularSpeed += self._angular_rate
            

        self._angular = self.angularSpeed
        self._linear = self.linearSpeed

    def _key_pressed(self, keycode):
        if keycode == ord('q'):
            self._running = False
            rospy.signal_shutdown('Bye')
        elif keycode in self.movement_bindings:
            self._last_pressed[keycode] = rospy.get_time()

    def _publish(self):
        self._interface.clear()
        self._interface.write_line(2, 'Linear: %f, Angular: %f' % (self._linear, self._angular))
        self._interface.write_line(5, 'Use arrow keys to move, q to exit.')
        self._interface.refresh()

        twist = self._get_twist(self._linear, self._angular)
        self._pub_cmd.publish(twist)


def main(stdscr):
    rospy.init_node('key_teleop')
    app = SimpleKeyTeleop(TextWindow(stdscr))
    app.run()

if __name__ == '__main__':
    try:
        curses.wrapper(main)
    except rospy.ROSInterruptException:
        pass
