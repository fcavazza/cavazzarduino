#qpy:kivy

from random import random
from math import sqrt
import time
from collections import deque

from kivy.app import App
from kivy.uix.widget import Widget
from kivy.uix.label import Label
from kivy.properties import NumericProperty, ReferenceListProperty,\
    ObjectProperty
from kivy.clock import Clock
from kivy.graphics import Color,Line


import serial
from serial.tools.list_ports import comports

__version__ = '0.0.1'

TARGET_ARDUINO = '55431313338351113152'
MAX_SENSOR_DISTANCE = 200

class RadarPulse(Widget):
    def __init__(self, parent, angle, distance, *args, **kwargs):
        self.angle = angle
        self.distance = distance
        self.center_x = parent.center_x
        self.center_y = parent.center_y - parent.width / 2 + 100
        super(RadarPulse, self).__init__(*args, **kwargs)
        print self.angle
        print self.distance
        print self.center
        
class RadarGrid(Widget):
    pass
    
class RadarPage(Widget):
    lbl_angle = ObjectProperty(None)
    lbl_dist = ObjectProperty(None)
    radar_grid = ObjectProperty(None)
    lbl_port = ObjectProperty(None)
    lbl_log = ObjectProperty(None)
    
    def __init__(self, *args, **kwargs):
        self.targets = {}
        super(RadarPage, self).__init__(*args, **kwargs)
        self.port = None
        self.serial = None
        self.lastConnectionAttempt = 0
        self.loglines = deque(maxlen=5)
        self.connectArduino()
        self.lastangle = None
        
    def on_touch_down(self, touch):
        #if self.lastangle:
        #    print self.targets.items()
        #    self.remove_widget(self.targets.pop(0))
        #    print self.targets.items()
        #    self.lastangle = False
        #else:
        #    angle = 90
        #    distance = 100
        #    self.lastangle = True
        #    self.updateTarget(angle, distance)
        
        newstep = int(random() * 30)
        self.log(newstep)
        self.writeArduino(newstep)
        for angle in self.targets.keys():
            self.remove_widget(self.targets.pop(angle))
            self.log('cancello angolo: %i' % angle)
        return True
        
    def updateTarget(self, angle, dist):
        if dist < 0: return
        if dist > MAX_SENSOR_DISTANCE: return
        angle = angle - 90 # arduino Servo works from 0 to 180 degrees, but circle widget from -90 to 90
        dist =  self.width / 2.0 / MAX_SENSOR_DISTANCE * dist # rescale distance to radar grid
        print "dist: %.2f" % dist

        if self.targets.get(angle): self.remove_widget(self.targets.pop(angle))
        self.targets[angle] = RadarPulse(self.radar_grid, angle, dist)
        self.add_widget(self.targets[angle])
        
        self.lbl_angle.text = '%.2f' % angle
        self.lbl_dist.text = '%.2f' % dist
        
    def connectArduino(self):
        if time.time() - self.lastConnectionAttempt < 1: return self.port
        self.lastConnectionAttempt = time.time()
        self.log('provo a connettere Arduino')
        port = [p for p in comports() if TARGET_ARDUINO in p[2]]
        # get the right port based on the SN of target arduino
        if port:
            self.port = port[0][0] 
            self.serial = serial.Serial(self.port, 250000, timeout=1)
            self.log('Arduino connesso su porta %s' % self.port)
        else:
            self.port = None
            self.serial = None
            self.log('non trovato Arduino')
            
        self.lbl_port.text = str(self.port or 'no connection to Arduino')
        return self.port
        
    def readArduino(self):
        if not self.serial:
            self.connectArduino()
            return
        line = self.serial.readline()
        if not line:
            return
        if line.startswith('#'):
            self.log(line)
            return
        print line
        angle, dist = [int(v) for v in line.split(';')]
        self.updateTarget(angle, dist)

    def writeArduino(self, newstep):
        if not self.serial: return
        self.serial.write('%i\n' % (newstep, ))

    def update(self, dt):
        self.readArduino()
        
    def log(self, txt):
        print txt
        self.loglines.append(str(txt))
        self.lbl_log.text = '\n'.join(self.loglines)

class RadarApp(App):
    def build(self):
        radar = RadarPage()
        Clock.schedule_interval(radar.update, 1.0 / 100.0)
        return radar
        


if __name__ == '__main__':
    RadarApp().run()