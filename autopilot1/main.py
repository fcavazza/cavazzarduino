#qpy:kivy

from random import random
from math import sqrt
import time
from collections import deque

from kivy.app import App
from kivy.uix.widget import Widget
from kivy.uix.label import Label
from kivy.properties import NumericProperty, ReferenceListProperty, ObjectProperty, StringProperty
from kivy.clock import Clock
from kivy.graphics import Color,Line
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.anchorlayout import AnchorLayout
from kivy.uix.floatlayout import FloatLayout
from kivy.uix.gridlayout import GridLayout
from kivy.event import EventDispatcher
#from kivy.garden import gauge

import serial
from serial.tools.list_ports import comports

from kivy.core.window import Window
#Window.clearcolor = (1, 1, 1, 1)

import xlsxwriter

__version__ = '0.0.1'

TARGET_ARDUINO = '55431313338351113152'
MAX_SENSOR_DISTANCE = 200



#class ArduinoData(EventDispatcher):
#    pass
    
    
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

class BottomCommands(BoxLayout):
    pass

        
class RadarGrid(FloatLayout):
    lbl_angle = ObjectProperty(None)
    lbl_dist = ObjectProperty(None)
    
    def __init__(self, *args, **kwargs):
        self.targets = {}
        super(RadarGrid, self).__init__(*args, **kwargs)

    def updateTarget(self, angle, dist):
        if dist < 0: return
        if dist > MAX_SENSOR_DISTANCE: return
        angle = angle - 90 # arduino Servo works from 0 to 180 degrees, but circle widget from -90 to 90
        dist =  self.width / 2.0 / MAX_SENSOR_DISTANCE * dist # rescale distance to radar grid

        #if self.targets.get(angle): self.remove_widget(self.targets.pop(angle))
        #self.targets[angle] = RadarPulse(self, angle, dist)
        self.add_widget(self.targets[angle])
        
        self.lbl_angle.text = '%.2f' % angle
        self.lbl_dist.text = '%.2f' % dist
        
#    def on_touch_down_(self, touch):
#        #if self.lastangle:
#        #    print self.targets.items()
#        #    self.remove_widget(self.targets.pop(0))
#        #    print self.targets.items()
#        #    self.lastangle = False
#        #else:
#        #    angle = 90
#        #    distance = 100
#        #    self.lastangle = True
#        #    self.updateTarget(angle, distance)
#        
#        newstep = int(random() * 30)
#        self.log(newstep)
#        self.writeArduino(newstep)
#        for angle in self.targets.keys():
#            self.remove_widget(self.targets.pop(angle))
#            self.log('cancello angolo: %i' % angle)
#        return True
        
class LeftCommands(BoxLayout):
    lbl_port = ObjectProperty(None)
    lbl_command = ObjectProperty(None)
    lbl_log = ObjectProperty(None)

class AutopilotPage(BoxLayout):
    radarGrid = ObjectProperty(None)
    leftCommands = ObjectProperty(None)

class AutopilotApp(App):
    #data = ObjectProperty(ArduinoData(), rebind=True)
    distance = NumericProperty(0)
    cur_steering = NumericProperty(0)
    dir_A = NumericProperty(1)
    brake_A = NumericProperty(0)
    speed_A = NumericProperty(0)
    target_speed_A = NumericProperty(70)
    current = NumericProperty(0)
    strategyLock = StringProperty('-')
    strategy = StringProperty('G')
    strategyTime = NumericProperty(0)
    strategyStep = StringProperty('-')
    strategyStepTime = NumericProperty(0)
    lastCommCicles = NumericProperty(0)
    lastCommTime = NumericProperty(0)
    auto = StringProperty('normal')
    loglines_text = StringProperty()
    last_command = StringProperty()
    port = StringProperty()
    
    def __init__(self, *args, **kwargs):
        super(AutopilotApp, self).__init__(*args, **kwargs)
        self.serial = None
        self.lastConnectionAttempt = 0
        self.loglines = deque(maxlen=12)
        self.recording = False
        self.rec_distance = {}
        self.rec_status = {}
        #self.segno = 1
        
    def build(self):
        radar = AutopilotPage()
        Clock.schedule_interval(self.update, 1.0 / 100.0)
        self.connectArduino()
        return radar
        
    @property
    def auto_mode(self):
        return self.auto == 'down'
    
    def readArduinoData(self, datastring):
        checksum = 0
        for el in datastring[1:]:
            checksum ^= ord(el)
        if chr(checksum) != datastring[0]:
            self.log('** checksum error: %s - %s' % (chr(checksum), datastring[0]))
            return
            #pass
        d = datastring[1:].split(';')
        millis = int(d[0])
        self.distance = int(d[1])
        if self.auto_mode:
            self.cur_steering = int(d[2])
            self.dir_A = int(d[3])
            self.brake_A = int(d[4])
            self.target_speed_A = int(d[6])
        self.speed_A = int(d[5])
        self.current = float(d[7])
        self.strategyLock = 'y' if int(d[8]) else '-'
        self.strategy = d[9]
        strategyStart = int(d[10])
        self.strategyTime = millis - strategyStart
        self.strategyStep = d[11]
        strategyStepStart = int(d[12])
        self.strategyStepTime = millis - strategyStepStart
        self.lastCommCicles = int(d[13])
        lastCommTime = int(d[14])
        self.lastCommTime = millis - lastCommTime
        if self.recording:
            self.rec_status[millis] = d
            for distRec in d[15:-1]:
                dist, clock = distRec.split(':')
                self.rec_distance[clock] = dist
        
    def writeRecords(self):
        wb = xlsxwriter.Workbook('arduino_records.xlsx')
        ws = wb.add_worksheet('recordings')
        
        riga = 0
        
        #for recDict in (self.rec_status, self.rec_distance):
        for millis,data in self.rec_distance.items():
            riga += 1
            ws.write(riga, 0, millis)
            ws.write(riga, 1, data)
        wb.close()
        
    def connectArduino(self):
        if time.time() - self.lastConnectionAttempt < 1: return self.port
        self.lastConnectionAttempt = time.time()
        self.log('provo a connettere Arduino')
        #port = [p for p in comports() if TARGET_ARDUINO in p[2]]
        port = [p for p in comports() if 'HC-06' in p[0]]
        # get the right port based on the SN of target arduino
        if port:
            self.port = port[0][0] 
            try:
                self.serial = serial.Serial(self.port, 57600, timeout=1)
                self.log('Arduino connesso su porta %s' % self.port)
            except:
                self.log('Errore di connessione su porta %s' % self.port)
                pass
            
        else:
            self.port = 'ND'
            self.serial = None
            self.log('non trovato Arduino')
            
        #self.lbl_port.text = str(self.port or 'no connection to Arduino')
        return self.port
        
    def readArduino(self):
        if not self.serial:
            self.connectArduino()
            return
        line = self.serial.readline()
        line = line.strip()
        print "letto: '%s'" % line
        if not line:
            return
        if line.startswith('# '):
            self.log(line)
            return
        #try:
        self.readArduinoData(line)
        #except:
        #    self.log('** transmission error **')
        #angle, dist = [int(v) for v in line.split(';')]
        #self.radarGrid.updateTarget(angle, dist)

    def writeArduino(self):
        if self.auto_mode:
            cmd = 'A'
        else:
            cmd = str('M%i%03i%+03i%i' % (self.dir_A, 
                    self.target_speed_A, self.cur_steering, self.brake_A))
        
        self.sendBTArduino(cmd)
        
    def sendBTArduino(self, cmd):
        if not self.serial: return
        checksum = 0
        for el in cmd:
            checksum ^= ord(el)
        cmd = '%s%s\n' % (chr(checksum), cmd)
        self.serial.write(cmd)
        print "scritto: %s" % cmd
        self.last_command = cmd


    def update(self, dt):
        self.readArduino()
        self.writeArduino()
        
        #if self.target_speed_A == 100:
        #    self.segno = -1
        #elif self.target_speed_A == 0:
        #    self.segno = 1
        #self.target_speed_A += (1 * self.segno)
        #self.cur_steering += (1 * self.segno)
        
    def log(self, txt):
        print txt
        self.loglines.append(unicode(txt, 'latin-1','ignore'))
        self.loglines_text = '\n'.join(self.loglines)
        #self.leftCommands.lbl_log.text = '\n'.join(self.loglines)
        
    def startRecording(self, state):
        self.recording = (state == 'down')
        if not self.recording and len(self.rec_distance) > 0: # stop recording
            self.writeRecords()
            

if __name__ == '__main__':
    AutopilotApp().run()