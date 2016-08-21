#! /usr/bin/env python
# -*- coding: UTF-8 -*-
 
#
# GTK imports
#
import pygtk
import gi
gi.require_version('Gtk', '3.0')
from gi.repository import GLib, Gtk, Gdk, GObject, GdkPixbuf
#
# DroneKit imports
#
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil
#
# Other imports
#
import sys, getopt, os, time, math, threading, urllib, struct, socket
import pyjulius, Queue
from time import gmtime, strftime
from subprocess import call

class Joystick(GObject.GObject): 
    #
    # The Joystick class is a GObject that sends signals that represent Joystick events 
    #
    EVENT_BUTTON = 0x01 #button pressed/released 
    EVENT_AXIS = 0x02  #axis moved  
    EVENT_INIT = 0x80  #button/axis initialized  
    #
    #see http://docs.python.org/library/struct.html for the format determination 
    #
    EVENT_FORMAT = "IhBB" 
    EVENT_SIZE = struct.calcsize(EVENT_FORMAT) 
    #
    # we need a few signals to send data to the main 
    # signals will return 4 variables as follows: 
    #         1. a string representing if the signal is from an axis or a button 
    #         2. an integer representation of a particular button/axis 
    #         3. an integer representing axis direction or button press/release 
    #         4. an integer representing the "init" of the button/axis 
    #
    __gsignals__ = { 
    'axis-release' : 
    (GObject.SIGNAL_RUN_LAST, GObject.TYPE_NONE, 
    (GObject.TYPE_INT,)),
    'axis-move' : 
    (GObject.SIGNAL_RUN_LAST, GObject.TYPE_NONE, 
    (GObject.TYPE_INT,GObject.TYPE_INT)),
    'button-release' : 
    (GObject.SIGNAL_RUN_LAST, GObject.TYPE_NONE, 
    (GObject.TYPE_INT,)),
    'button-press' : 
    (GObject.SIGNAL_RUN_LAST, GObject.TYPE_NONE, 
    (GObject.TYPE_INT,))
    } 

    def __init__(self,dev_num): 
        GObject.GObject.__init__(self)
        #
        # Define the device 
        #
        device = '/dev/input/js%s' % dev_num 
        #
        # Error check that this can be read ... is the Joystick connected??? 
        #
        try: 
            #
            # Open the joystick device 
            #
            self.device = open(device) 
            #
            # Keep an eye on the device, when there is data to read, execute the read function 
            #
            GObject.io_add_watch(self.device,GObject.IO_IN,self.read_buttons) 
            self.Connected=True
        except Exception,ex: 
            #
            # Raise an exception 
            #
            # raise Exception( ex )
            self.Connected=False

    def read_buttons(self, arg0='', arg1=''): 
        #
        # Read the button and axis press event from the joystick device 
        # and emit a signal containing the event data 
        #
        # Read self.EVENT_SIZE bytes from the joystick 
        #
        read_event = self.device.read(self.EVENT_SIZE)   
        #
        # Get the event structure values from  the read event 
        #
        time, value, type, number = struct.unpack(self.EVENT_FORMAT, read_event) 
        #
        # Get just the button/axis press event from the event type  
        #
        event = type & ~self.EVENT_INIT 
        #
        # Get just the INIT event from the event type 
        #
        init = type & ~event 
        if event == self.EVENT_AXIS:
           if value == 0.0:              # Release axis
              signal = "axis-release"
              self.emit(signal,number)
           else:                         # Move axis
              signal = "axis-move"
              self.emit(signal,number,value)
        elif event == self.EVENT_BUTTON:
           if value == 0.0:              # Release button
              signal = "button-release"
              self.emit(signal,number)
           else:                         # Press button
              signal = "button-press"
              self.emit(signal,number)
        return True
        # print("%s %s %s %s" % (signal,number,value,init) ) 
        # self.emit(signal,number,value,init)

class VideoThread(threading.Thread):
    '''
    A background thread that takes the MJPEG stream and
    updates the GTK image.
    '''
    def __init__(self, widget,STREAM_URL='http://8.8.8.8:8081'):
        super(VideoThread, self).__init__()
        self.widget = widget
        self.quit = False
        self.stream = urllib.urlopen(STREAM_URL)

    def get_raw_frame(self):
        '''
        Parse an MJPEG http stream and yield each frame.
        Source: http://stackoverflow.com/a/21844162
        :return: generator of JPEG images
        '''
        raw_buffer = ''
        while True:
            new = self.stream.read(256)
            if not new:
                # Connection dropped
                yield None
            raw_buffer += new
            a = raw_buffer.find('\xff\xd8')
            b = raw_buffer.find('\xff\xd9')
            if a != -1 and b != -1:
                frame = raw_buffer[a:b+2]
                raw_buffer = raw_buffer[b+2:]
                yield frame

    def run(self):
        for frame in self.get_raw_frame():
            if self.quit or frame is None:
                return
            #loader = Gtk.Gdk.PixbufLoader('jpeg')
            loader = GdkPixbuf.PixbufLoader()
            loader.write(frame)
            loader.close()
            pixbuf = loader.get_pixbuf()
            # Schedule image update to happen in main thread
            GObject.idle_add(self.widget.set_from_pixbuf, pixbuf)

class MainWindow:
     
    def __init__(self,ynetbook=False):
        #
        # Initialize Variables
        #
        self.InitializeVar()
        #
        # Initialize Widgets (from xml file)
        #
        self.InitializeWidgets()
        #
        # Connect Signals
        #
        self.builder.connect_signals(self)
        self.LogText.connect("size-allocate", self._autoscroll)
        #
        # Initial Properties
        #
        self.SensitiveAllButtons(False)
        self.ConnectButton.set_sensitive(True) 
        self.Protocol.set_sensitive(True) 
        self.ProtocolU.set_sensitive(True) 
        self.IPText.set_sensitive(True)
        self.IPText.grab_focus()
        self.PortText.set_sensitive(True) 
        self.PortText.set_sensitive(True)
        self.CloseButton.set_sensitive(True)
        self.MainWindow.show_all()

    def InitializeVar(self, *args):
        #
        # Initialize variables
        #
        self.video_port="8081"
        self.yVideo=True
        self.DefaultLength=15
        self.DefaultVelocity=1.5
        self.Defaultaltitude=5.0
        self.DefaultMaxaltitude=100.0
        self.DefaultAmplitude=1.0
        self.DefaultMode="GUIDED"
        self.Defaultrc_pitch=1500
        self.Defaultrc_roll=1500
        self.Defaultrc_yaw=1500
        self.Defaultrc_throttle=1360
        self.longitude=0.0
        self.latitude=0.0
        self.altitudeGPS=0.0
        self.altitudeREL=0.0
        self.climb=0.0
        self.pitch=0.0
        self.roll=0.0
        self.yaw=0.0
        self.rc_overrides=False
        self.rc_pitch=1500
        self.rc_roll=1500
        self.rc_yaw=1500
        self.rc_throttle=1000
        self.rc_throttle_max = 2000
        self.rc_throttle_min = 1000
        self.rc_throttle_inc = 1
        self.Defaultrc_throttle_alt_hold = 1610
        self.rc_roll_max = 2000
        self.rc_roll_min = 1000
        self.rc_pitch_max = 2000
        self.rc_pitch_min = 1000
        self.rc_yaw_max = 2000
        self.rc_yaw_min = 1000
        self.voltage=0.0
        self.current=0.0
        self.level=0.0
        self.altitudeError=0.25
        self.currentYaw=0
        self.DeltaDegrees=2.5
        self.LastAltitude=0
        self.Last_rc_throttle=0
        self.LastActiveIndex=0
        self.gpsError=0.40
        self.yRTL=False
        self.yLAND=False
        self.yWave=False
        self.yConnect=False
        self.timeout=1
        self.timeoutLog = 400
        self.timeVoice = 0.1
        self.message=" "
        self.id_callback=None
        self.id_log=None
        self.last_location=None
        self.failSafe_Battery=10
        self.iesdeltebreLon=40.721553
        self.iesdeltebreLat=0.722831
        self.iesdeltebreAlt=6.0
        self.iesdeltebreHeading=348
        self.ConnectionString=" "
        self.ProtocolString="tcp"
        self.yKeyboardControl=False
        self.yJoystickControl=False
        self.yVoiceControl=False
        self.yBCIControl=False
        self.a_press=False
        self.z_press=False
        self.s_press=False
        self.x_press=False
        self.m_press=False
        self.q_press=False
        self.space_press=False
        self.Up_press=False
        self.Down_press=False
        self.Right_press=False
        self.Left_press=False
        self.c_press=False
        self.n_press=False
        self.t_press=False
        self.w_press=False
        self.v_press=False
        self.Control_R_press=False
        self.axis_0=False
        self.value_axis_0=0
        self.axis_1=False
        self.value_axis_0=0
        self.axis_2=False
        self.value_axis_0=0
        self.axis_3=False
        self.value_axis_0=0
        self.axis_4=False
        self.value_axis_0=0
        self.axis_5=False
        self.value_axis_0=0
        self.axis_6=False
        self.value_axis_0=0
        self.axis_7=False
        self.value_axis_0=0
        self.button_0=False
        self.button_1=False
        self.button_2=False
        self.button_3=False
        self.button_4=False
        self.button_5=False
        self.button_6=False
        self.button_7=False
        self.button_8=False
        self.button_9=False
        self.button_10=False
        self.button_11=False
        self.button_12=False
        self.RangeJostick=32767
        self.DeltaJostick=200
        return True

    def InitializeWidgets(self, *args):
        #
        # XML file of GLADE for Gtk.Builder()
        #
        self.builderfile = "DroneNet.glade"
        self.builder = Gtk.Builder()
        self.builder.add_from_file(self.builderfile)
        #
        # Define the widgets from Gtk.Builder()
        #
        self.MainWindow = self.builder.get_object("MainWindow")
        self.Protocol = self.builder.get_object("ProtocolRadioButton")
        self.ProtocolU = self.builder.get_object("ProtocolRadioButtonU")
        self.IPText = self.builder.get_object("IPText")
        self.PortText = self.builder.get_object("PortText")
        self.ConnectButton = self.builder.get_object("ConnectButton")
        self.DisConnectButton = self.builder.get_object("DisConnectButton")
        self.CloseButton = self.builder.get_object("CloseButton")
        self.KeyboardButton = self.builder.get_object("KeyboardButton")
        self.JoystickButton = self.builder.get_object("JoystickButton")
        self.VoiceButton = self.builder.get_object("VoiceButton")
        self.BCIButton = self.builder.get_object("BCIButton")
        self.StopButton = self.builder.get_object("StopButton")
        self.LeftUpButton = self.builder.get_object("LeftUpButton")
        self.LeftUpButton = self.builder.get_object("LeftUpButton")
        self.UpButton = self.builder.get_object("UpButton")
        self.RightUpButton = self.builder.get_object("RightUpButton")
        self.LeftButton = self.builder.get_object("LeftButton")
        self.SpinLeftButton = self.builder.get_object("SpinLeftButton")
        self.StayButton = self.builder.get_object("StayButton")
        self.RightButton = self.builder.get_object("RightButton")
        self.SpinRightButton = self.builder.get_object("SpinRightButton")
        self.LeftDownButton = self.builder.get_object("LeftDownButton")
        self.DownButton = self.builder.get_object("DownButton")
        self.RightDownButton = self.builder.get_object("RightDownButton")
        self.AltitudeScale = self.builder.get_object("AltitudeScale")
        self.AltitudeScaleAdjust = self.builder.get_object("AltitudeScaleAdjust")
        self.ThrottleScaleAdjust = self.builder.get_object("ThrottleScaleAdjust")
        self.ArmButton = self.builder.get_object("ArmButton")
        self.StayButton = self.builder.get_object("StayButton")
        self.SquareButton = self.builder.get_object("SquareButton")
        self.CircleButton = self.builder.get_object("CircleButton")
        self.WaveButton = self.builder.get_object("WaveButton")
        self.TriangleButton = self.builder.get_object("TriangleButton")
        self.ModeComboBox = self.builder.get_object("ModeComboBox")
        self.ModeListStore = self.builder.get_object("ModeListStore")
        self.Logscrolledwindow = self.builder.get_object("Logscrolledwindow")
        self.LogText = self.builder.get_object("LogTextView")
        self.VideoImg = self.builder.get_object("VideoImg")
        self.TextBuffer = self.LogText.get_buffer() 
        self.tag = self.TextBuffer.create_tag("red_bg", foreground="red")
        #
        # Labels to update the info
        #
        self.LongitudeLabel = self.builder.get_object("LongitudeLabel")
        self.LatitudeLabel  = self.builder.get_object("LatitudeLabel")
        self.AltitudeGPSLabel  = self.builder.get_object("AltitudeGPSLabel")
        self.AltitudeRELLabel  = self.builder.get_object("AltitudeRELLabel")
        self.SpeedLabel  = self.builder.get_object("SpeedLabel")
        self.ClimbLabel  = self.builder.get_object("ClimbLabel")
        self.PitchLabel  = self.builder.get_object("PitchLabel")
        self.RollLabel  = self.builder.get_object("RollLabel")
        self.YawLabel  = self.builder.get_object("YawLabel")
        self.VoltageLabel  = self.builder.get_object("VoltageLabel")
        self.IntensityLabel  = self.builder.get_object("IntensityLabel")
        self.LevelLabel  = self.builder.get_object("LevelLabel")
        return True

#-----------------------------------------------------------------------------------------
#   The next def are the SIGNALS connections of the UI
#-----------------------------------------------------------------------------------------

    def on_KeyboardButton_clicked(self, *args):
        #
        # Control de UI with the keyboard and on_key_press function
        #
        #
        # Activate UI Buttons.
        #
        self.SensitiveAllButtons(False)
        self.yKeyboardControl=True
        self.handler_id1=self.MainWindow.connect('key-press-event', self.on_key_press)
        self.handler_id2=self.MainWindow.connect('key-release-event', self.on_key_release)
        self._log(" Activate the keyboard control. Press q(uit) to EXIT ...")
        return True

    def on_key_release(self, MainWindow, event):
        #
        # Control de UI with the keyboard and key_release
        #
        keyname = Gdk.keyval_name(event.keyval)
        if self.yKeyboardControl:
           if keyname == 'a' or keyname == 'A':
              self.a_press=False
           if keyname == 'z' or keyname == 'Z':
              self.z_press=False
           if keyname == 's' or keyname == 'S':
              self.s_press=False
           if keyname == 'x' or keyname == 'X':
              self.x_press=False
           if keyname == 'm' or keyname == 'M':
              self.m_press=False
           if keyname == 'q' or keyname == 'Q':
              self.q_press=False
           if keyname == 'space':
              self.space_press=False
           if keyname == 'Up':
              self.Up_press=False
           if keyname == 'Down':
              self.Down_press=False
           if keyname == 'Right':
              self.Right_press=False
           if keyname == 'Left':
              self.Left_press=False
           if keyname == 'Control_R':
              self.Control_R_press=False
           if keyname == 'c' or keyname == 'C':
              self.c_press=False
           if keyname == 'n' or keyname == 'N':
              self.n_press=False
           if keyname == 't' or keyname == 'T':
              self.t_press=False
           if keyname == 'w' or keyname == 'W':
              self.w_press=False
           if keyname == 'v' or keyname == 'V':
              self.v_press=False

    def on_key_press(self, MainWindow, event):
        #
        # Control de UI with the keyboard and key_press
        #
        keyname = Gdk.keyval_name(event.keyval)
        if self.yKeyboardControl:
           if keyname == 'a' or keyname == 'A':
              self.a_press=True
           if keyname == 'z' or keyname == 'Z':
              self.z_press=True
           if keyname == 's' or keyname == 'S':
              self.s_press=True
           if keyname == 'x' or keyname == 'X':
              self.x_press=True
           if keyname == 'm' or keyname == 'M':
              self.m_press=True
           if keyname == 'q' or keyname == 'Q':
              self.q_press=True
           if keyname == 'space':
              self.space_press=True
           if keyname == 'Up':
              self.Up_press=True
           if keyname == 'Down':
              self.Down_press=True
           if keyname == 'Right':
              self.Right_press=True
           if keyname == 'Left':
              self.Left_press=True
           if keyname == 'Control_R':
              self.Control_R_press=True
           if keyname == 'c' or keyname == 'C':
              self.c_press=True
           if keyname == 'n' or keyname == 'N':
              self.n_press=True
           if keyname == 't' or keyname == 'T':
              self.t_press=True
           if keyname == 'w' or keyname == 'W':
              self.w_press=True
           if keyname == 'v' or keyname == 'V':
              self.v_press=True
           #
           # Event treatment
           #
           if self.q_press:
              self.yKeyboardControl=False
              self.MainWindow.disconnect(self.handler_id1)
              self.MainWindow.disconnect(self.handler_id2)
              #
              # Activate UI Buttons.
              #
              self.ArmButton.set_sensitive(True)
              self.DisConnectButton.set_sensitive(True)
              self.ModeComboBox.set_sensitive(True)
              self.KeyboardButton.set_sensitive(True)
              self.JoystickButton.set_sensitive(True)
              self.VoiceButton.set_sensitive(True)
              self.BCIButton.set_sensitive(True)
              self.CloseButton.set_sensitive(True)
              return True
           if not self.vehicle.armed:
              if self.space_press:
                 self.on_ArmButton_clicked(self.ArmButton)
                 self.SensitiveAllButtons(False)
                 return True
              if self.m_press:
                 model = self.ModeComboBox.get_model()
                 active = self.ModeComboBox.get_active()
                 active=active+1
                 if active > 3:
                    active=0
                 self.ModeComboBox.set_active(active)
                 self.SensitiveAllButtons(False)
                 return True
              if self.n_press:
                 model = self.ModeComboBox.get_model()
                 active = self.ModeComboBox.get_active()
                 active=active-1
                 if active < 0:
                    active=3
                 self.ModeComboBox.set_active(active)
                 self.SensitiveAllButtons(False)
                 return True
           else:
              if self.space_press:
                 if self.vehicle.mode == "GUIDED":
                    self.on_StayButton_clicked(self.StayButton)
                 elif self.vehicle.mode == "STABILIZE" or self.vehicle.mode == "ALT_HOLD":
                    active=3
                    self.ModeComboBox.set_active(active)
                 self.SensitiveAllButtons(False)
                 return True
              if self.m_press:
                 model = self.ModeComboBox.get_model()
                 active = self.ModeComboBox.get_active()
                 active=active+1
                 if active > 3:
                    active=0
                 self.ModeComboBox.set_active(active)
                 self.SensitiveAllButtons(False)
                 return True
              if self.n_press:
                 model = self.ModeComboBox.get_model()
                 active = self.ModeComboBox.get_active()
                 active=active-1
                 if active < 0:
                    active=3
                 self.ModeComboBox.set_active(active)
                 self.SensitiveAllButtons(False)
                 return True
              if self.a_press:
                 self.AltitudeKey=self.AltitudeKey+self.AltitudeScaleAdjust.get_step_increment()
                 self.AltitudeScale.set_value(self.AltitudeKey)
                 return True
              if self.z_press:
                 self.AltitudeKey=self.AltitudeKey-self.AltitudeScaleAdjust.get_step_increment()
                 self.AltitudeScale.set_value(self.AltitudeKey)
                 return True
              if self.s_press:
                 self.on_SpinRightButton_clicked(self.SpinRightButton)
                 return True
              if self.x_press:
                 self.on_SpinLeftButton_clicked(self.SpinLeftButton)
                 return True
              if self.Up_press and self.Right_press:
                 self.on_RightUpButton_clicked(self.RightUpButton)
                 return True
              if self.Up_press and self.Left_press:
                 self.on_LeftUpButton_clicked(self.LeftUpButton)
                 return True
              if self.Down_press and self.Left_press:
                 self.on_LeftDownButton_clicked(self.LeftDownButton)
                 return True
              if self.Down_press and self.Right_press:
                 self.on_RightDownButton_clicked(self.RightDownButton)
                 return True
              if self.Up_press:
                 self.on_UpButton_clicked(self.UpButton)
                 return True
              if self.Down_press:
                 self.on_DownButton_clicked(self.DownButton)
                 return True
              if self.Right_press:
                 self.on_RightButton_clicked(self.RightButton)
                 return True
              if self.Left_press:
                 self.on_LeftButton_clicked(self.LeftButton)
                 return True
              if self.Control_R_press:
                 self.on_StopButton_clicked(self.StopButton)
                 return True
              if self.c_press:
                 if self.vehicle.mode == "GUIDED":
                    self.Trace_Circle()
                 return True
              if self.v_press:
                 if self.vehicle.mode == "GUIDED":
                    self.Trace_Square()
                 return True
              if self.t_press:
                 if self.vehicle.mode == "GUIDED":
                    self.Trace_Triangle()
                 return True
              if self.w_press:
                 if self.vehicle.mode == "GUIDED":
                    self.Trace_Wave()
                 return True
        return True

#-----------------------------------------------------------------------------------------

    def on_JoystickButton_clicked(self, *args):
        #
        # Control de UI with the Joystick and the Joystick Class
        #
        self.SensitiveAllButtons(False)
        self.yJoystickControl=True
        try:
            num_dev=0
            self.joystick=Joystick(num_dev)
            self.handler_id1=self.joystick.connect('axis-release', self.on_Joystick_axis_release, *args)
            self.handler_id2=self.joystick.connect('axis-move', self.on_Joystick_axis_move, *args)
            self.handler_id3=self.joystick.connect('button-release', self.on_Joystick_button_release, *args)
            self.handler_id4=self.joystick.connect('button-press', self.on_Joystick_button_press, *args)
            self._log(" Joystick connected on: /dev/input/js%s" % num_dev)
            self.yFirst_axis=True
        except:
            self._log(" No Joystick connected.")
            return False
        return True
    def on_Joystick_axis_release(self,signal,number,kk):
        if self.yFirst_axis:
           self.yFirst_axis=False
           return True
        if self.axis_2 and self.axis_3:
           self.axis_2=False
           self.value_axis_2=0
           self.axis_3=False
           self.value_axis_3=0
           self.rc_pitch=self.Defaultrc_pitch
           self.vehicle.channels.overrides[1] = self.rc_pitch
           self.rc_roll=self.Defaultrc_roll
           self.vehicle.channels.overrides[2] = self.rc_roll
           return True
        if number == 0:
           self.axis_0=False
           self.value_axis_0=0
           self.rc_yaw=self.Defaultrc_yaw
           self.vehicle.channels.overrides[4] = self.rc_yaw
        elif number == 1:
           self.axis_1=False
           self.value_axis_1=0
           if self.vehicle.mode == "GUIDED" or self.vehicle.mode == "LAND" or self.vehicle.mode == "RTL" :
              return
           if self.vehicle.mode == "STABILIZE":
              self.rc_throttle=self.Defaultrc_throttle
           if self.vehicle.mode == "ALT_HOLD":
              self.rc_throttle=self.Defaultrc_throttle_alt_hold
           self.AltitudeScale.set_value(self.rc_throttle)
        elif number == 2:
           self.axis_2=False
           self.value_axis_2=0
           self.rc_pitch=self.Defaultrc_pitch
           self.vehicle.channels.overrides[1] = self.rc_pitch
        elif number == 3:
           self.axis_3=False
           self.value_axis_3=0
           self.rc_roll=self.Defaultrc_roll
           self.vehicle.channels.overrides[2] = self.rc_roll
        elif number == 4:
           self.axis_4=False
           self.value_axis_4=0
        elif number == 5:
           self.axis_5=False
           self.value_axis_5=0
        elif number == 6:
           self.axis_6=False
           self.value_axis_6=0
        elif number == 7:
           self.axis_7=False
           self.value_axis_7=0
        return True
    def on_Joystick_axis_move(self,signal,number,value,kk):
        if number == 0:
           self.axis_0=True
           self.value_axis_0=value
        elif number == 1:
           self.axis_1=True
           self.value_axis_1=value
        elif number == 2:
           self.axis_2=True
           self.value_axis_2=value
        elif number == 3:
           self.axis_3=True
           self.value_axis_3=value
        elif number == 4:
           self.axis_4=True
           self.value_axis_4=value
        elif number == 5:
           self.axis_5=True
           self.value_axis_5=value
        elif number == 6:
           self.axis_6=True
           self.value_axis_6=value
        elif number == 7:
           self.axis_7=True
           self.value_axis_7=value
        #
        # Axis <--> Movement Assignement: axis_0 -> Yaw, axis_1 -> throttle (I), axis_2 -> pitch, axis_3 -> roll (I)
        #
        if self.axis_4 and self.button_2:
           self.yJoystickControl=False
           self.joystick.disconnect(self.handler_id1)
           self.joystick.disconnect(self.handler_id2)
           self.joystick.disconnect(self.handler_id3)
           self.joystick.disconnect(self.handler_id4)
           del self.joystick
           #
           # Activate UI Buttons.
           #
           self.ArmButton.set_sensitive(True)
           self.DisConnectButton.set_sensitive(True)
           self.ModeComboBox.set_sensitive(True)
           self.KeyboardButton.set_sensitive(True)
           self.JoystickButton.set_sensitive(True)
           self.VoiceButton.set_sensitive(True)
           self.BCIButton.set_sensitive(True)
           self.CloseButton.set_sensitive(True)
           return True
        if self.axis_5:
           if self.value_axis_5 < 0.0:
              model = self.ModeComboBox.get_model()
              active = self.ModeComboBox.get_active()
              active=active+1
              if active > 3:
                 active=0
              self.ModeComboBox.set_active(active)
              self.SensitiveAllButtons(False)
              return True
           elif self.value_axis_5 > 0.0:
              model = self.ModeComboBox.get_model()
              active = self.ModeComboBox.get_active()
              active=active-1
              if active < 0:
                 active=3
              self.ModeComboBox.set_active(active)
              self.SensitiveAllButtons(False)
              return True
           return True
        if not self.vehicle.armed:
           return True
        if self.axis_0:
           if self.vehicle.mode == "STABILIZE" or self.vehicle.mode == "ALT_HOLD":  
              self.rc_yaw=self.mapping(self.value_axis_0,-self.RangeJostick,self.RangeJostick,self.rc_yaw_min+self.DeltaJostick,self.rc_yaw_max-self.DeltaJostick)
              self.vehicle.channels.overrides[4] = self.rc_yaw
           elif self.vehicle.mode == "GUIDED":
              if self.value_axis_0 < 0.0:
                 self.on_SpinRightButton_clicked(self.SpinRightButton)
              elif self.value_axis_0 > 0.0:
                 self.on_SpinLeftButton_clicked(self.SpinLeftButton)
           return True
        if self.axis_1:
           if self.vehicle.mode == "STABILIZE":  
              self.rc_throttle=self.mapping(self.value_axis_1,self.RangeJostick,-self.RangeJostick,self.rc_throttle_min+self.DeltaJostick,self.rc_throttle_max-self.DeltaJostick)
              self.AltitudeScale.set_value(self.rc_throttle)
           if self.vehicle.mode == "ALT_HOLD":  
              self.rc_throttle=self.mapping(self.value_axis_1,self.RangeJostick,-self.RangeJostick,self.rc_throttle_min+self.DeltaJostick,self.rc_throttle_max-self.DeltaJostick)
              self.AltitudeScale.set_value(self.rc_throttle)
           elif self.vehicle.mode == "GUIDED":
              if self.value_axis_1 < 0.0:
                 self.AltitudeJoystick=self.AltitudeJoystick+self.AltitudeScaleAdjust.get_step_increment()
                 self.AltitudeScale.set_value(self.AltitudeJoystick)
              elif self.value_axis_1 > 0.0:
                 self.AltitudeJoystick=self.AltitudeJoystick-self.AltitudeScaleAdjust.get_step_increment()
                 self.AltitudeScale.set_value(self.AltitudeJoystick)
           return True
        if self.axis_2 and self.axis_3:
           if self.vehicle.mode == "STABILIZE" or self.vehicle.mode == "ALT_HOLD":
              if (self.value_axis_2 > 0.0 and self.value_axis_3 < 0.0) or (self.value_axis_3 > 0.0 and self.value_axis_2 < 0.0):
                 self.rc_pitch=self.mapping(self.value_axis_2,self.RangeJostick,-self.RangeJostick,self.rc_pitch_min+self.DeltaJostick,self.rc_pitch_max-self.DeltaJostick)
                 self.rc_roll=self.mapping(self.value_axis_3,self.RangeJostick,-self.RangeJostick,self.rc_roll_min+self.DeltaJostick,self.rc_roll_max-self.DeltaJostick)
              else:
                 self.rc_pitch=self.mapping(self.value_axis_2,-self.RangeJostick,self.RangeJostick,self.rc_pitch_min+self.DeltaJostick,self.rc_pitch_max-self.DeltaJostick)
                 self.rc_roll=self.mapping(self.value_axis_3,-self.RangeJostick,self.RangeJostick,self.rc_roll_min+self.DeltaJostick,self.rc_roll_max-self.DeltaJostick)
              self.vehicle.channels.overrides[1] = self.rc_roll
              self.vehicle.channels.overrides[2] = self.rc_pitch
           elif self.vehicle.mode == "GUIDED":
              if self.value_axis_3 < 0.0 and self.value_axis_2 > 0.0:
                 self.on_RightUpButton_clicked(self.RightUpButton)
              elif self.value_axis_3 < 0.0 and self.value_axis_2 < 0.0:
                 self.on_LeftUpButton_clicked(self.LeftUpButton)
              elif self.value_axis_3 > 0.0 and self.value_axis_2 < 0.0:
                 self.on_LeftDownButton_clicked(self.LeftDownButton)
              elif self.value_axis_3 > 0.0 and self.value_axis_2 > 0.0:
                 self.on_RightDownButton_clicked(self.RightDownButton)
           return True
        if self.axis_2:
           if self.vehicle.mode == "STABILIZE" or self.vehicle.mode == "ALT_HOLD":  
              self.rc_pitch=self.mapping(self.value_axis_2,-self.RangeJostick,self.RangeJostick,self.rc_pitch_min+self.DeltaJostick,self.rc_pitch_max-self.DeltaJostick)
              self.vehicle.channels.overrides[1] = self.rc_pitch
           elif self.vehicle.mode == "GUIDED":
              if self.value_axis_2 > 0.0:
                 self.on_RightButton_clicked(self.RightButton)
              elif self.value_axis_2 < 0.0:
                 self.on_LeftButton_clicked(self.LeftButton)
           return True
        if self.axis_3:
           if self.vehicle.mode == "STABILIZE" or self.vehicle.mode == "ALT_HOLD":  
              self.rc_roll=self.mapping(self.value_axis_3,-self.RangeJostick,self.RangeJostick,self.rc_roll_min+self.DeltaJostick,self.rc_roll_max-self.DeltaJostick)
              self.vehicle.channels.overrides[2] = self.rc_roll
           elif self.vehicle.mode == "GUIDED":
              if self.value_axis_3 < 0.0:
                 self.on_UpButton_clicked(self.UpButton)
              elif self.value_axis_3 > 0.0:
                 self.on_DownButton_clicked(self.DownButton)
           return True
        return True
    def on_Joystick_button_release(self,signal,number,kk):
        if number == 0:
           self.button_0=False
        elif number == 1:
           self.button_1=False
        elif number == 2:
           self.button_2=False
        elif number == 3:
           self.button_3=False
        elif number == 4:
           self.button_4=False
        elif number == 5:
           self.button_5=False
        elif number == 6:
           self.button_6=False
        elif number == 7:
           self.button_7=False
        elif number == 8:
           self.button_8=False
        elif number == 9:
           self.button_9=False
        elif number == 10:
           self.button_10=False
        elif number == 11:
           self.button_11=False
        elif number == 12:
           self.button_12=False
    def on_Joystick_button_press(self,signal,number,kk):
        if number == 0:
           self.button_0=True
        elif number == 1:
           self.button_1=True
        elif number == 2:
           self.button_2=True
        elif number == 3:
           self.button_3=True
        elif number == 4:
           self.button_4=True
        elif number == 5:
           self.button_5=True
        elif number == 6:
           self.button_6=True
        elif number == 7:
           self.button_7=True
        elif number == 8:
           self.button_8=True
        elif number == 9:
           self.button_9=True
        elif number == 10:
           self.button_10=True
        elif number == 11:
           self.button_11=True
        elif number == 12:
           self.button_12=True
        if self.axis_4 and self.button_2:
           self.yJoystickControl=False
           self.joystick.disconnect(self.handler_id1)
           self.joystick.disconnect(self.handler_id2)
           self.joystick.disconnect(self.handler_id3)
           self.joystick.disconnect(self.handler_id4)
           del self.joystick
           #
           # Activate UI Buttons.
           #
           self.ArmButton.set_sensitive(True)
           self.DisConnectButton.set_sensitive(True)
           self.ModeComboBox.set_sensitive(True)
           self.KeyboardButton.set_sensitive(True)
           self.JoystickButton.set_sensitive(True)
           self.VoiceButton.set_sensitive(True)
           self.BCIButton.set_sensitive(True)
           self.CloseButton.set_sensitive(True)
           return True
        if not self.vehicle.armed:
           if self.button_9:
              self.on_ArmButton_clicked(self.ArmButton)
              self.SensitiveAllButtons(False)
              return True
           if self.button_8:
              model = self.ModeComboBox.get_model()
              active = self.ModeComboBox.get_active()
              active=active+1
              if active > 3:
                 active=0
              self.ModeComboBox.set_active(active)
              self.SensitiveAllButtons(False)
              return True            
           return True
        else:
           if self.button_9:
              if self.vehicle.mode == "GUIDED":
                 self.on_StayButton_clicked(self.StayButton)
              elif self.vehicle.mode == "STABILIZE" or self.vehicle.mode == "ALT_HOLD":
                 active=3
                 self.ModeComboBox.set_active(active)
              self.SensitiveAllButtons(False)
              return True
           if self.button_8:
              model = self.ModeComboBox.get_model()
              active = self.ModeComboBox.get_active()
              active=active+1
              if active > 3:
                 active=0
              self.ModeComboBox.set_active(active)
              self.SensitiveAllButtons(False)
              return True
           if self.button_4 or self.button_5 or self.button_6 or self.button_7:
              self.on_StopButton_clicked(self.StopButton)
              return True
           if self.button_1:
              if self.vehicle.mode == "GUIDED":
                 self.Trace_Circle()
              return True
           if self.button_3:
              if self.vehicle.mode == "GUIDED":
                 self.Trace_Square()
              return True
           if self.button_0:
              if self.vehicle.mode == "GUIDED":
                 self.Trace_Triangle()
              return True
           if self.button_2:
              if self.vehicle.mode == "GUIDED":
                 self.Trace_Wave()
              return True
           return True
        return True
    def mapping(self,value,iMin,iMax,oMin,oMax):
        return ((value - iMin) * (oMax - oMin) / (iMax - iMin) + oMin)

#-----------------------------------------------------------------------------------------

    def on_VoiceButton_clicked(self, *args):
        #
        # Control de UI with the Voice Commands and Julius (Voxforge grammar own created) 
        #
        self._log(" Activating voice control ...")
        self.SensitiveAllButtons(False)
        self.yVoiceControl=True
        #
        # Initialize Julius: first killall julius instances, second excute julius with -module option 
        #
        command="killall julius 2> julius_output"
        os.system(command)
        time.sleep(1)
        command="julius -quiet -input mic -C drone.jconf -module -logfile ./julius_output &"
        os.system(command)
        time.sleep(2)
        #
        # Try to connect to Julius: localhost and port 10500
        #
        self.julius_client = pyjulius.Client('localhost', 10500)
        try:
           self.julius_client.connect()
           self._log(" Julius activated ...")
           self._log(" Client connected to Julius in port 10500 ...")
           self._log(" Speak loud and slowly!!")
           #
           # INIT de Voice Control loop in a new timer
           #
           self.julius_client.start()
           self.id_VoiceControl=GObject.timeout_add_seconds(self.timeVoice, self.VoiceControl)
        except pyjulius.ConnectionError:
           self._log(" Error to connect to Julius in port 10500")
           #
           # Activate UI Buttons.
           #
           self.ArmButton.set_sensitive(True)
           self.DisConnectButton.set_sensitive(True)
           self.ModeComboBox.set_sensitive(True)
           self.KeyboardButton.set_sensitive(True)
           self.JoystickButton.set_sensitive(True)
           self.VoiceButton.set_sensitive(True)
           self.BCIButton.set_sensitive(True)
           self.CloseButton.set_sensitive(True)
           return True
        #t10.join()
        return True
    def VoiceControl(self, *args):
        if not self.yVoiceControl:
           return True
        try:
            command=None
            command_first=""
            command_second=""
            score=0
            result = self.julius_client.results.get(False)
            if isinstance(result, pyjulius.Sentence):
               #print 'Sentence "%s" recognized with score %.2f' % (result, result.score)
               command=str(result).split()
               command_first=command[0]
               command_second=command[1]
               score=result.score
               if command_first == "mode":
                  if command_second == "rtl":
                     if self.vehicle.mode == "GUIDED":
                        self.on_StayButton_clicked(self.StayButton)
                        self.yVoiceControl=False
                        GObject.source_remove(self.id_VoiceControl)
                        self.julius_client.stop()
                        command="killall julius 2> julius_output"
                        os.system(command)
                        time.sleep(1)
                        return True               
                  if command_second == "guided":
                     active=0
                  elif command_second == "althold":
                     active=1
                  elif command_second == "stabilize":
                     active=2
                  elif command_second == "land":
                     active=3
                  self.ModeComboBox.set_active(active)
                  self.SensitiveAllButtons(False)
               elif command_first == "drone":
                    if command_second == "arm":
                       if not self.vehicle.armed:
                          if self.vehicle.mode <> "LAND" and self.vehicle.mode <> "RTL":
                             self.on_ArmButton_clicked(self.ArmButton)
                             self.SensitiveAllButtons(False)
                       elif command_second == "quit":
                          self._log(" DeActivating voice control ...")
                          self.yVoiceControl=False
                          GObject.source_remove(self.id_VoiceControl)
                          self.julius_client.stop()
                          command="killall julius 2> julius_output"
                          os.system(command)
                          time.sleep(1)
                          #
                          # Activate UI Buttons.
                          #
                          self.ArmButton.set_sensitive(True)
                          self.DisConnectButton.set_sensitive(True)
                          self.ModeComboBox.set_sensitive(True)
                          self.KeyboardButton.set_sensitive(True)
                          self.JoystickButton.set_sensitive(True)
                          self.VoiceButton.set_sensitive(True)
                          self.BCIButton.set_sensitive(True)
                          self.CloseButton.set_sensitive(True)
                          return True
               elif command_first == "move":
                    if command_second == "ascend":
                       self.AltitudeVoice=self.AltitudeVoice+self.AltitudeScaleAdjust.get_step_increment()
                       self.AltitudeScale.set_value(self.AltitudeVoice)
                    elif command_second == "descend":
                       self.AltitudeVoice=self.AltitudeVoice-self.AltitudeScaleAdjust.get_step_increment()
                       self.AltitudeScale.set_value(self.AltitudeVoice)
                    elif command_second == "right":
                       self.on_RightButton_clicked(self.RightButton)
                    elif command_second == "left":
                       self.on_LeftButton_clicked(self.LeftButton)
                    elif command_second == "forward":
                       self.on_UpButton_clicked(self.UpButton)
                    elif command_second == "backward":
                       self.on_DownButton_clicked(self.DownButton)
                    elif command_second == "stop":
                       self.on_StopButton_clicked(self.StopButton)
                    elif command_second == "bacright":
                       self.on_RightDownButton_clicked(self.RightDownButton)
                    elif command_second == "bacleft":
                       self.on_LeftDownButton_clicked(self.LeftDownButton)
                    elif command_second == "foright":
                       self.on_RightUpButton_clicked(self.RightUpButton)
                    elif command_second == "forleft":
                       self.on_LeftUpButton_clicked(self.LeftUpButton)
               elif command_first == "special" and self.vehicle.mode == "GUIDED":
                    if command_second == "square":
                       self.Trace_Square()
                    elif command_second == "circle":
                       self.Trace_Circle()
                    elif command_second == "triangle":
                       self.Trace_Triangle()
                    elif command_second == "wave":
                       self.Trace_Wave()
        except Queue.Empty:
            i=1
            #print "Queue empty"
        return True

#-----------------------------------------------------------------------------------------

    def on_BCIButton_clicked(self, *args):
        #
        # Control de UI with the BCI gadget and our program MovMind
        #
        #self.SensitiveAllButtons(False)
        self.yBCIControl=True
        return True

#-----------------------------------------------------------------------------------------

    def on_CloseButton_clicked(self, *args):
        if self.yConnect:
           t0 = threading.Thread(target=self.vehicle.close())
           t0.daemon = True
           t0.start()
        if self.id_callback:
           GObject.source_remove(self.id_callback)
        if self.id_log:
           GObject.source_remove(self.id_log)
        if self.yVoiceControl:
           self.yVoiceControl=False
           GObject.source_remove(self.id_VoiceControl)
           self.julius_client.stop()
           command="killall julius 2> julius_output"
           os.system(command)
           time.sleep(1)
        GObject.idle_add(quit)
        Gtk.main_quit(*args)
        call(["kill", "-9", str(os.getpid())])
        sys.exit(0)
        os._exit(0)
        return True

#-----------------------------------------------------------------------------------------

    def onCloseMainWindow(self, *args):
        if self.yConnect:
           t0 = threading.Thread(target=self.vehicle.close())
           t0.daemon = True
           t0.start()
        if self.id_callback:
           GObject.source_remove(self.id_callback)
        if self.id_log:
           GObject.source_remove(self.id_log)
        if self.yVoiceControl:
           self.yVoiceControl=False
           GObject.source_remove(self.id_VoiceControl)
           self.julius_client.stop()
           command="killall julius 2> julius_output"
           os.system(command)
           time.sleep(1)
        GObject.idle_add(quit)
        Gtk.main_quit(*args)
        call(["kill", "-9", str(os.getpid())])
        sys.exit(0)
        os._exit(0)
        return True

#-----------------------------------------------------------------------------------------

    def on_ModeComboBox_changed(self, widget, data=None):
        model = widget.get_model()
        active = widget.get_active()
        if active >= 0 and active <> self.LastActiveIndex:
           code = str(model[active][0])
           if active == 3:
              self.yLAND=True
              self.rc_overrides=False
           self.ChangeMode(code)
           self.LastActiveIndex=active
        else:
           print('No Mode selected')
        return True

#-----------------------------------------------------------------------------------------

    def onProtocolChanged(self, widget, data=None):
        if widget.get_active():
           self.ProtocolString=widget.get_label()
        return True

#-----------------------------------------------------------------------------------------

    def on_ConnectButton_clicked(self, widget, data=None):
        #
        # Get connection to the Vehicle
        #
        t0 = threading.Thread(target=self.getConnection)
        t0.daemon = True
        t0.start()
        t0.join()
        if not self.yConnect:
           return True
        if self.yVideo:
           self._log(" Initializing Video Streaming ...")
           self.STREAM_URL = 'http://'+self.IPText.get_text()+'/html/stream_oc.php'
           self.video = VideoThread(self.VideoImg,self.STREAM_URL)
           self.video.start()
           self._log(" ... done.")
        #
        # register a periodic timers
        #
        self.id_callback=GObject.timeout_add_seconds(self.timeout, self.callback)
        self.id_log=GObject.timeout_add_seconds(self.timeoutLog, self._log, self.message)
        #
        # Get Vehicle Home location - will be `None` until first set by autopilot
        #
        t2 = threading.Thread(target=self.getHomeLocation)
        t2.daemon = True
        t2.start()
        self._log(" Waiting for home location ...")
        self._log(" ... done.")
        t2.join()
        #
        # Set Vehicle Mode to Default "GUIDED"
        #
        mode=self.DefaultMode
        t1 = threading.Thread(target=self.ChangeMode, args=(mode,))
        t1.daemon = True
        t1.start()
        t1.join()
        #
        # We have a first home location. We can continue ... Print info
        #
        self.PrintFirstInfo()
        #
        # Register vehicle observers
        #
        self.getObservers()
        #
        # Deactivate-Activate UI Buttons.
        #
        self.SensitiveAllButtons(False)
        self.ArmButton.set_sensitive(True)
        self.DisConnectButton.set_sensitive(True)
        self.ModeComboBox.set_sensitive(True)
        self.KeyboardButton.set_sensitive(True)
        self.JoystickButton.set_sensitive(True)
        self.VoiceButton.set_sensitive(True)
        self.BCIButton.set_sensitive(True)
        return True

#-----------------------------------------------------------------------------------------

    def on_DisConnectButton_clicked(self, widget, data=None):
        #
        # Disconnect from the APM
        #
        self.SensitiveAllButtons(False)
        if self.vehicle.armed:
           self.yRTL=True
           self.SensitiveAllButtons(False)
           self._log(" Vehicle is armed. Return to home Location ... RTL in progress ")
           mode="RTL"
           t1 = threading.Thread(target=self.ChangeMode, args=(mode,))
           t1.daemon = True
           t1.start()
           t1.join()
           self.vehicle.armed=False
           while self.vehicle.armed:
               time.sleep(1)
           self._log(" Vehicle is disarmed.")
        if not self.vehicle.armed:
           self._log(" Reinitialize all variables.")
           self.removeObservers()
           self.vehicle.close()
           self.Protocol.set_sensitive(True) 
           self.ProtocolU.set_sensitive(True)
           self.IPText.set_sensitive(True)
           self.PortText.set_sensitive(True)
           self.ConnectButton.set_sensitive(True)
           self.DisConnectButton.set_sensitive(False)
           self.CloseButton.set_sensitive(True)
           if self.sitl is not None:
              self.sitl.stop()
           self.InitializeVar()
        return True

#-----------------------------------------------------------------------------------------

    def on_ArmButton_clicked(self, widget, data=None):
        self.SensitiveAllButtons(False)
        self.yRTL = False
        self.yLAND = False
        if self.DefaultMode <> "GUIDED" and self.DefaultMode <> "LAND":
           #
           # 1. RC Overrides
           #
           self.vehicle.parameters['ARMING_CHECK']=-9
           self.rc_overrides=True
           self.vehicle.channels.overrides = {}
           self.rc_throttle=1000
           self.vehicle.channels.overrides[1] = self.rc_roll
           self.vehicle.channels.overrides[2] = self.rc_pitch
           self.vehicle.channels.overrides[3] = self.rc_throttle
           self.vehicle.channels.overrides[4] = self.rc_yaw
           #
           # 2. Arm the Engines
           #
           self._log("       ---------- Arming Engines -------------")
           while not self.vehicle.armed:
                 print("Arming motors")
                 self.vehicle.armed = True
                 time.sleep(0.5)
           self._log("       --------------- done ------------------")
           self._log(" Vehicle is Armed in mode : %s " % self.vehicle.mode)
           self.rc_throttle=1200
           self.AltitudeScale.set_value(self.rc_throttle)
           time.sleep(1)
           self._log(" Set Throttle to: %s" % self.rc_throttle)
           self._log(" -------- Done --------")
           self.rc_throttle=1300
           self.AltitudeScale.set_value(self.rc_throttle)
           time.sleep(1)
           self._log(" Set Throttle to: %s" % self.rc_throttle)
           self._log(" -------- Done --------")
           self.rc_throttle=self.Defaultrc_throttle
           if self.vehicle.mode == "ALT_HOLD":
              self.rc_throttle=self.Defaultrc_throttle_alt_hold
           self.AltitudeScale.set_value(self.rc_throttle)
           time.sleep(1)
           self._log(" Set Throttle to: %s" % self.rc_throttle)
           self._log(" -------- Done --------")
           self.AltitudeKey=self.rc_throttle
           self.AltitudeJoystick=self.rc_throttle
           self.AltitudeVoice=self.rc_throttle
        else:
           #
           # 1. Prearm Checks.
           #
           self._log(" Basic Pre-Arm Checks: ")
           self._log("       -- Waiting for vehicle to initialise --")
           t1 = threading.Thread(target=self.PreArmChecks)
           t1.daemon = True
           t1.start()
           self._log("       --------------- done ------------------")
           t1.join()
           if self.vehicle.is_armable:
              #
              # 2. Arm the Engines
              #
              self._log("       ---------- Arming Engines -------------")
              t2 = threading.Thread(target=self.ArmEngines)
              t2.daemon = True
              t2.start()
              self._log("       --------------- done ------------------")
              self._log(" Vehicle is Armed in mode : %s " % self.vehicle.mode)
              t2.join()
              #
              # 3. Determine Current Location
              #
              self._log(" Waiting for current location...")
              t3 = threading.Thread(target=self.getCurrentLocation)
              t3.daemon = True
              t3.start()
              self._log(" -------- Done --------")
              #
              # 4. First altitude=self.Defaultaltitude
              #
              t4 = threading.Thread(target=self.DefaultAltitude)
              t4.daemon = True
              t4.start()
              self._log(" Set default/target airspeed to 3")
              self._log(" Getting first Altitude of %s :" % self.Defaultaltitude)
              self._log(" -------- Done --------")
              self.vehicle.airspeed = 3
              self.AltitudeKey=self.Defaultaltitude
              self.AltitudeJoystick=self.Defaultaltitude
              self.AltitudeVoice=self.Defaultaltitude    
           else:
              self._log(" Impossible to arm in GUIDED Mode")
              self._log(" GPS: %s" % self.vehicle.gps_0)  
              self._log(" EKF OK?: %s" % self.vehicle.ekf_ok)
              self._log(" Battery: %s" % self.vehicle.battery)
        if self.vehicle.armed:
           #
           # 4. Deactivate-Activate UI Buttons.
           #
           self.SensitiveAllButtons(True)
           self.ArmButton.set_sensitive(False)
           self.Protocol.set_sensitive(False) 
           self.ProtocolU.set_sensitive(False)
           self.IPText.set_sensitive(False)
           self.PortText.set_sensitive(False)
           self.ConnectButton.set_sensitive(False)
           self.DisConnectButton.set_sensitive(False)
        else:
           self.ArmButton.set_sensitive(True)
           self.ModeComboBox.set_sensitive(True)
           self.Protocol.set_sensitive(False) 
           self.ProtocolU.set_sensitive(False)
           self.IPText.set_sensitive(False)
           self.PortText.set_sensitive(False)
           self.ConnectButton.set_sensitive(False)
           self.DisConnectButton.set_sensitive(True)
        return True

#-----------------------------------------------------------------------------------------

    def on_AltitudeScale_value_changed(self, widget, data=None):
        #
        # Get new Altitude (Relative) or change Throttle
        #
        if self.yRTL or self.yWave or self.yLAND:
           newAlt=self.altitudeREL
           self.AltitudeScale.set_value(newAlt)
           return
        if self.rc_overrides:
           if not self.yJoystickControl:
              self.rc_throttle=widget.get_value()
           if self.Last_rc_throttle==self.rc_throttle:
              return
           self.vehicle.channels.overrides[3]=self.rc_throttle
           self._log(" Set Throttle to: %s" % self.rc_throttle)
           self._log(" -------- Done --------")
           self.AltitudeKey=self.rc_throttle
           self.AltitudeJoystick=self.rc_throttle
           self.AltitudeVoice=self.rc_throttle
           self.AltitudeScale.set_value(self.rc_throttle)
           self.Last_rc_throttle=self.rc_throttle
        else:
           newAlt = widget.get_value()
           if newAlt >= 1000.0:
              newAlt=self.LastAltitude
           dif = abs(self.altitudeREL - newAlt)
           if dif > .25:
              self._log(" Reaching new altitude of ... %s " % newAlt)
              t1 = threading.Thread(target=self.getNewAltitude, args=(newAlt,))
              t1.daemon = True
              t1.start()
              self.AltitudeScale.set_value(newAlt)
              self._log(" -------- Done --------")
              self.AltitudeKey=newAlt
              self.AltitudeJoystick=newAlt           
              self.AltitudeVoice=newAlt
        return True

#-----------------------------------------------------------------------------------------

    def on_StopButton_clicked(self, widget, data=None):
        #
        # Stop actual movement
        #
        if self.rc_overrides:
           self.vehicle.channels.overrides[1] = self.Defaultrc_roll
           self.vehicle.channels.overrides[2] = self.Defaultrc_pitch
           self.vehicle.channels.overrides[3] = self.LastThrottle
           self.vehicle.channels.overrides[4] = self.Defaultrc_yaw
           self.rc_roll=self.Defaultrc_roll
           self.rc_pitch=self.Defaultrc_pitch
           self.rc_yaw=self.Defaultrc_yaw
        else:
           vel_x=0
           vel_y=0
           vel_z=0
           duration=1
           self.send_ned_velocity(vel_x,vel_y,vel_z,duration)
        return True

#-----------------------------------------------------------------------------------------

    def on_StayButton_clicked(self, widget, data=None):
        #
        # Return To Launch (RTL) goto home_location
        #
        if self.rc_overrides:
           index=3
           self.ModeComboBox.set_active(index)    
        else:
           self.yRTL=True
           self.SensitiveAllButtons(False)
           self._log(" Return to home Location ... RTL in progress ")
           mode="RTL"
           t1 = threading.Thread(target=self.ChangeMode, args=(mode,))
           t1.daemon = True
           t1.start()
           t1.join()
        return True

#-----------------------------------------------------------------------------------------

    def on_UpButton_clicked(self, widget, data=None):
        #
        # send signal to the drone for go to front ... Using RC Overrides
        #
        if self.rc_overrides:
           self.rc_pitch -= 50
           if self.rc_pitch < self.rc_pitch_min:
              self.rc_pitch=self.rc_pitch_min
           if self.rc_pitch > self.rc_pitch_max:
              self.rc_pitch=self.rc_pitch_max
           self.vehicle.channels.overrides[2] = self.rc_pitch    
        else:
           vel_x=self.DefaultVelocity
           vel_y=0
           vel_z=0
           duration=1
           t1 = threading.Thread(target=self.send_ned_velocity, args=(vel_x,vel_y,vel_z,duration,))
           t1.daemon = True
           t1.start()
        return True

#-----------------------------------------------------------------------------------------

    def on_DownButton_clicked(self, widget, data=None):
        #
        # send signal to the drone for go to back
        #
        if self.rc_overrides:
           self.rc_pitch += 50
           if self.rc_pitch < self.rc_pitch_min:
              self.rc_pitch=self.rc_pitch_min
           if self.rc_pitch > self.rc_pitch_max:
              self.rc_pitch=self.rc_pitch_max
           self.vehicle.channels.overrides[2] = self.rc_pitch
        else:
           vel_x=-self.DefaultVelocity
           vel_y=0
           vel_z=0
           duration=1
           t1 = threading.Thread(target=self.send_ned_velocity, args=(vel_x,vel_y,vel_z,duration,))
           t1.daemon = True
           t1.start()
        return True

#-----------------------------------------------------------------------------------------

    def on_RightButton_clicked(self, widget, data=None):
        #
        # send signal to the drone for go to right
        #
        if self.rc_overrides:
           self.rc_roll += 50
           if self.rc_roll < self.rc_roll_min:
              self.rc_roll=self.rc_roll_min
           if self.rc_roll > self.rc_roll_max:
              self.rc_roll=self.rc_roll_max
           self.vehicle.channels.overrides[1] = self.rc_roll
        else:
           vel_x=0
           vel_y=self.DefaultVelocity
           vel_z=0
           duration=1
           t1 = threading.Thread(target=self.send_ned_velocity, args=(vel_x,vel_y,vel_z,duration,))
           t1.daemon = True
           t1.start()
        return True

#-----------------------------------------------------------------------------------------

    def on_LeftButton_clicked(self, widget, data=None):
        #
        # send signal to the drone for go to left
        #
        if self.rc_overrides:
           self.rc_roll -= 50
           if self.rc_roll < self.rc_roll_min:
              self.rc_roll=self.rc_roll_min
           if self.rc_roll > self.rc_roll_max:
              self.rc_roll=self.rc_roll_max
           self.vehicle.channels.overrides[1] = self.rc_roll
        else:
           vel_x=0
           vel_y=-self.DefaultVelocity
           vel_z=0
           duration=1
           t1 = threading.Thread(target=self.send_ned_velocity, args=(vel_x,vel_y,vel_z,duration,))
           t1.daemon = True
           t1.start()
        return True

#-----------------------------------------------------------------------------------------

    def on_LeftUpButton_clicked(self, widget, data=None):
        #
        # send signal to the drone for go to left-front
        #
        if self.rc_overrides:
           self.rc_pitch -= 50
           if self.rc_pitch < self.rc_pitch_min:
              self.rc_pitch=self.rc_pitch_min
           if self.rc_pitch > self.rc_pitch_max:
              self.rc_pitch=self.rc_pitch_max 
           self.rc_roll -= 50
           if self.rc_roll < self.rc_roll_min:
              self.rc_roll=self.rc_roll_min
           if self.rc_roll > self.rc_roll_max:
              self.rc_roll=self.rc_roll_max
           self.vehicle.channels.overrides[2] = self.rc_pitch
           self.vehicle.channels.overrides[1] = self.rc_roll
        else:
           vel=self.DefaultVelocity/math.sqrt(2)
           vel_x=vel
           vel_y=-vel
           vel_z=0
           duration=1
           t1 = threading.Thread(target=self.send_ned_velocity, args=(vel_x,vel_y,vel_z,duration,))
           t1.daemon = True
           t1.start()
        return True

#-----------------------------------------------------------------------------------------

    def on_RightUpButton_clicked(self, widget, data=None):
        #
        # send signal to the drone for go to right-front
        #
        if self.rc_overrides:
           self.rc_pitch -= 50
           if self.rc_pitch < self.rc_pitch_min:
              self.rc_pitch=self.rc_pitch_min
           if self.rc_pitch > self.rc_pitch_max:
              self.rc_pitch=self.rc_pitch_max 
           self.rc_roll += 50
           if self.rc_roll < self.rc_roll_min:
              self.rc_roll=self.rc_roll_min
           if self.rc_roll > self.rc_roll_max:
              self.rc_roll=self.rc_roll_max
           self.vehicle.channels.overrides[2] = self.rc_pitch
           self.vehicle.channels.overrides[1] = self.rc_roll
        else:
           vel=self.DefaultVelocity/math.sqrt(2)
           vel_x=vel
           vel_y=vel
           vel_z=0
           duration=1
           t1 = threading.Thread(target=self.send_ned_velocity, args=(vel_x,vel_y,vel_z,duration,))
           t1.daemon = True
           t1.start()
        return True

#-----------------------------------------------------------------------------------------

    def on_LeftDownButton_clicked(self, widget, data=None):
        #
        # send signal to the drone for go to left-back 
        #
        if self.rc_overrides:
           self.rc_pitch += 50
           if self.rc_pitch < self.rc_pitch_min:
              self.rc_pitch=self.rc_pitch_min
           if self.rc_pitch > self.rc_pitch_max:
              self.rc_pitch=self.rc_pitch_max 
           self.rc_roll -= 50
           if self.rc_roll < self.rc_roll_min:
              self.rc_roll=self.rc_roll_min
           if self.rc_roll > self.rc_roll_max:
              self.rc_roll=self.rc_roll_max
           self.vehicle.channels.overrides[2] = self.rc_pitch
           self.vehicle.channels.overrides[1] = self.rc_roll
        else:
           vel=self.DefaultVelocity/math.sqrt(2)
           vel_x=-vel
           vel_y=-vel
           vel_z=0
           duration=1
           t1 = threading.Thread(target=self.send_ned_velocity, args=(vel_x,vel_y,vel_z,duration,))
           t1.daemon = True
           t1.start()
        return True

#-----------------------------------------------------------------------------------------

    def on_RightDownButton_clicked(self, widget, data=None):
        #
        # send signal to the drone for go to right-back
        #
        if self.rc_overrides:
           self.rc_pitch += 50
           if self.rc_pitch < self.rc_pitch_min:
              self.rc_pitch=self.rc_pitch_min
           if self.rc_pitch > self.rc_pitch_max:
              self.rc_pitch=self.rc_pitch_max 
           self.rc_roll += 50
           if self.rc_roll < self.rc_roll_min:
              self.rc_roll=self.rc_roll_min
           if self.rc_roll > self.rc_roll_max:
              self.rc_roll=self.rc_roll_max
           self.vehicle.channels.overrides[2] = self.rc_pitch
           self.vehicle.channels.overrides[1] = self.rc_roll
        else:
           vel=self.DefaultVelocity/math.sqrt(2)
           vel_x=-vel
           vel_y=vel
           vel_z=0
           duration=1
           t1 = threading.Thread(target=self.send_ned_velocity, args=(vel_x,vel_y,vel_z,duration,))
           t1.daemon = True
           t1.start()
        return True

#-----------------------------------------------------------------------------------------

    def on_SpinRightButton_clicked(self, widget, data=None):
        #
        # send signal to the drone for spin DeltaDegrees degrees CW
        #
        if self.rc_overrides:
           self.rc_yaw += 5
           if self.rc_yaw < self.rc_yaw_min:
              self.rc_yaw=self.rc_yaw_min
           if self.rc_yaw > self.rc_yaw_max:
              self.rc_yaw=self.rc_yaw_max
           self.vehicle.channels.overrides[4] = self.rc_yaw
        else:
           heading=self.DeltaDegrees
           relative=True
           clock=1
           t1 = threading.Thread(target=self.condition_yaw, args=(heading,relative,clock,))
           t1.daemon = True
           t1.start()
        return True

#-----------------------------------------------------------------------------------------

    def on_SpinLeftButton_clicked(self, widget, data=None):
        #
        # send signal to the drone for spin DeltDegrees degrees CCW
        #
        if self.rc_overrides:
           self.rc_yaw -= 5
           if self.rc_yaw < self.rc_yaw_min:
              self.rc_yaw=self.rc_yaw_min
           if self.rc_yaw > self.rc_yaw_max:
              self.rc_yaw=self.rc_yaw_max
           self.vehicle.channels.overrides[4] = self.rc_yaw
        else:
           heading=self.DeltaDegrees
           relative=True
           clock=-1
           t1 = threading.Thread(target=self.condition_yaw, args=(heading,relative,clock,))
           t1.daemon = True
           t1.start()
        return True

#-----------------------------------------------------------------------------------------

    def on_SquareButton_clicked(self, widget, data=None):
        #
        # send signal to the drone for draw a Square of self.DefaultLength
        #
        t1 = threading.Thread(target=self.Trace_Square)
        t1.daemon = True
        t1.start()
        return True

#-----------------------------------------------------------------------------------------

    def on_CircleButton_clicked(self, widget, data=None):
        #
        # send signal to the drone for draw a Circle of self.DefaultLength Radius
        #
        t1 = threading.Thread(target=self.Trace_Circle)
        t1.daemon = True
        t1.start()
        return True

#-----------------------------------------------------------------------------------------

    def on_TriangleButton_clicked(self, widget, data=None):
        #
        # send signal to the drone for draw a Triangle of self.DefaultLength Length
        #
        t1 = threading.Thread(target=self.Trace_Triangle)
        t1.daemon = True
        t1.start()
        return True

#-----------------------------------------------------------------------------------------

    def on_WaveButton_clicked(self, widget, data=None):
        #
        # send signal to the drone for draw a Circle of self.DefaultLength/2 Radius
        #
        t1 = threading.Thread(target=self.Trace_Wave)
        t1.daemon = True
        t1.start()
        return True

#-----------------------------------------------------------------------------------------

    def Trace_Square(self):
         self.vehicle.groundspeed=self.DefaultVelocity
         distance=self.DefaultLength
         distance2=self.DefaultLength/2 
         self.goto(0, -distance2)
         self.goto(distance2, 0)
         self.goto(0, distance)
         self.goto(-distance, 0.0)
         self.goto(0.0, -distance)
         self.goto(distance2, 0)
         self.goto(0, distance2)

#-----------------------------------------------------------------------------------------

    def Trace_Triangle(self):
         self.vehicle.groundspeed=self.DefaultVelocity
         distance=self.DefaultLength
         distance2=self.DefaultLength/2 
         self.goto(0, -distance2)
         #self.goto(distance2, 0)
         self.goto(distance2, distance)
         self.goto(-distance, 0)
         self.goto(distance2, -distance)
         #self.goto(distance2, 0)
         self.goto(0, distance2)

#-----------------------------------------------------------------------------------------

    def Trace_Circle(self):
         self.vehicle.groundspeed=self.DefaultVelocity
         distance=self.DefaultLength
         distance2=self.DefaultLength/2
         self.goto(0, -distance)
         xn=0
         yn=-distance
         for i in range(180-15,-195,-15):
             east=distance*math.cos(math.radians(i))
             north=distance*math.sin(math.radians(i))
             xd=north-xn
             yd=east-yn
             #print i
             self.goto(xd, yd)
             xn=north
             yn=east
         self.goto(0, distance)
         return True

#-----------------------------------------------------------------------------------------

    def Trace_Wave(self):
        self.yWave=True
        if self.altitudeREL < 10.:
           self.getNewAltitude(10.)
        T=float(2*self.DefaultLength)
        for i in range(0,self.DefaultLength):
            vel=self.DefaultVelocity/2.
            vel_x=vel
            vel_y=0
            constant=2.0*math.pi/T
            vel_z=self.DefaultAmplitude*math.cos(constant*float(i))
            duration=1
            self.send_ned_velocity(vel_x,vel_y,vel_z,duration)
        for i in range(0,self.DefaultLength-1):
            vel=self.DefaultVelocity/2.
            vel_x=-vel
            vel_y=0
            constant=2.0*math.pi/T
            vel_z=self.DefaultAmplitude*math.cos(constant*float(i))
            duration=1
            self.send_ned_velocity(vel_x,vel_y,vel_z,duration)
        self.yWave=False
        for i in range(0,self.DefaultLength):
            vel=self.DefaultVelocity/2.
            vel_x=vel
            vel_z=0
            constant=2.0*math.pi/T
            vel_y=self.DefaultAmplitude*math.cos(constant*float(i))
            duration=1
            self.send_ned_velocity(vel_x,vel_y,vel_z,duration)
        for i in range(0,self.DefaultLength-1):
            vel=self.DefaultVelocity/2.
            vel_x=-vel
            vel_z=0
            constant=2.0*math.pi/T
            vel_y=self.DefaultAmplitude*math.cos(constant*float(i))
            duration=1
            self.send_ned_velocity(vel_x,vel_y,vel_z,duration)
        return True

#-----------------------------------------------------------------------------------------
#   The next def are the for retraiving the info of the drone in real time and show it 
#-----------------------------------------------------------------------------------------

    def callback(self):
        if not self.yConnect:
           return True
        if self.vehicle.armed:
           self.LastThrottle=self.vehicle.channels['3']
        self.current_location = self.vehicle.location.global_relative_frame
        if not self.last_location:
           self.last_location=self.current_location
        self.LongitudeLabel.set_text("Lon. (GPS)\n"+str(self.longitude))
        self.LatitudeLabel.set_text("Lat. (GPS)\n"+str(self.latitude))
        self.AltitudeGPSLabel.set_text("Alt. (GPS)\n"+str(self.altitudeGPS))
        self.AltitudeRELLabel.set_text("Alt. (Rel)\n"+str(self.altitudeREL))
        distance=self.get_distance_metres(self.current_location, self.last_location)
        self.speed='{:6.2f}'.format(distance)
        self.SpeedLabel.set_text("Speed (m/s)\n"+str(self.speed))
        self.climb=(self.altitudeREL-self.LastAltitude)
        self.ClimbLabel.set_text("Climb (m/s)\n"+str(self.climb))
        self.PitchLabel.set_text("Pitch (Deg)\n"+str(self.pitch))
        self.RollLabel.set_text("Roll (Deg)\n"+str(self.roll))
        self.YawLabel.set_text("Yaw (Deg)\n"+str(self.yaw))
        self.VoltageLabel.set_text("Voltage (V)\n"+str(self.voltage))
        self.IntensityLabel.set_text("Current (A)\n"+str(self.current))
        self.LevelLabel.set_text("Level (%)\n"+str(self.level))
        if self.yWave:
           self.AltitudeScale.set_value(self.current_location.alt)
        if self.yRTL or self.yLAND:
           newAlt=self.altitudeREL
           self.AltitudeScale.set_value(newAlt)
           if not self.vehicle.armed and (not self.yKeyboardControl) and (not self.yJoystickControl) and (not self.yVoiceControl):
              self.DisConnectButton.set_sensitive(True)
              self.ArmButton.set_sensitive(True)
              self.ModeComboBox.set_sensitive(True)
              self.KeyboardButton.set_sensitive(True)
              self.JoystickButton.set_sensitive(True)
              self.VoiceButton.set_sensitive(True)
              self.BCIButton.set_sensitive(True)
              self.CloseButton.set_sensitive(True)
              self.yRTL=False
              self.yLAND=False
              index=0
              self.ModeComboBox.set_active(index)
        self.LastAltitude=self.altitudeREL
        self.last_location=self.current_location
        return True

#-----------------------------------------------------------------------------------------

    def location_callback(self, vehicle, name, location):
        self.current_location = location.global_relative_frame
        self.longitude=self.current_location.lon
        self.latitude=self.current_location.lat
        self.altitudeGPS=location.global_frame.alt
        if location.global_relative_frame.alt is not None:
           self.altitudeREL=location.global_relative_frame.alt
        return True

#-----------------------------------------------------------------------------------------

    def attitude_callback(self, vehicle, name, attitude):
        self.current_attitude = attitude
        pitch=float(self.pitch)*float(57.2957795)
        roll=float(self.roll)*float(57.2957795)
        yaw=float(self.yaw)*float(57.2957795)
        if self.current_attitude.pitch < 0.0:
           self.pitch='{:6.2f}'.format(self.current_attitude.pitch*57.2957795)
        else:
           self.pitch='{:6.2f}'.format(self.current_attitude.pitch*57.2957795)
        if self.current_attitude.roll < 0.0:
           self.roll='{:6.2f}'.format(self.current_attitude.roll*57.2957795)
        else:
           self.roll='{:6.2f}'.format(self.current_attitude.roll*57.2957795)
        self.currentYaw=self.current_attitude.yaw*57.2957795
        if self.currentYaw < 0:
           self.currentYaw += 360.0;
        if self.current_attitude.yaw < 0.0:
           self.yaw='{:6.2f}'.format(self.currentYaw)
        else:
           self.yaw='{:6.2f}'.format(self.currentYaw)
        return True

#-----------------------------------------------------------------------------------------

    def battery_callback(self, vehicle, name, battery):
        self.current_battery = battery
        self.voltage=self.current_battery.voltage
        self.current=self.current_battery.current
        self.level=self.current_battery.level
        return True

#-----------------------------------------------------------------------------------------

    def _log(self, message=None, *args):
        if message:
           end_iter = self.TextBuffer.get_end_iter()
           start_iter = self.TextBuffer.get_start_iter()
           now=strftime("%d-%b-%Y at %H:%M:%S", gmtime())
           message=now+" --> "+message
           print "[DEBUG]: {0}".format(message)
           message="[DEBUG]: {0}".format(message)
           c_tag = self.tag
           self.TextBuffer.insert_with_tags(start_iter, message+"\n",c_tag)
        return True

#-----------------------------------------------------------------------------------------

    def _autoscroll(self, *args):
        adj = self.Logscrolledwindow.get_vadjustment()
        #adj.set_value(adj.get_upper() - adj.get_page_size())
        return True

#-----------------------------------------------------------------------------------------

    def SensitiveAllButtons(self,value):
        self.DisConnectButton.set_sensitive(value)
        self.Protocol.set_sensitive(value) 
        self.ProtocolU.set_sensitive(value)
        self.IPText.set_sensitive(value)
        self.PortText.set_sensitive(value)
        self.ConnectButton.set_sensitive(value)
        self.DisConnectButton.set_sensitive(value)
        self.CloseButton.set_sensitive(value)
        self.KeyboardButton.set_sensitive(value)
        self.JoystickButton.set_sensitive(value)
        self.VoiceButton.set_sensitive(value)
        self.BCIButton.set_sensitive(value)
        self.LeftUpButton.set_sensitive(value)
        self.UpButton.set_sensitive(value)
        self.RightUpButton.set_sensitive(value)
        self.LeftButton.set_sensitive(value)
        self.SpinLeftButton.set_sensitive(value)
        self.StayButton.set_sensitive(value)
        self.RightButton.set_sensitive(value)
        self.SpinRightButton.set_sensitive(value)
        self.LeftDownButton.set_sensitive(value)
        self.DownButton.set_sensitive(value)
        self.RightDownButton.set_sensitive(value)
        self.AltitudeScale.set_sensitive(value)
        self.ArmButton.set_sensitive(value)
        self.SquareButton.set_sensitive(value)
        self.CircleButton.set_sensitive(value)
        self.WaveButton.set_sensitive(value)
        self.StopButton.set_sensitive(value)
        self.TriangleButton.set_sensitive(value)
        self.ModeComboBox.set_sensitive(value)
        return True

#-----------------------------------------------------------------------------------------
#   The next def are for command de vehicle from one GPS point to other one
#-----------------------------------------------------------------------------------------

    def getConnection(self):
        self.ConnectionString=" "
        self.sitl = None
        #
        # Starts SITL if connection_string=Simul or Connect with Real Drone e.i. tcp:10.1.0.25:14550
        #
        if self.IPText.get_text() == "Simul":
           import dronekit_sitl
           self.sitl = dronekit_sitl.start_default()
           connection_string = self.sitl.connection_string()
           self.sitl.download('copter', '3.3', verbose=True)
           self.sitl_args = ['-I0', '--model', 'quad', '--home='+str(self.iesdeltebreLon)+','+str(self.iesdeltebreLat)+','+str(self.iesdeltebreAlt)+','+str(self.iesdeltebreHeading)]
           self.sitl.launch(self.sitl_args, await_ready=True, restart=True)
           connection_string='tcp:127.0.0.1:5760'
           self._log("Starting copter simulator (SITL)")
           print "Starting copter simulator (SITL)"
           self._log(connection_string)
           print connection_string
           message=" ".join(self.sitl_args)
           self._log(message)
           print self.sitl_args
           self.yVideo=False
        else:
           #
           # Verify IP and Port
           #
           protocol=self.ProtocolString
           if self.is_valid_ipv4_address(self.IPText.get_text()):
              ip=self.IPText.get_text()
           else:
              self.IPText.set_text("Invalid IP Address")
              return
           if self.is_valid_port(self.PortText.get_text()):
              port=self.PortText.get_text()
           else:
              self.PortText.set_text("Port between 1024-65535")
              return
           self.ConnectionString=protocol+":"+ip+":"+port
           connection_string = self.ConnectionString
        #
        # Here is where all is done ... 1. First connect with the vehicle
        #
        self._log(" Waiting for connect to APM ...")
        try:
           self.vehicle = connect(connection_string, wait_ready=True)
           self.yConnect=True
        except:
           self._log(" Impossible to connect with this address and port")
        self._log(" ... done.")
        return True

#-----------------------------------------------------------------------------------------

    def getObservers(self):
        self.vehicle.add_attribute_listener('location', self.location_callback)
        self.vehicle.add_attribute_listener('attitude', self.attitude_callback)
        self.vehicle.add_attribute_listener('battery' , self.battery_callback)
        return True

#-----------------------------------------------------------------------------------------

    def removeObservers(self):
        self.vehicle.remove_attribute_listener('location', self.location_callback)
        self.vehicle.remove_attribute_listener('attitude', self.attitude_callback)
        self.vehicle.remove_attribute_listener('battery' , self.battery_callback)
        return True

#-----------------------------------------------------------------------------------------

    def getHomeLocation(self):
        while not self.yConnect:
           while not self.vehicle.home_location:
               cmds = self.vehicle.commands
               cmds.download()
               cmds.wait_ready()
        return True

#-----------------------------------------------------------------------------------------

    def getCurrentLocation(self):
        while self.vehicle.location.global_frame.lat == 0:
            time.sleep(0.1)
        self.homeLocation = LocationGlobalRelative(self.vehicle.location.global_frame.lat,
                            self.vehicle.location.global_frame.lon, 0)
        return True

#-----------------------------------------------------------------------------------------

    def DefaultAltitude(self):
        self.current_heading=self.vehicle.heading
        self.vehicle.simple_takeoff(self.Defaultaltitude)
        #
        # we must to attend to get the first altitude
        #
        while True:
            dif = abs(self.current_location.alt - self.Defaultaltitude)
            if dif <= self.altitudeError:
               self.AltitudeScale.set_value(self.Defaultaltitude)
               break
            time.sleep(1)    
        return True

#-----------------------------------------------------------------------------------------

    def getNewAltitude(self, newAltitude):
        newAlt=newAltitude
        newLat = self.current_location.lat
        newLon = self.current_location.lon
        newLoc = LocationGlobalRelative (newLat, newLon, newAlt)
        GroundSpeed=5
        self.vehicle.simple_goto(newLoc,GroundSpeed)
        while True:
            dif = abs(self.current_location.alt - newAlt)
            if dif <= self.altitudeError:
            #   self.AltitudeScale.set_value(self.current_location.alt)
               break
            time.sleep(1)
        relative=False
        clock=1
        t1 = threading.Thread(target=self.condition_yaw, args=(self.current_heading,relative,clock,))
        t1.daemon = True
        t1.start()
        t1.join()
        return True

#-----------------------------------------------------------------------------------------
         
    def PreArmChecks(self):
        k=0
        while not self.vehicle.is_armable:
              time.sleep(1)
              k=k+1
              if k == 30:
                 if not self.vehicle.is_armable:
                    break
        return True

#-----------------------------------------------------------------------------------------
         
    def ArmEngines(self):
        self.vehicle.armed = True
        while not self.vehicle.armed:
              time.sleep(1)
        return True

#-----------------------------------------------------------------------------------------

    def ChangeMode(self, code):
        if code == "RTL" or code == "Land":
           if code == "Land":
              self.DefaultMode="LAND"
           elif code == "RTL":
              self.DefaultMode="RTL"
           self.SensitiveAllButtons(False)
        if code == "Guided (GPS)":
           self.DefaultMode="GUIDED"
           if not self.vehicle.is_armable:
              self._log(" Impossible to arm in GUIDED Mode")
              self._log(" GPS: %s" % self.vehicle.gps_0)  
              self._log(" EKF OK?: %s" % self.vehicle.ekf_ok)
              self._log(" Battery: %s" % self.vehicle.battery)
              return
        elif code == "Alt Hold":
           self.DefaultMode="ALT_HOLD"
        elif code == "Stabilize":
           self.DefaultMode="STABILIZE"
        self._log(" Changing to mode: {0}".format(self.DefaultMode))
        t1 = threading.Thread(target=self.mode_Thread)
        t1.daemon = True
        t1.start()
        t1.join()
        if self.DefaultMode == "STABILIZE" or self.DefaultMode == "ALT_HOLD":
           self.rc_overrides=True
           self.vehicle.channels.overrides = {}
           self.vehicle.channels.overrides[1] = self.vehicle.channels['1']
           self.vehicle.channels.overrides[2] = self.vehicle.channels['2']
           self.vehicle.channels.overrides[4] = self.vehicle.channels['4']
           self.AltitudeScale.set_fill_level(self.rc_throttle_max)
           self.AltitudeScale.set_adjustment(self.ThrottleScaleAdjust)
           if self.DefaultMode == "ALT_HOLD":
              self.rc_throttle=self.Defaultrc_throttle_alt_hold
              self.Last_rc_throttle=0
           if self.DefaultMode == "STABILIZE":
              self.AltitudeScaleAdjust.get_step_increment()
              self.rc_throttle=self.Defaultrc_throttle
              self.Last_rc_throttle=0
           self.AltitudeScale.set_value(self.rc_throttle)
           time.sleep(0.1)
        else:
           self.rc_overrides=False
           self.AltitudeScale.set_fill_level(self.DefaultMaxaltitude)
           self.AltitudeScale.set_adjustment(self.AltitudeScaleAdjust)
           self.AltitudeKey=self.LastAltitude
           self.AltitudeJoystick=self.LastAltitude
           self.AltitudeVoice=self.LastAltitude
           self.AltitudeScale.set_value(self.LastAltitude)
        self._log('  ... polled mode: {0}'.format(self.DefaultMode))
        self._log(" -------- Done --------")
        return True
    def mode_Thread(self):
        self.vehicle.mode = VehicleMode(self.DefaultMode)
        while self.vehicle.mode.name != self.DefaultMode:
              time.sleep(1)
#-----------------------------------------------------------------------------------------

    def goto(self, dNorth, dEast, gotoFunction=None):
        #
        # Moves the vehicle to a position dNorth metres North and dEast metres East of the current position.
        #
        currentLocation = self.vehicle.location.global_relative_frame
        targetLocation = self.get_location_metres(currentLocation, dNorth, dEast)
        targetDistance = self.get_distance_metres(currentLocation, targetLocation)
        if gotoFunction == None:
           self.vehicle.simple_goto(targetLocation)
        elif gotoFunction == "goto_position_target_global_int":
           x=1
        remainingDistanceOld=0
        n=0
        while self.vehicle.mode.name=="GUIDED":
            remainingDistance=abs(self.get_distance_metres(self.vehicle.location.global_relative_frame, targetLocation))
            #print remainingDistance,remainingDistanceOld
            if remainingDistance<=self.gpsError:
                break;
            remainingDistanceOld=remainingDistance
            n=n+1
            if n >= 15:
               break;
            time.sleep(2)

#-----------------------------------------------------------------------------------------

    def get_distance_metres(self,aLocation1, aLocation2):
        dlat        = aLocation2.lat - aLocation1.lat
        dlong       = aLocation2.lon - aLocation1.lon
        return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

#-----------------------------------------------------------------------------------------

    def get_location_metres(self, original_location, dNorth, dEast):
        #
        # Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
        # specified `original_location`. The returned LocationGlobal has the same `alt` value
        # as `original_location`.
        #
        # The function is useful when you want to move the vehicle around specifying locations relative to 
        # the current vehicle position.
        #
        # The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
        #
        # For more information see:
        # http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
        #
        earth_radius = 6378137.0
        dLat = dNorth/earth_radius
        dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))
        newlat = original_location.lat + (dLat * 180/math.pi)
        newlon = original_location.lon + (dLon * 180/math.pi)
        if type(original_location) is LocationGlobal:
            targetlocation=LocationGlobal(newlat, newlon,original_location.alt)
        elif type(original_location) is LocationGlobalRelative:
            targetlocation=LocationGlobalRelative(newlat, newlon,original_location.alt)
        else:
            raise Exception("Invalid Location object passed")   
        return targetlocation

#-----------------------------------------------------------------------------------------

    def send_ned_velocity(self, velocity_x, velocity_y, velocity_z, duration, *args):
        #   
        #  Move vehicle in direction based on specified velocity vectors.
        #   
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,                                                              # time_boot_ms (not used)
            0, 0,                                                           # target system, target component
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,                      # type of frame MAV_FRAME_BODY_OFFSET_NED or MAV_FRAME_LOCAL_NED
            0b0000111111000111,                                             # type_mask (only speeds enabled)
            0, 0, 0,                                                        # x, y, z positions (not used)
            velocity_x, velocity_y, velocity_z,                             # x, y, z velocity in m/s
            0, 0, 0,                                                        # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)                                                           # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
        #
        # send command to vehicle on 1 Hz cycle
        #
        for x in range(0,duration):
            self.vehicle.send_mavlink(msg)
            time.sleep(1)
        return True

#-----------------------------------------------------------------------------------------

    def send_global_velocity(self, velocity_x, velocity_y, velocity_z, duration):
        #
        # Move vehicle in direction based on specified velocity vectors.
        #
        # This uses the SET_POSITION_TARGET_GLOBAL_INT command with type mask enabling only 
        # velocity components 
        # (http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/#set_position_target_global_int).
        #
        # Note that from AC3.3 the message should be re-sent every second (after about 3 seconds
        # with no message the velocity will drop back to zero). In AC3.2.1 and earlier the specified
        # velocity persists until it is canceled. The code below should work on either version 
        # (sending the message multiple times does not cause problems).
        #
        # See the above link for information on the type_mask (0=enable, 1=ignore). 
        # At time of writing, acceleration and yaw bits are ignored.
        #
        msg = self.vehicle.message_factory.set_position_target_global_int_encode(
            0,       # time_boot_ms (not used)
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
            0b0000111111000111, # type_mask (only speeds enabled)
            0, # lat_int - X Position in WGS84 frame in 1e7 * meters
            0, # lon_int - Y Position in WGS84 frame in 1e7 * meters
            0, # alt - Altitude in meters in AMSL altitude(not WGS84 if absolute or relative)
            # altitude above terrain if GLOBAL_TERRAIN_ALT_INT
            velocity_x, # X velocity in NED frame in m/s
            velocity_y, # Y velocity in NED frame in m/s
            velocity_z, # Z velocity in NED frame in m/s
            0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 

        # send command to vehicle on 1 Hz cycle
        for x in range(0,duration):
            self.vehicle.send_mavlink(msg)
            time.sleep(1)    

#-----------------------------------------------------------------------------------------

    def condition_yaw(self,heading, relative=False, clock=1, *args):
        #
        #   Send MAV_CMD_CONDITION_YAW message to point vehicle at a specified heading (in degrees).
        # 
        #   This method sets an absolute heading by default, but you can set the `relative` parameter
        #   to `True` to set yaw relative to the current yaw heading.
        # 
        #   By default the yaw of the vehicle will follow the direction of travel. After setting 
        #   the yaw using this function there is no way to return to the default yaw "follow direction 
        #   of travel" behaviour (https://github.com/diydrones/ardupilot/issues/2427)
        # 
        #   For more information see: 
        #   http://copter.ardupilot.com/wiki/common-mavlink-mission-command-messages-mav_cmd/#mav_cmd_condition_yaw
        #
        if relative:
            is_relative = 1 #yaw relative to direction of travel
        else:
            is_relative = 0 #yaw is an absolute angle
        #
        # create the CONDITION_YAW command using command_long_encode()
        #
        msg = self.vehicle.message_factory.command_long_encode(
                                                               0, 0,                                  # target system, target component
                                                               mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
                                                               0,                                     #confirmation
                                                               heading,                               # param 1, yaw in degrees
                                                               0,                                     # param 2, yaw speed deg/s
                                                               clock,                                 # param 3, direction -1 ccw, 1 cw
                                                               is_relative,                           # param 4, relative offset 1, absolute angle 0
                                                               0, 0, 0                                # param 5 ~ 7 not used
                                                              )
        #
        # send command to vehicle
        #
        self.vehicle.send_mavlink(msg)

#-----------------------------------------------------------------------------------------

    def goto_position_target_global_int(self,aLocation):
        msg = self.vehicle.message_factory.set_position_target_global_int_encode(
            0,       # time_boot_ms (not used)
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame      
            0b0000111111111000, # type_mask (only speeds enabled)
            aLocation.lat*1e7, # lat_int - X Position in WGS84 frame in 1e7 * meters
            aLocation.lon*1e7, # lon_int - Y Position in WGS84 frame in 1e7 * meters
            aLocation.alt, # alt - Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_INT
            0, # X velocity in NED frame in m/s
            0, # Y velocity in NED frame in m/s
            0, # Z velocity in NED frame in m/s
            0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
        #
        # send command to vehicle
        #
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()
        return True

#-----------------------------------------------------------------------------------------

    def goto_position_target_local_ned(self, north, east, down):
        #	
        # Send SET_POSITION_TARGET_LOCAL_NED command to request the vehicle fly to a specified 
        # location in the North, East, Down frame.
        #
        # It is important to remember that in this frame, positive altitudes are entered as negative 
        # "Down" values. So if down is "10", this will be 10 metres below the home altitude.
        #
        # Starting from AC3.3 the method respects the frame setting. Prior to that the frame was
        # ignored. For more information see: 
        # http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/#set_position_target_local_ned
        #
        # See the above link for information on the type_mask (0=enable, 1=ignore). 
        # At time of writing, acceleration and yaw bits are ignored.
        #
        #
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,       # time_boot_ms (not used)
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
            0b0000111111111000, # type_mask (only positions enabled)
            north, east, down, # x, y, z positions (or North, East, Down in the MAV_FRAME_BODY_NED frame
            0, 0, 0, # x, y, z velocity in m/s  (not used)
            0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
        # send command to vehicle
        self.vehicle.send_mavlink(msg)

#-----------------------------------------------------------------------------------------

    def get_bearing(self, aLocation1, aLocation2):
        #
        # Returns the bearing between the two LocationGlobal objects passed as parameters.
        #
        # This method is an approximation, and may not be accurate over large distances and close to the 
        # earth's poles. It comes from the ArduPilot test code: 
        # https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
        #	
        off_x = aLocation2.lon - aLocation1.lon
        off_y = aLocation2.lat - aLocation1.lat
        bearing = 90.00 + math.atan2(-off_y, off_x) * 57.2957795
        if bearing < 0:
            bearing += 360.00
        return bearing;

#-----------------------------------------------------------------------------------------

    def PrintFirstInfo(self, *args):
        self._log(" Home location: %s" % self.vehicle.home_location)
        self._log(" Connecting to vehicle on: %s" % self.ConnectionString)
        self._log(" Connected to vehicle.")
        self._log(" Global Location: %s" % self.vehicle.location.global_frame)
        self._log(" Global Location (relative altitude): %s" % self.vehicle.location.global_relative_frame)
        self._log(" Local Location: %s" % self.vehicle.location.local_frame)
        self._log(" Attitude: %s" % self.vehicle.attitude)
        self._log(" Velocity: %s" % self.vehicle.velocity)
        self._log(" GPS: %s" % self.vehicle.gps_0)
        self._log(" Battery: %s" % self.vehicle.battery)
        self._log(" EKF OK?: %s" % self.vehicle.ekf_ok)
        self._log(" Last Heartbeat: %s" % self.vehicle.last_heartbeat)
        self._log(" Rangefinder: %s" % self.vehicle.rangefinder)
        self._log(" Rangefinder distance: %s" % self.vehicle.rangefinder.distance)
        self._log(" Rangefinder voltage: %s" % self.vehicle.rangefinder.voltage)
        self._log(" Heading: %s" % self.vehicle.heading)
        self._log(" Is Armable?: %s" % self.vehicle.is_armable)
        self._log(" System status: %s" % self.vehicle.system_status.state)
        self._log(" Groundspeed: %s" % self.vehicle.groundspeed)
        self._log(" Airspeed: %s" % self.vehicle.airspeed)
        self._log(" Mode: %s" % self.vehicle.mode.name)
        self._log(" Armed: %s" % self.vehicle.armed)
        return True

#-----------------------------------------------------------------------------------------

    def is_valid_port(self,port):
        test_port=int(port)
        if test_port > 1023 and test_port < 65535:
           return True
        else:
            return False
        return True

#-----------------------------------------------------------------------------------------

    def is_valid_ipv4_address(self,address):
        try:
            socket.inet_pton(socket.AF_INET, address)
        except AttributeError:  # no inet_pton here, sorry
            try:
                socket.inet_aton(address)
            except socket.error:
                return False
            return address.count('.') == 3
        except socket.error:  # not a valid address
            return False
        return True

#-----------------------------------------------------------------------------------------

    def is_valid_ipv6_address(self,address):
        try:
            socket.inet_pton(socket.AF_INET6, address)
        except socket.error:  # not a valid address
            return False
        return True

#-----------------------------------------------------------------------------------------

def main(argv):   
    myopts, args = getopt.getopt(sys.argv[1:],"h:s:")
    ynetbook=False
    for opt, val in myopts:
       if opt == '-h':
          print 'Help Usage: python ./Drone.py -s <screen: netbook or normal> (netbook for netbook screens)'
          sys.exit(0)
       elif opt == '-s':
          screen=str(val)
          if screen=="netbook":
             ynetbook=True
          elif screen=="normal":
             ynetbook=False
       else:
          print argv
          print 'Error --> Usage: python ./Drone.py -s  <screen: netbook or normal> (netbook for netbook screens)'
          print "          Use -h option to help"
          sys.exit(0)
    controler=MainWindow(ynetbook)
    try:
       Gtk.main()
    except (KeyboardInterrupt, SystemExit):
       sys.exit(0)
       raise

if __name__ == "__main__":
    try:
        GObject.threads_init()
        main(sys.argv[1:])
    except (KeyboardInterrupt, SystemExit):
        sys.exit(0)
        raise
