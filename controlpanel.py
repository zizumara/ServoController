# This module provides a graphical display for controlling multiple channels of an Adafruit
# 16-channel servo controller (based on the PCA9685 MCU).  Each servo is controlled by a
# slider which sets the angle of the servo in degrees.  Buttons are provided at each end
# of the slider to set minimum and maximum angles according to the mechanical limits of
# the mechanisms to which each servo is attached.  A JSON configuration file (pca9685.conf)
# determines the labeling and min/max limits for each servo.  All current settings may be
# exported to a JSON configuration file (pca9685Save.conf), which may be renamed to
# pca9685.conf and used for initial configuration of the servos when loading this module.
# The interface also provides buttons to activate presets (i.e. groups of servo settings)
# which are configured via the JSON file presets.conf.

from tkinter import *
from quitter import Quitter
from servoslider import ServoSlider
from pca9685 import PCA9685Controller, Servo

class PresetsPanel(Frame):
    """
    This class defines a graphical panel containing buttons for applying a list of preset
    angles to each servo.  The presets are contained in a dictionary in the controller and
    their associated buttons are labeled by the same name used to index the dictionary.
    """

    def __init__(self, controller, sliderDict, parent=None, **options):
        Frame.__init__(self, parent, **options)
        self.controller = controller
        self.sliderDict = sliderDict
        Label(self, text='Presets', bg=options.get('bg', None), font=("Arial 11 bold")).pack(side=LEFT)
        for name in controller.presetDict.keys():
            Button(self, text=name, command=lambda name=name: self.onClick(name)).pack(side=LEFT)

    def onClick(self, name):
        self.controller.applyPreset(name)
        for (channel, servo) in self.controller.servoDict.items():
            self.sliderDict[channel].setMarker(servo.angle)


class PCA9685ServoSlider(ServoSlider):
    """
    This class defines a graphical scale of angles that may be applied to an associated
    servo, including controls for dynamically setting the allowed minimum and maximum
    angles.  Changes in the slider position result in calls to the servo controller
    to apply the servo angle (after converting to pulse width).
    """

    def __init__(self, parent, labelText=' ', angle=90.0, min=0.0, max=180.0,
                 bgColor='#a0b0ff', channel=0, controller=None, **options):
        ServoSlider.__init__(self, parent, labelText=labelText, angle=angle,
                             min=min, max=max, bgColor=bgColor, **options)
        self.channel = channel
        self.controller = controller

    def onMove(self, value):
        if not self.controller == None:
            angle = float(value)
            pulseWidthMs = self.controller.pulseWidthMsFromAngle(angle)
            self.controller.setPulseWidth(self.channel, pulseWidthMs, angle)


class PCA9685Panel(Frame):
    """
    This class defines a graphical control panel containing controls for applying
    settings to a list of servos managed by a servo controller.  Names, initial
    angles, and min/max limits are acquired from a configuration file.  This panel
    may be used to establish the desired servo min/max angles and initial values
    according to the mechanism to which each servo is attached (eg. a servo may
    be constrained to a range of positions well inside of the range 0 to 180
    degrees).  An export button permits these settings to be saved in a new
    version of the configuration file that may be used in subsequent sessions.
    The panel also provides a list of preset actions (via buttons) read from
    a presets configuration file.  When clicked, each button applies one or more
    sets of preset angles to the controller servos with a slight delay between
    each set.
    """

    BG_COLOR1 = '#a0a0ff'
    BG_COLOR2 = '#a0ffa0'
    BG_COLOR3 = '#ff80c0'

    def __init__(self, configFile, i2cBaseAddr, updateFreqHz, parent=None, presetFile=None):
        Frame.__init__(self, parent)
        self.controller = PCA9685Controller(i2cBaseAddr)
        success = self.controller.loadConfig(configFile)
        if success == False:
            print(f'Controller not configured.  Quitting.')
            _exit(1)
        havePresets = False
        if not presetFile == None:
            havePresets = self.controller.loadPresets(presetFile)
            if havePresets == False:
                print(f'Load of preset file {presetFile} failed.')
        bg = self.BG_COLOR1
        self.sliderDict = {}
        for (channel, servo) in self.controller.servoDict.items():
            slider = PCA9685ServoSlider(self, labelText=f'Channel {channel}: {servo.name}',
                                        angle=servo.angle, min=servo.minAngle, max=servo.maxAngle,
                                        bgColor=bg, channel=channel, controller=self.controller,
                                        bg=bg, bd=4, relief=GROOVE)
            slider.pack(side=TOP)
            self.sliderDict[channel] = slider
            if bg == self.BG_COLOR1:
                bg = self.BG_COLOR2
            else:
                bg = self.BG_COLOR1
        PresetsPanel(self.controller, self.sliderDict, parent=self, bd=4, bg=self.BG_COLOR3, relief=GROOVE,
                     padx=3, pady=3).pack(fill=X)
        saver = Button(self, text='Export', command=self.export)
        saver.pack(side=LEFT)
        Quitter(self, parent).pack(side=BOTTOM)
        self.controller.getStatus()
        self.controller.printStatus()
        self.controller.start(updateFreqHz)

        # Set each servo to the initial position of its associated servo slider.
        for (channel, servo) in self.controller.servoDict.items():
            self.sliderDict[channel].onMove(servo.angle)

    def export(self):
        """
        Export the control panel settings to a configuration file after retrieving the
        min and max values from each slider and saving them in the controller servos.
        """
        for (channel, slider) in self.sliderDict.items():
            (minAngle, maxAngle) = slider.getMinMax()
            self.controller.setServoMinAngle(channel, minAngle)
            self.controller.setServoMaxAngle(channel, maxAngle)
        self.controller.saveConfig('pca9685Save.conf')

if __name__ == '__main__':
    UPDATE_FREQ_HZ = 50
    I2C_BASE_ADDR = 0x40

    root = Tk()
    panel = PCA9685Panel('pca9685.conf', I2C_BASE_ADDR, UPDATE_FREQ_HZ, parent=root,
                         presetFile='presets.conf')
    panel.pack()
    panel.mainloop()

