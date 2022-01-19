# This module serves as the control interface to an Adafruit PCA9685 16-channel servo
# controller.  A function is provided to load configuration data (pca9685.conf) for up
# to 16 channels, defining minimum and maximum angles in degrees as well as short names
# for each servo.  Functions are also provided to load and apply presets (groups of
# pre-defined servo settings).  Note that the I2C address is hard-coded and presumed
# to be 0x40 (the default).

import smbus
import time
import json
from os import _exit, path


SERVO_MIN_ANGLE = 0.0
SERVO_MAX_ANGLE = 180.0

class ConfigMissingError(Exception):
    """
    Raised when a variable is missing in the configuration file during load.
    """
    def __init__(self, name, fileType):
        self.message = f'Missing {name} variable in {fileType}.'
        super().__init__(self.message)


class ConfigTypeError(Exception):
    """
    Raised when a variable type is incorrect in the configuration file during load.
    """
    def __init__(self, name, value, expType, fileType):
        self.message = (f'Expected {expType} type for variable {name} '
                        f'with value {value} in {fileType}.')
        super().__init__(self.message)


class Servo:
    """
    Used to store information about the configuration and state of a servo.
    """
    def __init__(self, name, angle=0.0, minAngle=SERVO_MIN_ANGLE, maxAngle=SERVO_MAX_ANGLE):
        self.name = name
        self.pulseWidthMs = 0.0
        self.angle = angle
        self.minAngle = minAngle
        self.maxAngle = maxAngle


class PCA9685Controller:

    OSCILLATOR_FREQ   = 25000000
    REG_MODE1         = 0x00
    REG_MODE2         = 0x01
    REG_LED0_BASE     = 0x06
    REG_ALL_LED_OFF_H = 0xfd
    REG_PRESCALE      = 0xfe

    OFFSET_ON_COUNT_L  = 0
    OFFSET_ON_COUNT_H  = 1
    OFFSET_OFF_COUNT_L = 2
    OFFSET_OFF_COUNT_H = 3
    OFFSET_BLOCK_SIZE  = 4

    CMD_MODE_SLEEP    = 0x10
    CMD_MODE_NORMAL   = 0x00
    CMD_MODE_RESTART  = 0x80
    CMD_LED_ALL_OFF   = 0x10

    STS_MASK_SLEEPING = 0x10
    STS_MASK_RESTART  = 0x80

    def __init__(self, i2cBaseAddr, debug=False):
        self.i2c = smbus.SMBus(1)
        self.i2cBaseAddr = i2cBaseAddr
        self.debug = debug
        self.connected = False
        self.restart = False
        self.sleeping = False
        self.updateFreqHz = 0
        self.preScale = 0
        self.mode1 = 0
        self.mode2 = 0
        self.configData = {}
        self.servoDict = {}
        self.presetData = {}
        self.presetDict = {}

    def loadConfig(self, configFilePath):
        """
        Load the configuration file identifying the attached servos and their associated
        data.  Check beyond the basic JSON formatting to ensure that all the required
        attributes are present and that they are the expected type.  Servo data is
        saved in a configuration dictionary where it may be later modified.
        """
        success = False
        self.servoDict = {}
        self.configData = {}
        if not path.exists(configFilePath):
            msg(LVL_ERROR, f'Cannot find configuration file {configFilePath}.')
        else:
            msg(LVL_NONE, f'Loading configuration file {configFilePath}...')
            try:
                with open(configFilePath, 'r') as configFileH:
                    try:
                        self.configData = json.load(configFileH)
                    except:
                        msg(LVL_ERROR, f'Malformed JSON file.')
                    if not 'servos' in self.configData.keys():
                        msg(LVL_ERROR, f'No list named "servos" in file {configFilePath}.')
                    else:
                        try:
                            for servoCfg in self.configData['servos']:
                                if not 'name' in servoCfg.keys():
                                    raise ConfigMissingError('name', 'configuration')
                                if not 'deviceNum' in servoCfg.keys():
                                    raise ConfigMissingError('deviceNum', 'configuration')
                                if not 'angle' in servoCfg.keys():
                                    raise ConfigMissingError('angle', 'configuration')
                                if not 'minAngle' in servoCfg.keys():
                                    raise ConfigMissingError('minAngle', 'configuration')
                                if not 'maxAngle' in servoCfg.keys():
                                    raise ConfigMissingError('maxAngle', 'configuration')
                                if not isinstance(servoCfg['name'], str):
                                    raise ConfigTypeError('name', servoCfg['name'], str,
                                                          'configuration')
                                if not isinstance(servoCfg['deviceNum'], int):
                                    raise ConfigTypeError('deviceNum', servoCfg['deviceNum'], int,
                                                          'configuration')
                                if not isinstance(servoCfg['angle'], float):
                                    raise ConfigTypeError('angle', servoCfg['angle'], float,
                                                          'configuration')
                                if not isinstance(servoCfg['minAngle'], float):
                                    raise ConfigTypeError('minAngle', servoCfg['minAngle'], float,
                                                          'configuration')
                                if not isinstance(servoCfg['maxAngle'], float):
                                    raise ConfigTypeError('maxAngle', servoCfg['maxAngle'], str,
                                                          'configuration')

                                # Create a Servo object from each configured servo.  Note that the
                                # configuration dictionary is used only for loading and unloading
                                # JSON configuration, while the servo dictionary is used to maintain
                                # live servo status.
                                self.servoDict[servoCfg['deviceNum']] = Servo(servoCfg['name'],
                                                                              angle=servoCfg['angle'],
                                                                              minAngle=servoCfg['minAngle'],
                                                                              maxAngle=servoCfg['maxAngle'])
                                msg(LVL_OK, f'Configured servo {servoCfg["deviceNum"]} as {servoCfg["name"]}')
                            msg(LVL_NONE, f'Configuration loaded.')
                            success = True
                        except ConfigMissingError as e:
                            msg(LVL_ERROR, e.message)
                        except ConfigTypeError as e:
                            msg(LVL_ERROR, e.message)

            except OSError:
                msg(LVL_ERROR, f'Cannot load configuration file {configFilePath}.')

        return success

    def saveConfig(self, configFilePath):
        """
        Copy current servo angle, minimum angle, and maximum angle to the configuration
        dictionary, then export it to a new JSON-formatted configuration file.
        """
        msg(LVL_NONE, f'Saving configuration file {configFilePath}...')
        for servoCfg in self.configData['servos']:
            servoCfg['angle'] = self.servoDict[servoCfg['deviceNum']].angle
            servoCfg['minAngle'] = self.servoDict[servoCfg['deviceNum']].minAngle
            servoCfg['maxAngle'] = self.servoDict[servoCfg['deviceNum']].maxAngle
        try:
            with open(configFilePath, 'w') as configFileH:
                try:
                    json.dump(self.configData, configFileH, indent=4, default=lambda o:o.__dict__)
                except:
                    msg(LVL_ERROR, f'Unable to export configuration as {configFilePath}.')
        except:
            msg(LVL_ERROR, f'Cannot open configuration file {configFilePath} for writing.')

    def loadPresets(self, presetFilePath):
        """
        Load the presets file containing groups of servo settings for pre-defined actions.
        Check beyond the basic JSON formatting to ensure that all the required attributes
        are present and that they are the expected type.  The structure of the file should
        be a list named 'presets', with each list item being a dictionary containing
        'name' (str) and 'actions' (list).  Each actions list item should be a list of
        dictionaries, where each dictionary contains a 'deviceNum' (int) and 'angle' (float)
        used to set each servo.
        """
        success = False
        self.presetData = {}
        self.presetDict = {}
        if not path.exists(presetFilePath):
            msg(LVL_ERROR, f'Cannot find presets file {presetFilePath}.')
        else:
            msg(LVL_NONE, f'Loading configuration file {presetFilePath}...')
            try:
                with open(presetFilePath, 'r') as presetFileH:
                    try:
                        self.presetData = json.load(presetFileH)
                    except:
                        msg(LVL_ERROR, f'Malformed JSON file.')
                    if not 'presets' in self.presetData.keys():
                        msg(LVL_ERROR, f'No list named "presets" in file {presetFilePath}.')
                    else:
                        try:
                            for presetCfg in self.presetData['presets']:
                                if not 'name' in presetCfg.keys():
                                    raise ConfigMissingError('name', 'presets')
                                if not 'actions' in presetCfg.keys():
                                    raise ConfigMissingError('actions', 'presets')
                                if not isinstance(presetCfg['name'], str):
                                    raise ConfigTypeError('name', presetCfg['name'], str, 'presets')
                                if not isinstance(presetCfg['actions'], list):
                                    raise ConfigTypeError('actions', 'unknown', list, 'presets')
                                for action in presetCfg['actions']:
                                    if not isinstance(action, list):
                                        raise ConfigTypeError('actions[]', 'unknown', list, 'presets')

                                presetName = presetCfg['name']
                                self.presetDict[presetName] = presetCfg['actions']
                                actionCnt = len(presetCfg['actions'])
                                msg(LVL_OK, f'Loaded preset {presetName} ({actionCnt} actions).')
                            msg(LVL_NONE, f'Presets loaded.')
                            success = True
                        except ConfigMissingError as e:
                            msg(LVL_ERROR, e.message)
                        except ConfigTypeError as e:
                            msg(LVL_ERROR, e.message)

            except OSError:
                msg(LVL_ERROR, f'Cannot load presets file {presetFilePath}.')

        return success

    def applyPreset(self, name):
        """
        For the given preset name, apply the stored servo settings for each action of
        the preset with a slight delay between each action.
        """
        success = True
        if not name in self.presetDict.keys():
            msg(LVL_ERROR, f'No preset named {name} in preset configuration data.')
            success = False
        else:
            actionCnt = len(self.presetDict[name])
            msg(LVL_OK, f'Applying preset {name} ({actionCnt} actions).')
            for action in self.presetDict[name]:
                for servoSetting in action:
                    channel = servoSetting['deviceNum']
                    angle = servoSetting['angle']
                    if angle < self.servoDict[channel].minAngle:
                        msg(LVL_ERROR, f'Channel {channel} preset angle {angle} less '
                            f'than current minimum {self.servoDict[channel].minAngle}.')
                    elif angle > self.servoDict[channel].maxAngle:
                        msg(LVL_ERROR, f'Channel {channel} preset angle {angle} greater '
                            f'than current maximum {self.servoDict[channel].maxAngle}.')
                    else:
                        pw = self.pulseWidthMsFromAngle(servoSetting['angle'],
                                                        minAngle=SERVO_MIN_ANGLE,
                                                        maxAngle=SERVO_MAX_ANGLE,
                                                        minWidthMs=0.5, maxWidthMs=2.5)
                        self.setPulseWidth(servoSetting['deviceNum'], pw,
                                           servoSetting['angle'])
                time.sleep(0.12)

        return success

    def getStatus(self):
        """
        Read and decode the controller mode register values.
        """
        try:
            if not self.debug:
                self.mode1 = self.i2c.read_byte_data(self.i2cBaseAddr, self.REG_MODE1)
                self.mode2 = self.i2c.read_byte_data(self.i2cBaseAddr, self.REG_MODE2)
            if self.mode1 & self.STS_MASK_RESTART == 0:
                self.restart = False
            else:
                self.restart = True
            if self.mode1 & self.STS_MASK_SLEEPING == 0:
                self.sleeping = False
            else:
                self.sleeping = True
        except OSError:
            msg(LVL_WARN, f'Unable to connect to I2C address 0x{self.i2cBaseAddr:02x}.')
            self.connected = False
        else:
            self.connected = True

    def printStatus(self):
        """
        Print information about the controller connection, state, and configuration.
        """
        if self.connected:
            connStr = f'{C_GREEN}connected{C_END}'
        else:
            connStr = f'{C_RED}NOT connected{C_END}'
        if not self.restart:
            startStr = f'{C_GREEN}running{C_END}'
        else:
            startStr = f'{C_RED}stopped{C_END}'
        if not self.sleeping:
            modeStr = f'{C_GREEN}normal mode{C_END}'
        else:
            modeStr = f'{C_RED}sleep mode{C_END}'
        msg(LVL_NONE,
            f'Status: {connStr}, {startStr}, {modeStr}, '
            f'prescale={self.preScale}(0x{self.preScale:02x}), '
            f'mode 1=0x{self.mode1:02x}, mode 2=0x{self.mode2:02x}')

    def start(self, updateFreqHz):
        """
        Set the PWM update frequency and start controller operation of all channels.
        """
        if updateFreqHz < 40 or updateFreqHz > 60:
            msg(LVL_ERROR, f'Update frequency {updateFreqHz} is out of range 40-60 Hz.')
        else:
            self.updateFreqHz = updateFreqHz
            self.preScale = int((self.OSCILLATOR_FREQ / (self.updateFreqHz * 4096)) - 1)
            try:
                if not self.debug:
                    self.i2c.write_byte_data(self.i2cBaseAddr, self.REG_PRESCALE, self.preScale)
                    self.i2c.write_byte_data(self.i2cBaseAddr, self.REG_MODE1, self.CMD_MODE_NORMAL)
                    time.sleep(0.1)
                    self.i2c.write_byte_data(self.i2cBaseAddr, self.REG_MODE1, self.CMD_MODE_RESTART)
                    time.sleep(0.1)
                self.getStatus()
            except OSError:
                msg(LVL_WARN, f'Unable to write to I2C address 0x{self.i2cBaseAddr:02x}.')
                self.connected = False
            else:
                self.connected = True
                msg(LVL_OK, f'Set PWM prescale to {self.preScale}(0x{self.preScale:02x}).')

    def reset(self):
        """
        Issue a controller restart while ensuring that a complete PWM cycle is completed.
        """
        if not self.debug:
            try:
                mode = self.i2c.read_byte_data(self.i2cBaseAddr, self.REG_MODE1)
                if (mode & 0x80) == 0:
                    self.i2c.write_byte_data(self.i2cBaseAddr, self.REG_MODE1, (mode & 0x04))
                    time.sleep(0.1)
                    self.i2c.write_byte_data(self.i2cBaseAddr, self.REG_MODE1, 0x80)
                else:
                    msg(LVL_WARN, f'Unable to restart controller - restart bit is set.')
            except OSError:
                msg(LVL_ERROR, f'Unable to restart controller - not connected.')

    def stop(self):
        """
        Put the controller in sleep mode, ending all PWM output.
        """
        try:
            if not self.debug:
                self.i2c.write_byte_data(self.i2cBaseAddr, self.REG_ALL_LED_OFF_H, self.CMD_LED_ALL_OFF)
                self.i2c.write_byte_data(self.i2cBaseAddr, self.REG_MODE1, self.CMD_MODE_SLEEP)
                time.sleep(0.1)
            self.getStatus()
        except OSError:
            msg(LVL_WARN, f'Unable to write to I2C address 0x{self.i2cBaseAddr:02x}.')
            self.connected = False
        else:
            self.connected = True

    def pulseWidthMsFromAngle(self, angle, minAngle=0.0, maxAngle=180.0,
                              minWidthMs=0.5, maxWidthMs=2.5):
        """
        Given a desired servo angle, the range of possible angles, and the corresponding
        range of pulse width values, calculate and return the pulse width necessary to
        produce the servo angle.
        """
        if angle < minAngle or angle > maxAngle:
            pulseWidthMs = 0.0
        else:
            pulseWidthRangeMs = maxWidthMs - minWidthMs
            pulseWidthMs = (minWidthMs + pulseWidthRangeMs * (angle / (maxAngle-minAngle)))

        return pulseWidthMs

    def setPulseWidth(self, channel, pulseWidthMs, angle):
        """
        Set the PWM pulse width for a given channel by calculating on which clock ticks
        to set the output high or low and writing the data to the corresponding channel
        registers.
        """
        updateIntervalMs = 1000.0 / self.updateFreqHz
        oneCountDurationMs = updateIntervalMs / 4096
        totalUpdIntervalCount = int(updateIntervalMs / oneCountDurationMs)
        totalOnCount = int(pulseWidthMs / oneCountDurationMs)
        totalOffCount = totalUpdIntervalCount - totalOnCount
        turnOnOffset = int(( totalUpdIntervalCount / 2 ) - ( totalOnCount / 2))
        turnOnOffsetLow = 0xff & turnOnOffset
        turnOnOffsetHigh = turnOnOffset >> 8
        turnOffOffset = turnOnOffset + totalOnCount
        turnOffOffsetLow = 0xff & turnOffOffset
        turnOffOffsetHigh = turnOffOffset >> 8
        if self.debug:
            msg(LVL_DEBUG, f'channel {channel} pulse width={pulseWidthMs:.3f} ms, '
                f'on at {turnOnOffset} for {totalOnCount}, '
                f'off at {turnOffOffset} for {totalOffCount}.')
        try:
            if not self.debug:
                blockAddr = self.REG_LED0_BASE + (self.OFFSET_BLOCK_SIZE * channel)
                self.i2c.write_byte_data(self.i2cBaseAddr, blockAddr + self.OFFSET_ON_COUNT_L,
                                         turnOnOffsetLow)
                self.i2c.write_byte_data(self.i2cBaseAddr, blockAddr + self.OFFSET_ON_COUNT_H,
                                         turnOnOffsetHigh)
                self.i2c.write_byte_data(self.i2cBaseAddr, blockAddr + self.OFFSET_OFF_COUNT_L,
                                         turnOffOffsetLow)
                self.i2c.write_byte_data(self.i2cBaseAddr, blockAddr + self.OFFSET_OFF_COUNT_H,
                                         turnOffOffsetHigh)
        except OSError:
            msg(LVL_WARN, f'Unable to write to I2C address 0x{self.i2cBaseAddr:02x}.')
            self.connected = False
        else:
            self.connected = True

        self.servoDict[channel].angle = angle
        self.servoDict[channel].pulseWidthMs = pulseWidthMs

    def setServoMinAngle(self, channel, minAngle):
        if channel in self.servoDict.keys():
            self.servoDict[channel].minAngle = minAngle
        else:
            msg(LVL_ERROR, f'Channel {channel} not found.  Cannot set minimum angle.')

    def setServoMaxAngle(self, channel, maxAngle):
        if channel in self.servoDict.keys():
            self.servoDict[channel].maxAngle = maxAngle
        else:
            msg(LVL_ERROR, f'Channel {channel} not found.  Cannot set maximum angle.')

# end of class PCA9658Controller


# Message severities.
LVL_NONE = 0
LVL_OK = 1
LVL_WARN = 2
LVL_ERROR = 3
LVL_DEBUG = 4

# ANSI escape sequences for text colors.
C_RED = '\033[91m'      # red
C_BLUE = '\033[94m'     # blue
C_GREEN = '\033[92m'    # green
C_YELLOW = '\033[93m'   # yellow
C_MAGENTA = '\033[95m'  # magenta
C_END = '\033[0m'       # end coloring

def msg(level, text):
    """
    Display a message with severity level and color it according to severity level.
    """
    if level == LVL_OK:
        print(f'{C_GREEN}OK: {text}{C_END}')
    elif level == LVL_WARN:
        print(f'{C_YELLOW}WARNING: {text}{C_END}')
    elif level == LVL_ERROR:
        print(f'{C_RED}ERROR: {text}{C_END}')
    elif level == LVL_DEBUG:
        print(f'{C_MAGENTA}DEBUG: {text}{C_END}')
    else:
        print(text)

def inputServoAngle(pulseWidthMinMs, pulseWidthMaxMs):
    """
    Accept user input for a desired servo angle and convert the angle into a pulse
    width in milliseconds.
    """
    angle = 0.0
    pulseWidthMs = 0
    pulseWidthRangeMs = pulseWidthMaxMs - pulseWidthMinMs
    while True:
        menuInput = input(f'Enter desired angle in degrees ({SERVO_MIN_ANGLE}-'
                          f'{SERVO_MAX_ANGLE} or Q to quit):')
        if menuInput == 'Q' or menuInput == 'q' or menuInput == '':
            break
        try:
            tryAngle = float(menuInput)
            if tryAngle < SERVO_MIN_ANGLE or tryAngle > SERVO_MAX_ANGLE:
                msg(LVL_WARN, f'Value {tryAngle} out of range.')
            else:
                pulseWidthMs = (pulseWidthMinMs + pulseWidthRangeMs *
                                (tryAngle / (SERVO_MAX_ANGLE-SERVO_MIN_ANGLE)))
                angle = tryAngle
                break
        except:
            msg(LVL_WARN, f'{tryAngle} is not a valid input.  Enter a floating point value.')

    return (angle, pulseWidthMs, menuInput)

def selectChannel(servoDict):
    menuInput = ''
    channel = '0'
    while True:
        msg(LVL_NONE, '\nServo List:')
        for (deviceNum, servo) in servoDict.items():
            msg(LVL_NONE, f'{deviceNum} - {servo.name}')
        menuInput = input('Enter device number, a for all, or q to quit menu:')
        if menuInput == 'Q' or menuInput == 'q' or menuInput == 'A' or menuInput == 'a':
            break
        try:
            chkChannel = int(menuInput)
        except ValueError:
            msg(LVL_WARN, f'{menuInput} is not an integer.')
            continue
        if chkChannel in servoDict.keys():
            channel = chkChannel
            break
        else:
            msg(LVL_WARN, f'{menuInput} is not a valid channel.')

    return (channel, menuInput)


######################################################################################
# MAIN
#
if __name__ == '__main__':

    updateFreqHz = 50
    pulseWidthMinMs = 0.5
    pulseWidthMaxMs = 2.5
    i2cAddr = 0x40

    controller = PCA9685Controller(i2cAddr, debug=False)
    success = controller.loadConfig('pca9685.conf')
    if success == False:
        msg(LVL_ERROR, f'Controller not configured.  Quitting.')
        _exit(1)
    success = controller.loadPresets('presets.conf')
    controller.start(updateFreqHz)

    while True:
        msg(LVL_NONE, f'\n{C_BLUE}Current servo settings:{C_END}')
        for (channel, servo) in controller.servoDict.items():
            msg(LVL_NONE,
                f'  channel {channel:>2}, angle {servo.angle:>6.2f}, '
                f'pulse width {servo.pulseWidthMs:>5.3f}, name {servo.name}')
        controller.getStatus()
        controller.printStatus()
        (angle, pulseWidthMs, menuInput) = inputServoAngle(pulseWidthMinMs, pulseWidthMaxMs)
        if menuInput == 'Q' or menuInput == 'q':
            msg(LVL_OK, 'Quitting.')
            break
        else:
            msg(LVL_NONE, '\n')
            (channel, menuInput) = selectChannel(controller.servoDict)
            if not menuInput == 'Q' and not menuInput == 'q':
                if menuInput == 'A' or menuInput == 'a':
                    for (deviceNum, servo) in controller.servoDict.items():
                        channel = int(deviceNum)
                        controller.setPulseWidth(channel, pulseWidthMs, angle)
                else:
                    controller.setPulseWidth(channel, pulseWidthMs, angle)

    controller.saveConfig('pca9685save.conf')
    controller.stop()


