Coded for python 3.7.  To use, run controlpanel.py with Python3.7 on a Raspberry Pi desktop.
The Adafruit 16-channel servo controller should be connected to the Raspberry Pi using the
default address of 0x40 (hard-coded in pca9685.py).  The I2C interface requires the smbus
Python module.  The graphical interface requires Tkinter.

