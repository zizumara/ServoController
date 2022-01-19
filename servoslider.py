from tkinter import *
from quitter import Quitter


class ServoSlider(Frame):
    """
    This is a graphical slider for tuning and controlling the angle of a servo for which
    the range of possible angles needs to be limited due to mechanical constraints.  The
    minimum and maximum allowed values can be decreased or increased using buttons at
    each end of the scale.
    """
    def __init__(self, parent, labelText=' ', minLimit=0.0, maxLimit=180.0, angle=90.0,
                 min=0.0, max=180.0, bgColor='#a0b0ff', **options):
        Frame.__init__(self, parent, **options)
        self.pack()
        self.bgColor = bgColor
        self.minLimit = minLimit
        self.maxLimit = maxLimit
        self.minAngle = min
        self.maxAngle = max
        self.angle = DoubleVar()
        Button(self, text='<<', command=lambda: self.decrMinimum(5.0)).pack(side=LEFT)
        Button(self, text='<', command=lambda: self.decrMinimum(1.0)).pack(side=LEFT)
        Button(self, text='>', command=lambda: self.incrMinimum(1.0)).pack(side=LEFT)
        Button(self, text='>>', command=lambda: self.incrMinimum(5.0)).pack(side=LEFT)
        self.minLabel = Label(self, text=f'Min {self.minAngle:3.0f}', bg=self.bgColor)
        self.minLabel.pack(side=LEFT)
        Button(self, text='>>', command=lambda: self.incrMaximum(5.0)).pack(side=RIGHT)
        Button(self, text='>', command=lambda: self.incrMaximum(1.0)).pack(side=RIGHT)
        Button(self, text='<', command=lambda: self.decrMaximum(1.0)).pack(side=RIGHT)
        Button(self, text='<<', command=lambda: self.decrMaximum(5.0)).pack(side=RIGHT)
        self.maxLabel = Label(self, text=f'{self.maxAngle:3.0f} Max', bg=self.bgColor)
        self.maxLabel.pack(side=RIGHT)
        self.slider = Scale(self, label=labelText, command=self.onMove, variable=self.angle,
                            from_=self.minAngle, to=self.maxAngle, length=640, tickinterval=10,
                            showvalue=YES, orient='horizontal', bg=self.bgColor,
                            font=("Arial 11 bold"))
        self.slider.pack()
        self.angle.set(angle)

    def onMove(self, value):
        # Override this method in a subclass to take action on the new slider value.
        print('No move action implemented')

    def setMarker(self, angle):
        """
        Manually set the position of the marker.
        """
        self.angle.set(angle)

    def decrMinimum(self, byValue):
        """
        Decrement the minimum value of the slider by the given amount and redraw it.
        """
        if self.minAngle - byValue >= self.minLimit:
            self.minAngle -= byValue
            self.slider.configure(from_=self.minAngle)
            self.minLabel.configure(text=f'Min {self.minAngle:3.0f}')

    def incrMinimum(self, byValue):
        """
        Increment the minimum value of the slider by the given amount and redraw it.
        """
        if self.minAngle + byValue <= self.maxAngle - 5.0:
            self.minAngle += byValue
            if self.angle.get() < self.minAngle:
                self.slider.set(self.minAngle)
            self.slider.configure(from_=self.minAngle)
            self.minLabel.configure(text=f'Min {self.minAngle:3.0f}')

    def incrMaximum(self, byValue):
        """
        Increment the maximum value of the slider by the given amount and redraw it.
        """
        if self.maxAngle + byValue <= self.maxLimit:
            self.maxAngle += byValue
            self.slider.configure(to=self.maxAngle)
            self.maxLabel.configure(text=f'{self.maxAngle:3.0f} Max')

    def decrMaximum(self, byValue):
        """
        Decrement the maximum value of the slider by the given amount and redraw it.
        """
        if self.maxAngle - byValue >= self.minAngle + 5.0:
            self.maxAngle -= byValue
            if self.angle.get() > self.maxAngle:
                self.slider.set(self.maxAngle)
            self.slider.configure(to=self.maxAngle)
            self.maxLabel.configure(text=f'{self.maxAngle:3.0f} Max')

    def getMinMax(self):
        return (self.minAngle, self.maxAngle)


if __name__ == '__main__':
    root = Tk()
    slider = ServoSlider(root, labelText='Channel 0', angle=90.0, min=0.0, max=180.0)
    Quitter(slider).pack(side=BOTTOM)
    root.mainloop()
