from tkinter import *
from tkinter.messagebox import askokcancel

class Quitter(Frame):

    def __init__(self, root, parent=None):
        Frame.__init__(self, parent)
        self.pack()
        self.root = root
        widget = Button(self, text='Quit', command=self.quit)
        widget.pack(side=LEFT, expand=YES, fill=BOTH)

    def quit(self):
        answer = askokcancel('Verify exit', 'OK to quit?')
        if answer:
            Frame.quit(self)
            self.root.destroy()

if __name__ == '__main__':
    quitter = Quitter()
    quitter.mainloop()
