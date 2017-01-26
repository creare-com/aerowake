from Tkinter import *


def callback_takeoff():
    print "Takeoff Vehicle"

def callback_land():
    print "Land Vehicle"

def callback_miss():
    print "Advance Mission"

root = Tk()
frame = Frame(root)
frame.pack()

b1 = Button(frame, 
                     text="QUIT", fg="red",
                     command=frame.quit)
b1.pack(fill=X)

b2 = Button(frame, text="Takeoff", command=callback_takeoff)
b2.pack(fill=X)

b3 = Button(frame, text="Land", command=callback_land)
b3.pack(fill=X)

l1 = Label(root, fg="dark green")
l1.pack(fill=X)

curr = "[%.2f, %.2f, %.2f]"%(0,1,2)
l1.config(text=curr)

b4 = Button(frame, text="Advance Mission", command=callback_miss)
b4.pack(fill=X)

root.mainloop()





















