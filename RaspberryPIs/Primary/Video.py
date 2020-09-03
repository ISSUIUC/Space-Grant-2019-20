import cv2
import threading
import datetime
from time import time
from queue import Queue

cap = cv2.VideoCapture(0)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
fsize = (int(cap.get(3)), int(cap.get(4)))
frameRate = int(cap.get(5))

fontface = cv2.FONT_HERSHEY_SIMPLEX
fontscale = 1
fontcolor = (255, 255, 255)

class Writer(threading.Thread):
    def __init__(this, o, f):
        threading.Thread.__init__(this)
        this.out = o
        this.frames = f
        this.running = True
    def stop(this):
        this.running = False
    def run(this):
        while this.running or this.frames.qsize() > 0:
            print(this.frames.qsize())
            this.out.write(this.frames.get())

class Video:
    data = [0] * 10

    def __init__(this):
        this.frames = Queue(50)
        fourc = cv2.VideoWriter_fourcc(*'H264')
        date = datetime.datetime.now()
        #name = date.strftime("UIUC-SG-%h/%M-%m/%d/%y.mp4")
        name = "Final.mp4"
        this.out = cv2.VideoWriter(name, apiPreference = cv2.CAP_ANY, fourcc = fourc, fps = frameRate, frameSize = fsize)
        this.writer = Writer(this.out, this.frames)
        this.writer.start()
        this.t = time()

    def nextFrame(this):
        text = ["Axial Acceleration:", "X: " + str(this.data[3]), "Y: " + str(this.data[4]),"Z: " + str(this.data[5])]
        text.append("Radial Acceleration: " + str(this.data[8]))
        text.append("Internal Pressure: " + str(this.data[9]))
        text.append(str(time() - this.t))
        this.t = time()
        ret, frame = cap.read()
        y = 25
        for t in range(len(text)):
            cv2.putText(frame, text[t], (10, y), fontface, fontscale, fontcolor)
            y = y + 25
        #this.out.write(frame)
        this.frames.put(frame)

    def stop(this):
        this.writer.stop()
        this.nextFrame()

    def updateData(this, newData):
        this.data = newData

