from threading import Thread
from time import *
from Comm import *

class Master(Thread):
    def __init__(this, d):
        Thread.__init__(this)
        this.data = d
        this.running = True
    def stop(this):
        this.running = False

class VideoThread(Master):
    def __init__(this, d, v):
        Master.__init__(this, d)
        this.video = v
    def run(this):
        while this.running:
            this.video.nextFrame()

class SensorThread(Master):
    def __init__(this, d):
        Master.__init__(this, d)
    def run(this):
        current = time()
        while this.running:
            current = current + 1.0/30
            sleep(max(current - time(), 0))
            raw = getData()
            for i in range(len(raw)):
                this.data[i] = raw[i]

class ArduinoThread(Master):
    def __init__(this, d):
        Master.__init__(this, d)
    def run():
        while this.running:
            print('run')
