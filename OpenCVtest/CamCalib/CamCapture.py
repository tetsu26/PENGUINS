import picamera
from time import sleep

camera = picamera.PiCamera()

camera.start_preview()

for i in range(20):
    filename = "image{0}.jpg" .format(i)
    camera.capture(filename)
    print "hai!!"
    sleep(3)

print "Owari"

camera.stop_preview()
