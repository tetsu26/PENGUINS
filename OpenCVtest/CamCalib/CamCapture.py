import picamera
from time import sleep

camera = picamera.PiCamera()

sleep(5)

camera.resolution = (1920,1080)
camera.start_preview()

for i in range(30):
    filename = "image{0}.jpg" .format(i)
    camera.capture(filename)
    print "hai!!"
    sleep(2)

print "Owari"

camera.stop_preview()
