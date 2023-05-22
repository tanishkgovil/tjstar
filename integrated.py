from roboflow import Roboflow
from water_blast import getAngle
from pyfirmata import Arduino, SERVO, util
import time

board = Arduino('COM6')
it = util.Iterator(board)
it.start()
board.digital[9].mode = SERVO
rf = Roboflow(api_key="1NZtlP9KhqoJYeTjyy3v")
project = rf.workspace().project("deerwatch")
model = project.version(4).model

# inputted image
def predictAngle(image):
    prediction = model.predict(image, confidence=40, overlap=30).json()

    # if deer is in image, then use x and y coordinates to figure out how far deer is
    if prediction['predictions']:
        x = prediction['predictions'][0]['x']
        y = prediction['predictions'][0]['y']
        height = prediction['predictions'][0]['height']
        width = prediction['predictions'][0]['width']
        printPrediction = {'x': x, 'y': y, 'height': height, 'width':width}
        realX = x/height
        realY = y/height
        angle_of_tube = getAngle(realX, realY, 24.93822910822762)
        print(angle_of_tube)
    print(printPrediction)
    return angle_of_tube

angle1 = predictAngle('test.jpg')
angle2 = predictAngle('test2.jpg')
angle3 = predictAngle('test3.jpg')
print([angle1, angle2, angle3])
while True:
    board.digital[9].write((180 -angle1))
    time.sleep(1)
    board.digital[9].write((180 -angle2))
    time.sleep(1)
    board.digital[9].write((180 -angle3))
    time.sleep(1)