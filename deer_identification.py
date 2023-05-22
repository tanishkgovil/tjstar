from roboflow import Roboflow
import math

# inputted image
def predictAngle(image):
    rf = Roboflow(api_key="1NZtlP9KhqoJYeTjyy3v")
    project = rf.workspace().project("deerwatch")
    model = project.version(4).model
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
        angle_of_tube = getAngle(realX, realY)
    print(printPrediction)
    return angle_of_tube

def getAngle(x, y):
    g = 9.80665 # m/s^2
    water_density = 997.77 # kg/m^3

    water_pressure = 45 * 6894.76 # pascals
    
    Vo = math.sqrt( (2 * water_pressure) / (water_density))
    a = ( ( (g * ( x**2 )) / (Vo**2) ) + y) / ( math.sqrt( ( y**2 ) + ( x**2 ) ) )
    b = math.acos(a)
    c = math.atan((x)/(y))
    rad = (b + c) / 2
    return 90-((rad*180)/(math.pi)) # converts to degrees and returns

print(predictAngle('test2.jpg'))