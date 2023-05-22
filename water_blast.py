import math
# assumptions: tube is lined up with cylinder in x-direction, tube is at same height as LiDAR (will have to be adjusted)
# inputs: y-coordinate and z-coordinate of cylinder to spray water in
# outputs: angle at which tube must be aimed to spray directly into cylinder

# givens/measured values
tolerance = 0.015 # cm
tube_diameter = 1+tolerance # cm
water_pressure_psi = 45 # pressure in psi
flow_rate_lpm = 11.3 # flow rate in L/min
g = 9.80665 # value of gravitational acceleration in m/s^2
water_density = 997.77 # density of water in kg/m^3

# derived values
water_pressure = water_pressure_psi * 6894.76 # convert to pascal
flow_rate = (flow_rate_lpm / 60) / 1000 # flow rate in m^3/s
tube_radius = (tube_diameter/2) / 100 # tube radius in m
cross_sectional_area = math.pi * (tube_radius**2) # crossectional area of tube in m^2

# formula to get initial speed of water given pressure: sqrt( 2 * pressure / (density) )
initial_speed = math.sqrt( (2 * water_pressure) / (water_density))
#initial_speed = flow_rate/(cross_sectional_area)

#returns angle tube must be positioned at for water to land in target, given horizontal and vertical displacement and initial speed
def getAngle(x, y, Vo):
    a = ( ( (g * ( x**2 )) / (Vo**2) ) + y) / ( math.sqrt( ( y**2 ) + ( x**2 ) ) )
    b = math.acos(a)
    c = math.atan((x)/(y))
    rad = (b + c) / 2
    return 90-((rad*180)/(math.pi)) # converts to degrees and returns

# test data
# x = 7
# y = 5
# print(f'Initial speed of water: {initial_speed} m/s')
# print(f'Horizontal distance to target: {x} m')
# print(f'Vertical distance to target: {y} m')
# print(f'Angle tube must be positioned at: {getAngle(x, y, initial_speed)} degrees')