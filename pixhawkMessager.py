import time
from pymavlink import mavutil
import sys; args = sys.argv[1:]
pixhawkConnection = None
returnPoints = dict()

if args: port = args[0]
else: port = 5

def init() -> None:
	global pixhawkConnection

	# Create the connection
	pixhawkConnection = mavutil.mavlink_connection('COM5')
	# Wait a heartbeat before sending commands
	pixhawkConnection.wait_heartbeat()
	# Make sure the pixhawk is armed
	pixhawkConnection.armed = True

def get_system_status():
	systemStatusMessage = pixhawkConnection.mav.command_long_encode(1, 0,
		mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0,
		1,
		0, 0, 0, 0, 0, 0)
	pixhawkConnection.mav.send(systemStatusMessage)
	return pixhawkConnection.recv_match(type = "SYS_STATUS")


def get_location() -> tuple:
	gpsMessage = pixhawkConnection.mav.command_long_encode(1, 0,
		mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0,
		24,
		0, 0, 0, 0, 0, 0)
	pixhawkConnection.mav.send(gpsMessage)
	return pixhawkConnection.recv_match(type = "GLOBAL_POSITION_INT")

def set_control_mode_autopilot():
	# controlModeMessage = pixhawkConnection.mav.command_long_encode(
	# 	pixhawkConnection.target_system, pixhawkConnection.target_component,
	# 	mavutil.mavlink.SET_MODE, 0,
	# 	pixhawkConnection.target_system,		
	# 	0,		
	# 	0, 		
	# 	0,
	# 	0,
	# 	0,
	# 	0
	# )
	controlModeMessage = pixhawkConnection.mav.command_long_encode(
		pixhawkConnection.target_system, pixhawkConnection.target_component,
		mavutil.mavlink.MAV_CMD_DO_SET_MODE,
		10,
		MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1,
		MAV_MODE_FLAG_CUSTOM_SUBMODE_ENABLED = 1)
	pixhawkConnection.mav.send(controlModeMessage)
def set_servo_pwm(microseconds: int, pwmPort: int = 5) -> None:
	# pixhawkConnection.mav.command_long_send(
    #     	pixhawkConnection.target_system, pixhawkConnection.target_component,
    #     	mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
    #     	0,            # first transmission of this command
    #     	pwmPort+8,    # servo instance, offset by 8 MAIN outputs
    #     	microseconds, # PWM pulse-width
    #     	0,0,0,0,0     # unused parameters
    # 	)
	pixhawkConnection.set_servo(pwmPort+8, microseconds)

def create_waypoint(latitude: float, longitude: float):
	latitude = int(latitude * 10**7); longitude = int(longitude * 10**7)
	# https://www.ardusub.com/developers/pymavlink.html#send-rangefindercomputer-vision-distance-measurement-to-the-autopilot
	pixhawkConnection.mav.gps_input_send(
		0,  # Timestamp (micros since boot or Unix epoch)
		0,  # ID of the GPS for multiple GPS inputs
		# Flags indicating which fields to ignore (see GPS_INPUT_IGNORE_FLAGS enum).
		# All other fields must be provided.
		(mavutil.mavlink.GPS_INPUT_IGNORE_FLAG_VEL_HORIZ |
		mavutil.mavlink.GPS_INPUT_IGNORE_FLAG_VEL_VERT |
		mavutil.mavlink.GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY),
		0,  # GPS time (milliseconds from start of GPS week)
		0,  # GPS week number
		3,  # 0-1: no fix, 2: 2D fix, 3: 3D fix. 4: 3D with DGPS. 5: 3D with RTK
		latitude,  # Latitude (WGS84), in degrees * 1E7
		longitude,  # Longitude (WGS84), in degrees * 1E7
		0,  # Altitude (AMSL, not WGS84), in m (positive for up)
		1,  # GPS HDOP horizontal dilution of position in m
		1,  # GPS VDOP vertical dilution of position in m
		0,  # GPS velocity in m/s in NORTH direction in earth-fixed NED frame
		0,  # GPS velocity in m/s in EAST direction in earth-fixed NED frame
		0,  # GPS velocity in m/s in DOWN direction in earth-fixed NED frame
		0,  # GPS speed accuracy in m/s
		0,  # GPS horizontal accuracy in m
		0,  # GPS vertical accuracy in m
		7   # Number of satellites visible.
    )
	# different method
	# https://www.colorado.edu/recuv/2015/05/25/mavlink-protocol-waypoints
	# wp = mavutil.mavlink.MAVLink_mission_item_message(pixhawkConnection.target_system,
    #     pixhawkConnection.target_component,
    #     1,
    #     mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
    #     mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
    #     0, 0, 0, 10, 0, 0,
    #     latitude,longitude, 0
	# )
	# msg = pixhawkConnection.recv_match(type=['MISSION_REQUEST'],blocking=True)             
	# pixhawkConnection.mav.send(wp(msg.seq)) 
	# print('Sending waypoint {0}'.format(msg.seq))
	print('sent')

# THIS METHOD DOESNT WORK YET
def send_gps_coordinates(latitude: float, longitude: float) -> None:
	latitude = int(latitude * 10**7); longitude = int(longitude * 10**7)

	gpsMessage = pixhawkConnection.mav.command_long_encode(
		pixhawkConnection.target_system, pixhawkConnection.target_component,
		mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0,
		10,		# either useless or time to stay at waypoint
		1,		# accept radius
		0, 		# orbit radius around the waypoint
		0,
		latitude,
		longitude,
		0
	)
	# note to self use commands not messages
	# 82? 25, probably 192 would work
	# right now this is using command 16

	pixhawkConnection.mav.send(gpsMessage)
	# print(pixhawkConnection.recv_match(type = "SET_POSITION_TARGET_GLOBAL_INT"))
	
	# gpsConfirmationMessage = pixhawkConnection.mav.command_long_encode(1, 0,
	# 	mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0,
	# 	87,
	# 	0, 0, 0, 0, 0, 0)
	# pixhawkConnection.mav.send(gpsConfirmationMessage)

	# print(pixhawkConnection.messages)
	# return pixhawkConnection.recv_match(type = "POSITION_TARGET_GLOBAL_INT")
	
	# pixhawkConnection.mav.send(
	# 	mavutil.mavlink.MAVLink_set_position_target_global_int_message(
	# 		10, 
	# 		pixhawkConnection.target_system, pixhawkConnection.target_component, 
	# 		mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 	# reference frame
	# 		int(0b110111111000),
	# 		latitude, longitude,
	# 		0,0,0,0,0,0,0,1.57,0.5     			# unused parameters
	# ))


class pixhawkMessager:
	def __init__(self):
		init()
	def send_gps_coordinates(self, latitude: float, longitude: float):
		send_gps_coordinates(latitude, longitude)
	def set_servo_pwm(microseconds: int, pwmPort: int = 5):
		set_servo_pwm(microseconds, pwmPort)

	def set_return_point(name: str, location: tuple = ()):
		if not location: location = get_location()
		returnPoints[name] = location
		
	def get_return_point(name: str) -> tuple:
		return returnPoints[name][0], returnPoints[name][1]
	
	#def set_control_mode(control_mode):
	#	if control_mode == "RC":
	#		control_with_RC()
	#	elif control_mode == "GPS":
		# for RC control, we could potentially use the message RC_CHANNELS_SCALED
		


def main():
	init()
	# set_servo_pwm(1600)
	print(get_location())
	# send_gps_coordinates(22, 33)
	create_waypoint(22, 33)
	# this piece of code should print out the distance to the next waypoint among other things
	testMessage = pixhawkConnection.mav.command_long_encode(1, 0,
		mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0,
		62,
		0, 0, 0, 0, 0, 0)
	pixhawkConnection.mav.send(testMessage)
	print(testMessage)
	print(pixhawkConnection.recv_match(type = "NAV_CONTROLLER_OUTPUT"))


	# while True:
	# 	for i in range(1000, 2000):
	# 		set_servo_pwm(i)

if __name__== "__main__": main()