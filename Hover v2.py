import krpc, time, math, os
conn = krpc.connect(name='Hover V2')
vessel = conn.space_center.active_vessel
control = vessel.control
flight = vessel.flight(vessel.orbit.body.reference_frame)
print(vessel.name)
os.system("cls")

ref_frame = conn.space_center.ReferenceFrame.create_hybrid(
    position=vessel.orbit.body.reference_frame,
    rotation=vessel.surface_reference_frame)

vessel.control.input_mode = vessel.control.input_mode.override
control.sas = True
time.sleep(0.1)
control.sas_mode = control.sas_mode.radial
control.rcs = True
control.activate_next_stage()

latitude = vessel.flight().latitude
longitude = vessel.flight().longitude

print(str(latitude)+", "+str(longitude))

# Target Locations
#
# Name latitude, longitude
#
# Just Hover Mode:  Nothing, Nothing
# Tracking Station:  -0.12718455163649123, -74.60535376109283
# VAB:  -0.09664795580728258, -74.61999866061524
# Admin Building:  -0.09217390347059799, -74.66250078566117

# Variables
drop = False
land = False
longLat = True
useSeaLevel = False

targetHeight = 200 # target altitude above the surface, in meters

# For adding custom targets use this format:
# "Name of target":[latitude, longitude, altitude used in flight, use sea level altitude for flight, do you land or not (always True unless hovering)]
#
# Select Target
targets = {"Just Hover":[-1, -1, 40, False, False], "Tracking Station":[-0.12718455163649123, -74.60535376109283, 40, False, True], "Administration Building":[-0.09217390347059799, -74.66250078566117, 200, True, True], "VAB":[-0.09664795580728258, -74.61999866061524, 200, True, True]}
print("Targets list:\n")
i = 0
for x in targets:
	print(str(i) + ": " + x)
	i += 1
print("\n")
selection = str(input("Please select a target's number from the list: "))

targetLatitude, targetLongitude = 0, 0
getNumber = []
for x in targets:
	getNumber.append(x)

i = 0
for x in getNumber:
	if getNumber[int(selection)] == str(x):
		if targets[getNumber[i]][0] != -1:
			targetLatitude, targetLongitude = targets[x][0], targets[x][1]
			useSealevel = targets[x][3]
			targetHeight = targets[x][2]
			land = targets[x][4]
			print(getNumber[i])
		else:
			targetLatitude, targetLongitude = vessel.flight().latitude, vessel.flight().longitude
			useSeaLevel = False
			targetHeight = 40
			land = False
			print("Just Hover")
	i += 1


# More variable setup
origTargHeight = targetHeight
g = 9.81

# Boost the rocket into the air
if str(vessel.situation)[16:] == "landed":
    control.throttle = 1
    time.sleep(0.25)

# Main loop
while True:
    # Change Targets
    heightControl = control.forward

    # Get data about flight
    latitude = vessel.flight().latitude
    longitude = vessel.flight().longitude
    latitudeDiff = targetLatitude - latitude
    longitudeDiff = targetLongitude - longitude
    heading = vessel.flight().heading
    angvel = vessel.angular_velocity(ref_frame)

    targetHeight += heightControl / 4

    # This is for the change form sea-level to surface based altitude measuring
    if useSeaLevel:
        alt_error = targetHeight - flight.mean_altitude
    else:
        alt_error = targetHeight - flight.surface_altitude

    # compute the desired acceleration:
    #   g   to counteract gravity
    #   -v  to push the vertical speed towards 0
    #   e   to push the altitude error towards 0
    a = g - flight.vertical_speed + alt_error

    # Compute throttle setting using newton's law F=ma and change throttle
    F = vessel.mass * a
    if str(vessel.situation)[16:] != "landed":
        if not drop:
            control.throttle = (F / vessel.available_thrust)
        else:
            control.throttle = (F / vessel.available_thrust) / 4
    else:
        if longLat:
            control.throttle = 0
            break

    # Position Control
    velocity = vessel.flight(ref_frame).velocity
    if longLat:
        if abs(velocity[2]) < 15:
            control.right = (((targetLongitude - longitude)*16)*50) - (velocity[2]/2)
        else:
            control.right = -velocity[2] / 15 / 2
        if abs(velocity[1]) < 15:
            control.up = -((((targetLatitude - latitude)*16)*50) - (velocity[1]/2))
        else:
            control.up = velocity[1] / 15 / 2
    print(abs(round(latitudeDiff + longitudeDiff, 4))) # Display distance to target
    # This is the landing procedure
    if land:
        if abs(round(latitudeDiff + longitudeDiff, 4)) <= 0.0001:
            targetHeight = 2.5
            drop = True
        if abs(round(latitudeDiff + longitudeDiff, 4)) <= 0.0002:
            targetHeight = 10
        if abs(round(latitudeDiff + longitudeDiff, 4)) < 0.003:
            control.gear = True
            targetHeight = 40
            useSeaLevel = False
        else:
            control.gear = False
            targetHeight = origTargHeight
            useSealevel = True
    # This is a terrible system for keeping the rocket in a specific direction
    # if you experience problems with rotation comment the following if statement out and make sure not to touch the controls in flight
    if longLat:
        if abs(angvel[2]) < 0.1:
            control.roll = -( -(round(vessel.flight().rotation[2], 4) - -0.5)**2)
        else:
            control.roll = 0
    time.sleep(0.01)