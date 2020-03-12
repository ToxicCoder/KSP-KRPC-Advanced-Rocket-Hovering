import krpc, time, math, os, numpy
conn = krpc.connect(name='Hover V2')
vessel = conn.space_center.active_vessel
control = vessel.control
flight = vessel.flight(vessel.orbit.body.reference_frame)
os.system("cls")

print(vessel.name)

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
targetNames = ["Just Hover", "Tracking Station", "Administration Building", "VAB", "Landing Pad"]
targets = [[-1, -1, 40, False, False], # Just Hover
		[-0.09260748710094725, -74.66306148797543, 40, False, True], # Tracking Station
		[-0.09234810450805865, -74.66295876716225, 200, True, True], # Administration Building
		[-0.09664795580728258, -74.61999866061524, 200, True, True], # VAB
		[-0.09720758699224381, -74.55768331492169, 40, True, True]] # Landing Pad

print("Targets list:\n")
i = 0
for x in targetNames:
	print(str(i) + ": " + x)
	i += 1
print("\n")
selection = int(input("Please select a target's number from the list: "))

targetLatitude, targetLongitude = 0, 0

targetData = targets[selection]

if targetData[0] != -1:
	targetLatitude, targetLongitude = targetData[0], targetData[1]
	useSealevel = targetData[3]
	targetHeight = targetData[2]
	land = targetData[4]
	print(targetNames[selection])
else:
	targetLatitude, targetLongitude = vessel.flight().latitude, vessel.flight().longitude
	useSeaLevel = False
	targetHeight = 40
	land = False
	print("Just Hover")


# More variable setup
origTargHeight = targetHeight
g = 9.81

# Boost the rocket into the air
if str(vessel.situation)[16:] == "landed":
    control.throttle = 1
    time.sleep(250000/vessel.available_thrust)

targetRoll = 0

rollCurrent = 0
rollDifference = 1
rollSpeed = 0

startTime = 0
endTime = 1
duration = 1

rollError = 0
rollControl = 0

def quaternion_to_euler(quaternionInput):
    x, y, z, w = quaternionInput[0], quaternionInput[1], quaternionInput[2], quaternionInput[3]

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return [yaw, pitch, roll]

def listAverage(listInput):
    output = 0
    for x in listInput:
        output += x
    output = output / len(listInput)
    return output

average = [0, 0, 0, 0]

vessel.auto_pilot.engage()

# Main loop
while True:
    #print(quaternion_to_euler(flight.rotation)[2] - -1.587310489250823)
    startTime = time.time()

    rollPrevious = rollCurrent
    rollCurrent = quaternion_to_euler(flight.rotation)[0]#vessel.flight().roll
    rollDifference = rollCurrent - rollPrevious
    rollSpeed = rollDifference / duration
    time.sleep(0.025)
    endTime = time.time()
    duration = endTime - startTime
    del average[0]
    average.append(rollSpeed)
    #print(round(listAverage(average), 2))

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

    # This is for the change from sea-level to surface based altitude measuring
    if useSeaLevel:
        alt_error = targetHeight - flight.mean_altitude
    else:
        alt_error = targetHeight - flight.surface_altitude

    # This part uses equations from gist.github.com/djungelorm/01b597163491a3ed0bce
    # compute the desired acceleration:
    #   g   to counteract gravity
    #   -v  to push the vertical speed towards 0
    #   e   to push the altitude error towards 0
    a = g - (flight.vertical_speed*2) + alt_error

    # Compute throttle setting using newton's law F=ma and change throttle
    F = vessel.mass * a
    if str(vessel.situation)[16:] != "landed":
        if not drop:
            ht = (F / vessel.available_thrust)
            control.throttle = ht+(ht*(flight.pitch/90))
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

    # This section keeps the rocket stable
    vessel.auto_pilot.target_roll = 0
    vessel.auto_pilot.target_pitch = 90

    #print((((targetLongitude - longitude)*16)*50) - (velocity[2]/2))


    #print(abs(round(latitudeDiff + longitudeDiff, 4))) # Display distance to target
    # This is the landing procedure
    if land:
        if abs(round(latitudeDiff + longitudeDiff, 4)) < 0.0002 or abs(round(latitudeDiff + longitudeDiff, 4)) == 0.0:
            targetHeight = 2.5
            drop = True
        elif abs(round(latitudeDiff + longitudeDiff, 4)) <= 0.0002:
            targetHeight = 15
        elif abs(round(latitudeDiff + longitudeDiff, 4)) < 0.003:
            control.gear = True
            targetHeight = 40
            useSeaLevel = False
        else:
            control.gear = False
            targetHeight = origTargHeight
            useSealevel = True
    time.sleep(0.01)