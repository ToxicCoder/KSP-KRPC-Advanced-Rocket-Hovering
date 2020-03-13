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

frameRate = 60

# Target Locations
#
# Name latitude, longitude
#
# Just Hover Mode:  Nothing, Nothing
# Tracking Station:  -0.12707740447720042, -74.60547772969107
# VAB:  -0.09664795580728258, -74.61999866061524
# Admin Building:  -0.09260748710094725, -74.66306148797543
# Landing Pad: -0.09720758699224381, -74.55768331492169

# Variables
drop = False
land = False
longLat = True
useSeaLevel = False

targetHeight = 200 # target altitude above the surface, in meters

# For adding custom targets use this format:
# Add "Name of target" to targetNames
# [[latitude, longitude], [waypointLatitude, waypointLongitude], altitude used in flight, use sea level altitude for flight, do you land or not (always True unless hovering), use waypoints]
#
# Select Target
targetNames = ["Just Hover", "Tracking Station", "Administration Building", "VAB", "Landing Pad"]
targets = [[[-1, -1], [[-1, -1]], 40, False, False, False], # Just Hover
        [[-0.12707740447720042, -74.60547772969107], [], 40, False, True, False], # Tracking Station
        [[-0.09260748710094725, -74.66306148797543], [], 40, False, True, False], # Administration Building
        [[-0.09664795580728258, -74.61999866061524], [], 200, True, True, False], # VAB
        [[-0.09720758699224381, -74.55768331492169], [], 40, True, True, False]] # Landing Pad

print("Targets list:\n")
i = 0
for x in targetNames:
    print(str(i) + ": " + x)
    i += 1
print("\n")
selection = int(input("Please select a target's number from the list: "))

def waypointSelect(targets, selection, loop=False):
    print(targets[selection])
    if not loop:
        choice = input("Do you want to go via a waypoint? ").lower()
    else:
        choice = "yes"
    if choice == "y" or choice == "yes":
        waypointSelection = int(input("Please select a waypoint to go via: "))
        targets[selection][1].append(targets[waypointSelection][0])
        targets[selection][5] = True
        print("Waypoint added")
        choice = input("Do you want another waypoint? ").lower()
        if choice == "y" or choice == "yes":
            targets = waypointSelect(targets, selection, True)
    else:
        targets[selection][5] = False
        print("No waypoint added")
    return targets

targets = waypointSelect(targets, selection)

targetLatitude, targetLongitude = 0, 0

targetData = targets[selection]

if targetData[0] != -1:
    if targetData[5]:
        targetLatitude, targetLongitude = targetData[1][0][0], targetData[1][0][1]
        waypointLatLon = targetData[1]
        waypointLatLon.append(targetData[0])
    else:
        targetLatitude, targetLongitude = targetData[0][0], targetData[0][1]
        waypointLatLon = [targetData[0]]
    useSealevel = targetData[3]
    targetHeight = targetData[2]
    land = targetData[4]
    print(targetNames[selection])
else:
    destinationLatitude, destinationLongitude = vessel.flight().latitude, vessel.flight().longitude
    targetLatitude, targetLongitude = vessel.flight().latitude, vessel.flight().longitude
    waypointLatLon = [[vessel.flight().latitude, vessel.flight().longitude]]
    useSeaLevel = False
    targetHeight = 40
    land = False
    print("Just Hover")


# More variable setup
origTargHeight = targetHeight
g = 9.81

# Boost the rocket into the air
control.gear = False
if str(vessel.situation)[16:] == "landed":
    control.throttle = 1
    while flight.surface_altitude < 10:
        time.sleep(0.1)

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

currentWaypointID = 0
finals = False

# Main loop
while True:
    startTime = time.time()

    rollPrevious = rollCurrent
    rollCurrent = quaternion_to_euler(flight.rotation)[0]
    rollDifference = rollCurrent - rollPrevious
    rollSpeed = rollDifference / duration
    time.sleep(0.025)
    endTime = time.time()
    duration = endTime - startTime
    del average[0]
    average.append(rollSpeed)

    # Change Targets
    heightControl = control.forward

    # Get data about flight
    latitude = vessel.flight().latitude
    longitude = vessel.flight().longitude

    latitudeDiff = waypointLatLon[currentWaypointID][0] - latitude
    longitudeDiff = waypointLatLon[currentWaypointID][1] - longitude

    targetLatitude, targetLongitude = waypointLatLon[currentWaypointID][0], waypointLatLon[currentWaypointID][1]

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
            control.throttle = (F / vessel.available_thrust) / abs(-abs(flight.vertical_speed)+7.5)
    else:
        if land:
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

    # Physics warp control
    if finals:
        if abs(round(latitudeDiff + longitudeDiff, 4)) < 0.001:
            if conn.space_center.physics_warp_factor != 0:
                conn.space_center.physics_warp_factor = 0
        elif abs(round(latitudeDiff + longitudeDiff, 4)) < 0.01:
            if conn.space_center.physics_warp_factor != 1:
                conn.space_center.physics_warp_factor = 1
    
    if abs(round(latitudeDiff + longitudeDiff, 4)) >= 0.01:
        if conn.space_center.physics_warp_factor != 2:
            conn.space_center.physics_warp_factor = 2

    # This is the landing procedure
    if land:
        if finals:
            if abs(round(latitudeDiff + longitudeDiff, 4)) < 0.0002 or abs(round(latitudeDiff + longitudeDiff, 4)) == 0.0:
                targetHeight = 40
                drop = True
            elif abs(round(latitudeDiff + longitudeDiff, 4)) <= 0.0002:
                targetHeight = 40
            elif abs(round(latitudeDiff + longitudeDiff, 4)) < 0.003:
                control.gear = True
                targetHeight = 40
                useSeaLevel = False
            elif not abs(round(latitudeDiff + longitudeDiff, 4)) < 0.003:
                control.gear = False
                targetHeight = origTargHeight
                useSealevel = True
        else:
            if abs(round(latitudeDiff + longitudeDiff, 4)) < 0.01:
                targetLatitude, targetLongitude = waypointLatLon[currentWaypointID][0], waypointLatLon[currentWaypointID][1]
                currentWaypointID += 1
                if currentWaypointID >= len(waypointLatLon)-1:
                    finals = True
                    currentWaypointID = len(waypointLatLon)-1
    try:
        time.sleep((1/frameRate)-(time.time()-startTime))
    except:
        pass
