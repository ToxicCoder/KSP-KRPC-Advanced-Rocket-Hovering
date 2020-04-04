import krpc, time, math, os, numpy
import numpy as np
from math import *
conn = krpc.connect(name='Hover V2')
vessel = conn.space_center.active_vessel
control = vessel.control
flight = vessel.flight(vessel.orbit.body.reference_frame)
if os.name == 'posix':
    os.system("clear")
else:
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

# Customisation
frameRate = 60 # Can be adjusted depending on situation - Min: 2 Max: 120 default: 60
legDeployTime = 10 # Actual deploy time is 6 seconds but it's modified to improve timing
maxHorizSpeed = 100 # Doesn't need to be set as the equations factor in enough data to stay safe
dropSpeed = -7.5 # Can be adjusted to give cooler results
overrideLandOnEngine = False # Change this to True to have manual control over landing on engine

# Target Locations
#
# Name latitude, longitude
#
# Just Hover Mode:  Nothing, Nothing
# Tracking Station:  -0.12707740447720042, -74.60547772969107
# VAB:  -0.09664795580728258, -74.61999866061524
# Admin Building:  -0.09260748710094725, -74.66306148797543
# Landing Pad: -0.09720758699224381, -74.55768331492169
# Secret mode: enter the number -1 on the selection screen and say no to waypoints

# Variables
drop = False
land = False
longLat = True
useSeaLevel = False
dropAccuracy = 1
targetHeight = 200 # target altitude above the surface, in metres

# For adding custom targets use this format:
# Add "Name of target" to targetNames
# [[latitude, longitude], [waypointLatitude, waypointLongitude], altitude used in flight, use sea level altitude for flight, do you land or not (always True unless hovering), use waypoints, accuracy for landing drop]
#
# Select Target
targetNames = np.load("targets/targetNames.npy", allow_pickle=True)
targets = np.load("targets/targets.npy", allow_pickle=True)

print("Targets list:\n")
i = 0
for x in targetNames:
    print(str(i) + ": " + x)
    i += 1

def getChoice():
    try:
        print("\n")
        selection = int(input("Please select a target's number from the list: "))
    except ValueError:
        print("INVALID CHOICE")
        selection = getChoice()
    return selection
selection = getChoice()

def waypointSelect(targets, selection, loop=False):
    if not loop:
        choice = input("Do you want to go via a waypoint? (y/n): ").lower()
    else:
        choice = "yes"
    if choice == "y" or choice == "yes":
        waypointSelection = int(input("Please select a waypoint to go via: "))
        targets[selection][1].append(targets[waypointSelection][0])
        targets[selection][5] = True
        print("Waypoint added")
        choice = input("Do you want another waypoint? (y/n): ").lower()
        if choice == "y" or choice == "yes":
            targets = waypointSelect(targets, selection, True)
    else:
        targets[selection][5] = False
        print("No waypoint added")
    return targets

targets = waypointSelect(targets, selection)

targetLatitude, targetLongitude = 0, 0

targetData = targets[selection]

if selection != -1:
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
        longLat = True
        if targetData[6] >= 4:
            dropAccuracy = 2
        else:
            dropAccuracy = 1
        print(targetNames[selection])
    else:
        destinationLatitude, destinationLongitude = vessel.flight().latitude, vessel.flight().longitude
        targetLatitude, targetLongitude = vessel.flight().latitude, vessel.flight().longitude
        waypointLatLon = [[vessel.flight().latitude, vessel.flight().longitude]]
        useSeaLevel = False
        targetHeight = 40
        land = False
        longLat = False
        print("Just Hover")
else:
    destinationLatitude, destinationLongitude = vessel.flight().latitude, vessel.flight().longitude
    targetLatitude, targetLongitude = vessel.flight().latitude, vessel.flight().longitude
    waypointLatLon = [[vessel.flight().latitude, vessel.flight().longitude]]
    useSeaLevel = True
    targetHeight = 100
    land = False
    longLat = False
    print("Secret: No location hold")

print()
land = input("Land at destination? (y/n - default = y): ").lower()
if land == "y" or land == "yes":
    land = True
elif land == "":
    land = True
else:
    land = False
if land:
    if overrideLandOnEngine:
        landEngine = input("Balance on engine when landed? (y/n): ").lower()
        if landEngine == "y" or landEngine == "yes":
            landEngine = True
        else:
            landEngine = False
    else:
        landEngine = targetData[6] >= 4

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

speedCurrent = 0
speedDifference = 1

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

# Calculation from https://andrew.hedges.name/experiments/haversine/
def getDistance(lat1, lon1, lat2, lon2, R=600000): # R = radius in metres
    dlon = radians(lon2) - radians(lon1)
    dlat = radians(lat2) - radians(lat1)
    a = (sin(dlat/2))**2 + cos(lat1) * cos(lat2) * (sin(dlon/2))**2
    c = 2 * atan2( sqrt(a), sqrt(1-a) )
    d = R * c
    return d

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

a = 0
openLegs = False

control.rcs = True

# Main loop
while True:
    startTime = time.time()

    speedPrevious = speedCurrent
    speedCurrent = flight.vertical_speed
    speedDifference = (speedCurrent - speedPrevious)/duration
    speedNext = speedCurrent + speedDifference

    # Telemetry
    landed = str(vessel.situation)[16:] == "landed"
    latitude = vessel.flight().latitude
    longitude = vessel.flight().longitude

    latitudeDiff = waypointLatLon[currentWaypointID][0] - latitude
    longitudeDiff = waypointLatLon[currentWaypointID][1] - longitude

    targetLatitude, targetLongitude = waypointLatLon[currentWaypointID][0], waypointLatLon[currentWaypointID][1]

    heading = vessel.flight().heading
    angvel = vessel.angular_velocity(ref_frame)

    # Calculate distance to target
    distance = abs(getDistance(latitude, longitude, waypointLatLon[currentWaypointID][0], waypointLatLon[currentWaypointID][1]))

    # User input
    targetHeight += control.forward / 4

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
    a = ((g - (flight.vertical_speed*5) + alt_error)+a)/2

    # Compute throttle setting using newton's law F=ma and change throttle
    F = vessel.mass * a
    if vessel.available_thrust == 0:
        raise Exception("Loss of thrust detected! Please check your engines still exist.")
    if not landed:
        if not drop:
            try:
                ht = ((F+(F-vessel.thrust)/2) / vessel.available_thrust)
            except ZeroDivisionError:
                raise Exception("Loss of thrust detected! Please check your engines still exist.")
                break
            control.throttle = ht+(ht*(flight.pitch/90))
        else:
            try:
                control.throttle = (F / vessel.available_thrust) / (8/(dropSpeed - flight.vertical_speed))
            except:
                control.throttle = (F / vessel.available_thrust) / 4
    else:
        if land:
            control.throttle = 0
            break

    # Position Control
    velocity = vessel.flight(ref_frame).velocity
    touchDown = finals and abs(flight.surface_altitude / flight.vertical_speed) < 3
    if longLat and not touchDown:
        if abs(velocity[2]) < maxHorizSpeed:
            longitudeControl = ((longitudeDiff*16)*35) - (velocity[2]/2) #((np.clip(longitudeDiff, -0.02, 0.02)*16)*35) - (velocity[2]/2)
        else:
            longitudeControl = -velocity[2] / maxHorizSpeed
        if abs(velocity[1]) < maxHorizSpeed:
            latitudeControl = -(((latitudeDiff*16)*35) - (velocity[1]/2)) #-(((np.clip(latitudeDiff, -0.02, 0.02)*16)*35) - (velocity[1]/2))
        else:
            latitudeControl = velocity[1] / maxHorizSpeed

        if finals:
            controlRot = [90, -latitudeControl*10, longitudeControl*10]
        else:
            controlRot = [90, (-latitudeControl*5)/(abs(ht)*2), (longitudeControl*5)/(abs(ht)*2)]
        controlRot[1] = np.clip(controlRot[1], -45, 45)
        controlRot[2] = np.clip(controlRot[2], -45, 45)
        vessel.auto_pilot.target_direction = controlRot
        apDirection = vessel.auto_pilot.target_direction
    else:
        vessel.auto_pilot.target_roll = 0
        vessel.auto_pilot.target_heading = 0
        vessel.auto_pilot.target_pitch = 90

    # Physics warp control
    if longLat:
        if distance < 2:
            if conn.space_center.physics_warp_factor != 0:
                if finals:
                    conn.space_center.physics_warp_factor = 0
        elif distance < 10:
            if conn.space_center.physics_warp_factor != 1:
                conn.space_center.physics_warp_factor = 1
        else:
            if conn.space_center.physics_warp_factor != 2:
                conn.space_center.physics_warp_factor = 2

    # This is the landing procedure
    if land:
        if finals:
            if distance <= 1/dropAccuracy or distance == 0.0:
                targetHeight = 40
                drop = True
                if abs(flight.surface_altitude / flight.vertical_speed) <= legDeployTime and not landEngine:
                    control.gear = True
            elif distance < 10:
                drop = False
                control.gear = False
                targetHeight = 40
                useSeaLevel = False
            elif not distance < 10:
                drop = False
                control.gear = False
                targetHeight = origTargHeight
                useSealevel = True
        else:
            if distance < 100:
                drop = False
                targetLatitude, targetLongitude = waypointLatLon[currentWaypointID][0], waypointLatLon[currentWaypointID][1]
                currentWaypointID += 1
                if currentWaypointID >= len(waypointLatLon)-1:
                    finals = True
                    currentWaypointID = len(waypointLatLon)-1
    try:
        time.sleep((1/frameRate)-(time.time()-startTime))
    except:
        pass
    endTime = time.time()
    duration = endTime - startTime

vessel.auto_pilot.disengage()

try:
    if landed and not vessel.parts.legs[0].deployed and not landEngine:
        control.gear = True
        control.sas = True
        time.sleep(0.1)
        control.sas_mode = control.sas_mode.radial
    if landEngine:
        control.sas = True
        time.sleep(0.1)
        control.sas_mode = control.sas_mode.radial
except:
    pass
