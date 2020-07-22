# LocatiOn netWork caLculation - OWL
import krpc, time, math, os, numpy
import numpy as np
from math import *
conn = krpc.connect(name='Network Calculation')
vessel = conn.space_center.active_vessel
control = vessel.control
body = vessel.orbit.body
create_relative = conn.space_center.ReferenceFrame.create_relative
flight = vessel.flight(vessel.orbit.body.reference_frame)
if os.name == 'posix':
    os.system("clear")
else:
    os.system("cls")

targetNames = np.load("targets/targetNames.npy", allow_pickle=True)
targets = np.load("targets/targets.npy", allow_pickle=True)

print("Targets list:\n")
i = 0
for x in targetNames:
    print(str(i) + ": " + x)
    i += 1
#print('(%.1f, %.1f, %.1f)' % vessel.position(vessel.orbit.body.reference_frame))

# Add useful functions
def polar_cartesian(rotation, distance):
	rotation = radians(rotation)
	return (sin(rotation)*distance, cos(rotation)*distance)

# Calculation from https://andrew.hedges.name/experiments/haversine/
def getDistance(lat1, lon1, lat2, lon2, R=600000): # R = radius in metres
    dlon = radians(lon2) - radians(lon1)
    dlat = radians(lat2) - radians(lat1)
    a = (sin(dlat/2))**2 + cos(lat1) * cos(lat2) * (sin(dlon/2))**2
    c = 2 * atan2( sqrt(a), sqrt(1-a) )
    d = R * c
    return d

# From https://www.movable-type.co.uk/scripts/latlong.html
def calculateAzimuth(lat1, lon1, lat2, lon2):
	φ1 = radians(lat1)
	λ1 = radians(lon1)
	φ2 = radians(lat2)
	λ2 = radians(lon2)
	Δλ = λ2-λ1
	θ = atan2(sin(Δλ) * cos(φ2), cos(φ1) * sin(φ2) - sin(φ1) * cos(φ2) * cos(Δλ)) # Azimuth
	return degrees(θ) # Return output in degrees

def calculateLatLon(lat1, lon1, lat2, lon2, R=600000): # R = radius in metres
	return (calculateAzimuth(lat1, lon1, lat2, lon2), getDistance(lat1, lon1, lat2, lon2, R))

#Note in these scripts
#I generally use lat/lon for lati­tude/longi­tude in degrees, and φ/λ for lati­tude/longi­tude in radians
#having found that mixing degrees & radians is often the easiest route to head-scratching bugs...

# Steps:
# maybe 1. Calculate azimuth (angle) between longitude and latitudes Done
# maybe 2. Calculate distance between longitude and latitudes Done
# maybe 3. Convert direction from angle to cartesian Done
# 4. Fire raycast (using direction from step 3)
# 5. If raycast is shorter than calculated distance (step 2) no connection
# 5. If greater than or equal to calculated distance then add connection

azimuth, distance = calculateLatLon(0, 0, 1, 0)
rayDirection = list(polar_cartesian(azimuth, 1))
rayDirection.append(0)
rayDirection = tuple(rayDirection)

print(rayDirection)

#for x in range(90):
#	print(polar_cartesian(x*4, 1))

# Define the landing site as the top of the VAB
landing_latitude = -(0+(5.0/60)+(48.38/60/60))
landing_longitude = -(74+(37.0/60)+(12.2/60/60))

# Determine landing site reference frame
# (orientation: x=zenith, y=north, z=east)
start_position = body.position_at_altitude(
    targets[4][0][0], targets[4][0][1], 40, body.reference_frame)
landing_position = body.position_at_altitude(
    targets[1][0][0], targets[1][0][1], 40, body.reference_frame)

print(start_position, landing_position, [a - b for a, b in zip(landing_position, start_position)])
