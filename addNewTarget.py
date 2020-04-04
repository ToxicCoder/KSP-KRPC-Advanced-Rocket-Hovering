import numpy as np
import krpc, time
conn = krpc.connect(name='Hover V2')
vessel = conn.space_center.active_vessel
control = vessel.control
flight = vessel.flight(vessel.orbit.body.reference_frame)

vessel.control.input_mode = vessel.control.input_mode.override
control.sas = True
time.sleep(0.1)
control.sas_mode = control.sas_mode.radial
control.rcs = True
control.activate_next_stage()

print(r"Please follow the instructions on the wiki page or the 'guides/Saving New Locations.md' file\n\n")

def displayTargets(targetNames):
	print("Current Targets:\n")
	i = 0
	for x in targetNames:
	    print(str(i) + ": " + x)
	    i += 1

targetNames = np.load("targets/targetNames.npy", allow_pickle=True)
targets = np.load("targets/targets.npy", allow_pickle=True)

displayTargets(targetNames)

targetHeight = 100
a = 0
try:
	while True:
		if vessel.available_thrust == 0:
		    raise Exception("Loss of thrust detected! Please check your engines still exist.")
	
		targetHeight += control.forward / 4
		# This is for the change from sea-level to surface based altitude measuring
		alt_error = targetHeight - flight.mean_altitude
		# This part uses equations from gist.github.com/djungelorm/01b597163491a3ed0bce
		# compute the desired acceleration:
		#   g   to counteract gravity
		#   -v  to push the vertical speed towards 0
		#   e   to push the altitude error towards 0
		a = ((9.81 - (flight.vertical_speed*5) + alt_error)+a)/2
		# Compute throttle setting using newton's law F=ma and change throttle
		F = vessel.mass * a
		ht = (F / vessel.available_thrust)
		control.throttle = ht+(ht*(flight.pitch/90))
		latLon = [flight.latitude, flight.longitude]
		altitude = flight.mean_altitude
except KeyboardInterrupt:
	if input("Would you like to save this location? (y/n): ").lower() == "y":
		name = input("Please enter a name for this location: ")
		name = name[0].upper() + name[1:]
		if input("Is this location on a small landing pad? (y/n): ").lower() == "y":
			landingAccuracy = 4
		else:
			landingAccuracy = 1
		targetNames = np.append(targetNames, name)
		targets = list(targets)
		targets.append([latLon, [], (round(altitude/10)*10)+10, True, True, False, landingAccuracy])
		np.save("targets/targetNames.npy", targetNames)
		np.save("targets/targets.npy", targets)
		print("Location saved!")
		targetNames = np.load("targets/targetNames.npy", allow_pickle=True)
		targets = np.load("targets/targets.npy", allow_pickle=True)
		print()
		displayTargets(targetNames)
	else:
		print("No worries, you can restart this program to try again")
		print("\nNo Changed made")

# Un-comment the following section to set the targets list to the defaults
#
#targetNames = ["Just Hover", "Tracking Station", "Administration Building", "VAB", "Landing Pad", "Mission Control", "Water Tower SPH", "Water Tower Launchpad"]
#targets = [[[-1, -1], [[-1, -1]], 40, True, False, False, 1, 1], # Just Hover
#        [[-0.12707740447720042, -74.60547772969107], [], 40, False, True, False, 1], # Tracking Station
#        [[-0.09260748710094725, -74.66306148797543], [], 40, False, True, False, 1], # Administration Building
#        [[-0.09664795580728258, -74.61999866061524], [], 200, True, True, False, 1], # VAB
#        [[-0.09720758699224381, -74.55768331492169], [], 40, False, True, False, 1], # Landing Pad
#        [[-0.07486285149048191, -74.61355530825483], [], 40, False, True, False, 1], # Mission Control
#        [[-0.058009709890787194, -74.64144279145307], [], 40, False, True, False, 4], # Water Tower SPH
#        [[-0.09212809071597255, -74.55249954150645], [], 40, False, True, False, 4]] # Water Tower launchpad
#
#np.save("targets/targetNames.npy", targetNames)
#np.save("targets/targets.npy", targets)
