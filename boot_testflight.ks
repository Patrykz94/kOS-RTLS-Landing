// initialisation
@lazyglobal off.
clearscreen.
set ship:control:pilotmainthrottle to 0.


wait until ag1. // The program will wait until you press the Action Group 1 button. If you don't want this, please delete/comment this line.
unlock all.
wait 1.

// default launch parameters can be changed while while starting the program.
// You can do it by typing "run boot_testflight(250,90,1)."

parameter orbAlt is 100. // Default target altitude
parameter orbDir is 90.  // Default launch direction (landing only works on launching to 90 degrees)
parameter landReq is 1.  // Landing site

// requiring libraries

function has_file {
	parameter name.
	local allFiles is 0.
	
	list files in allFiles.
	for file in allFiles {
		if file:name = name {
			return true.
		}
	}
	return false.
}

if not has_file("telemetry.ks") {
	copy telemetry.ks from 0.
}
if not has_file("flight_display.ks") {
	copy flight_display.ks from 0.
}
if not has_file("lib_navball.ks") {
	copy lib_navball.ks from 0.
}
if not has_file("functions.ks") {
	copy functions.ks from 0.
}
if not has_file("maneuvers.ks") {
	copy maneuvers.ks from 0.
}

run telemetry.ks. // general telemetry library
run flight_display.ks.
run lib_navball.ks.
run functions.ks.
run maneuvers.ks.

// setting the ship to a known state

rcs off.
sas off.

// declaring variables

set orbAlt to orbAlt * 1000.
local expTWR is 0.
local compass is orbDir.
local pitch is 90.
local runmode is 1.
local tval is 0.
local clearRequired is false.
local mN is 0.
local nD is 0.
local mnvInProgress is false.
local velLat is 0.
local velLng is 0.
local dt is 0.
local steerPitch is 0.
local steerYaw is 0.
local steer is up.
local gearStatus is 0.
local targetAlt is 0.
local curAlt is 0.
local targetPos is 0.
local altParam is 0.
local targStatus is 0.
local missionT is 0.
local startT is 0.
local launchT is 0.
local oldT is 0.
local prevPos is 0.
local curPos is 0.
local gravTurn is false.
local tempPitch is 0.

// landing pads

local KSCLaunchPad is latlng(-0.0972080884740584, -74.5576970966038).
local KSCVABLandingPad1 is latlng(-0.0968010588104255, -74.6174316020627).
local KSCVABLandingPad2 is latlng(-0.0967649062924497, -74.6200692295964).
local KSCAstrComp is latlng(-0.0925723018712286, -74.6631085805855).
if landReq = 0 {
	local landingSite is 0.
} else if landReq = 1 {
	local landingSite is KSCLaunchPad.
} else if landReq = 2 {
	local landingSite is KSCVABLandingPad1.
} else if landReq = 3 {
	local landingSite is KSCVABLandingPad2.
} else if landReq = 4 {
	local landingSite is KSCAstrComp.
}

// preparing PID loops

// --- ascending loops ---

local PitThr_PID is pidloop(0.1, 0.001, 0.05, 0.2, 1).
local ApoThr_PID is pidloop(0.2, 0, 0.1, 0, 1).
//local Roll_PID is PID_init(0.1, 0, 0.05, -1, 1).
//local Yaw_PID is PID_init(0.1, 0, 0.05, -1, 1).

// --- END ascending loops ---

// final preparation

set startT to time:seconds.
set oldT to startT.
set prevPos to ship:geoposition.
set launchT to startT + 10.

lock throttle to max(0, min(1, tval)).
lock steering to steer.

wait 0.001. // waiting 1 physics tick

// ----------------================ Main loop starts here ================----------------

until runmode = 0 {
	
	// stuff that needs to update before every iteration
	set missionT to time:seconds.
	set dt to missionT - oldT.
	set curPos to ship:geoposition.
	set curAlt to ship:altitude.
	set velLat to (prevPos:lat - curPos:lat)/dt.
	set velLng to (prevPos:lng - curPos:lng)/dt.
	
	// runmodes
	
	if runmode = 1 // Engine ignition
	{
		if alt:radar > 200 {
			set runmode to 3.
		} else if missionT >= launchT - 2 {
			set tval to 1.
			stage.
			set runmode to 2.
		}
	}
	else if runmode = 2 // Liftoff!!
	{
		if missionT >= launchT {
			//sas on.
			stage.
			set runmode to 3.
		}
	}
	else if runmode = 3 // Initiating pitch-over
	{
		if curAlt >= 10000 {
			set runmode to 4.
		} else {
			//sas on.
			if airspeed >= 60 {
				set pitch to max(0, 90 * (1 - ship:altitude / 40000)).
			}
			set PitThr_PID:setpoint to 22 + (ship:altitude / 500).
			set tval to PitThr_PID:update(time:seconds, eta:apoapsis).
			if airspeed < 100 {
				set tval to 1.
			}
		}
	}
	else if runmode = 4 // initiating gravity turn
	{
		if curAlt >= 40000 or ship:apoapsis > (orbAlt * 0.75) {
			set runmode to 6.
		} else {
			set pitch to max(0, 90 * (1 - ship:altitude / 40000)).
			if tval <= 0.99 {
				set tval to tval + 0.01.
			} else {
				set tval to 1.
			}
		}
	}
	else if runmode = 5 // controling gravity turn
	{
		
	}
	else if runmode = 6 // continue until desired apoapsis and cut the throttle
	{
		set tval to 1.
		sas off.
		set pitch to 0.
		if ship:apoapsis >= orbAlt * 0.95 {
			set tval to 0.
			set runmode to 7.
		}
	}
	else if runmode = 7 // coast until above the atmosphere, warp if needed
	{
		if ship:apoapsis > atmHeight() and curAlt < atmHeight() {
			if not(warpmode = "PHYSICS") {
				set warpmode to "PHYSICS".
			}
			//if not(warp = 3) {
			//	set warp to 3.
			//}
		} else if ship:apoapsis < atmHeight() + 500 {
			if not(warp = 0) {
				set warp to 0.
			}
			set runmode to 6. // if ship apoapsis fals into the atmosphere, go back to runmode 6
		} else if curAlt >= atmHeight() {
			if not(warp = 0) {
				set warp to 0.
			}
			set runmode to 8.
		}
	}
	else if runmode = 8
	{
		if curAlt >= atmHeight() {
			if ship:apoapsis < orbAlt {
				set ApoThr_PID:setpoint to orbAlt + 10.
				set tval to ApoThr_PID:update(time:seconds, ship:apoapsis).
			} else {
				set tval to 0.
				set runmode to 9.
			}
		} else {
			set runmode to 7.
		}
	}
	else if runmode = 9
	{
		set mN to node(time:seconds + eta:apoapsis, 0, 0, getCircularizationDeltaV(ship:apoapsis)).
		add mN.
		set clearRequired to true.
		set runmode to 10.
	}
	else if runmode = 10
	{
		set runmode to 0.
	}
	
	// stuff that needs to update after every iteration
	if engineFlameout() {
		lock throttle to 0.
		wait 0.5.
		sas off.
		stage.
		wait 4.
		lock throttle to tval.
	}
	if clearRequired {
		clearscreen.
		set clearRequired to false.
	}
	displayFlightData().
	displayLaunchData().
	set steer to heading(compass, pitch).
	if curAlt >= 35000 {
		if not ag9 {
			toggle ag9.
		}
	}
	//if gravTurn {
	//	set ship:control:roll to PID_seek(Roll_PID, 0, roll_for(ship)).
	//	set ship:control:yaw to PID_seek(Yaw_PID, orbDir, compass_for(ship)).
	//}
	
	set oldT to missionT.
	set prevPos to curPos.
	wait 0.001. // waiting 1 physics tick
}

unlock all.





