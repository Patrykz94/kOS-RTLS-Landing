@lazyglobal off.
clearscreen.

function has_file { // function that checks if a file exists on the hard drive
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

if not has_file("lib_navball.ks") { // downloading needed function libraries
	copy lib_navball.ks from 0.
}
if not has_file("telemetry.ks") {
	copy telemetry.ks from 0.
}
if not has_file("flight_display.ks") {
	copy flight_display.ks from 0.
}

run lib_navball. // running libraries
run telemetry.
run flight_display.

local compass is 0.
local pitch is 90.
local runmode is 1.
local tval is 0.
local clearRequired is false.
local velLat is 0. // change in latitude since last physics tick
local velLng is 0. // change in longitude...
local dt is 0. // delta time (time between physics ticks)
local steerPitch is 0. // pitch offset
local steerYaw is 0. // yaw offset
local steer is up. // steering direction
local curAlt is 0. // current altitude (basically same as ship:altitude)
local missionT is 0. // current time
local launchT is 0. // time until/since launch
local startT is 0. // time when the program started
local oldT is 0. // time of last physics tick
local prevPos is 0. // geoposition of last physics tick
local curPos is 0. // current geoposition
local landReq is 1. // landing position. 0 if landing is not required
local lqdFuel is core:part:parent:resources[0]. // selecting liquid fuel in stage 1 tank
local fins is 0. // gridfins
local finsDeployed is false. // state of gridfins
local impactTime is 0. // time untill crossing the landing altitude
local impactPos is 0. // landing position but before accounting for planet rotation
local landingPos is 0. // landing position
local latOffset is 0. // something to do with steering
local landingSite is 0. // geoposition of landing site
local landingAltitude is 0. // sea level altitude of the landing site
local coreName is core:part:tag. // name of kOS cpu
local stageApo is 0. // not important anymore, its the apoapsis at the moment of stage separation
local desPitch is 10. // pitch during boostback
local posMod1 is 0.15. // landing position modifier during boostback (something to offset the aero drag)
local posMod2 is 0.115. // same as above but for reentry burn
local externalTanks is ship:partstagged("ext"). // external fuel tank. This is the one attached to launch clamp
local falconTanks is ship:partstagged("falcon-tank"). // stage 1 fuel tank(s)
local transl is 0. // liqd fuel transfer
local transo is 0. // oxidizer transfer
local MECO is 0. // Main Engine Cutoff (how much fuel needs to be left for boostback and landing)

// list of potential landing sites
local KSCLaunchPad is latlng(-0.0972080884740584, -74.5576970966038).
local KSCVABLandingPad1 is latlng(-0.0968010588104255, -74.6174316020627).
local KSCVABLandingPad2 is latlng(-0.0967649062924497, -74.6200692295964).
local KSCAstrComp is latlng(-0.0925723018712286, -74.6631085805855).
if landReq = 0 { // setting the landing site
	set landingSite to 0.
} else if landReq = 1 {
	set landingSite to KSCLaunchPad.
} else if landReq = 2 {
	set landingSite to KSCVABLandingPad1.
} else if landReq = 3 {
	set landingSite to KSCVABLandingPad2.
} else if landReq = 4 {
	set landingSite to KSCAstrComp.
}
if landingSite = 0 {
	set landingAltitude to 0. // if no landing site i.e. landing not required. the landing altitude is set to 0.
} else {
	set landingAltitude to landingSite:terrainheight + 7. // terrain altitude at landing pos + offset(because ksp measures altitude from center of mass)
}

if landingSite = 0 {
	set MECO to 0.
} else {
	lock MECO to 500 + (groundspeed * 1.5). // amount of fuel needed for boostback and landing
}
when lqdFuel:amount <= MECO then { set lqdFuel:enabled to false. } // cutting liquid fuel when MECO reached (this cause launch script to stage)
when alt:radar < 40 and runmode > 2 then { ag10 on. } // deploying landing legs (they need to be set to ag10)

// preparing PID loops
// --- landing loops ---

// Altitude control
local AltVel_PID is pidloop(0.2, 0, 0.04, -200, 200). // calculating desired velocity during landing
local VelThr_PID is pidloop(0.05, 0.01, 0.005, 0, 1). // adjusting thrust to reach that velocity

// Latitude control
local LatVel_PID is pidloop(6, 0, 40, -0.15, 0.15). // same as above but for steering during landing
local VelPit1_PID is pidloop(700, 0, 200, -5, 5).
local VelPit2_PID is pidloop(700, 0, 200, -15, 15).

// Longditude control
local LngVel_PID is pidloop(6, 0, 40, -0.15, 0.15).
local VelYaw1_PID is pidloop(500, 0, 200, -5, 5).
local VelYaw2_PID is pidloop(500, 0, 200, -15, 15).

// --- END landing loops ---

// final preparation - setting up for launch

set startT to time:seconds.
set oldT to startT.
set prevPos to ship:geoposition.

wait 0.001. // waiting 1 physics tick to avoid errors

// ----------------================ Magic. Do not touch! ================----------------

until runmode = 0 {
	
	// stuff that needs to update before every iteration
	set missionT to time:seconds.
	set dt to missionT - oldT.
	set curPos to ship:geoposition.
	set curAlt to ship:altitude.
	set velLat to (prevPos:lat - curPos:lat)/dt.
	set velLng to (prevPos:lng - curPos:lng)/dt.
	if runmode >= 2 {
		set impactTime to timeToAltitude(landingAltitude). // time untin impact
		set impactPos to body:geopositionof(positionat(ship, time:seconds + impactTime)). // position of impact
		set landingPos to latlng(impactPos:lat, impactPos:lng - impactTime * 0.01666666). // adding planet rotation
	}
	
	// The main code
	if runmode = 1 // keep checking if stage 1 separated
	{
		// displaying data about fuel
		print "Amount of fuel left: " + round(lqdFuel:amount, 2) + "          " at (3,20).
		print "Fuel cut-off amount: " + round(MECO, 2) + "          " at (3,21).
		
		// during launch, engines start 2 seconds before liftoff,
		// this bit of code keeps transfering fuel from the external tank,
		// to keep stage 1 full untill liftoff. I had issues with fuel lines.
		if coreName = "Falcon" and externalTanks:length > 0 {
			if ship:status = "prelaunch" or ship:status = "landed" {
				set transl to transferall("liquidfuel", externalTanks, falconTanks).
				set transo to transferall("oxidizer", externalTanks, falconTanks).
				set transl:active to true.
				set transo:active to true.
			} else {
				set transl:active to false.
				set transo:active to false.
			}
		}
		
		if lqdFuel:enabled = false { // chceck if liquid fuel has been cut, indicating staging
			set stageApo to ship:apoapsis.
			if stageApo < 20000 { set desPitch to 35. }
			if stageApo < 70000 and groundspeed > 1000 { set desPitch to 15. } // changing boostback pitch depending on speed/distance
			wait 1. // waiting 1 second to make sure that ship has completed staging
			set lqdFuel:enabled to true. // enabling fuel again
			rcs on.
			sas off.
			set ship:control:fore to -1. // reversing the booster from seconds stage so it wont get damaged by exhaust
			wait 3.
			set ship:control:fore to 0.
			set runmode to 2.
			set clearRequired to true. // clearing screen
		}
	}
	else if runmode = 2
	{
		set ship:control:pitch to 1. // turning booster around
		wait 5.
		set ship:control:pitch to 0.
		wait 4.
		ship:partstagged("S1E")[0]:getmodule("MultiModeEngine"):doevent("toggle mode"). // switching engine mode to center only
		set runmode to 3.
	}
	else if runmode = 3
	{
		if 	pitch_for(ship) <= (desPitch + 2) // making sure we point in the right direction before boostback
			and pitch_for(ship) >= (desPitch - 2)
			and compass_for(ship) <= (landingSite:heading + latOffset + 5)
			and compass_for(ship) >= (landingSite:heading + latOffset - 5)
		{
			if abs(abs(landingPos:lng + posMod1) - abs(landingSite:lng)) >= 0.5 { // changing thrust depending on how far the landing position is from landing site (will need to change it to PID loop)
				set tval to 1.
			} else if abs(abs(landingPos:lng + posMod1) - abs(landingSite:lng)) < 0.05 {
				set tval to 0.05.
			} else {
				set tval to 0.2.
			}
			
			if landingPos:lng < longitude {
				if landingPos:lat > landingSite:lat { // offsetting steering slightly
					set latOffset to -0.5.
				} else {
					set latOffset to 0.5.
				}
			} else {
				set latOffset to 0.
			}
		}
		set steer to heading(landingSite:heading + latOffset, desPitch). // steering towards the target
		if (landingPos:lng + posMod1) <= landingSite:lng {
			set tval to 0.
			set latOffset to 0.
			unlock steering.
			set runmode to 3.5.
		}
	}
	else if runmode = 3.5 {
		set ship:control:pitch to 1. // turning around after boostback
		wait 4.
		set ship:control:pitch to 0.
		wait 4.
		set runmode to 4.
	}
	else if runmode = 4
	{
		if curAlt <= 45000 { // reentry burn when under 45k (it really just finetunes the landing position)
			if abs(abs(landingPos:lng + posMod2) - abs(landingSite:lng)) >= 0.05 {
				set tval to 0.75.
			} else {
				set tval to 0.3.
			}
			
			if (landingPos:lng + posMod2) >= landingSite:lng {
				set tval to 0.
				set runmode to 5.
				rcs off.
			}
			
			if landingPos:lat > landingSite:lat {
				set latOffset to 0.5.
			} else {
				set latOffset to -0.5.
			}
		}
		set steer to ship:srfretrograde + r(latOffset,0,0).
	}
	else if runmode = 5
	{
		set steer to ship:srfretrograde + r(0,0,0).
		if not finsDeployed and curAlt <= 45000 { // deploying gridfins
			set fins to ship:partsnamed("gridfin").
			for fin in fins {
				if fin:getmodule("ModuleAeroSurface"):getfield("deploy") = "False" {
					fin:getmodule("ModuleAeroSurface"):doaction("extend", true).
				}
			}
			set finsDeployed to true.
		}
		if curAlt < 25000 {
			set runmode to 6.
		}
	}
	else if runmode = 6 // once below 25k, fine tune landing position
	{
		set LatVel_PID:setpoint to landingSite:lat. // steering
		set VelPit1_PID:setpoint to LatVel_PID:update(missionT, landingPos:lat).
		set steerPitch to VelPit1_PID:update(missionT, velLat).
		
		set LngVel_PID:setpoint to landingSite:lng.
		if curAlt < 5000 { // if below 5k, use actual longditude instead of the expected one
			rcs on.
			set VelYaw1_PID:setpoint to LngVel_PID:update(missionT, longitude).
		} else {
			set VelYaw1_PID:setpoint to LngVel_PID:update(missionT, landingPos:lng + posMod2).
		}
		set steerYaw to VelYaw1_PID:update(missionT, velLng).
		
		set steer to ship:srfretrograde + r(steerPitch, steerYaw, 0).
		if (curAlt <= 3000) or ((longitude - 0.03) <= landingSite:lng) // if below 3k or almost directly over landing pos, it's time to slow down
		{
			set runmode to 7.
		}
	}
	else if runmode = 7 // Landing
	{
		set AltVel_PID:setpoint to landingAltitude. // landing burn
		set VelThr_PID:setpoint to AltVel_PID:update(missionT, ship:altitude).
		set tval to VelThr_PID:update(missionT, verticalspeed).
		
		set LatVel_PID:setpoint to landingSite:lat.
		set VelPit2_PID:setpoint to LatVel_PID:update(missionT, latitude).
		set steerPitch to -VelPit2_PID:update(missionT, velLat).
		
		set LngVel_PID:setpoint to landingSite:lng.
		set VelYaw2_PID:setpoint to LngVel_PID:update(missionT, longitude).
		set steerYaw to -VelYaw2_PID:update(missionT, velLng).
		
		if tval < 0.5 and alt:radar > 750 { // if engines are on low thrust, aerodynamics will be stronger so reverse steering
			set steerPitch to -steerPitch.
			set steerYaw to -steerYaw.
		}
		
		set steer to up + r(steerPitch, steerYaw, 90).
		if ship:status = "Landed" { // checking if we have landed already
			set runmode to 0.
			set tval to 0.
			set steer to up.
		}
	}
	
	// stuff that needs to update after every iteration
	if clearRequired {
		clearscreen.
		set clearRequired to false.
	}
	displayFlightData().
	if runmode >= 3 and runmode <> 3.5 {
		lock throttle to max(0, min(1, tval)).
		lock steering to steer.
	}
	
	if runmode >= 3 {
		print "Current Position:     " + round(longitude, 5) + "          " at (3,23).
		print "Landing Position (r): " + round(landingPos:lng, 5) + "          " at (3,24).
		print "Steer Pitch:          " + round(steerPitch, 2) + "     " at (3, 26).
		print "Steer Yaw:            " + round(steerYaw, 2) + "     " at (3, 27).
		print "Impact Time:          " + round(impactTime, 2) + "     " at (3, 28).
	}
	
	// setting variables to new values
	set oldT to missionT.
	set prevPos to curPos.
	wait 0.001. // waiting 1 physics tick
}

unlock all. // we are done!