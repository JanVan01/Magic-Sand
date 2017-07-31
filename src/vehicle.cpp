/***********************************************************************
vehicle.cpp - vehicle class (fish & rabbits moving in the sandbox)
Copyright (c) 2016 Thomas Wolf

This file is part of the Magic Sand.

The Magic Sand is free software; you can redistribute it
and/or modify it under the terms of the GNU General Public License as
published by the Free Software Foundation; either version 2 of the
License, or (at your option) any later version.

The Magic Sand is distributed in the hope that it will be
useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
General Public License for more details.

You should have received a copy of the GNU General Public License along
with the Augmented Reality Sandbox; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
***********************************************************************/

#include "vehicle.h"


Vehicle::Vehicle(std::shared_ptr<KinectProjector> const& k, ofPoint slocation, ofRectangle sborders, float sangle) {
    kinectProjector = k;
    location = slocation;
    borders = sborders;
    angle = sangle;
    globalVelocityChange.set(0, 0);
    velocity.set(0.0, 0.0);
    wandertheta = 0;
}

void Vehicle::updateBeachDetection(){
    // Find sandbox gradients and elevations in the next 10 steps of vehicle v, update vehicle variables
    ofPoint futureLocation;
    futureLocation = location;
    beachSlope = ofVec2f(0);
    beach = false;
    int i = 1;
    while (i < 10 && !beach)
    {
        bool water = kinectProjector->elevationAtKinectCoord(futureLocation.x, futureLocation.y) < 0;
        if (water)
        {
            beach = true;
            beachDist = i;
            beachSlope = kinectProjector->gradientAtKinectCoord(futureLocation.x,futureLocation.y);
        }
        futureLocation += velocity; // Go to next future location step
        i++;
    }
}

ofPoint Vehicle::bordersEffect(){
    ofPoint desired, futureLocation;
    
    // Predict location 10 (arbitrary choice) frames ahead
    futureLocation = location + velocity*10;
    
    ofPoint target = location;
    if (!internalBorders.inside(futureLocation)){ // Go to the opposite direction
        border = true;
        if (futureLocation.x < internalBorders.getLeft())
            target.x = borders.getRight();
        if (futureLocation.y < internalBorders.getTop())
            target.y = borders.getBottom();
        if (futureLocation.x > internalBorders.getRight())
            target.x = borders.getLeft();
        if (futureLocation.y > internalBorders.getBottom())
            target.y = borders.getTop();
    } else {
        border = false;
    }
    
    //desired = target - location;
    //desired.normalize();
    //desired *= topSpeed;
    
    ofPoint velocityChange(0);
    //velocityChange = desired - velocity;
    //velocityChange.limit(maxVelocityChange);
    return velocityChange;
}

ofPoint Vehicle::wanderEffect(){
    
    ofPoint velocityChange, desired;

    wandertheta += ofRandom(-change,change);     // Randomly change wander theta
    
    ofPoint front = velocity;
    front.normalize();
    front *= wanderD;
    ofPoint circleloc = location + front;
    
	float h = front.angle(ofVec2f(1,0)); // We need to know the heading to offset wandertheta
    
    ofPoint circleOffSet = ofPoint(wanderR*cos(wandertheta+h),wanderR*sin(wandertheta+h));
    ofPoint target = circleloc + circleOffSet;
    
    desired = target - location;
    desired.normalize();
    desired *= topSpeed;
    
    velocityChange = desired - velocity;
    velocityChange.limit(maxVelocityChange);
    return velocityChange;
}

// Effect to change the speed of the agent depending on topography
ofPoint Vehicle::hillEffect() {
	ofPoint velocityChange, futureLocation, currentLocation;
	float futureelevation, currentelevation;

	currentLocation = location;
	futureLocation = location + velocity * 10;
	currentelevation = kinectProjector->elevationAtKinectCoord(currentLocation.x, currentLocation.y);
	futureelevation = kinectProjector->elevationAtKinectCoord(futureLocation.x, futureLocation.y);


	float currDir = ofDegToRad(angle);
	ofPoint front = ofVec2f(cos(currDir), sin(currDir));
	front.normalize();

	if (currentelevation > futureelevation) {
		// Inverse Direction when moving downhill,
		front *= -1;
		// limits the direction change in one step, so the fire does not change the direction immediately
		ofPoint dirChange = front.limit(maxVelocityChange);
		return dirChange;
	}
	if (currentelevation < futureelevation) {
		// Increase Speed when moving uphill
		front *= 3;
		return front;
	}
	if (currentelevation == futureelevation) {
		// no effect when moving on a plane
		return ofPoint(0);
	}
}

// Calculates a vector for the effect of the wind.
ofPoint Vehicle::windEffect(float windspeed, float winddirection) {
	ofPoint velocityChange;
	int windForce;
	// Sets factor for the wind speed
	if (windspeed > 1) {
		windForce = 1.25;
	} else if (windspeed > 4) {
		windForce = 1.5;
	} else if (windspeed > 6) {
		windForce = 1.75;
	} else if (windspeed > 8) {
		windForce = 2;
	} else {
		windForce = 0;
	}

	// Calculates the direction change
	float windDir = ofDegToRad(winddirection);
	ofPoint desired = ofVec2f(cos(windDir), sin(windDir));
	// Normalisation of the vector so that it only influences the direction
	desired.normalize();
	desired *= windForce;

	return desired.limit(maxVelocityChange);
}


ofPoint Vehicle::slopesEffect(){
    ofPoint desired, velocityChange;
    
    desired = beachSlope;
    desired.normalize();
    desired *= topSpeed;
    if(beach){
        desired /= beachDist; // The closest the beach is, the more we want to avoid it
    }
    velocityChange = desired - velocity;
    velocityChange.limit(maxVelocityChange);

    return velocityChange;
}

void Vehicle::applyVelocityChange(const ofPoint & velocityChange){
    globalVelocityChange += velocityChange;
}
// Movement: vehicle update rotation ...
void Vehicle::update(){
    projectorCoord = kinectProjector->kinectCoordToProjCoord(location.x, location.y);
    velocity += globalVelocityChange;
    velocity.limit(topSpeed);
    location += velocity;
    globalVelocityChange *= 0;

    float desiredAngle = ofRadToDeg(atan2(velocity.y,velocity.x));
    float angleChange = desiredAngle - angle;
    angleChange += (angleChange > 180) ? -360 : (angleChange < -180) ? 360 : 0; // To take into account that the difference between -180 and 180 is 0 and not 360
    angleChange *= velocity.length();
    angleChange /= topSpeed;
    angleChange = max(min(angleChange, maxRotation), -maxRotation);
    angle += angleChange;
}

//==============================================================
// Derived class Fire
//==============================================================

void Fire::setup(){
    minborderDist = 50;
    internalBorders = borders;
    internalBorders.scaleFromCenter((borders.width-minborderDist)/borders.width, (borders.height-minborderDist)/borders.height);
    // Randomness parameters
    wanderR = 50;         // Radius for our "wander circle"
    wanderD = 0;         // Distance for our "wander circle"
    change = 1;
    
    r = 12;
    maxVelocityChange = 1;
    maxRotation = 360;
    topSpeed = 1;
    velocityIncreaseStep = 2;
    minVelocity = velocityIncreaseStep;
    
    intensity = 3;
	alive = true;
}

// Effect introduce some randomness in the direction changes
ofPoint Fire::wanderEffect(){
    
    ofPoint velocityChange, desired;
	// Randomly change wander theta
    wandertheta = ofRandom(-change,change);     
    
    float currDir = ofDegToRad(angle);
    ofPoint front = ofVec2f(cos(currDir), sin(currDir));
    
    front.normalize();
    front *= wanderD;
    ofPoint circleloc = location + front;
    
    ofPoint circleOffSet = ofPoint(wanderR*cos(wandertheta+currDir),wanderR*sin(wandertheta+currDir));
    ofPoint target = circleloc + circleOffSet;
    
    desired = target - location;
    desired.normalize();
    desired *= topSpeed;
    
    velocityChange = desired;// - velocity;
    velocityChange.limit(maxVelocityChange);
    
    return velocityChange;
}


void Fire::applyBehaviours() {

}
// Function that applies the different forces that effect the agent
void Fire::applyBehaviours(float windspeed, float winddirection) {
    updateBeachDetection();
    // Function calls for the different effects
	windF = windEffect(windspeed, winddirection);
    bordersF = bordersEffect();
    slopesF = slopesEffect();
    wanderF = wanderEffect();
	hillF = hillEffect();
    
    ofPoint littleSlopeF = slopesF;
    
	// Balancing of the different effects
    bordersF *=0.5;
    slopesF *= 2;
    wanderF *= 1;
    littleSlopeF *= 1;
	hillF *= 3;
	windF *= 0.6;
    
    float currDir = ofDegToRad(angle);
    ofPoint oldDir = ofVec2f(cos(currDir), sin(currDir));
    oldDir.scale(velocityIncreaseStep);
	ofPoint newDir;
	newDir += wanderF;
	newDir += hillF;
	newDir += windF;

    
	if (beach)
        oldDir.scale(velocityIncreaseStep/beachDist);

	if (!beach && !border)
		{   
		applyVelocityChange(newDir);
		applyVelocityChange(oldDir); // Just accelerate
		} else { // Wee need to decelerate and then change direction
		    if (velocity.lengthSquared() > minVelocity*minVelocity) // We are not stopped yet
		    {
		        applyVelocityChange(-oldDir); // Just deccelerate
				applyVelocityChange(-newDir);
		    }  else {
				// Stops the Fire agent
				velocity = ofPoint(0);
				alive = false;
			}
		}
}

void Fire::draw(){
    if(!alive){
        intensity--;
    }
    // saves the current coordinate system
    ofPushMatrix();
    ofTranslate(projectorCoord);
    ofRotate(angle);
    ofColor color = getFlameColor();
    
    float sc = 2;
    
    ofFill();
    
    ofPath flame;
    flame.arc(0,-3*sc,3*sc,3*sc,90,270);
    flame.arc(0,-5*sc,sc,sc,90,270);
    flame.arc(0,-2*sc,2*sc,2*sc,270,90);
    flame.setFillColor(color);
    flame.setStrokeWidth(0);
    flame.draw();
    
    ofNoFill();
    
    // restore the pushed state
    ofPopMatrix();
}

void Fire::kill(){
    alive = false;
}

ofColor Fire::getFlameColor(){
    float intensityFactor = intensity <= 0 ? 0 : intensity*0.33;
    int red = 255 * intensityFactor;
    int green = 64 * intensityFactor;
    int blue = 0 * intensityFactor;
    return ofColor(red, green, blue);
}
