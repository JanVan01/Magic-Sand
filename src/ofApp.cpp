/***********************************************************************
ofApp.cpp - main openframeworks app
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

#include "ofApp.h"

/**
 * @fn	void ofApp::setup()
 *
 * @brief	Setups for the Interface.
 *
 */

void ofApp::setup() {
	ofSetFrameRate(15);
	ofBackground(0);
	ofSetVerticalSync(true);
	ofSetLogLevel("ofThread", OF_LOG_WARNING);

	// Setup kinectProjector
	kinectProjector = std::make_shared<KinectProjector>(projWindow);
	kinectProjector->setup(true);
	
	// Setup sandSurfaceRenderer
	sandSurfaceRenderer = new SandSurfaceRenderer(kinectProjector, projWindow);
	sandSurfaceRenderer->setup(true);

    // Setup Model
    model = new Model(kinectProjector);

	// Retrieve variables
	ofVec2f projRes = ofVec2f(projWindow->getWidth(), projWindow->getHeight());
    kinectROI = kinectProjector->getKinectROI();

	//Interface FBO 
	fboInterface.allocate(projRes.x, projRes.y, GL_RGBA);
	fboInterface.begin();
	ofClear(0, 0, 0, 0);
	fboInterface.end();

	//Vehicles FBO
	fboVehicles.allocate(projRes.x, projRes.y, GL_RGBA);
	fboVehicles.begin();
	ofClear(0,0,0,0);
	fboVehicles.end();

	//RiskZone FBO
	fboRiskZone.allocate(projRes.x, projRes.y, GL_RGBA);
	fboRiskZone.begin();
	ofClear(0, 0, 0, 0);
	fboRiskZone.end();


	//Initialize interface parameters without slider movement
    runstate = false;
	firePos.set(kinectROI.width / 2, kinectROI.height / 2);
    windSpeed = 5;
    windDirection = 180;

	model->setWindSpeed(windSpeed);
	model->setWindDirection(windDirection);

	setupGui();
}

/**
 * @fn	void ofApp::update()
 *
 * @brief	Update call.
 *
 */

void ofApp::update() {
    // Call kinectProjector->update() first during the update function()
	kinectProjector->update();
    
	sandSurfaceRenderer->update();
	
	
	
    if (kinectProjector->isROIUpdated())
        kinectROI = kinectProjector->getKinectROI();

    if (!model->isRunning()) {
        gui->getButton("Start fire")->setLabel("Start fire");
        runstate = false;
    }

	if (kinectProjector->isImageStabilized()) {
		drawWindArrow();

        if(runstate){
			model->update();
			drawVehicles();
			setStatistics();
		}
	}
	gui->update();
}

/**
 * @fn	void ofApp::draw()
 *
 * @brief	Draws main window and gui.
 *
 */

void ofApp::draw() {
    drawMainWindow(300, 30, 600, 450);
	gui->draw();
}

void ofApp::drawMainWindow(float x, float y, float width, float height){
    sandSurfaceRenderer->drawMainWindow(x, y, width, height);
    kinectProjector->drawMainWindow(x, y, width, height);
	fboRiskZone.draw(x, y, width, height);
	fboVehicles.draw(x, y, width, height);
	fboInterface.draw(x, y, width, height);
}

void ofApp::drawProjWindow(ofEventArgs &args) {
	kinectProjector->drawProjectorWindow();
	
	if (!kinectProjector->isCalibrating()){
	    sandSurfaceRenderer->drawProjectorWindow();
		fboRiskZone.draw(0, 0);
	    fboVehicles.draw(0, 0);
		fboInterface.draw(0, 0);
	}
}

void ofApp::drawVehicles()
{
    fboVehicles.begin();
    model->draw();
    fboVehicles.end();
}

void ofApp::drawWindArrow()
{
	fboInterface.begin();   
	ofClear(0, 0, 0, 0);  
	ofVec2f projectorCoord = kinectProjector->kinectCoordToProjCoord(75, 125);
	ofTranslate(projectorCoord);
	ofRotate(windDirection);
	ofScale(windSpeed/5, 1, 1);
	ofFill();
    
	ofDrawArrow(ofVec3f(20, 2.5, 0), ofVec3f(50, 2.5, 0), 15.05);
	ofPath arrow;
	arrow.rectangle(ofPoint(0, 0), 50, 5);
    arrow.setFillColor(ofColor::white);
	arrow.setStrokeWidth(0);
	arrow.draw();

	ofNoFill();
	fboInterface.end();
}

void ofApp::drawPositioningTarget(ofVec2f firePos) 
{
	ofVec2f projectorCoord = kinectProjector->kinectCoordToProjCoord(firePos.x, firePos.y);
    
    fboVehicles.begin();
    ofClear(0, 0, 0, 0);
    ofPushMatrix();
    ofTranslate(projectorCoord.x, projectorCoord.y);

    float scale = 5;
    
    ofPath target;
    target.circle(0, 0, 4 * scale);
    target.circle(0, 0, 2 * scale);
    target.moveTo(0, 5 * scale);
    target.lineTo(0, -5 * scale);
    target.moveTo(5 * scale, 0);
    target.lineTo(-5 *scale, 0);
    
    target.setStrokeColor(ofColor::black);
    target.setStrokeWidth(2);
    target.setFilled(false);
    
    target.draw();
    
    ofPopMatrix();
	fboVehicles.end();
}

void ofApp::setStatistics() {
	// Set model information
	gui2->getValuePlotter("Fire intensity")->setValue(model->getNumberOfAgents());
	gui2->getLabel("Burned Area:")->setLabel(model->getPercentageOfBurnedArea());
	time = "Timestep: " + std::to_string(model->getTimestep()) + " steps";
	gui2->getLabel("Timestep: Model not running")->setLabel(time);
}

void ofApp::keyPressed(int key) {

}

void ofApp::keyReleased(int key) {

}

void ofApp::mouseMoved(int x, int y) {

}

void ofApp::mouseDragged(int x, int y, int button) {

}

void ofApp::mousePressed(int x, int y, int button) {

}

void ofApp::mouseReleased(int x, int y, int button) {

}

void ofApp::mouseEntered(int x, int y) {

}

void ofApp::mouseExited(int x, int y) {

}

void ofApp::windowResized(int w, int h) {

}

void ofApp::gotMessage(ofMessage msg) {

}

void ofApp::dragEvent(ofDragInfo dragInfo) {

}

/**
 * @fn	void ofApp::setupGui()
 *
 * @brief	Sets up the graphical user interface.
 *
 */

void ofApp::setupGui(){
	
	//Fire Simulation GUI : Simon
	gui = new ofxDatGui();
	gui->setTheme(new ofxDatGuiThemeAqua());
	gui->addToggle("Calculate Risk Zones");
	gui->add2dPad("Fire position", kinectROI);
	ofxDatGuiSlider* windSpeedSlider = gui->addSlider("Wind speed", 0, 10, windSpeed);
	windSpeedSlider->bind(windSpeed);
	ofxDatGuiSlider* windDirectionSlider = gui->addSlider("Wind direction", 0, 360, windDirection);
	windDirectionSlider->bind(windDirection);
	gui->addButton("Start fire");
	gui->addButton("Reset");
	gui->addHeader(":: Fire simulation ::", false);

	// once the gui has been assembled, register callbacks to listen for component specific events //
	gui->onButtonEvent(this, &ofApp::onButtonEvent);
	gui->on2dPadEvent(this, &ofApp::on2dPadEvent);
	gui->onSliderEvent(this, &ofApp::onSliderEvent);
	gui->onToggleEvent(this, &ofApp::onToggleEvent);
    gui->setLabelAlignment(ofxDatGuiAlignment::CENTER);
    gui->setPosition(ofxDatGuiAnchor::TOP_RIGHT);
	// Fire statistics GUI
	gui2 = new ofxDatGui();
	gui2->setTheme(new ofxDatGuiThemeAqua());
	gui2->addLabel("Timestep: Model not running");
	ofxDatGuiValuePlotter* areaBurnedPlot = gui2->addValuePlotter("Fire intensity", 0, 150);
	areaBurnedPlot->setValue(0);
	gui2->addLabel("Burned area:");
	gui2->addHeader(":: Fire statistics::", false);
	gui2->setPosition(ofxDatGuiAnchor::BOTTOM_RIGHT);
	
	gui->setAutoDraw(false); // troubles with multiple windows drawings on Windows
}

void ofApp::onButtonEvent(ofxDatGuiButtonEvent e) {
	if (e.target->is("Start fire")) {
		// Button functionality depending on State
		if (gui->getButton("Start fire")->getLabel() == "Start fire") {
			runstate = true;
			// Clear vehicles FBO of target arrow
			fboVehicles.begin();
			ofClear(0, 0, 0, 0);
			fboVehicles.end();

			// Start fire
			model->addNewFire(firePos);
			gui->getButton("Start fire")->setLabel("Pause");

			//Toggle Calc Risk Zones
			gui->getToggle("Calculate Risk Zones")->setChecked(!runstate);
			fboRiskZone.begin();
			ofClear(0, 0, 0, 0);
			fboRiskZone.end();
		}
		else if (gui->getButton("Start fire")->getLabel() == "Pause") {
			runstate = false;
			gui->getButton("Start fire")->setLabel("Resume");
			gui2->getLabel("Timestep: Model not running")->setLabel(time + " paused");
		}
		else if (gui->getButton("Start fire")->getLabel() == "Resume") {
			runstate = true;
			gui->getButton("Start fire")->setLabel("Pause");		
		}
	}

	if (e.target->is("Reset")) {
		model->clear();
		fboVehicles.begin();
		ofClear(0, 0, 0, 0);
		fboVehicles.end();
		gui->getButton("Start fire")->setLabel("Start fire");
		gui->get2dPad("Fire position")->reset();
		gui2->getLabel("Timestep: Model not running")->setLabel("Timestep: Model not running");
		gui2->getLabel("Burned area:")->setLabel("Burned area:");
		firePos.set(kinectROI.width / 2, kinectROI.height / 2);
		gui2->getValuePlotter("Fire intensity")->setValue(0);
		runstate = false;
		
	}
}

void ofApp::onToggleEvent(ofxDatGuiToggleEvent e) {
	if (e.target->is("Calculate Risk Zones")) {
		if (e.checked) {
			model->calculateRiskZones();
			fboRiskZone.begin();
			ofClear(0, 0, 0, 0);
			model->drawRiskZones();
			fboRiskZone.end();
		} else {
			fboRiskZone.begin();
			ofClear(0, 0, 0, 0);
			fboRiskZone.end();
		}
	}
}

void ofApp::on2dPadEvent(ofxDatGui2dPadEvent e) {
	if (e.target->is("Fire position")) {
		firePos.set(e.x, e.y);
		if (!model->isRunning()) {
			drawPositioningTarget(firePos);
		}
	}
}

void ofApp::onSliderEvent(ofxDatGuiSliderEvent e) {
	if (e.target->is("Wind speed")) {
		model->setWindSpeed(e.value);
	}

	if (e.target->is("Wind direction")) {
		model->setWindDirection(e.value);		
	}
}
