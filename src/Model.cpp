//
//  Model.cpp
//  Magic-Sand
//
//  Created by Jan on 06/06/17.
//
//

#include "Model.h"

Model::Model(std::shared_ptr<KinectProjector> const& k){
    kinectProjector = k;
    
    
    // Retrieve variables
    kinectROI = kinectProjector->getKinectROI();
    spreadTrees();

}

bool Model::isRunning() {
	return fires.size() > 0;
}

void Model::addNewFire(){
    ofVec2f location;
    setRandomVehicleLocation(kinectROI, false, location);
    auto f = Fire(kinectProjector, location, kinectROI);
    f.setup();
    fires.push_back(f);
}

void Model::addNewFire(ofVec2f fireSpawnPos) {
    addNewFire(fireSpawnPos, 0);
}

void Model::addNewFire(ofVec2f fireSpawnPos, float angle){
    if (kinectProjector->elevationAtKinectCoord(fireSpawnPos.x, fireSpawnPos.y) < 0){
        return;
    }
    auto f = Fire(kinectProjector, fireSpawnPos, kinectROI, angle);
    f.setup();
    fires.push_back(f);
}

bool Model::setRandomVehicleLocation(ofRectangle area, bool liveInWater, ofVec2f & location) {
	bool okwater = false;
	int count = 0;
	int maxCount = 100;
	while (!okwater && count < maxCount) {
		count++;
		float x = ofRandom(area.getLeft(), area.getRight());
		float y = ofRandom(area.getTop(), area.getBottom());
		bool insideWater = kinectProjector->elevationAtKinectCoord(x, y) < 0;
		if ((insideWater && liveInWater) || (!insideWater && !liveInWater)) {
			location = ofVec2f(x, y);
			okwater = true;
		}
	}
	return okwater;
}

// SETTERS and GETTERS Fire Parameters:
void Model::setWindspeed(float uiwindspeed) {
	windspeed = uiwindspeed;
}

void Model::setTemp(float uiTemp) {
	temperature = uiTemp;
}

void Model::setWinddirection(float uiWinddirection) {
	winddirection = uiWinddirection;
}

void Model::update(){
    // kinectROI updated
    if (kinectProjector->getKinectROI() != kinectROI){
        kinectROI = kinectProjector->getKinectROI();
        spreadTrees();
    }
    
    //spread fires
    int size = fires.size();
    int i = 0;
    while(i < size){
        embers.push_back(fires[i]);
        ofPoint location = fires[i].getLocation();
        
        int rand = std::rand() % 100;
        if (firePotential[floor(location.x)][floor(location.y)] == 0 || !fires[i].isAlive()){
            // kill fires where no firePotential is left
            fires.erase(fires.begin() + i);
            size--;
        } else {
            // set propability for new fires to firePotential
            // set firePotential to zero afterwards
            int spreadFactor = firePotential[floor(location.x)][floor(location.y)];
            firePotential[floor(location.x)][floor(location.y)] = 0;
            int rand = std::rand() % 100;
            if (fires[i].isAlive() && rand < spreadFactor){
                int angle = fires[i].getAngle();
                addNewFire(location, (angle + 90)%360);
                addNewFire(location, (angle + 270)%360);
            }
            i++;
        }
    }
    
    for (auto & f : fires){
        f.applyBehaviours(temperature,windspeed,winddirection);
        f.update();
    }
}

void Model::draw(){
    drawEmbers();
    for (auto & f : fires){
        f.draw();
    }
}

void Model::clear(){
    fires.clear();
	embers.clear();
    spreadTrees();
}


void Model::calculateRiskZones() {
	for (int x = 0; x <= kinectROI.getRight(); x++) {
		vector<bool> row;
		for (int y = 0; y <= kinectROI.getBottom(); y++) {
			if (x == 0 || x == kinectROI.getRight() || y == 0 || y == kinectROI.getBottom()) {
				row.push_back(false);
				continue;
			}
			float cell_aspect;
			float cell_slope;
			//assignment of the neighborhood
			float a = kinectProjector->elevationAtKinectCoord(x - 1, y - 1);
			float b = kinectProjector->elevationAtKinectCoord(x, y - 1);
			float c = kinectProjector->elevationAtKinectCoord(x + 1, y - 1);
			float d = kinectProjector->elevationAtKinectCoord(x - 1, y);
			float e = kinectProjector->elevationAtKinectCoord(x, y);
			float f = kinectProjector->elevationAtKinectCoord(x + 1, y);
			float g = kinectProjector->elevationAtKinectCoord(x - 1, y + 1);
			float h = kinectProjector->elevationAtKinectCoord(x, y + 1);
			float i = kinectProjector->elevationAtKinectCoord(x + 1, y + 1);
			float changeRateInXDirection = ((c + 2 * f + i) - (a + 2 * d + g)) / 8;
			float changeRateInYDirection = ((g + 2 * h + i) - (a + 2 * b + c)) / 8;
			//calculation of south aspects
			float aspect = 180 / PI * atan2(changeRateInYDirection, -changeRateInXDirection);
			if (aspect < 0) {
				cell_aspect = 90.0 - aspect;
			}
			else if (aspect > 90.0) {
				cell_aspect = 360.0 - aspect + 90.0;
			}
			else {
				cell_aspect = 90.0 - aspect;
			}
			//calculation of slopes >= 10 degrees
			cell_slope = atan(sqrt(pow(changeRateInXDirection, 2) + pow(changeRateInYDirection, 2))) * (180 / PI);

			//identification of risk zones
			if (cell_aspect >= 157.5 && cell_aspect <= 202.5 && cell_slope >= 10) {
				row.push_back(true);
			}
			else {
				row.push_back(false);
			}
		}
		riskZones.push_back(row);
	}
}

void Model::drawEmbers(){
    int i = 0;
    int size = embers.size();
    while(i < size){
        if(embers[i].isAlive()){
            embers[i].kill();
        }
        embers[i].draw();
        if(embers[i].getIntensity() <= 0){
            embers.erase(embers.begin() + i);
            size--;
        } else {
            i++;
        }
    }
}

void Model::spreadTrees() {
    firePotential.clear();
    for (int x = 0; x <= kinectROI.getRight(); x++){
        vector<int> row;
        for (int y = 0; y <= kinectROI.getBottom(); y++) {
            if (kinectProjector->elevationAtKinectCoord(x, y) > 200) {
                // if the height is above 200 the firepotential is lower
                row.push_back(5);
            } else {
                row.push_back(10);
            }
        }
        firePotential.push_back(row);
    }

    // corresponding to the treePotential the firePotential is 15%, positions around tree have similar values
    int treePotential = 2;
    for (int x = 0; x < firePotential.size(); x++){
        for (int y = 0; y < firePotential[x].size(); y++) {
            if (x == 0 || y == 0 || x == firePotential.size()-1 || y == firePotential[x].size()-1){
                continue;
            }
            if (treePotential >= (std::rand() % 100)){
                firePotential[x][y] = 15;
                firePotential[x][y + 1] = 12;
                firePotential[x][y - 1] = 12;
                firePotential[x + 1][y] = 12;
                firePotential[x + 1][y + 1] = 12;
                firePotential[x + 1][y - 1] = 12;
                firePotential[x - 1][y + 1] = 12;
                firePotential[x - 1][y - 1] = 12;
                firePotential[x - 1][y] = 12;
            }
        }
    }		
}

void Model::drawRiskZones() {
	for (int x = 0; x < riskZones.size(); x++) {
		vector<bool> row;
		for (int y = 0; y < riskZones[x].size(); y++) {
			if (riskZones[x][y]) {
				ofPushMatrix();
				ofColor color = ofColor(255, 0, 0, 200);
				ofPoint coord = kinectProjector->kinectCoordToProjCoord(x, y);
				ofFill();

				ofPath riskZone;
				riskZone.rectangle(coord.x - 2, coord.y - 2, 4, 4);
				riskZone.setFillColor(color);
				riskZone.setStrokeWidth(0);
				riskZone.draw();

				ofNoFill();

				// restore the pushed state
				ofPopMatrix();
			}
			else {
				//Keine Gefahr Zeichne Grün an Position XY
			}
		}
	}
}
void Model::drawTrees() {
	for (int x = 0; x < firePotential.size(); x++) {
		for (int y = 0; y < firePotential[x].size(); y++) {
			if (firePotential[x][y] == 15) {
				ofColor color = ofColor(0, 153, 0, 200);
				ofPoint coord = kinectProjector->kinectCoordToProjCoord(x, y);
				ofFill();

				ofPath treeZone;
				treeZone.circle(coord.x, coord.y, 4);
				treeZone.setFillColor(color);
				treeZone.setStrokeWidth(0);
				treeZone.draw();

				ofNoFill();
			}
		}
	}
}
