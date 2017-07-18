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
    resetFirePotential();

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

void Model::update() {
	// kinectROI updated
	if (kinectProjector->getKinectROI() != kinectROI) {
		kinectROI = kinectProjector->getKinectROI();
		resetFirePotential();
	}

	//spread trees
	int size = fires.size();
	for (int i = 0; i < size; i++) {
		int spreadFactor = 0;
		ofPoint location = fires[i].getLocation();

		// kill fires where no firePotential is left
		if ((firePotential[floor(location.x)][floor(location.y)]) == 0) {
			fires[i].kill();
		}
		// set propability for new fires to firePotential 
		// st firePotential to zero afterwards
		else {
			spreadFactor = firePotential[floor(location.x)][floor(location.y)];
			firePotential[floor(location.x)][floor(location.y)] = 0;
		}

		int rand = std::rand() % 100;

		// set new fires with propability depending on spreadFactor 
		if (fires[i].isAlive() && rand <= spreadFactor) {
			int angle = fires[i].getAngle();
			addNewFire(location, (angle + 90) % 360);
			addNewFire(location, (angle + 270) % 360);
		}
	}

	deleteDeadFires();
	for (auto & f : fires) {
		if (f.isAlive()) {
			f.applyBehaviours(temperature, windspeed, winddirection);
			f.update();
		}
	}
}

void Model::draw() {
	for (auto & f : fires) {
		f.draw();
	}
}

void Model::clear() {
	fires.clear();
	resetFirePotential();
}

void Model::deleteDeadFires() {
	vector<int> deadFires;

	for (int i = 0; i < fires.size(); i++) {
		if (!fires[i].isAlive()) {
			deadFires.push_back(i);
		}
	}
	for (int i = 0; i < deadFires.size(); i++) {
		fires.erase(fires.begin() + (deadFires[i] - i));
	}
}

void Model::spreadTrees() {
	firePotential.clear(); 
	for (int x = 0; x <= kinectROI.getRight(); x++){
		vector<int> row;
		for (int y = 0; y <= kinectROI.getBottom(); y++) {
			// above a hight of 200 and under a hight of 0 the firePotential should be 5%
			// hight (200) good??
			if (elevationAtKinectCoord(currentLocation.x, currentLocation.y) > 200) {
				firePotential[floor(location.x)][floor(location.y)] = (5);
			}
			// if not, the firePotential should be 10% 
			else {
				row.push_back(10);
			}
		}
		firePotential.push_back(row);
	}
	// corresponding to the treePotential the firePotential is 15%, positions around tree have similar values
	treePotential = 5
	for (int x = 0; x <= kinectROI.getRight(); x++){
		vector<int> row;
		for (int y = 0; y <= kinectROI.getBottom(); y++) {
			if (treePotential < (std::rand() % 100)  ){
				firePotential[floor(location.x)][floor(location.y)] = 15;
				firePotential[floor(location.x)][floor(location.y + 1)] = 12;
				firePotential[floor(location.x)][floor(location.y - 1)] = 12;
				firePotential[floor(location.x + 1)][floor(location.y)] = 12;
				firePotential[floor(location.x + 1)][floor(location.y + 1)] = 12;
				firePotential[floor(location.x + 1)][floor(location.y - 1)] = 12;
				firePotential[floor(location.x - 1)][floor(location.y + 1)] = 12;
				firePotential[floor(location.x - 1)][floor(location.y - 1)] = 12;
				firePotential[floor(location.x - 1)][floor(location.y)] = 12;
			}
		}
		firePotential.push_back(row);
	}		
}



