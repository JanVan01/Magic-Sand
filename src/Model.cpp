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
            spreadFactor = firePotential[floor(location.x)][floor(location.y)];
            firePotential[floor(location.x)][floor(location.y)] = 0;
            burnedArea[floor(location.x)][floor(location.y)] = true;
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
    float treePotential = 5;
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

