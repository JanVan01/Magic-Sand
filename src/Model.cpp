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
    int treePotential = 5;
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

