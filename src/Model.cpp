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

bool Model::setRandomVehicleLocation(ofRectangle area, bool liveInWater, ofVec2f & location){
    bool okwater = false;
    int count = 0;
    int maxCount = 100;
    while (!okwater && count < maxCount) {
        count++;
        float x = ofRandom(area.getLeft(),area.getRight());
        float y = ofRandom(area.getTop(),area.getBottom());
        bool insideWater = kinectProjector->elevationAtKinectCoord(x, y) < 0;
        if ((insideWater && liveInWater) || (!insideWater && !liveInWater)){
            location = ofVec2f(x, y);
            okwater = true;
        }
    }
    return okwater;
}

void Model::update(){
    // kinectROI updated
    if (kinectProjector->getKinectROI() != kinectROI){
        kinectROI = kinectProjector->getKinectROI();
        resetBurnedArea();
    }
    
    
    
    //spread fires
    int size = fires.size();
    for (int i = 0; i < size ; i++){
        Fire f = fires[i];
        ofPoint location = f.getLocation();
        if (burnedArea[location.x][location.y]){
            f.kill();
        } else {
            burnedArea[location.x][location.y] = true;
        }
        
        int rand = std::rand() % 100;
        if (f.isAlive() && rand < 10){
            int angle = f.getAngle();
            addNewFire(location, (angle + 90)%360);
            addNewFire(location, (angle + 270)%360);
        }
    }

    for (auto & f : fires){
        f.applyBehaviours();
        f.update();
    }
}

void Model::draw(){
    for (auto & f : fires){
        f.draw();
    }
}

void Model::clear(){
    fires.clear();
    resetBurnedArea();
}

void Model::resetBurnedArea(){
    burnedArea.clear();
    for(int x = kinectROI.getLeft(); x <= kinectROI.getRight(); x++ ){
        for (int y = kinectROI.getTop(); y <= kinectROI.getBottom(); y++ ){
            burnedArea[x][y] = false;
        }
    }
}

