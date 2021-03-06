#pragma once

#include "ofMain.h"
#include "KinectProjector/KinectProjector.h"
#include "vehicle.h"


class Model{
public:
    Model(std::shared_ptr<KinectProjector> const& k);

	bool isRunning();

    void setWindSpeed(float v);
    void setWindDirection(float d);

    void addNewFire(ofVec2f fireSpawnPos);
    void addNewFire(ofVec2f fireSpawnPos, float angle);
    void addNewFireInRiskZone();

    void calculateRiskZones();
	void drawRiskZones();
	
	int getNumberOfAgents();
	int getTimestep();
	string getPercentageOfBurnedArea();

    void update();
    void draw();
    void clear();

private:
    std::shared_ptr<KinectProjector> kinectProjector;
    ofRectangle kinectROI;
    
    vector<Fire> fires;
    vector<Fire> embers;
	vector<ofVec2f> riskZones;
    vector< vector<bool> > burnedArea;
    
    float windSpeed;
    float windDirection;
    
    int timestep;

	int burnedAreaCounter;
	float completeArea;

	void resetBurnedArea();
    void drawEmbers();
};
