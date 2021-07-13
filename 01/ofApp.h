#pragma once

#include "ofMain.h"

#include "ofxKinect.h"
#include "ofxCv.h"
#include "ofxOsc.h"
#include "ofxGui.h"
#include "ofxTskokmtMath.h"
#include "ofxTskokmtTool.h"

class ofApp: public ofBaseApp {

public:
    void setup();
    void update();
    void draw();
    void exit();
    void keyPressed(int key);
    void keyReleased(int key);
    void mouseMoved(int x, int y );
    void mouseDragged(int x, int y, int button);
    void mousePressed(int x, int y, int button);
    void mouseReleased(int x, int y, int button);
    void mouseEntered(int x, int y);
    void mouseExited(int x, int y);
    void windowResized(int w, int h);
    void dragEvent(ofDragInfo dragInfo);
    void gotMessage(ofMessage msg);
		
private:
    //mode
    int mode = 0;
    
    //kinect
    ofxKinect kinect;
    int kinectIndex = 0;
    int kinectWidth, kinectHeight;
    ofParameter<bool> bRight;
    ofParameterGroup kinectGroup;
    
    //scope
    ofBoxPrimitive scope;
    ofParameter<int> scopeWidth, scopeHeight, scopeDepth, scopeZ;
    ofParameterGroup scopeGroup;
    
    //pointCloud
    ofMesh pointCloudZ, pointCloudX;
    vector<ofPoint> points;
    ofParameter<bool> bDrawPointCloud;
    ofParameter<float> pointCloudRotation;
    ofParameter<float> pointCloudScale;
    ofParameter<int> pointCloudDensityReferenceDistance;
    ofParameter<int> pointCloudSkip;
    ofParameterGroup pointCloudGroup;
    
    //hand
    vector<ofPoint> hands, handsRaw;
    ofParameter<int> minPointsNumForAHandSeed;
    int minPointsNumForAHand;
    ofParameter<float> minJoinDistance;
    ofParameterGroup handGroup;
    
    //osc
    ofxOscSender sender;
    ofxOscMessage message;
    ofParameter<string> host;
    ofParameterGroup oscGroup;
    
    //gui
    ofxPanel gui;
    
    //xml
    ofxXmlSettings xml;
    
    //math
    ofxTskokmtMath math;
    
    //tool
    ofxTskokmtTool tool;
};
