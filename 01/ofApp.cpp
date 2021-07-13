#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup() {
    
    ofSetFrameRate(60);
    ofSetVerticalSync(true);
    ofSetWindowPosition(ofGetScreenWidth() / 18., ofGetScreenHeight() / 18.);
    ofSetWindowShape(ofGetScreenWidth() / 18. * 16, ofGetScreenHeight() / 18. * 16);
    ofBackground(0);
    ofSetCircleResolution(32);
    glPointSize(2);
    
    //load kinectIndex
    xml.clear();
    xml.load("kinectIndexSaveData.xml");
    kinectIndex = xml.getValue("kinectIndex", 0);
    
    //setup kinect
    kinect.init();
    mode = kinect.open(kinectIndex);
    kinect.setRegistration(true);
    kinectWidth = kinect.width, kinectHeight = kinect.height;
    
    //setup pointCloud
    pointCloudZ.setMode(OF_PRIMITIVE_POINTS);
    pointCloudX.setMode(OF_PRIMITIVE_POINTS);
    
    //setup osc
    sender.setup("localhost", 50100);
    
    //setup gui
    kinectGroup.add(bRight.set("bRight", true));
    
    scopeGroup.setName("scope");
    scopeGroup.add(scopeZ.set("z", 0, -2000, 2000));
    scopeGroup.add(scopeWidth.set("width", 0, 0, 4000));
    scopeGroup.add(scopeHeight.set("height", 0, 0, 4000));
    scopeGroup.add(scopeDepth.set("depth", 10, 1, 200));
    
    pointCloudGroup.setName("pointCloud");
    pointCloudGroup.add(bDrawPointCloud.set("bDraw", true));
    pointCloudGroup.add(pointCloudRotation.set("rotation", 0, 0, 90));
    pointCloudGroup.add(pointCloudScale.set("scale", 1, .1, 1));
    pointCloudGroup.add(pointCloudDensityReferenceDistance.set("densityReferenceDistance", 10000, 0, 10000));
    pointCloudGroup.add(pointCloudSkip.set("skip", 1, 1, 100));
    
    handGroup.setName("hand");
    handGroup.add(minPointsNumForAHandSeed.set("minPointsNumForAHandSeed", 500, 1, 5000));
    handGroup.add(minJoinDistance.set("minJoinDistance", 250, 0, 500));
    
    oscGroup.setName("osc");
    oscGroup.add(host.set("host", "192.168."));
    
    gui.setup();
    gui.add(kinectGroup);
    gui.add(scopeGroup);
    gui.add(pointCloudGroup);
    gui.add(handGroup);
    gui.add(oscGroup);
    gui.loadFromFile("guiSaveData.xml");
    
    //load windowRect
    tool.loadWindowRect();
}

//--------------------------------------------------------------
void ofApp::update() {
    
    ofSetWindowTitle("touch | " + ofToString(round(ofGetFrameRate())));
    
    if (mode == 0) return;
    
    //update osc
    string hostString = host;
    if (hostString != sender.getHost()) sender.setup(hostString, 50100);
    
    //update kinect
    kinect.update();
    ofPoint depthes[kinectHeight][kinectWidth];
    for (int y = 0; y < kinectHeight; y++) {
        for (int x = 0; x < kinectWidth; x++) {
            depthes[y][x] = kinect.getWorldCoordinateAt(x, y);
        }
    }
    
    //set scope
    scope.setPosition(scopeWidth * ofMap(bRight, 0, 1, 0, -1) + scopeWidth / 2., -scopeHeight + scopeHeight / 2., scopeZ + scopeDepth / 2.);
    scope.set(scopeWidth, scopeHeight, scopeDepth);
    
    //set points
    points.clear();
    for (int y = 0; y < kinectHeight; y++) {
        for (int x = 0; x < kinectWidth; x++) {
            if ((int)ofRandom(pointCloudSkip) == 0 && (int)ofRandom(pow(pointCloudDensityReferenceDistance / kinect.getDistanceAt(x, y), 2)) == 0) {
                ofPoint point = math.rotatedPoint(depthes[y][x].z, -depthes[y][x].x, pointCloudRotation * (bRight ? 1 : -1) + (bRight ? 180 : 0));
                point.z = depthes[y][x].y;
                if (math.isInBox(point, scope)) points.push_back(point);
            }
        }
    }
    
    //set pointCloud
    if (bDrawPointCloud) {
        pointCloudZ.clear();
        pointCloudX.clear();
        for (int y = 0; y < kinectHeight; y++) {
            for (int x = 0; x < kinectWidth; x++) {
                if ((int)ofRandom(pointCloudSkip) == 0 && (int)ofRandom(pow(pointCloudDensityReferenceDistance / kinect.getDistanceAt(x, y), 2)) == 0) {
                    ofPoint point = math.rotatedPoint(depthes[y][x].z, -depthes[y][x].x, pointCloudRotation * (bRight ? 1 : -1) + (bRight ? 180 : 0));
                    point.z = depthes[y][x].y;
                    pointCloudX.addVertex(ofPoint(point.z * (bRight ? -1 : 1), point.y, 0));
                    pointCloudZ.addVertex(ofPoint(point.x, point.y, 0));
                    if (math.isInBox(point, scope)) {
                        pointCloudZ.addColor(ofFloatColor(0, 1, 0));
                        pointCloudX.addColor(ofFloatColor(0, 1, 0));
                    } else {
                        pointCloudZ.addColor(ofFloatColor(1, 0, 0));
                        pointCloudX.addColor(ofFloatColor(1, 0, 0));
                    }
                }
            }
        }
    }
    
    //nearJoin
    if (points.size() >= 2) {
        vector<vector<ofPoint>> handGroups;
        handGroups.push_back(vector<ofPoint>());
        handGroups.back().push_back(points.front());
        for (int i = 1; i < points.size(); i++) {
            bool bNear;
            for (int j = 0; j < handGroups.size(); j++) {
                bNear = false;
                for (int k = 0; k < handGroups[j].size(); k++) {
                    if (ofDist(points[i], handGroups[j][k]) <= minJoinDistance) {
                        bNear = true;
                        break;
                    }
                }
                if (bNear) {
                    handGroups[j].push_back(points[i]);
                    break;
                }
            }
            if (!bNear) {
                handGroups.push_back(vector<ofPoint>());
                handGroups.back().push_back(points[i]);
            }
        }
        handsRaw.clear();
        minPointsNumForAHand = minPointsNumForAHandSeed / pointCloudSkip / pow(pointCloudDensityReferenceDistance / 1000., 2);
        for (int i = 0; i < handGroups.size(); i++) {
            if (handGroups[i].size() >= minPointsNumForAHand) handsRaw.push_back(math.mean(handGroups[i]));
        }
    } else {
        handsRaw = points;
    }
    hands = handsRaw;
    
    //enplane & translate hands
    for (int i = 0; i < hands.size(); i++) {
        hands[i].z = 0;
        hands[i] -= ofPoint(scope.getX() - scope.getWidth() / 2., scope.getY() - scope.getHeight() / 2.);
    }
    
    //send hands
    message.clear();
    if (bRight) message.setAddress("handR");
    else message.setAddress("handL");
    message.addIntArg(hands.size());
    for (int i = 0; i < hands.size(); i++) {
        message.addFloatArg(hands[i].x);
        message.addFloatArg(hands[i].y);
    }
    sender.sendMessage(message);
}

//--------------------------------------------------------------
void ofApp::draw() {
    
    float radius = sqrt(scopeWidth * scopeHeight) / 90.;
    
    if (mode == 0) {
        //draw alart
        ofSetColor(255, 0, 0);
        ofDrawRectangle(ofGetWindowRect());
    } else {
        ofPushMatrix();
        ofTranslate(ofGetWidth() * bRight, ofGetHeight());
        ofScale(pointCloudScale);
        
        //draw pointCloudZ
        if (bDrawPointCloud) {
            ofSetColor(255);
            pointCloudZ.draw();
        }
        
        //draw scope
        ofPushStyle();
        ofNoFill();
        ofSetColor(255);
        ofDrawRectangle(scope.getX() - scope.getWidth() / 2., scope.getY() - scope.getHeight() / 2., scope.getWidth(), scope.getHeight());
        ofPopStyle();
        
        //draw points
        if (bDrawPointCloud) {
            ofSetColor(127);
            for (int i = 0; i < points.size(); i++) {
                ofDrawCircle(points[i].x, points[i].y, radius);
                ofPushStyle();
                ofNoFill();
                ofDrawCircle(points[i].x, points[i].y, minJoinDistance);
                ofPopStyle();
            }
        }
        
        //draw handsRaw
        for (int i = 0; i < handsRaw.size(); i++) {
            ofColor color;
            color.setHsb((int)ofMap(i, 0, 6, 0, 256) % 256, 255, 255);
            ofSetColor(color);
            ofDrawCircle(handsRaw[i].x, handsRaw[i].y, radius);
        }
        
        ofPopMatrix();
        
        ofPushMatrix();
        ofTranslate(ofGetWidth() * ofMap(bRight, 0, 1, .75, .25), ofGetHeight());
        ofScale(pointCloudScale);
        
        //draw pointCloudX
        if (bDrawPointCloud) {
            ofSetColor(255);
            pointCloudX.draw();
        }
        
        ofPushMatrix();
        ofScale(ofMap(bRight, 0, 1, 1, -1), 1);
        
        //draw scope
        ofPushStyle();
        ofNoFill();
        ofSetColor(255);
        ofDrawRectangle(scope.getZ() - scope.getDepth() / 2., scope.getY() - scope.getHeight() / 2., scope.getDepth(), scope.getHeight());
        ofPopStyle();
        
        //draw points
        if (bDrawPointCloud) {
            ofSetColor(127);
            for (int i = 0; i < points.size(); i++) {
                ofDrawCircle(points[i].z, points[i].y, radius);
                ofPushStyle();
                ofNoFill();
                ofDrawCircle(points[i].z, points[i].y, minJoinDistance);
                ofPopStyle();
            }
        }
        
        //draw handsRaw
        for (int i = 0; i < handsRaw.size(); i++) {
            ofColor color;
            color.setHsb((int)ofMap(i, 0, 6, 0, 256) % 256, 255, 255);
            ofSetColor(color);
            ofDrawCircle(handsRaw[i].z, handsRaw[i].y, radius);
        }
        
        ofPopMatrix();
        
        ofPopMatrix();
        
        //draw ui
        gui.draw();
    }
}

//--------------------------------------------------------------
void ofApp::exit() {
    
    //save kinectIndex
    xml.clear();
    xml.setValue("kinectIndex", mode);
    xml.save("kinectIndexSaveData.xml");
    
    //save gui
    gui.saveToFile("guiSaveData.xml");
    
    //save windowRect
    tool.saveWindowRect();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key) {
    
    //switch kinect
    if (key == ' ') {
        if (mode == 0) {
            kinectIndex = 0;
        } else {
            kinectIndex++;
            if (kinectIndex >= kinect.numTotalDevices()) kinectIndex = 0;
        }
        kinect.close();
        kinect.init();
        mode = kinect.open(kinectIndex);
        kinect.setRegistration(true);
        kinectWidth = kinect.width, kinectHeight = kinect.height;
    }
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key) {
    
}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ) {
    
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button) {
    
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button) {
    
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button) {
    
}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y) {
    
}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y) {
    
}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h) {
    
}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg) {
    
}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo) {
    
}
