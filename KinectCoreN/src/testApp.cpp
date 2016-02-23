#include "testApp.h"
#include "stdio.h"
#include "ofUtils.h"
//--------------------------------------------------------------
void testApp::setup()
{
    //ofSetDataPathRoot("data/");
	ckv = new ofxKCoreVision(debug);
	ckv->setup();

}

//--------------------------------------------------------------
void testApp::update(){
	
	ckv->update();

}

//--------------------------------------------------------------
void testApp::draw(){

	ckv->draw();

}

//--------------------------------------------------------------
void testApp::keyPressed  (int key){


}

//--------------------------------------------------------------
void testApp::keyReleased  (int key){}

//--------------------------------------------------------------
void testApp::mouseMoved(int x, int y ){}

//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button){}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button){}

//--------------------------------------------------------------
void testApp::mouseReleased(){}

/*****************************************************************************
 *	TOUCH EVENTS
 *****************************************************************************/
void testApp::TouchDown(Blob b){}

void testApp::TouchUp(Blob b){}

void testApp::TouchMoved( Blob b){}


