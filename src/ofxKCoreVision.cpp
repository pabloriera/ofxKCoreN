/*
*  ofxKCoreVision.cpp
*  based on ofxNCoreVision.cpp
*  NUI Group Community Core Vision
*
*  Created by NUI Group Dev Team A on 2/1/09.
*  Copyright 2009 NUI Group. All rights reserved.
*
*/
 
#include "ofxKCoreVision.h"
#include "Controls/gui.h"
#include <fstream>

using namespace std;

/******************************************************************************
* The setup function is run once to perform initializations in the application
*****************************************************************************/
void ofxKCoreVision::_setup(ofEventArgs &e){
	threshold       = 80;
	nearThreshold   = 550;
	farThreshold    = 650;

	//set the title
	ofSetWindowTitle("Kinect Vision based on CCV v2.1");

	//create filter
	if(filter == NULL)
		filter = new ProcessFilters();

	//Load Settings from config.xml file
	loadXMLSettings();

	if(debugMode){
		printf("DEBUG MODE : Printing to File\n");
		/*****************************************************************************************************
		* LOGGING
		******************************************************************************************************/
		/* alright first we need to get time and date so our logs can be ordered */
		time ( &rawtime );
		timeinfo = localtime ( &rawtime );
		strftime (fileName,80,"../logs/log_%B_%d_%y_%H_%M_%S.txt",timeinfo);
		FILE *stream;
		sprintf(fileName, ofToDataPath(fileName).c_str());
		if((stream = freopen(fileName, "w", stdout)) == NULL){}
		/******************************************************************************************************/
	}


	//  Setup Window Properties
    //
	ofSetWindowShape(winWidth,winHeight);
	ofSetVerticalSync(false);	            //Set vertical sync to false for better performance?

	//  Init Kinect Sensor Device
    //
	//  save/update log file
	if(debugMode) if( (stream = freopen(fileName, "a", stdout) ) == NULL){}
    cameraInited = false;

    ifstream numKinectsfile;
    string str;

    numKinectsfile.open("numKinects.txt");
    if (numKinectsfile.is_open())
        numKinectsfile >> str >> numOfKinects >> str >> kN_X >>str>> kN_Y;
        else {numOfKinects=1;kN_X=1;kN_Y=1;}

    numKinectsfile.close();

    cout << "Number of Kinects " <<  numOfKinects << endl;
    cout << "nx grid " <<  kN_X << endl;
    cout << "ny grid " <<  kN_Y << endl;

    /*numOfKinects =  1;
    kN_X = 1;
    kN_Y = 1;*/

    sceneWidth            = 640;
    sceneHeight           = 480;


    srcPoints[0] = dstPoints[0] = ofPoint(0,0); //SET DIMENSION
    srcPoints[1] = dstPoints[1] = ofPoint(sceneWidth,0); //SET DIMENSION
    srcPoints[2] = dstPoints[2] = ofPoint(sceneWidth,sceneHeight); //SET DIMENSION
    srcPoints[3] = dstPoints[3] = ofPoint(0,sceneHeight); //SET DIMENSION

    extranearThreshold = 2000;

    kinect = new ofxKinect[numOfKinects];

    for(int kN = 0 ; kN<numOfKinects; kN++){

        //kinect[kN].setRegistration(true);
        if ( kinect[kN].init(false,false,false) ){

/*           switch (kN){
                case 0:
                    cameraInited    =   kinect[kN].open("A00366A14864045A");

                case 1:

                    cameraInited    =   kinect[kN].open("B00362717909103B");
                }
*/

                cameraInited    =   kinect[kN].open();

                camWidth		=	kinect[kN].getWidth();
                camHeight	    =	kinect[kN].getHeight();
                int camRate     =   30;

                ofSetFrameRate(camRate * 1.3);			//  This will be based on camera fps in the future


                int t = kinect[kN].getDeviceId();

                cout<<"Kinect Id "<<kN<<" getDeviceId "<<t<<endl;

//                string s = kinect[kN].getSerial();

 //               cout<<"Kinect Serial Num "<<kN <<" getSerial "<<s.c_str()<<endl;

                printf("Kinect %d Initialised...\n",kN);
        }

    }

    kinectsWidth                = camWidth *kN_X;
    kinectsHeight               = camHeight*kN_Y;

    cout<<"SCc WH "<<camWidth<< "  "<<camHeight<<endl;

	//  Allocate images (needed for drawing/processing images)
    //

    /***  1 Kinects 640 x 480      **/

/*
    sourceImg.allocate(camWidth, camHeight);    //  Source Image
	sourceImg.setUseTexture(false);				//  We don't need to draw this so don't create a texture
	processedImg.allocate(camWidth, camHeight); //  main Image that'll be processed.
	processedImg.setUseTexture(false);			//  We don't need to draw this so don't create a texture

*/
    /***  N Kinects 640 x 480  **/

    sourceImg.allocate(kinectsWidth, kinectsHeight);    //  Source Image
	sourceImg.setUseTexture(false);				//  We don't need to draw this so don't create a texture
	processedImg.allocate(kinectsWidth, kinectsHeight); //  main Image that'll be processed.
	processedImg.setUseTexture(false);			//  We don't need to draw this so don't create a texture

	//  GUI Stuff
    //
	verdana.loadFont("verdana.ttf", 8, true, true);
	background.loadImage("images/background.jpg");
	controls = ofxGui::Instance(this);
	setupControls();

	//  Setup Calibration
    //
	calib.setup(kinectsWidth, kinectsHeight, &tracker);
	//calib.setup(camWidth, camHeight, &tracker);

	//  Allocate Filters
    //
	filter->allocate(kinectsWidth,kinectsHeight);

	/*****************************************************************************************************
	* Startup Modes
	******************************************************************************************************/

	//If Standalone Mode (not an addon)
	if (bStandaloneMode){
		printf("Starting in standalone mode...\n\n");
		showConfiguration = true;
	}
	if (bMiniMode) {
		showConfiguration = true;
		bShowInterface = false;
		printf("Starting in Mini Mode...\n\n");
		ofSetWindowShape(190, 200); //minimized size
		filter->bMiniMode = bMiniMode;
	} else {
		bShowInterface = true;
		printf("Starting in full mode...\n\n");
	}

	//If Object tracking activated
	if(contourFinder.bTrackObjects) {
		templates.loadTemplateXml();
	}

	contourFinder.setTemplateUtils(&templates);

	printf("Kinect 2 Community Core Vision is setup!\n\n");
}

/****************************************************************
*	Load/Save config.xml file Settings
****************************************************************/
bool ofxKCoreVision::loadXMLSettings(){
	message = "Loading config.xml...";

    ofxXmlSettings		XML;
    if ( XML.loadFile("config.xml") ){
        if ( XML.pushTag("CONFIG") ){

            winWidth					= XML.getValue("WINDOW:WIDTH", 950);
            winHeight					= XML.getValue("WINDOW:HEIGHT", 600);

            nearThreshold				= XML.getValue("KINECT:NEAR",600);
            farThreshold				= XML.getValue("KINECT:FAR",700);

            XML.pushTag("WARP");
            for(int i = 0; i < 4; i++){
                XML.pushTag("POINT",i);
                srcPoints[i].x = XML.getValue("X", 0.0);
                srcPoints[i].y = XML.getValue("Y", 0.0);
                XML.popTag();
            }
            XML.popTag();

            maxBlobs					= XML.getValue("BLOBS:MAXNUMBER", 20);

            backgroundLearnRate			= XML.getValue("INT:BGLEARNRATE", 0.01f);

            bMiniMode                   = XML.getValue("BOOLEAN:MINIMODE",0);

            bShowLabels					= XML.getValue("BOOLEAN:LABELS",0);
            bDrawOutlines				= XML.getValue("BOOLEAN:OUTLINES",0);

            //  Pre-Filter
            //
            filter->bLearnBakground		= XML.getValue("BOOLEAN:LEARNBG",0);
            filter->bVerticalMirror		= XML.getValue("BOOLEAN:VMIRROR",0);
            filter->bHorizontalMirror	= XML.getValue("BOOLEAN:HMIRROR",0);

            //  Filters
            //
            filter->bTrackDark			= XML.getValue("BOOLEAN:TRACKDARK", 0);
            filter->bHighpass			= XML.getValue("BOOLEAN:HIGHPASS",1);
            filter->bAmplify			= XML.getValue("BOOLEAN:AMPLIFY", 1);
            filter->bSmooth				= XML.getValue("BOOLEAN:SMOOTH", 1);
            filter->bDynamicBG			= XML.getValue("BOOLEAN:DYNAMICBG", 1);

            //  Filter Settings
            //
            filter->threshold			= XML.getValue("INT:THRESHOLD",0);
            filter->highpassBlur		= XML.getValue("INT:HIGHPASSBLUR",0);
            filter->highpassNoise		= XML.getValue("INT:HIGHPASSNOISE",0);
            filter->highpassAmp			= XML.getValue("INT:HIGHPASSAMP",0);
            filter->smooth				= XML.getValue("INT:SMOOTH",0);

            //  CounterFinder
            //
            minTempArea					= XML.getValue("INT:MINTEMPAREA",0);
            maxTempArea					= XML.getValue("INT:MAXTEMPAREA",0);
            MIN_BLOB_SIZE				= XML.getValue("INT:MINBLOBSIZE",2);
            MAX_BLOB_SIZE				= XML.getValue("INT:MAXBLOBSIZE",100);
            hullPress					= XML.getValue("INT:HULLPRESS",20.0);
            tracker.MOVEMENT_FILTERING	= XML.getValue("INT:MINMOVEMENT",0);

            //  Tracking Options
            //
            contourFinder.bTrackBlobs	= XML.getValue("BOOLEAN:TRACKBLOBS",0);
            contourFinder.bTrackFingers	= XML.getValue("BOOLEAN:TRACKFINGERS",0);
            contourFinder.bTrackObjects	= XML.getValue("BOOLEAN:TRACKOBJECTS",0);

            //  NETWORK SETTINGS
            //
            bTUIOMode					= XML.getValue("BOOLEAN:TUIO",0);
            myTUIO.bOSCMode				= XML.getValue("BOOLEAN:OSCMODE",1);
            myTUIO.bTCPMode				= XML.getValue("BOOLEAN:TCPMODE",1);
            myTUIO.bBinaryMode			= XML.getValue("BOOLEAN:BINMODE",1);
            myTUIO.bHeightWidth			= XML.getValue("BOOLEAN:HEIGHTWIDTH",0);
            tmpLocalHost				= XML.getValue("NETWORK:LOCALHOST", "localhost");
            tmpPort						= XML.getValue("NETWORK:TUIOPORT_OUT", 3333);
            tmpFlashPort				= XML.getValue("NETWORK:TUIOFLASHPORT_OUT", 3000);

            XML.popTag();

            myTUIO.setup(tmpLocalHost.c_str(), tmpPort, tmpFlashPort); //have to convert tmpLocalHost to a const char*
            message = "Settings Loaded!\n\n";
            return true;
        } else {
            message = "The settings file was empty!\n\n";
            return false;
        }

	} else {
		message = "No Settings Found...\n\n"; //FAIL
        return false;
    }
}

bool ofxKCoreVision::saveXMLSettings(){
    ofxXmlSettings		XML;

    XML.loadFile("config.xml");

	XML.setValue("CONFIG:KINECT:NEAR", nearThreshold);
	XML.setValue("CONFIG:KINECT:FAR", farThreshold);

	XML.setValue("CONFIG:BOOLEAN:PRESSURE",bShowPressure);
	XML.setValue("CONFIG:BOOLEAN:LABELS",bShowLabels);
	XML.setValue("CONFIG:BOOLEAN:OUTLINES",bDrawOutlines);
	XML.setValue("CONFIG:BOOLEAN:LEARNBG", filter->bLearnBakground);
	XML.setValue("CONFIG:BOOLEAN:VMIRROR", filter->bVerticalMirror);
	XML.setValue("CONFIG:BOOLEAN:HMIRROR", filter->bHorizontalMirror);
	XML.setValue("CONFIG:BOOLEAN:TRACKDARK", filter->bTrackDark);
	XML.setValue("CONFIG:BOOLEAN:HIGHPASS", filter->bHighpass);
	XML.setValue("CONFIG:BOOLEAN:AMPLIFY", filter->bAmplify);
	XML.setValue("CONFIG:BOOLEAN:SMOOTH", filter->bSmooth);
	XML.setValue("CONFIG:BOOLEAN:DYNAMICBG", filter->bDynamicBG);
	XML.setValue("CONFIG:INT:MINMOVEMENT", tracker.MOVEMENT_FILTERING);
	XML.setValue("CONFIG:INT:MINBLOBSIZE", MIN_BLOB_SIZE);
	XML.setValue("CONFIG:INT:MAXBLOBSIZE", MAX_BLOB_SIZE);
	XML.setValue("CONFIG:INT:BGLEARNRATE", backgroundLearnRate);
	XML.setValue("CONFIG:INT:THRESHOLD", filter->threshold);
	XML.setValue("CONFIG:INT:HIGHPASSBLUR", filter->highpassBlur);
	XML.setValue("CONFIG:INT:HIGHPASSNOISE", filter->highpassNoise);
	XML.setValue("CONFIG:INT:HIGHPASSAMP", filter->highpassAmp);
	XML.setValue("CONFIG:INT:SMOOTH", filter->smooth);
	XML.setValue("CONFIG:INT:MINTEMPAREA", minTempArea);
	XML.setValue("CONFIG:INT:MAXTEMPAREA", maxTempArea);
	XML.setValue("CONFIG:INT:HULLPRESS", hullPress);
	XML.setValue("CONFIG:BOOLEAN:MINIMODE", bMiniMode);
	XML.setValue("CONFIG:BOOLEAN:TUIO",bTUIOMode);
	XML.setValue("CONFIG:BOOLEAN:TRACKBLOBS",contourFinder.bTrackBlobs);
	XML.setValue("CONFIG:BOOLEAN:TRACKFINGERS",contourFinder.bTrackFingers);
	XML.setValue("CONFIG:BOOLEAN:TRACKOBJECTS",contourFinder.bTrackObjects);
	XML.setValue("CONFIG:BOOLEAN:HEIGHTWIDTH", myTUIO.bHeightWidth);
	XML.setValue("CONFIG:BOOLEAN:OSCMODE", myTUIO.bOSCMode);
	XML.setValue("CONFIG:BOOLEAN:TCPMODE", myTUIO.bTCPMode);
	XML.setValue("CONFIG:BOOLEAN:BINMODE", myTUIO.bBinaryMode);

    XML.pushTag("CONFIG");
    XML.pushTag("WARP");
    for(int i = 0 ; i < 4; i++){
        XML.setValue("POINT:X", srcPoints[i].x,i);
        XML.setValue("POINT:Y", srcPoints[i].y,i);
    }
    XML.popTag();
    XML.popTag();

    return XML.saveFile("config.xml");
}

/******************************************************************************
* The update function runs continuously. Use it to update states and variables
*****************************************************************************/
void ofxKCoreVision::_update(ofEventArgs &e){
	if(debugMode) if((stream = freopen(fileName, "a", stdout)) == NULL){}

    //  Dragable Warp points
    //
    if ( !bCalibration && !bMiniMode && !bBigMode && bShowInterface && ofGetMousePressed() ){
        ofPoint mouse = ofPoint(ofGetMouseX(),ofGetMouseY());
        mouse -= ofPoint(30, 15);   //  Translate
        mouse *= 2.0;               //  Scale

        if ( pointSelected == -1 ){
            for(int i = 0; i < 4; i++){
                if ( srcPoints[i].distance(mouse) < 20){
                    srcPoints[i].x = mouse.x;
                    srcPoints[i].y = mouse.y;
                    pointSelected = i;
                    break;
                }
            }
        } else {
            if (mouse.x < 0)
                mouse.x = 0;

            if (mouse.x > sceneWidth)
                mouse.x = sceneWidth;

            if (mouse.y < 0)
                mouse.y = 0;

            if (mouse.y > sceneHeight)
                mouse.y = sceneHeight;

            srcPoints[pointSelected].x = mouse.x;
            srcPoints[pointSelected].y = mouse.y;
        }
    } else {
        pointSelected = -1;
    }
    for(int kN=0; kN<numOfKinects;kN++)
        kinect[kN].update();

	bNewFrame = true;
	if(!bNewFrame){
		return;			//if no new frame, return
	} else {			//else process camera frame
		ofBackground(0, 0, 0);

		// Calculate FPS of Camera
        //
		frames++;
		float time = ofGetElapsedTimeMillis();
		if (time > (lastFPSlog + 1000)) {
			fps = frames;
			frames = 0;
			lastFPSlog = time;
		}
        // End calculation

		float beforeTime = ofGetElapsedTimeMillis();

        //  Get Pixels from the Kinect
        //
        frameNew_bool = false;

        for(int kN=0; kN<numOfKinects;kN++)
            frameNew_bool = frameNew_bool || kinect[kN].isFrameNew();

        if(frameNew_bool) {

            unsigned char * depthPixels = sourceImg.getPixels();

          //  cout << sourceImg.getWidth() << " " << sourceImg.getHeight() << endl;

            int numPixels = camWidth * camHeight;

            const float* depthRaw;

            for(int kN=0; kN<numOfKinects; kN++){

                depthRaw = kinect[kN].getDistancePixels();

                for(int i = 0; i < numPixels; i++)
                {
                    int i_x = i % camWidth;
                    int i_y = i / camWidth;
                    int x = i + (i_y)*camWidth*(kN_X-1) + (kN%kN_X)*camWidth + (kN/kN_X)*kN_X*numPixels;

                   // cout << i_x<<" "<< i_y << " "<< kN<< " " <<i<< " "<<x << endl;

                    if((depthRaw[i] <= farThreshold) && (depthRaw[i] >= nearThreshold))
                        depthPixels[x] = ofMap(depthRaw[i], nearThreshold, farThreshold, 255,0);

                    else if ((depthRaw[i] < nearThreshold) && (depthRaw[i] > extranearThreshold))
                        depthPixels[x] = 255;
                    else if (depthRaw[i] > farThreshold)
                        depthPixels[x] = 0;
                    else if (depthRaw[i] < extranearThreshold)
                        depthPixels[x] = 0;
                }

            }

            sourceImg.flagImageChanged();
        }

        processedImg = sourceImg;

        //  Filter and Process
        //
/*
        cout << sourceImg.getWidth() << " " << sourceImg.getHeight() << endl;
        cout <<  processedImg.getWidth() << " " <<  processedImg.getHeight()  << endl;

        cout << srcPoints[0].x << " " << srcPoints[0].y << endl;
        cout << srcPoints[1].x << " " << srcPoints[1].y << endl;
        cout << srcPoints[2].x << " " << srcPoints[2].y << endl;
        cout << srcPoints[3].x << " " << srcPoints[3].y << endl;


        cout << dstPoints[0].x << " " << dstPoints[0].y << endl;
        cout << dstPoints[1].x << " " << dstPoints[1].y << endl;
        cout << dstPoints[2].x << " " << dstPoints[2].y << endl;
        cout << dstPoints[3].x << " " << dstPoints[3].y << endl;
*/

        filter->applyFilters( processedImg, srcPoints, dstPoints );


        contourFinder.findContours(processedImg,  (MIN_BLOB_SIZE * 2) + 1, ((camWidth * camHeight) * .4) * (MAX_BLOB_SIZE * .001), maxBlobs, (double) hullPress, false);

     //   if(contourFinder.bTrackBlobs)
       //     for (int i=0; i<contourFinder.nBlobs; i++)
      //          cout << contourFinder.blobs[i].second.centroid.x << " " << contourFinder.blobs[i].second.centroid.y << endl;
		//  If Object tracking or Finger tracking is enabled
		//
        if( contourFinder.bTrackBlobs || contourFinder.bTrackFingers || contourFinder.bTrackObjects){
			tracker.track(&contourFinder);
        }

        //  Get DSP time
        //
        differenceTime = ofGetElapsedTimeMillis() - beforeTime;

        //Dynamic Background subtraction LearRate
        //
        if (filter->bDynamicBG){
            filter->fLearnRate = backgroundLearnRate * .0001; //If there are no blobs, add the background faster.

            if ((contourFinder.nBlobs > 0 )|| (contourFinder.nFingers > 0 ) ) //If there ARE blobs, add the background slower.
				filter->fLearnRate = backgroundLearnRate * .0001;
        }

		//Sending TUIO messages
        //
		if (myTUIO.bOSCMode || myTUIO.bTCPMode || myTUIO.bBinaryMode){
			myTUIO.setMode(contourFinder.bTrackBlobs, contourFinder.bTrackFingers , contourFinder.bTrackObjects);
			myTUIO.sendTUIO(tracker.getTrackedBlobsPtr(), tracker.getTrackedFingersPtr(), tracker.getTrackedObjectsPtr() );
		}
	}
}

void ofxKCoreVision::_draw(ofEventArgs &e){
	if (showConfiguration){

        //  Check the mode
        //
		if (bCalibration){
			//  Don't draw main interface
			calib.passInContourFinder(contourFinder.nBlobs, contourFinder.blobs);
			calib.doCalibration();
        } else if (bBigMode){
            drawBigMode();
		} else if (bMiniMode){
			drawMiniMode();
		} else if (bShowInterface){
			drawFullMode();

			if(bDrawOutlines || bShowLabels)
				drawOutlines();

			if(contourFinder.bTrackObjects && isSelecting){
				ofNoFill();
				ofSetColor(255, 0, 0);
				ofRect(rect.x,rect.y,rect.width,rect.height);
				ofSetColor(0, 255, 0);
				ofRect(minRect.x,minRect.y,minRect.width, minRect.height);
				ofSetColor(0, 0, 255);
				ofRect(maxRect.x, maxRect.y, maxRect.width, maxRect.height);
			}
		}

		//  Draw gui controls
        //
		if (!bCalibration && !bMiniMode && !bBigMode)
			controls->draw();
	}
}

void ofxKCoreVision::drawFullMode(){
	ofSetColor(255);

	//  Draw Background Image
    //
	background.draw(0,0);

    //  Draw Images
    //
    filter->draw();

    //  Draw Warp Area
    //
    ofPushStyle();
    ofPushMatrix();
    ofTranslate(30, 15);
    ofScale(0.5, 0.5);
    for(int i = 0; i < 4; i++){
        ofSetColor(255,0,0,100);

        if (i == pointSelected)
            ofFill();
        else
            ofNoFill();

        ofRect(srcPoints[i].x-7, srcPoints[i].y-7,14,14);
        ofLine(srcPoints[i], srcPoints[(i+1)%4]);
    }
    ofPopMatrix();
    ofPopStyle();

    //  Draw Info panel
    //
	string str0 = "FPS: ";
	str0+= ofToString(fps, 0)+"\n";
	string str1 = "Resolution: ";
	str1+= ofToString(camWidth, 0) + "x" + ofToString(camHeight, 0)  + "\n";
	string str2 = "Blobs: ";
	str2+= ofToString(contourFinder.nBlobs,0)+", "+ofToString(contourFinder.nObjects,0)+"\n";
	string str3 = "Fingers: ";
	str3+= ofToString(contourFinder.nFingers,0)+"\n";
	ofSetHexColor(0x969696);
	verdana.drawString( str0 + str1 + str2 + str3, 570, 430);

	//  Draw TUIO data
    //
    char buf[256]="";
    if(myTUIO.bOSCMode && myTUIO.bTCPMode)
        sprintf(buf, "Dual Mode");
    else if(myTUIO.bOSCMode)
        sprintf(buf, "Host: %s\nProtocol: UDP [OSC]\nPort: %i", myTUIO.localHost, myTUIO.TUIOPort);
    else if(myTUIO.bTCPMode){
        if(myTUIO.bIsConnected)
            sprintf(buf, "Host: %s\nProtocol: TCP [XML]\nPort: %i", myTUIO.localHost, myTUIO.TUIOFlashPort);
        else
            sprintf(buf, "Binding Error\nHost: %s\nProtocol: TCP [XML]\nPort: %i", myTUIO.localHost, myTUIO.TUIOFlashPort);
    } else if(myTUIO.bBinaryMode){
        if(myTUIO.bIsConnected)
            sprintf(buf, "Host: %s\nProtocol: Binary\nPort: %i", myTUIO.localHost, myTUIO.TUIOFlashPort);
        else
            sprintf(buf, "Binding Error\nHost: %s\nProtocol: Binary\nPort: %i", myTUIO.localHost, myTUIO.TUIOFlashPort);
    }
	ofSetHexColor(0x969696);
    verdana.drawString(buf, 573, 515);
}
void ofxKCoreVision::drawBigMode(){

    filter->drawOrg();
    ofSetColor(255);

}

void ofxKCoreVision::drawMiniMode(){
	//  Black background
    //
	ofSetColor(0);
	ofRect(0,0,ofGetWidth(), ofGetHeight());

	//  Draw outlines
	if (bDrawOutlines){
		for (int i=0; i<contourFinder.nBlobs; i++)
			contourFinder.blobs[i].drawContours(0,0, sceneWidth, sceneHeight, ofGetWidth(), ofGetHeight()-100);

		for (int i=0; i<contourFinder.nFingers; i++)
			contourFinder.fingers[i].drawCenter(0,0, sceneWidth, sceneHeight, ofGetWidth(), ofGetHeight()-100);

		for (int i=0;i<contourFinder.nObjects; i++)
			contourFinder.objects[i].drawBox(0,0, sceneWidth, sceneHeight, ofGetWidth(), ofGetHeight()-100);
	}

	//  Draw grey rectagles for text information
    //
	ofSetColor(128);
	ofFill();
	ofRect(0,ofGetHeight() - 83, ofGetWidth(), 20);
	ofRect(0,ofGetHeight() - 62, ofGetWidth(), 20);
	ofRect(0,ofGetHeight() - 41, ofGetWidth(), 20);
	ofRect(0,ofGetHeight() - 20, ofGetWidth(), 20);

	//  Draw text
    //
	ofSetColor(250);
	verdana.drawString(" [ms]:" + ofToString(differenceTime,0),10, ofGetHeight() - 70 );
	verdana.drawString("[fps]:" + ofToString(fps,0),10, ofGetHeight() - 50 );

	//  Draw green tuio circle
	//
    if((myTUIO.bIsConnected || myTUIO.bOSCMode) && bTUIOMode)
		ofSetColor(0,255,0);	//green = connected
	else
		ofSetColor(255,0,0);	//red = not connected
	ofFill();
	ofCircle(ofGetWidth() - 17 , ofGetHeight() - 10, 5);
	ofNoFill();
}

void ofxKCoreVision::drawOutlines(){
    ofPushStyle();

	//  Find the blobs for drawing
    //
	if(contourFinder.bTrackBlobs){
		for (int i=0; i<contourFinder.nBlobs; i++){

			if (bDrawOutlines) //Draw contours (outlines) on the source image
				contourFinder.blobs[i].drawContours(375, 15, 320, 240, MAIN_WINDOW_WIDTH/(2.0*kN_X), MAIN_WINDOW_HEIGHT/(2.0*kN_Y));



			if (bShowLabels){ //Show ID label
				float xpos = contourFinder.blobs[i].centroid.x * (MAIN_WINDOW_WIDTH);
				float ypos = contourFinder.blobs[i].centroid.y * (MAIN_WINDOW_HEIGHT);

				ofSetColor(200,255,200);
				char idStr[1024];

				sprintf(idStr, "id: %i", contourFinder.blobs[i].id);
				verdana.drawString(idStr, xpos + 365, ypos + contourFinder.blobs[i].boundingRect.height/2 + 15);
			}
		}
	}

	//  Find the blobs for drawing
    //
	if(contourFinder.bTrackFingers){
		for (int i=0; i<contourFinder.nFingers; i++) {

			if (bDrawOutlines) //Draw contours (outlines) on the source image
				contourFinder.fingers[i].drawCenter(375, 15, camWidth, camHeight, MAIN_WINDOW_WIDTH, MAIN_WINDOW_HEIGHT);

			if (bShowLabels){ //Show ID label
				float xpos = contourFinder.fingers[i].centroid.x * (MAIN_WINDOW_WIDTH/camWidth);
				float ypos = contourFinder.fingers[i].centroid.y * (MAIN_WINDOW_HEIGHT/camHeight);

				ofSetColor(200,255,200);
				char idStr[1024];

				sprintf(idStr, "id: %i", contourFinder.fingers[i].id);

				verdana.drawString(idStr, xpos + 365, ypos + contourFinder.fingers[i].boundingRect.height/2 + 15);
			}
		}
	}

	//  Object Drawing
    //
	if(contourFinder.bTrackObjects){
		for (int i=0; i<contourFinder.nObjects; i++){

			if (bDrawOutlines) //Draw contours (outlines) on the source image
				contourFinder.objects[i].drawBox(375, 30, camWidth, camHeight, MAIN_WINDOW_WIDTH, MAIN_WINDOW_HEIGHT);

			if (bShowLabels){ //Show ID label
				float xpos = contourFinder.objects[i].centroid.x * (MAIN_WINDOW_WIDTH/camWidth);
				float ypos = contourFinder.objects[i].centroid.y * (MAIN_WINDOW_HEIGHT/camHeight);

				ofSetColor(200,255,200);
				char idStr[1024];

				sprintf(idStr, "id: %i", contourFinder.objects[i].id);

				verdana.drawString(idStr, xpos + 365, ypos + contourFinder.objects[i].boundingRect.height/2 + 45);
			}
		}
	}
	ofPopStyle();
}

/*****************************************************************************
* KEY EVENTS
*****************************************************************************/
void ofxKCoreVision::_keyPressed(ofKeyEventArgs &e)
{
	if (showConfiguration){
		switch (e.key){
		case 'b':
			filter->bLearnBakground = true;
			break;
		case 'o':
			bDrawOutlines ? bDrawOutlines = false : bDrawOutlines = true;
			controls->update(appPtr->trackedPanel_outlines, kofxGui_Set_Bool, &appPtr->bDrawOutlines, sizeof(bool));
			break;
		case 'h':
			filter->bHorizontalMirror ? filter->bHorizontalMirror = false : filter->bHorizontalMirror = true;
			controls->update(appPtr->propertiesPanel_flipH, kofxGui_Set_Bool, &appPtr->filter->bHorizontalMirror, sizeof(bool));
			break;
		case 'j':
			filter->bVerticalMirror ? filter->bVerticalMirror = false : filter->bVerticalMirror = true;
			controls->update(appPtr->propertiesPanel_flipV, kofxGui_Set_Bool, &appPtr->filter->bVerticalMirror, sizeof(bool));
			break;
		case 't':
			myTUIO.bOSCMode = !myTUIO.bOSCMode;
			myTUIO.bTCPMode = false;
			myTUIO.bBinaryMode = false;
			bTUIOMode = myTUIO.bOSCMode;
			controls->update(appPtr->optionPanel_tuio_tcp, kofxGui_Set_Bool, &appPtr->myTUIO.bTCPMode, sizeof(bool));
			controls->update(appPtr->optionPanel_tuio_osc, kofxGui_Set_Bool, &appPtr->myTUIO.bOSCMode, sizeof(bool));
			controls->update(appPtr->optionPanel_bin_tcp, kofxGui_Set_Bool, &appPtr->myTUIO.bBinaryMode, sizeof(bool));
			//clear blobs
//			myTUIO.blobs.clear();
//			myTUIO.fingers.clear();
//			myTUIO.objects.clear();
			break;
		case 'r':
			myTUIO.bOSCMode = false;
			myTUIO.bTCPMode = !myTUIO.bTCPMode;
			myTUIO.bBinaryMode = false;
			bTUIOMode = myTUIO.bTCPMode;
			controls->update(appPtr->optionPanel_tuio_tcp, kofxGui_Set_Bool, &appPtr->myTUIO.bTCPMode, sizeof(bool));
			controls->update(appPtr->optionPanel_tuio_osc, kofxGui_Set_Bool, &appPtr->myTUIO.bOSCMode, sizeof(bool));
			controls->update(appPtr->optionPanel_bin_tcp, kofxGui_Set_Bool, &appPtr->myTUIO.bBinaryMode, sizeof(bool));
			//clear blobs
//			myTUIO.blobs.clear();
//			myTUIO.fingers.clear();
//			myTUIO.objects.clear();
			break;
		case 'y':
			myTUIO.bOSCMode = false;
			myTUIO.bTCPMode = false;
			myTUIO.bBinaryMode = !myTUIO.bBinaryMode;
			bTUIOMode = myTUIO.bBinaryMode;
			controls->update(appPtr->optionPanel_tuio_tcp, kofxGui_Set_Bool, &appPtr->myTUIO.bTCPMode, sizeof(bool));
			controls->update(appPtr->optionPanel_tuio_osc, kofxGui_Set_Bool, &appPtr->myTUIO.bOSCMode, sizeof(bool));
			controls->update(appPtr->optionPanel_bin_tcp, kofxGui_Set_Bool, &appPtr->myTUIO.bBinaryMode, sizeof(bool));
			//clear blobs
//			myTUIO.blobs.clear();
//			myTUIO.fingers.clear();
//			myTUIO.objects.clear();
			break;
		case 'l':
			bShowLabels ? bShowLabels = false : bShowLabels = true;
			controls->update(appPtr->trackedPanel_ids, kofxGui_Set_Bool, &appPtr->bShowLabels, sizeof(bool));
			break;
		case 'p':
			bShowPressure ? bShowPressure = false : bShowPressure = true;
			break;
		case ' ':
			if (bMiniMode && !bCalibration) // NEED TO ADD HERE ONLY GO MINI MODE IF NOT CALIBRATING
			{
				bMiniMode = false;
				bShowInterface = true;
				filter->bMiniMode = bMiniMode;
				ofSetWindowShape(950,600); //default size
			}
			else if(!bCalibration)
			{
				bMiniMode = true;
				bShowInterface = false;
				filter->bMiniMode = bMiniMode;
				ofSetWindowShape(190,200); //minimized size
			}
			break;
		case 'x': //Exit Calibrating
			if (bCalibration){
				bShowInterface = true;
				bCalibration = false;
				calib.calibrating = false;
				tracker.isCalibrating = false;
				if (bFullscreen == true)
					ofToggleFullscreen();
				bFullscreen = false;
			}
			break;
		case OF_KEY_RETURN: //Close Template Selection and save it
			if( contourFinder.bTrackObjects && isSelecting ){
				isSelecting = false;
				templates.addTemplate(rect,minRect,maxRect,camWidth/320,camHeight/240);
				rect = ofRectangle();
				minRect = rect;
				maxRect = rect;
				minTempArea = 0;
				maxTempArea = 0;
				controls->update(appPtr->TemplatePanel_minArea, kofxGui_Set_Float, &appPtr->minTempArea, sizeof(float));
				controls->update(appPtr->TemplatePanel_maxArea, kofxGui_Set_Float, &appPtr->maxTempArea, sizeof(float));
			}
			break;
		case OF_KEY_UP:
			farThreshold++;
			break;
		case OF_KEY_DOWN:
			farThreshold--;
			break;
		case OF_KEY_RIGHT:
			nearThreshold++;
			break;
		case OF_KEY_LEFT:
			nearThreshold--;
			break;
        case 'i':
			angle++;
			if(angle>30) angle=30;
			kinect[0].setCameraTiltAngle(angle);
			break;
		case 'k':
			angle--;
			if(angle<-30) angle=-30;
			kinect[0].setCameraTiltAngle(angle);
			break;

        case 'f':
            if (bBigMode && !bCalibration) // NEED TO ADD HERE ONLY GO MINI MODE IF NOT CALIBRATING
			{
				bBigMode = false;
				bShowInterface = true;
				filter->bBigMode = bBigMode;
				ofSetWindowShape(950,600); //default size
			}
			else if(!bCalibration)
			{
				bBigMode = true;
				bShowInterface = false;
				filter->bBigMode = bBigMode;
				ofSetWindowShape(950,600); //minimized size
			}
			break;

		default:
			break;
		}
	}
}

void ofxKCoreVision::_keyReleased(ofKeyEventArgs &e){
	if (showConfiguration){
		if ( e.key == 'c' && !bCalibration){
			bShowInterface = false;
			// Enter/Exit Calibration
			bCalibration = true;
			calib.calibrating = true;
			tracker.isCalibrating = true;
			if (bFullscreen == false) ofToggleFullscreen();
			bFullscreen = true;
		}
	}
	if ( e.key == '~' || e.key == '`' && !bMiniMode && !bCalibration) showConfiguration = !showConfiguration;
}

void ofxKCoreVision::_mouseDragged(ofMouseEventArgs &e){
	if (showConfiguration)
		controls->mouseDragged(e.x, e.y, e.button); //guilistener

	if(contourFinder.bTrackObjects){
		if( e.x > 385 && e.x < 705 && e.y > 30 && e.y < 270 ){
			if( e.x < rect.x || e.y < rect.y ){
				rect.width = rect.x - e.x;
				rect.height = rect.y - e.y;

				rect.x = e.x;
				rect.y =  e.y;
			} else {
				rect.width = e.x - rect.x;
				rect.height = e.y - rect.y;
			}
		}
	}
}

void ofxKCoreVision::_mousePressed(ofMouseEventArgs &e)
{
	if (showConfiguration)
	{
		controls->mousePressed( e.x, e.y, e.button ); //guilistener
		if ( contourFinder.bTrackObjects )
		{
			if ( e.x > 385 && e.x < 705 && e.y > 30 && e.y < 270 )
			{
				isSelecting = true;
				rect.x = e.x;
				rect.y = e.y;
				rect.width = 0;
				rect.height = 0;
			}
		}
		//Launch the website in browser
		if ( e.x > 723 && e.y > 549 )
			if ( e.x > 830 ) ofLaunchBrowser("http://www.patriciogonzalezvivo.com");
			else ofLaunchBrowser("http://ccv.nuigroup.com/");
	}
}

void ofxKCoreVision::_mouseReleased(ofMouseEventArgs &e)
{
	if (showConfiguration)
		controls->mouseReleased(e.x, e.y, 0); //guilistener

	if( e.x > 385 && e.x < 705 && e.y > 30 && e.y < 270 ){
		if	( contourFinder.bTrackObjects && isSelecting ){
			minRect = rect;
			maxRect = rect;
		}
	}
}

void ofxKCoreVision::_exit(ofEventArgs &e){

    for(int kN=0; kN<numOfKinects;kN++)
        kinect[kN].close();

	//  Save Settings
	saveXMLSettings();
    if(contourFinder.bTrackObjects)
		templates.saveTemplateXml();

	delete filter;
    filter = NULL;

    printf("Vision module has exited!\n");
}

ofxKCoreVision::~ofxKCoreVision(){
        delete filter;
        filter = NULL;

        for(size_t kN = 0;kN<numOfKinects;kN++)
            kinect[kN].close();
}
