#include "ofMain.h"
#include "ofApp.h"
#include <ros/ros.h>

//========================================================================

int main(int argc, char *argv[]){
    //ros::init(argc, argv, "ofxRosTemplate");  // ノード名の初期化

	ofSetupOpenGL(720,720,OF_WINDOW);// <-------- setup the GL context

	ofRunApp(new ofApp(argc, argv));
}
