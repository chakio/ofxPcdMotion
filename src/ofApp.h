#pragma once

#include "ofMain.h"
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <iostream>
#include <sstream>
#include <string>
#include <math.h>
#include <stdlib.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/io/pcd_io.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/ModelCoefficients.h>

using namespace std;

//極座標系の位置
class ofPolePoint
{
public:
	ofPolePoint(double Radius= 0, double Theta1 = 0, double Theta2 = 0);
	double radius;
	double theta1;
	double theta2;
};

class particle
{
private:
	ofVec3f pos;
	ofVec3f vel;
	ofColor col;

	ofVec3f 	spherePos;//デカルト座標系の球上の点
	ofPolePoint spherePolePos;//極座標系の球上の点
	ofVec3f 	modelPos;//デカルト座標系のPCD上の点

public:
	particle();
	void setPos(ofVec3f position);//パーティクルの位置
	void setCol(ofColor color);//色の設定

	void setSpherePos(ofPolePoint position);//球上のパーティクルの位置
	void setModelPos(ofVec3f position);//PCD上のパーティクルの位置

	ofVec3f getPos(double rate);
	ofVec3f getModelPos();
	ofPolePoint getSpherePos();
	
	ofVec3f getVel();
	ofColor getCol();
};

class cameraControl : public ofEasyCam
{
private:
	ofVec3f basePos;
	ofVec3f targetPos;
	double minDistance;
	double maxDistance;
	bool changeTarget;
public:
	void setDistanceRange(double minDistance,double maxDistance);
	void update(double rate);
};

class ofApp : public ofBaseApp
{
	public:
		ofApp(int argc, char *argv[]);
		void setup();
		void update();
		void draw();

		void getMessage(const std_msgs::String::ConstPtr& msg);

		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void mouseEntered(int x, int y);
		void mouseExited(int x, int y);
		void windowResized(int w, int h);

		pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
		pcl::PCLPointCloud2 cloud_filtered;

		vector<particle> particles;

		int count = 0;
		int step=0;

		double enlargeRate = 500;
		ofMesh mesh;

		double modelMaxHeight;
		double modelMinHeight;

		double frequency = 9000;
		double cSigmoid = 50;
		double noiseLevel = 100;
		double noiseRate = 0.7;//positional noise per spherical noise 

		cameraControl cam;
		double minDistance = 600;
		double maxDistance = 700;
		double angleChangeFrequency = 6000;
};
