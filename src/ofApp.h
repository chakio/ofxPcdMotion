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

class particle
{
private:
	ofVec3f pos;
	ofVec3f vel;
	ofColor col;

	ofVec3f spherePos;
	ofVec3f modelPos;
	ofVec2f theta;

public:
	particle();
	void setPos(ofVec3f position);
	void setCol(ofColor color);

	void setSpherePos(ofVec3f position);
	void setSphereTheta(double theta1,double theta2);
	void setModelPos(ofVec3f position);
	ofVec3f getPos(double rate);
	ofVec3f getModelPos();
	ofVec3f getVel();
	ofColor getCol();
	ofVec2f getTheta();
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

class ofApp : public ofBaseApp{

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

		cameraControl cam;
		double minDistance = 600;
		double maxDistance = 700;
		double angleChangeFrequency = 6000;
};
