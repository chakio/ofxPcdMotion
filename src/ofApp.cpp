#include "ofApp.h"


ofPolePoint::ofPolePoint(double Radius,double Theta1,double Theta2)
{
	radius = Radius;
	theta1 = Theta1;
	theta2 = Theta2;
}
//--------------------------------------------------------------
particle::particle()
{
	col 	= ofColor(255,255,255);
	vel 	= ofVec3f(0,0,0);
}
void particle::setPos(ofVec3f position)
{
	pos = position;
}
void particle::setModelPos(ofVec3f position)
{
	modelPos = position;
}
void particle::setSpherePos(ofPolePoint position)
{
	spherePolePos = position;
	spherePos = ofVec3f(spherePolePos.radius*sin(spherePolePos.theta1),spherePolePos.radius*cos(spherePolePos.theta1)*cos(spherePolePos.theta2),spherePolePos.radius*cos(spherePolePos.theta1)*sin(spherePolePos.theta2));
}
void particle::setCol(ofColor color)
{
	col = color;
}

ofVec3f particle::getPos(double rate)
{
	pos = rate*spherePos+(1-rate)*modelPos;
	return pos;
}
ofVec3f particle::getModelPos()
{
	return modelPos;
}
ofPolePoint particle::getSpherePos()
{
	return spherePolePos;
}
ofVec3f particle::getVel()
{
	return vel;
}
ofColor	particle::getCol()
{
	return col;
}

//--------------------------------------------------------------
void cameraControl::setDistanceRange(double min,double max)
{
	minDistance = min;
	maxDistance = max;
	basePos = ofVec3f(700,0,0);
	targetPos = ofVec3f(700,0,0);
}

void cameraControl::update(double rate)
{
	if(rate<0.5 && !changeTarget)
	{
		basePos = targetPos;
		double theta1 = ofRandom(-M_PI*0.25,M_PI*0.25);
		double theta2= ofRandom(0.0,M_PI*0.5);
		double distance= ofRandom(minDistance,maxDistance);
		targetPos = ofVec3f(distance*sin(theta1),distance*cos(theta1)*cos(theta2),distance*cos(theta1)*cos(theta2));
		changeTarget = true;
	}
	else if(rate>0.5)
	{
		changeTarget = false;
	}
	double sigmoidRate = 1/(1+exp(-40*(rate-0.5)));//球とモデルをシグモイドで切り替え
	ofVec3f pos = rate*targetPos+(1-sigmoidRate)*basePos;

	this->setPosition(pos);
	this->lookAt(ofVec3f(0,0,0), ofVec3f(0,1,0));
}


//--------------------------------------------------------------
ofApp::ofApp(int argc, char *argv[])
{

}

//--------------------------------------------------------------
void ofApp::setup(){
	ofEnableDepthTest();
	ofHideCursor();
    ofSetBackgroundColor(250);
	count = 0;

	mesh.setMode(OF_PRIMITIVE_POINTS);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	// 作成したPointCloudを読み込む
	// pcl::io::loadPCDFile("p_cloud_ascii.pcd", *p_cloud);
	pcl::io::loadPCDFile("data/1.pcd", *inputCloud);
	cout<<inputCloud->points.size()<<endl;

	//まず飛び値の削除
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PassThrough<pcl::PointXYZRGB> passThrough;
	passThrough.setInputCloud (inputCloud);
	passThrough.setFilterFieldName ("z");
	passThrough.setFilterLimits (0.3, 2.0);//PCDの読み込む範囲の指定
	//pass.setFilterLimitsNegative (true);
	passThrough.filter (*filteredCloud);

	//次に重心を求める
	pcl::CentroidPoint<pcl::PointXYZRGB> centroid;
	for(int i=0;i<filteredCloud->points.size();i++)
	{
		//重心計算用に追加
		centroid.add(filteredCloud->points[i]);
	}
	pcl::PointXYZ c1;
	centroid.get(c1);
	ofVec3f centroidVec3f(c1.x,c1.y,c1.z);

	//各点群をparticleとして読み込み
	modelMinHeight=100000000;
	modelMaxHeight=-10000000;
	for(int i=0;i<filteredCloud->points.size();i++)
	{
		particle p;
		//モデル上の点の指定
		p.setModelPos(ofVec3f((filteredCloud->points[i].x-centroidVec3f.x)*enlargeRate,-(filteredCloud->points[i].y-centroidVec3f.y)*enlargeRate,-(filteredCloud->points[i].z-centroidVec3f.z)*enlargeRate));
		
		//ランダムに球上の点の指定
		double theta1=ofRandom(0.0,2*M_PI);
		double theta2=ofRandom(0.0,M_PI);
		double R=enlargeRate/2.0;
		
		p.setSpherePos(ofPolePoint(R,theta1,theta2));
		p.setCol(ofColor(filteredCloud->points[i].r,filteredCloud->points[i].g,filteredCloud->points[i].b));
		particles.push_back(p);
	}

	//読み込んだモデルの区中で最大の高さと最小の高さを取得
	for(int i=0;i<filteredCloud->points.size();i++)
	{
		if(-(filteredCloud->points[i].y-centroidVec3f.y)*enlargeRate<modelMinHeight)
		{
			modelMinHeight = -(filteredCloud->points[i].y-centroidVec3f.y)*enlargeRate;
		}
		else if(-(filteredCloud->points[i].y-centroidVec3f.y)*enlargeRate>modelMaxHeight)
		{
			modelMaxHeight = -(filteredCloud->points[i].y-centroidVec3f.y)*enlargeRate;
		}
	}
	cout<<modelMinHeight<<endl;
	cout<<modelMaxHeight<<endl;
	cam.setDistanceRange(minDistance,maxDistance);
	cout<<"done"<<endl;
}

//--------------------------------------------------------------
void ofApp::update(){
	double timeRate = (double)((int)ofGetElapsedTimeMillis()%(int)frequency)/frequency;
	double cameraTimeRate = (double)((int)ofGetElapsedTimeMillis()%(int)angleChangeFrequency)/angleChangeFrequency;
	
	mesh.clear();

	for(int particleIndex=0; particleIndex<particles.size(); particleIndex++)
	{
		//時間に基づくモデルの形状の切り替え
		double modelTimeRate = (timeRate -  ((particles[particleIndex].getModelPos().y-modelMinHeight)/(modelMaxHeight-modelMinHeight)-0.5)*0.2);
		if (modelTimeRate<0)
		{
			modelTimeRate +=1;
		}
		else if(modelTimeRate>1)
		{
			modelTimeRate -= 1;
		}

		//球とモデルをシグモイドで切り替え
		double rate = 1/(1+exp(-cSigmoid*(std::abs(modelTimeRate-0.5)-0.25)));

		//	rateに応じた点群の位置の取得
		ofVec3f pos = particles[particleIndex].getPos(rate);

		//点群の位置に対してノイズをのせる
		//点群に割り当てられたindexに基づくノイズ
		ofVec3f positionalNoise = ofVec3f(ofNoise(particleIndex,0,ofGetElapsedTimef()),ofNoise(particleIndex,1,ofGetElapsedTimef()),ofNoise(particleIndex,2,ofGetElapsedTimef()));
		//点群の球面上の点の位置に基づくノイズ
		ofPolePoint polePos = particles[particleIndex].getSpherePos(); 
		ofVec3f sphericalNoise = pos.normalized()*ofNoise(sin(polePos.theta1),cos(polePos.theta1)*cos(polePos.theta2),cos(polePos.theta1)*sin(polePos.theta2),ofGetElapsedTimef());

		//meshに追加
		mesh.addVertex(pos+noiseLevel*rate*(positionalNoise*(1-noiseRate) + sphericalNoise*noiseRate));
		mesh.addColor(particles[particleIndex].getCol());
	}

	cam.update(cameraTimeRate );
}
//--------------------------------------------------------------
void ofApp::draw(){
	
	cam.begin();
	glPointSize(2);
	mesh.draw();
	cam.end();
}

//--------------------------------------------------------------
void ofApp::getMessage(const std_msgs::String::ConstPtr& msg){
 
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){
    
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------

