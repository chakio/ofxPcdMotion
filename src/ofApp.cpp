#include "ofApp.h"

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
void particle::setSpherePos(ofVec3f position)
{
	spherePos = position;
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
ofVec3f particle::getVel()
{
	return vel;
}
ofColor	particle::getCol()
{
	return col;
}
//--------------------------------------------------------------
ofApp::ofApp(int argc, char *argv[])
{

}

//--------------------------------------------------------------
void ofApp::setup(){
	ofHideCursor();
    ofSetBackgroundColor(0);
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
	passThrough.setFilterLimits (0.3, 2.0);
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
	ofVec3f centroidVec3f =  ofVec3f(c1.x,c1.y,c1.z);
	cout<<centroidVec3f<<endl;
	//重心が原点になるように並進移動してから読み込み
	for(int i=0;i<filteredCloud->points.size();i++)
	{
		particle p;
		//モデル上の点
		p.setModelPos(ofVec3f((filteredCloud->points[i].x-centroidVec3f.x)*enlargeLate,-(filteredCloud->points[i].y-centroidVec3f.y)*enlargeLate,-(filteredCloud->points[i].z-centroidVec3f.z)*enlargeLate));
		
		//球上の点
		double theta1=ofRandom(0,M_PI*2);
		double theta2=ofRandom(0,M_PI);
		double R=enlargeLate/2;
		
		p.setSpherePos(ofVec3f(R*sin(theta1)*sin(theta2),R*sin(theta1)*cos(theta2),R*cos(theta1)));
		p.setCol(ofColor(filteredCloud->points[i].r,filteredCloud->points[i].g,filteredCloud->points[i].b));
		particles.push_back(p);
	}
	cout<<"done"<<endl;
}

//--------------------------------------------------------------
void ofApp::update(){
	double timeRate = (double)((int)ofGetElapsedTimeMillis()%(int)frequency)/frequency;
	
	double rate = 1/(1+exp(-cSigmoid*(std::abs(timeRate-0.5)-0.25)));//球とモデルをシグモイドで切り替え
	mesh.clear();

	for(int i=0; i<particles.size(); i++)
	{
		ofVec3f noise = ofVec3f(ofNoise(i,0,ofGetElapsedTimef()),ofNoise(i,1,ofGetElapsedTimef()),ofNoise(i,2,ofGetElapsedTimef()));
		ofVec3f pos = particles[i].getPos(rate);

		mesh.addVertex(pos+noiseLevel*rate*noise);
		mesh.addColor(particles[i].getCol());
	}
}
//--------------------------------------------------------------
void ofApp::draw(){
	
	cam.begin();
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

