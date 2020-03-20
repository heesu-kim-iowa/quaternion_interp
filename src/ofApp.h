#pragma once

#include "ofMain.h"
#include "ofxAssimpModelLoader.h"
#include "Eigen/Dense"
#include "Eigen/Geometry"

class ofApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();

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
		
		void transform(ofMesh& mesh, Eigen::Matrix4f& M);

		ofEasyCam cam;
		ofLight light;
		ofMesh mesh;
		ofxAssimpModelLoader loader;

		Eigen::Matrix4f T0, T1;
		ofMesh target;
		ofMesh source;
		ofMesh interp;

		float t;

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
