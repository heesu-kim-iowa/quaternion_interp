#include "ofApp.h"

//--------------------------------------------------------------
Eigen::Quaternionf quaternion_exp(Eigen::Quaternionf v) {
	// TODO: Implement the exponential map for quaternions
	// HINT: q = exp(v), where v is a pure imaginary quaternion
	//       magnitude of v == angle theta
	//       direction of v (unit vector) == vhat
	//       q = exp(v) = exp(theta*vhat) = [cos(theta), vhat*sin(theta)]
	float theta = v.vec().norm();
	Eigen::Vector3f vhat(1, 1, 1);
	Eigen::Quaternionf q(cos(theta), vhat(0) * sin(theta), vhat(1) * sin(theta), vhat(2) * sin(theta));
	return q;
}

//--------------------------------------------------------------
Eigen::Quaternionf quaternion_log(Eigen::Quaternionf q) {
	// TODO: Implement the logarithmic map for quaternions
	// HINT: v = log(q), where q is a unit quaternion
	//       Since q is a unit quaternion, q must be in a form [cos(theta), vhat*sin(theta)]
	//       Then, v = theta * vhat
	Eigen::Vector3f vec = q.vec();
	Eigen::Quaternionf v(log(q.norm()), vec(0)*acos(q.w()/q.norm()), vec(1)*acos(q.w()/q.norm()), vec(2)*acos(q.w()/q.norm()));
	return v;
}

//--------------------------------------------------------------
void ofApp::transform(ofMesh& mesh, Eigen::Matrix4f& M) {
	for (int i = 0; i < mesh.getNumVertices(); i++) {
		ofVec3f p_ = mesh.getVertex(i);
		Eigen::Vector4f p(p_.x, p_.y, p_.z, 1);
		Eigen::Vector4f p_trans = M*p;
		mesh.setVertex(i, ofVec3f(p_trans[0], p_trans[1], p_trans[2]));
	}
}

//--------------------------------------------------------------
void ofApp::setup(){
	t = 0;
	if (loader.loadModel(ofToDataPath("bunny.obj"))) {
		mesh = loader.getMesh(0);
		
		// Compute the centroid of the mesh
		ofVec3f centroid(0,0,0);
		for (int i = 0; i < mesh.getNumVertices(); i++) {
			centroid += mesh.getVertex(i);
		}
		centroid /= mesh.getNumVertices();


		/* Set the local origin of the mesh to its centroid. Scale up by the factor of 1000
		The code below does essentially the same thing as:
		for (int i = 0; i < mesh.getNumVertices(); i++) {
			mesh.setVertex(i, (mesh.getVertex(i)-centroid) * 1000);
		}*/
		Eigen::Matrix4f M;
		M.setIdentity();
		M(0, 3) = -centroid.x;
		M(1, 3) = -centroid[1];
		M(2, 3) = -centroid[2];
		M *= 1000;
		M(3, 3) = 1;
		transform(mesh, M);

		// source is the one displayed in yellow
		// target is the one displayed in gray
		source = mesh;
		target = mesh;

		// T0 is the 4x4 transformation for the source mesh
		T0.setIdentity();
		T0(0, 3) = -350;   // move it to the left by 350 units
		transform(source, T0);

		// T1 is the 4x4 transformation for the target mesh
		T1.setIdentity();
		T1(0, 3) = 350;    // move it to the right by 350 units
		// TODO: apply random transformation
		// HINT: Define 'float angle' and 'Eigen::Vector3f axis'.
		//       Initialize them with random values (look up 'ofRandom').
		//       Define 'Eigen::Quaternionf q' using 'angle' and 'axis'.
		//       Note that axis has to be a unit vector.
		//       Use quaternion_exp() function to convert the angle & axis to a unit quaternion
		//       Note that the input to the exponential has to be "half" of the angle
		float angle(ofRandom(2*PI));           // do something here
		Eigen::Vector3f axis(ofRandom(10), ofRandom(10), ofRandom(10));  // do something here
		axis.normalize();

		Eigen::Quaternionf q_(cos(angle/2), axis(0)*sin(angle/2), axis(1)* sin(angle / 2), axis(2)* sin(angle / 2));
		Eigen::Quaternionf q = quaternion_exp(quaternion_log(q_)); // replace 'XXX' with an appropriate code

		T1.block<3,3>(0,0) = q.toRotationMatrix();
		transform(target, T1);
	}
}

//--------------------------------------------------------------
void ofApp::update(){
	//Eigen::Matrix4f T = (1-t)*T0 + t*T1;   // TODO: Replace this line with quaternion interpolation.
	Eigen::Matrix4f T = (1 - t) * T0 + t * T1;   // TODO: Replace this line with quaternion interpolation.
	interp = mesh;
	transform(interp, T);
}

//--------------------------------------------------------------
void ofApp::draw(){
	cam.enableOrtho();
	ofEnableDepthTest();
	ofEnableLighting();
	light.enable();
	light.setPosition(cam.getPosition());
	cam.begin();
	ofSetColor(255, 200, 50);
	interp.draw();
	ofSetColor(200);
	source.draw();
	target.draw();
	cam.end();
	
	ofDisableDepthTest();
	ofDisableLighting();
	char buffer[100];
	sprintf(buffer, "t=%.2f", t);
	ofSetColor(255);
	ofDrawBitmapString(buffer, 10, 20);
	ofDrawBitmapString("Press 's' to increase t", 10, 32);
	ofDrawBitmapString("Press 'a' to decrease t", 10, 44);
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
	if (key == 'a')
		t -= 0.05;
	if (key == 's')
		t += 0.05;
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
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}
