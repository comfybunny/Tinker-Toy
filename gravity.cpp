#include "gravity.h"
#include <fstream>
#include <iostream>

using namespace std;

Gravity::Gravity():Force(Eigen::Vector3d (0.0, -9.8, 0.0)){
}

Eigen::Vector3d Gravity::calculateForceAdded(Particle* particle){
	// cout << particle->mMass*getAcceleration()[1] << endl;
	return particle->mMass*getAcceleration();
}