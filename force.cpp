#include "force.h"

Force::Force(Eigen::Vector3d accel) {
	acceleration = accel;
}

std::vector<int> Force::getParticlesImpacted(){
	return particles_impacted;
}

Eigen::Vector3d Force::getAcceleration(){
	return acceleration;
}

void Force::addParticlesImpacted(int particleNum){
	particles_impacted.push_back(particleNum);
}

void Force::resetParticlesImpacted(){
	particles_impacted.clear();
}

