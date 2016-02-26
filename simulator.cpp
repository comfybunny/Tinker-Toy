#include "simulator.h"
#include <fstream>
#include <iostream>

using namespace std;


Simulator::Simulator() {
    // initialize the particles
	reset();
}

int Simulator::getNumParticles() {
    return mParticles.size();
}

Particle* Simulator::getParticle(int index) {
    return &mParticles[index];
}

double Simulator::getTimeStep() {
    return mTimeStep;
}

void Simulator::reset() {
	mParticles.resize(2);

	// Init particle positions (default is 0, 0, 0)
	mParticles[0].mPosition[1] = 0.0;
	mParticles[0].mPosition[0] = 0.2;
	mParticles[1].mPosition[0] = 0.2;
	mParticles[1].mPosition[1] = -0.1;

	mTimeStep = 0.0003;
	forces.clear();
	gravity.resetParticlesImpacted();

    for (int i = 0; i < mParticles.size(); i++) {
        mParticles[i].mVelocity.setZero();
        mParticles[i].mAccumulatedForce.setZero();
		// everything has gravity
		gravity.addParticlesImpacted(i);
		// cout << gravity.getParticlesImpacted()[i] << endl;
    }
	addForce(&gravity);
	// cout << (*forces.begin())->getAcceleration() << endl;
}

void Simulator::addForce(Force * newForce){
	forces.push_back(newForce);
}

void Simulator::simulate() {
	// clear force accumulator from previous iteration and update
	// cout << (*forces.begin())->getAcceleration() << endl;

	std::vector<Eigen::VectorXd> derivatives;
	// cout << derivatives.size() << endl;
	derivatives.resize(mParticles.size());
	// cout << derivatives[0] << endl;

	for (int i = 0; i < mParticles.size(); i++) {
		mParticles[i].mAccumulatedForce.setZero();
		mParticles[i].update_accumulated_forces(i, forces);

		derivatives[i] = solver.solve_X_dot(&mParticles[i]);
	}
    
    for (int i = 0; i < mParticles.size(); i++) {
        mParticles[i].mPosition += Eigen::Vector3d (derivatives[i][0], derivatives[i][1], derivatives[i][2]) * mTimeStep;
        mParticles[i].mVelocity += Eigen::Vector3d(derivatives[i][3], derivatives[i][4], derivatives[i][5]) * mTimeStep;
    }

}


