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
	W = Eigen::MatrixXd::Identity((mParticles.size() - 1) * 3, (mParticles.size() - 1) * 3);
	Eigen::VectorXd weights((mParticles.size() - 1) * 3);
	for (int i = 1; i < (mParticles.size() - 1); i++ ) {
		// cout << "TEST" << endl;
		for (int xyz = 0; xyz < 3; xyz++) {
			W(i-1 + xyz, i - 1 + xyz) = mParticles[i].mMass;
		}
	}
	// cout << "TEST" << endl;

	// Init particle positions (default is 0, 0, 0)
	mParticles[0].mPosition[0] = 0.2;
	mParticles[0].mPosition[1] = 0.0;
	
	mParticles[1].mPosition[0] = 0.4;
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

	constraints.push_back(Constraint(&mParticles[0], &mParticles[1]));
}

void Simulator::addForce(Force * newForce){
	forces.push_back(newForce);
}

void Simulator::simulate() {
	// clear force accumulator from previous iteration
	// cout << (*forces.begin())->getAcceleration() << endl;

	std::vector<Eigen::VectorXd> derivatives;
	derivatives.resize(mParticles.size());
	// cout << derivatives[0] << endl;

	// update applied forces here
	for (int i = 0; i < mParticles.size(); i++) {
		mParticles[i].mAccumulatedForce.setZero();
		mParticles[i].legal_force.setZero();
		mParticles[i].update_accumulated_forces(i, forces);
		derivatives[i] = solver.solve_X_dot(&mParticles[i]);
	}
    
	Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(constraints.size(), (mParticles.size()-1)*3);
	Eigen::MatrixXd jacobianDot = Eigen::MatrixXd::Zero(constraints.size(), (mParticles.size() - 1) * 3);;
	
	// not including the center circle which is at index 0 ; the center circle is a fixed point
	for (int i = 1; i < mParticles.size(); i++) {
		for (int j = 0; j < constraints.size(); j++) {
			Constraint currConstraint = constraints[j];
			Eigen::Vector3d currJ = currConstraint.dCdx2();
			Eigen::Vector3d currJdot = currConstraint.dCdotdx2();
			for (int temp = 0; temp < 3; temp++) {
				jacobian(j, (i - 1)*3 + temp) = currJ(temp);
				jacobianDot(j, (i - 1)*3 + temp) = currJdot(temp);
			}
		}
	}

	//cout << jacobianDot << endl;
	//cout << endl;

	Eigen::VectorXd Q((mParticles.size() - 1) * 3);
	Eigen::VectorXd qdot((mParticles.size() - 1) * 3);


	for (int i = 1; i < mParticles.size(); i++) {
		for (int temp = 0; temp < 3; temp++) {
			Q((i - 1) * 3 + temp) = mParticles[i].mAccumulatedForce(temp);
			qdot((i - 1) * 3 + temp) = mParticles[i].mVelocity(temp);
		}
	}

	

	Eigen::MatrixXd lambda = (jacobian*W*(jacobian.transpose())).inverse()*(- jacobianDot*qdot - jacobian*W*Q);
	// one lambda for each particle
	// cout << lambda.rows() << endl;
	// cout << lambda.cols() << endl;


	for (int i = 1; i < mParticles.size(); i++) {
		for (int j = 0; j < constraints.size(); j++) {
		}
	}

	Eigen::Vector3d durp = constraints[0].dCdx2()*lambda;
	cout << durp << endl;

	mParticles[1].legal_force = durp;
	mParticles[1].mPosition += Eigen::Vector3d(derivatives[1][0], derivatives[1][1], derivatives[1][2]) * mTimeStep;
	mParticles[1].mVelocity += Eigen::Vector3d(durp[0], durp[1], durp[2]) * mTimeStep;
    for (int i = 0; i < mParticles.size(); i++) {
        
    }

}


