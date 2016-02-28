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
    return mParticles[index];
}

double Simulator::getTimeStep() {
    return mTimeStep;
}

void Simulator::reset() {
	mParticles.clear();
	mParticles.push_back(new Particle());
	mParticles.push_back(new Particle());
	W = Eigen::MatrixXd::Identity((mParticles.size() - 1) * 3, (mParticles.size() - 1) * 3);
	Eigen::VectorXd weights((mParticles.size() - 1) * 3);
	for (int i = 1; i < (mParticles.size() - 1); i++ ) {
		// cout << "TEST" << endl;
		for (int xyz = 0; xyz < 3; xyz++) {
			W(i-1 + xyz, i - 1 + xyz) = 1.0/mParticles[i]->mMass;
		}
	}
	// cout << "TEST" << endl;

	// Init particle positions (default is 0, 0, 0)
	mParticles[0]->mPosition[0] = 0.0;
	mParticles[0]->mPosition[1] = 0.0;
	mParticles[0]->mPosition[2] = 0.0;

	mParticles[1]->mPosition[0] = 0.2;
	mParticles[1]->mPosition[1] = 0.0;
	mParticles[1]->mPosition[2] = 0.0;

	mTimeStep = 0.0003;
	forces.clear();
	gravity.resetParticlesImpacted();

    for (int i = 0; i < mParticles.size(); i++) {
        mParticles[i]->mVelocity.setZero();
        mParticles[i]->mAccumulatedForce.setZero();
		// everything has gravity
		gravity.addParticlesImpacted(i);
		// cout << gravity.getParticlesImpacted()[i] << endl;
    }
	forces.push_back(&gravity);
	// cout << (*forces.begin())->getAcceleration() << endl;
	constraints.clear();
	constraints.push_back(Constraint(mParticles[0], mParticles[1]));
}

int Simulator::getSelectedParticle(){
	return selected_particle;
}

void Simulator::addParticle(float x_pos, float y_pos){
	mParticles.push_back(new Particle(x_pos, y_pos));
}


void Simulator::simulate() {
	// clear force accumulator from previous iteration and update applied forces here
	for (int i = 0; i < mParticles.size(); i++) {
		mParticles[i]->mAccumulatedForce.setZero();
		mParticles[i]->fhat.setZero();
		mParticles[i]->update_accumulated_forces(i, forces);
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

	Eigen::VectorXd Q((mParticles.size() - 1) * 3, mParticles.size() - 1);
	Eigen::VectorXd qdot((mParticles.size() - 1) * 3, mParticles.size() - 1);

	for (int i = 1; i < mParticles.size(); i++) {
		for (int temp = 0; temp < 3; temp++) {
			Q((i - 1) * 3 + temp) = mParticles[i]->mAccumulatedForce(temp);
			qdot((i - 1) * 3 + temp) = mParticles[i]->mVelocity(temp);
		}
	}

	// one lambda for each particle
	Eigen::MatrixXd lambda = (jacobian*W*(jacobian.transpose())).inverse()*(- jacobianDot*qdot - jacobian*W*Q);
	cout << lambda << endl;
	cout << endl;
	for (int i = 1; i < mParticles.size(); i++) {
		for (int j = 0; j < constraints.size(); j++) {
		}
	}

	mParticles[1]->fhat = constraints[0].dCdx2()*lambda;

	std::vector<Eigen::VectorXd> derivatives;
	derivatives.resize(mParticles.size()-1);

	for (int i = 1; i < mParticles.size(); i++) {
		derivatives[i-1] = solver.solve_X_dot(mParticles[i]);
	}

	mParticles[1]->mPosition += Eigen::Vector3d(derivatives[0][0], derivatives[0][1], derivatives[0][2]) * mTimeStep;
	// cout << mParticles[0].mPosition << endl;
	// cout << mParticles[1].mPosition << endl;
	// cout << endl;

	mParticles[1]->mVelocity += Eigen::Vector3d(derivatives[0][3], derivatives[0][4], derivatives[0][5]) * mTimeStep;
    for (int i = 0; i < mParticles.size(); i++) {
        
    }

}


