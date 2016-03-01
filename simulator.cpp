#include "simulator.h"
#include <fstream>
#include <iostream>

using namespace std;


Simulator::Simulator() {
    // initialize the particles
	reset();
	/**
	selected_particle = 0;
	feedback = false;
	ks = 50;
	kd = 30;
	editing_force = false;
	selecting = false;
	**/
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
	selected_particle = 0;
	feedback = false;
	ks = 50;
	kd = 30;
	editing_force = false;

	mParticles.clear();
	mParticles.push_back(new Particle());
	mParticles.push_back(new Particle());
	W = Eigen::MatrixXd::Identity((mParticles.size() - 1) * 3, (mParticles.size() - 1) * 3);
	for (int i = 1; i < (mParticles.size() - 1); i++ ) {
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
	//Force* gravity = new Force(Eigen::Vector3d(0.0, -9.8, 0.0));
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
	// forces[0] will always be gravity
	forces[0]->addParticlesImpacted(mParticles.size());
	mParticles.push_back(new Particle(x_pos, y_pos));
	// update W matrix
	W = Eigen::MatrixXd::Identity((mParticles.size() - 1) * 3, (mParticles.size() - 1) * 3);
	for (int i = 1; i < (mParticles.size() - 1); i++) {
		for (int xyz = 0; xyz < 3; xyz++) {
			W(i - 1 + xyz, i - 1 + xyz) = 1.0 / mParticles[i]->mMass;
		}
	}
	// add new constraint
	constraints.push_back(Constraint(mParticles[mParticles.size()-2], mParticles[mParticles.size() - 1]));
}

bool Simulator::hasFeedback(){
	return feedback;
}

void Simulator::toggleFeedback(){
	feedback = !feedback;
}

void Simulator::toggleSolver(){
	solver.toggle();
}

char * Simulator::getSolverName(){
	return solver.getSolverName();
}



Eigen::Vector3d Simulator::getForceEnd(){
	return force_end;
}

bool Simulator::getEditingForceBoolean(){
	return editing_force;
}

void Simulator::toggleEditForceBoolean(){
	editing_force = !editing_force;
}

void Simulator::setForceEnd(Eigen::Vector3d newForce){
	force_end = newForce;
}

void Simulator::updateSelectedParticle(Eigen::Vector3d click_pt){
	// find closest particle to this point and then set it
	int closestPointIndex = 0;
	float distance = FLT_MAX;
	for (int i = 0; i < mParticles.size(); i++) {
		float currDistance = sqrt(pow(click_pt[0] - mParticles[i]->mPosition[0], 2) + pow(click_pt[1] - mParticles[i]->mPosition[1], 2));
		if (currDistance < distance) {
			distance = currDistance;
			closestPointIndex = i;
		}
	}
	selected_particle = closestPointIndex;
}

void Simulator::addForce(Eigen::Vector3d mouseVector){
	// cout << mouseVector << endl;
	Force* tempForce = new Force(mouseVector*1000);
	//cout << mouseVector * 1000 << endl;
	tempForce->addParticlesImpacted(selected_particle);
	forces.push_back(tempForce);
}


void Simulator::simulate() {
	if (forces.size() == 2) {
		//cout << forces.size() << endl;
	}
	// clear force accumulator from previous iteration and update applied forces here
	for (int i = 0; i < mParticles.size(); i++) {
		mParticles[i]->mAccumulatedForce.setZero();
		mParticles[i]->fhat.setZero();
		for (int j = 0; j < forces.size(); j++) {
			std::vector<int> currForceParticlesImpacted = forces[j]->getParticlesImpacted();
			if (std::find(currForceParticlesImpacted.begin(), currForceParticlesImpacted.end(), i) != currForceParticlesImpacted.end()) {
				if (j == 1) {
					cout << forces[j]->getAcceleration() << endl;
				}
				mParticles[i]->addForce(forces[j]->getAcceleration()*mParticles[i]->mMass);
			}
		}
	}
	
	mParticles[1]->mAccumulatedForce.setZero();

	Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(constraints.size(), (mParticles.size()-1)*3);
	Eigen::MatrixXd jacobianDot = Eigen::MatrixXd::Zero(constraints.size(), (mParticles.size() - 1) * 3);;

	// not including the center circle which is at index 0 ; the center circle is a fixed point
	for (int i = 1; i < mParticles.size(); i++) {
		for (int j = 0; j < constraints.size(); j++) {
			Constraint currConstraint = constraints[j];
			if (mParticles[i] == currConstraint.getParticle2()) {
				Eigen::Vector3d currJ = currConstraint.dCdx2();
				Eigen::Vector3d currJdot = currConstraint.dCdotdx2();
				for (int temp = 0; temp < 3; temp++) {
					jacobian(j, (i - 1) * 3 + temp) = currJ[temp];
					jacobianDot(j, (i - 1) * 3 + temp) = currJdot[temp];
				}
			}
			else if (mParticles[i] == currConstraint.getParticle1()) {
				Eigen::Vector3d currJ = currConstraint.dCdx1();
				Eigen::Vector3d currJdot = currConstraint.dCdotdx1();
				for (int temp = 0; temp < 3; temp++) {
					jacobian(j, (i - 1) * 3 + temp) = currJ[temp];
					jacobianDot(j, (i - 1) * 3 + temp) = currJdot[temp];
				}
			}
			
		}
	}

	// cout << jacobian << endl;

	Eigen::VectorXd Q((mParticles.size() - 1) * 3);
	Eigen::VectorXd qdot((mParticles.size() - 1) * 3);

	for (int i = 1; i < mParticles.size(); i++) {
		for (int temp = 0; temp < 3; temp++) {
			Q((i - 1) * 3 + temp) = mParticles[i]->mAccumulatedForce(temp);
			qdot((i - 1) * 3 + temp) = mParticles[i]->mVelocity(temp);
		}
	}
	
	// one lambda for each particle
	Eigen::MatrixXd tempCequation = -jacobianDot*qdot - jacobian*W*Q;
	// if feedback then need to so -ksC-kdCdot
	
		Eigen::VectorXd C(constraints.size());
		Eigen::VectorXd Cdot(constraints.size());
		for (int i = 0; i < constraints.size(); i++) {
			C(i) = constraints[i].C();
			Cdot(i) = constraints[i].Cdot();
		}
	if (feedback) {
		tempCequation = tempCequation - ks*C - kd*Cdot;
	}
	Eigen::MatrixXd lambda = (jacobian*W*(jacobian.transpose())).ldlt().solve(tempCequation);
	//Eigen::MatrixXd lambda = (jacobian*W*(jacobian.transpose())).inverse()*(tempCequation);
	Eigen::MatrixXd legal_forces = jacobian.transpose()*lambda;
	
	/**
	cout << "JACOBIAN" << endl;
	cout << jacobian.rows() << endl;
	cout << jacobian.cols() << endl;
	cout << "W" << endl;
	cout << W.rows() << endl;
	cout << W.cols() << endl;
	cout << "qdot" << endl;
	cout << qdot.rows() << endl;
	cout << qdot.cols() << endl;
	cout << "Q" << endl;
	cout << Q.rows() << endl;
	cout << Q.cols() << endl;
	cout << lambda.rows() << endl;
	cout << lambda.cols() << endl;
	**/

	// cout << legal_forces << endl;
	
	for (int i = 1; i < mParticles.size(); i++) {
		mParticles[i]->fhat[0] = legal_forces((i - 1) * 3);
		mParticles[i]->fhat[1] = legal_forces((i - 1) * 3 + 1);
		mParticles[i]->fhat[2] = legal_forces((i - 1) * 3 + 2);
	}
	std::vector<Eigen::VectorXd> derivatives;
	derivatives.resize(mParticles.size()-1);

	for (int i = 1; i < mParticles.size(); i++) {
		derivatives[i-1] = solver.solve_X_dot(mParticles[i]);
		mParticles[i]->mPosition += Eigen::Vector3d(derivatives[i-1][0], derivatives[i - 1][1], derivatives[i - 1][2]) * mTimeStep;
		mParticles[i]->mVelocity += Eigen::Vector3d(derivatives[i-1][3], derivatives[i - 1][4], derivatives[i - 1][5]) * mTimeStep;
	}

	for (int i = 1; i < forces.size(); i++) {
		delete forces[i];
	}

	forces.resize(1);
}

