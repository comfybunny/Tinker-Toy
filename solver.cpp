#ifndef SOLVER_H
#define SOLVER_H

#include "solver.h"
#include <fstream>
#include <iostream>

using namespace std;

Solver::Solver() {
	type = explicit_euler;
}

Eigen::VectorXd Solver::solve_X_dot(Particle* particle) {

	Eigen::VectorXd toReturn(6);

	switch (type) {
		case explicit_euler:

			toReturn[0] = particle->mVelocity[0];
			toReturn[1] = particle->mVelocity[1];
			toReturn[2] = particle->mVelocity[2];

			toReturn[3] = (particle->fhat[0] + particle->mAccumulatedForce[0]) / particle->mMass;
			toReturn[4] = (particle->fhat[1] + particle->mAccumulatedForce[1]) / particle->mMass;
			toReturn[5] = (particle->fhat[2] + particle->mAccumulatedForce[2]) / particle->mMass;

			break;
		case midpoint_method:

			Eigen::VectorXd euler_step(6);
			euler_step[0] = particle->mVelocity[0];
			euler_step[1] = particle->mVelocity[1];
			euler_step[2] = particle->mVelocity[2];
			euler_step[3] = (particle->fhat[0] + particle->mAccumulatedForce[0]) / particle->mMass;
			euler_step[4] = (particle->fhat[1] + particle->mAccumulatedForce[1]) / particle->mMass;
			euler_step[5] = (particle->fhat[2] + particle->mAccumulatedForce[2]) / particle->mMass;

			Eigen::VectorXd deltaXdiv2 = euler_step*0.0003 / 2.0;

			Eigen::VectorXd xPlusHalf(6);
			xPlusHalf[0] = particle->mPosition[0];
			xPlusHalf[1] = particle->mPosition[1];
			xPlusHalf[2] = particle->mPosition[2];
			xPlusHalf[3] = particle->mVelocity[0];
			xPlusHalf[4] = particle->mVelocity[1];
			xPlusHalf[5] = particle->mVelocity[2];

			xPlusHalf = xPlusHalf + deltaXdiv2;

			toReturn[0] = xPlusHalf[3];
			toReturn[1] = xPlusHalf[4];
			toReturn[2] = xPlusHalf[5];

			toReturn[3] = (particle->fhat[0] + particle->mAccumulatedForce[0]) / particle->mMass;
			toReturn[4] = (particle->fhat[1] + particle->mAccumulatedForce[1]) / particle->mMass;
			toReturn[5] = (particle->fhat[2] + particle->mAccumulatedForce[2]) / particle->mMass;

			break;
	}
	return toReturn;
}

void Solver::toggle(){
	switch (type) {
		case explicit_euler:
			type = midpoint_method;
			break;
		case midpoint_method:
			type = explicit_euler;
			break;
	}
}

char * Solver::getSolverName(){
	switch (type) {
		case explicit_euler:
			return "explicit euler";
		case midpoint_method:
			return "midpoint method";
	}
}

#endif