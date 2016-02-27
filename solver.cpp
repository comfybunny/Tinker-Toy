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

			toReturn[3] = particle->legal_force[0] / particle->mMass;
			toReturn[4] = particle->legal_force[1] / particle->mMass;
			toReturn[5] = particle->legal_force[2] / particle->mMass;

			break;
		case midpoint_method:
			cout << "HAHA I DIDNT DO IT" << endl;
			break;
	}
	return toReturn;
}

#endif