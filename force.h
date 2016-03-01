#ifndef FORCE_H
#define FORCE_H

#include <Eigen/Dense>
#include <vector>
#include <string>
#include "particle.h"
/* This is called a "forward declaration".  We use it to tell the compiler that the
identifier "Particle" will from now on stand for a class, and this class will be defined
later.  We will not be able to make any use of "Particle" before it has been defined, but
we will at least be able to declare pointers to it.
class Particle; */

// class for a force
class Force{

	// public functions
public:
	Force(Eigen::Vector3d accel);
	std::vector<int> getParticlesImpacted();
	Eigen::Vector3d getAcceleration();
	void addParticlesImpacted(int particleNum);
	void resetParticlesImpacted();
private:
	std::vector<int> particles_impacted;
	Eigen::Vector3d acceleration;
};

#endif