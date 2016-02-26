#ifndef GRAVITY_H
#define GRAVITY_H

#include <Eigen/Dense>
#include <vector>
#include <string>
#include "force.h"
#include "particle.h"

class Gravity : public Force{
public:
	Gravity();
	virtual Eigen::Vector3d calculateForceAdded(Particle* particle);
private:
	Eigen::Vector3d gravity;
};

#endif