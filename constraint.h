#include <Eigen/Dense>
#include <vector>
#include <string>
#include "particle.h"

class Constraint {

public:
	Constraint(Particle* meow, Particle* ruff);
	Particle* getParticle1();
	Particle* getParticle2();
	Eigen::Vector3d dCdx1();
	Eigen::Vector3d dCdx2();
	Eigen::Vector3d dCdotdx1();
	Eigen::Vector3d dCdotdx2();

private:
	Particle* particle1;
	Particle* particle2;
};