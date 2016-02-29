#include "particle.h"
#include <vector>
#include <string>

class Solver {

public:
	Solver();

	Eigen::VectorXd solve_X_dot(Particle* particle);

	enum Integrator {explicit_euler, midpoint_method};

	void toggle();

	char* getSolverName();
	
private:
	Integrator type;
};