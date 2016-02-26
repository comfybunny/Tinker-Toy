#include "particle.h"
#include <vector>
#include <string>

class Solver {

public:
	Solver();

	void solve_X_dot(Particle* particle);

	enum Integrator {explicit_euler, midpoint_method};
	
private:
	Integrator type;
};