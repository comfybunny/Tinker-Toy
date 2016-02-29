#ifndef SIMULATOR_H
#define SIMULATOR_H

#define EIGEN_DONT_VECTORIZE
#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT

#include <Eigen/Dense>
#include <vector>
#include <string>

#include "particle.h"
#include "force.h"
#include "gravity.h"
#include "solver.h"
#include "constraint.h"

// class containing objects to be simulated
class Simulator {
public:
    Simulator();
        
    void simulate();
    
    int getNumParticles();
    
    Particle* getParticle(int);
    
    double getTimeStep();
    
    void reset();

	int getSelectedParticle();

	void addParticle(float x_pos, float y_pos);

	bool hasFeedback();

	void toggleFeedback();

	void toggleSolver();

	char* getSolverName();

private:
    double mTimeStep;       // time step
    std::vector<Particle*> mParticles;
	std::vector<Force*> forces;
	std::vector<Constraint> constraints;
	Gravity gravity;
	Solver solver;
	Eigen::MatrixXd W;
	int selected_particle;
	bool feedback;
	double ks;
	double kd;
};


#endif  // SIMULATOR_H
