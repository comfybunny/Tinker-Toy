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

	Eigen::Vector3d getForceEnd();

	bool getEditingForceBoolean();

	void toggleEditForceBoolean();

	void setForceEnd(Eigen::Vector3d newForce);

	void updateSelectedParticle(Eigen::Vector3d click_pt);

	void addForce(Eigen::Vector3d mouseVector);

private:
    double mTimeStep;       // time step
    std::vector<Particle*> mParticles;
	std::vector<Force*> forces;
	std::vector<Constraint> constraints;
	Force gravity = Force(Eigen::Vector3d(0.0, -9.8, 0.0));
	Solver solver;
	Eigen::MatrixXd W;
	int selected_particle;
	bool feedback;
	double ks;
	double kd;
	Eigen::Vector3d force_end;
	bool editing_force;

};


#endif  // SIMULATOR_H
