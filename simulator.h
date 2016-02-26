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

// class containing objects to be simulated
class Simulator {
public:
    Simulator();
        
    void simulate();
    
    int getNumParticles();
    
    Particle* getParticle(int);
    
    double getTimeStep();
    
    void reset();

	void addForce(Force* newForce);

private:
    double mTimeStep;       // time step
    std::vector<Particle> mParticles;
	std::vector<Force*> forces;
	Gravity gravity;
};


#endif  // SIMULATOR_H
