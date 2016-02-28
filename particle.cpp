#include "particle.h"
#include <fstream>
#include <iostream>

#if defined(__APPLE__) && defined(__MACH__)
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

using namespace std;

Particle::Particle(float x, float y){
	Particle();
	mPosition[0] = x;
	mPosition[y] = y;
}

void Particle::draw() {
    glColor4d(mColor[0], mColor[1], mColor[2], mColor[3]);
    glPushMatrix();
    glTranslated(mPosition[0], mPosition[1], mPosition[2]);
    glutSolidSphere(0.01, 20, 20);
    glPopMatrix();
}

void Particle::update_accumulated_forces(int indexNum, std::vector<Force*> forces_list){
	// cout << forces_list.size()<< endl;
	for (std::vector<Force*>::iterator i = forces_list.begin(); i != forces_list.end(); ++i) {
		Force* currForce = *i;
		std::vector<int> currForceParticlesImpacted = currForce->getParticlesImpacted();
		// check to see if force is applicable to this particle
		if (std::find(currForceParticlesImpacted.begin(), currForceParticlesImpacted.end(), indexNum) != currForceParticlesImpacted.end()){
			mAccumulatedForce += currForce->calculateForceAdded(this);
		}
	}
	// cout << (*forces_list.begin())->getAcceleration() << endl;
	
}
