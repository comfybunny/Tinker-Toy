#include "constraint.h"
#include <fstream>
#include <iostream>

using namespace std;

Constraint::Constraint(Particle* meow, Particle* ruff){
	particle1 = meow;
	particle2 = ruff;
	Eigen::Vector3d x = particle1->mPosition - particle2->mPosition;
	radius2 = x.dot(x);
}

Particle* Constraint::getParticle1(){
	return particle1;
}

Particle* Constraint::getParticle2(){
	return particle2;
}

Eigen::Vector3d Constraint::dCdx1(){
	return particle1->mPosition-particle2->mPosition;
}

Eigen::Vector3d Constraint::dCdx2(){
	return particle2->mPosition - particle1->mPosition;
}

Eigen::Vector3d Constraint::dCdotdx1(){
	return particle1->mVelocity - particle2->mVelocity;
}

Eigen::Vector3d Constraint::dCdotdx2(){
	return particle2->mVelocity - particle1->mVelocity;
}

float Constraint::C(){
	Eigen::Vector3d x = particle1->mPosition-particle2->mPosition;
	return 0.5*(x.dot(x)-radius2);
}

float Constraint::Cdot(){
	Eigen::Vector3d x = particle1->mPosition - particle2->mPosition;
	Eigen::Vector3d v = particle1->mVelocity - particle2->mVelocity;
	return x.dot(v);
}

