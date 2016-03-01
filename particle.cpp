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
	mMass = 1.0;
	mPosition.setZero();
	mVelocity.setZero();
	mAccumulatedForce.setZero();
	fhat.setZero();
	mColor << 0.9, 0.2, 0.2, 1.0; // Red
	mPosition[0] = x;
	mPosition[1] = y;
}

void Particle::draw() {
    glColor4d(mColor[0], mColor[1], mColor[2], mColor[3]);
    glPushMatrix();
    glTranslated(mPosition[0], mPosition[1], mPosition[2]);
    glutSolidSphere(0.01, 20, 20);
    glPopMatrix();
}

void Particle::addForce(Eigen::Vector3d toAdd){
	mAccumulatedForce += toAdd;
}