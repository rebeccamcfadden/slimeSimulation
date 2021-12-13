#include <iostream>

#define GLEW_STATIC
#include <GL/glew.h>
#include <GLFW/glfw3.h>

#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "SlimeParticle.h"
#include "Shape.h"
#include "Texture.h"
#include "Program.h"
#include "MatrixStack.h"
#include "GLSL.h"

using namespace std; 
using namespace Eigen;


std::shared_ptr<Shape> SlimeParticle::slimeShape;
std::shared_ptr<Texture> SlimeParticle::slimeTexture;

SlimeParticle::SlimeParticle() : Particle()
{
	dead = false;
}

SlimeParticle::SlimeParticle(Eigen::Vector3d x0, Eigen::Vector3d v0) : Particle(slimeShape, slimeTexture)
{
	this->x0 = x0;
	this->v0 = v0;
	this->x = x0;
	this->v = v0;

	dead = false;
	this->r = 0.1;
}

SlimeParticle::~SlimeParticle()
{
}

void SlimeParticle::init()
{
}

void SlimeParticle::tare()
{
}

void SlimeParticle::reset()
{
}

bool SlimeParticle::step(double h, const Eigen::Vector3d& grav)
{
	v += h * grav;
	x += h * v;
	t += h;
	if (t > 10.0 || x[1] < -0.1) {
		dead = true;
		return true;
	}
	return false;
}

void SlimeParticle::draw(std::shared_ptr<MatrixStack> MV, const std::shared_ptr<Program> p) const
{
	Particle::draw(MV, p);
}
