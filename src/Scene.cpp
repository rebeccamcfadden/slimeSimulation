#include <iostream>

#include "Scene.h"
#include "SlimeParticle.h"
#include "Shape.h"
#include "Slime.h"
#include "Texture.h"
#include "Program.h"

using namespace std;
using namespace Eigen;

Scene::Scene() :
	t(0.0),
	h(1e-2),
	grav(0.0, 0.0, 0.0)
{
}

Scene::~Scene()
{
}

void Scene::load(const string &RESOURCE_DIR)
{
	// Units: meters, kilograms, seconds
	h = 2e-3;
	
	grav << 0.0, -9.8 * 2.9, 0.0;

	int x = 8;
	float mass = 0.1;
	float stiffness = 2e1;
	Vector3d x000(-0.25, 0.5, -0.25);
	Vector3d x010(-0.25, 1.0, -0.25);
	Vector3d x100(0.25, 0.5, -0.25);
	Vector3d x110(0.25, 1.0, -0.25);
	Vector3d x001(-0.25, 0.5, 0.25);
	Vector3d x011(-0.25, 1.0, 0.25);
	Vector3d x101(0.25, 0.5, 0.25);
	Vector3d x111(0.25, 1.0, 0.25);
	slime = make_shared<Slime>(x, x, x, x000, x010, x100, x110, x001, x011, x101, x111, mass, stiffness);
	
	auto textureKd = make_shared<Texture>();
	textureKd->setFilename(RESOURCE_DIR + "slime.png");
	textureKd->init();
	textureKd->setUnit(1); // Bind to unit 1
	textureKd->setWrapModes(GL_CLAMP_TO_EDGE, GL_CLAMP_TO_EDGE);
	slime->setTexture(textureKd);
	SlimeParticle::slimeTexture = textureKd;

	SlimeParticle::slimeShape = make_shared<Shape>();
	SlimeParticle::slimeShape->loadMesh(RESOURCE_DIR + "slimeparticle.obj");
	
	//auto sphere = make_shared<Particle>(sphereShape);
	//spheres.push_back(sphere);
	//sphere->r = 0.1;
	//sphere->x = Vector3d(0.0, 0.2, 0.0);
}

void Scene::init()
{
	if (SlimeParticle::slimeShape) SlimeParticle::slimeShape->init();
	slime->init();
}

void Scene::tare()
{
	for(int i = 0; i < (int)spheres.size(); ++i) {
		spheres[i]->tare();
	}
	slime->tare();
}

void Scene::reset()
{
	t = 0.0;
	for(int i = 0; i < (int)spheres.size(); ++i) {
		spheres[i]->reset();
	}
	slime->reset();
}

void Scene::step()
{
	t += h;
	
	// Move the sphere
	 if(!spheres.empty()) {
	 	auto s = spheres.front();
	 	Vector3d x0 = s->x;
	 	s->x(2) = 0.5 * sin(t);
	 }	
	slime->step(h, grav, spheres);
}

void Scene::draw(shared_ptr<MatrixStack> MV, const shared_ptr<Program> prog, const shared_ptr<Program> prog2, const shared_ptr<Program> progSimple)
{
	glUniform3fv(prog->getUniform("kdFront"), 1, Vector3f(1.0, 1.0, 1.0).data());
	for(int i = 0; i < (int)spheres.size(); ++i) {
		spheres[i]->draw(MV, prog);
	}
	slime->draw(MV, prog, prog2, progSimple);
}
