#include <iostream>

#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <omp.h>

#include "Slime.h"
#include "SlimeParticle.h"
#include "Spring.h"
#include "MatrixStack.h"
#include "Texture.h"
#include "Program.h"
#include "GLSL.h"

using namespace std;
using namespace Eigen;

shared_ptr<Spring> _createSpring(const shared_ptr<Particle> p0, const shared_ptr<Particle> p1, double E)
{
	auto s = make_shared<Spring>(p0, p1);
	s->E = E;
	Vector3d x0 = p0->x;
	Vector3d x1 = p1->x;
	Vector3d dx = x1 - x0;
	s->L = dx.norm();
	return s;
}
void Slime::_updateTexBuf(vector<float>& _texBuf, Vector2d offset, float width, float height) {
	// left
	Vector2d bottomLeft = Vector2d(0.0, 0.0) + offset;
	_interpolateTexCoords(
		_texBuf, 
		bottomLeft, 
		Vector2d(0.25, 0.0) + bottomLeft, 
		Vector2d(0.0, 0.25) + bottomLeft, 
		Vector2d(0.25, 0.25) + bottomLeft, 
		width, height);
	// right
	bottomLeft = Vector2d(0.5, 0.0) + offset;
	_interpolateTexCoords(
		_texBuf, 
		Vector2d(0.25, 0.0) + bottomLeft,
		bottomLeft,  
		Vector2d(0.25, 0.25) + bottomLeft, 
		Vector2d(0.0, 0.25) + bottomLeft, 
		width, height);
	// back
	bottomLeft = Vector2d(0.75, 0.0) + offset;
	_interpolateTexCoords(
		_texBuf, 
		bottomLeft, 
		Vector2d(0.25, 0.0) + bottomLeft, 
		Vector2d(0.0, 0.25) + bottomLeft, 
		Vector2d(0.25, 0.25) + bottomLeft, 
		width, height);
	// front
	bottomLeft = Vector2d(0.25, 0.0) + offset;
	_interpolateTexCoords(
		_texBuf, 
		bottomLeft, 
		Vector2d(0.0, 0.25) + bottomLeft, 
		Vector2d(0.25, 0.0) + bottomLeft, 
		Vector2d(0.25, 0.25) + bottomLeft, 
		width, height);
	// bottom
	bottomLeft = Vector2d(0.5, 0.25) + offset;
	_interpolateTexCoords(
		_texBuf, 
		bottomLeft, 
		Vector2d(0.25, 0.0) + bottomLeft, 
		Vector2d(0.0, 0.25) + bottomLeft, 
		Vector2d(0.25, 0.25) + bottomLeft, 
		width, height);
	// top 
	bottomLeft = Vector2d(0.25, 0.25) + offset;
	_interpolateTexCoords(
		_texBuf, 
		bottomLeft, 
		Vector2d(0.25, 0.0) + bottomLeft, 
		Vector2d(0.0, 0.25) + bottomLeft, 
		Vector2d(0.25, 0.25) + bottomLeft, 
		width, height);
}
	
void Slime::_interpolateTexCoords(vector<float>& _texBuf, Vector2d p00, Vector2d p01, Vector2d p10, Vector2d p11, float width, float height) {
	Vector2d xdif = (p01 - p00) / (width - 1.0);
	Vector2d ydif = (p10 - p00) / (height - 1.0);
	for (int j = 0; j < height - 1.0; j++) {
		for (int i = 0; i < width - 1.0; i++) {
			Vector2d u = p00 + xdif * i + ydif * j;
			Vector2d u1 = u + xdif;
			Vector2d v = u + ydif;
			Vector2d v1 = v + xdif;

			// 0 1 2
			_texBuf.push_back(u[0]);
			_texBuf.push_back(u[1]);
			_texBuf.push_back(u1[0]);
			_texBuf.push_back(u1[1]);
			_texBuf.push_back(v[0]);
			_texBuf.push_back(v[1]);

			// 1 2 3
			_texBuf.push_back(u1[0]);
			_texBuf.push_back(u1[1]);
			_texBuf.push_back(v[0]);
			_texBuf.push_back(v[1]);
			_texBuf.push_back(v1[0]);
			_texBuf.push_back(v1[1]);
		}
	}
}

Slime::Slime(int _x, int _y, int _z,
		const Eigen::Vector3d &x000,
		const Eigen::Vector3d &x010,
		const Eigen::Vector3d &x100,
		const Eigen::Vector3d &x110,
		const Eigen::Vector3d &x001,
		const Eigen::Vector3d &x011,
		const Eigen::Vector3d &x101,
		const Eigen::Vector3d &x111,
		double mass,
		double stiffness)
{
	assert(_x > 1);
	assert(_y > 1);
	assert(_z > 1);
	assert(mass > 0.0);
	assert(stiffness > 0.0);
	
	this->xdim = _x;
	this->ydim = _y;
	this->zdim = _z;
	this->c = 8.0;
	this->d = 5e-4;

	this->jumpFrame = false;
	this->jumpInt = -1;

	this->moveFrame = false;
	this->moveInt = -1;

	this->fixInvertFrame = false;

	float strucStiffness = stiffness / 15.0;
	float bendStiffness = stiffness * 2.0;
	float shearStiffness = stiffness / 3.0;
	float fixStiffness = stiffness * 5;
	float outerMultiplier = 2.0;
	
	// Create particles - Trilinear interpolation distribution
	// fill particle cube in x then z then y (bottom to top)
	n = 0;
	double r = 0.02; // Used for collisions
	int nParticles = xdim * ydim * zdim;
	int nVerts = (2 * xdim * ydim) + (2 * ydim * zdim) + (2 * xdim * zdim);
	for (int j = 0; j < xdim; j++) {
		// y - last
		double v = j / (ydim - 1.0);
		Vector3d x00 = (1-v)*x000 + v*x100;
		Vector3d x01 = (1-v)*x001 + v*x101;
		Vector3d x10 = (1-v)*x010 + v*x110;
		Vector3d x11 = (1-v)*x011 + v*x111;

		for (int k = 0; k < zdim; k++) {
			// z - second
			double w = k / (zdim - 1.0);
			Vector3d x0 = (1-w)*x00 + w*x10;
			Vector3d x1 = (1-w)*x01 + w*x11;

			for (int i = 0; i < xdim; i++) {
				// x - first
				double u = i / (xdim - 1.0);
				Vector3d x = (1-u)*x0 + u*x1;
				auto p = make_shared<Particle>();
				if (v == 0.0) {
					if (w == 0.0) {
						if (u == 0.0) particle000 = p;
						else if (u == 1.0) particle100 = p;
					}
					else if (w == 1.0) {
						if (u == 0.0) particle001 = p;
						else if (u == 1.0) particle101 = p;
					}
				}
				else if (v == 1.0) {
					if (w == 0.0) {
						if (u == 0.0) particle010 = p;
						else if (u == 1.0) particle110 = p;
					}
					else if (w == 1.0) {
						if (u == 0.0) particle011 = p;
						else if (u == 1.0) particle111 = p;
					}
				}
				particles.push_back(p);
				p->r = r;
				p->x = x;
				p->v << 0.0, 0.0, 0.0;
				p->m = mass/(nParticles);
				// Determine particles for mesh verts
				if(u == 0.0 || u == 1.0 || w == 0.0 || w == 1.0 || v == 1.0) {
					p->outer = true;
				}
				if (j == 0) {
					p->outer = true;
					p->bottom = true;
				}
				p->i = n;
				n += 3;
			}
		}
	}
	
	// Structural Springs - (6 connections per node, unless surface node)
	{
		// x springs
		for (int j = 0; j < ydim; j++) {
			for (int k = 0; k < zdim; k++) {
				for (int i = 0; i < xdim - 1; i++) {
					int p0 = (k * xdim) + (j * xdim * zdim) + i;
					int p1 = p0 + 1;
					bool outer = particles[p0]->outer || particles[p1]->outer;
					springs.push_back(_createSpring(particles[p0], particles[p1], outer ? strucStiffness / outerMultiplier : strucStiffness));
				}
			}
		}
		// z springs
		for (int j = 0; j < ydim; j++) {
			for (int i = 0; i < xdim; i++) {
				for (int k = 0; k < zdim - 1; k++) {
					int p0 = (k * xdim) + (j * xdim * zdim) + i;
					int p1 = p0 + xdim;
					bool outer = particles[p0]->outer || particles[p1]->outer;
					springs.push_back(_createSpring(particles[p0], particles[p1], outer ? strucStiffness / outerMultiplier : strucStiffness));
				}
			}
		}
		// y springs
		for (int k = 0; k < zdim; k++) {
			for (int i = 0; i < xdim; i++) {
				for (int j = 0; j < ydim - 1; j++) {
					int p0 = (k * xdim) + (j * xdim * zdim) + i;
					int p1 = p0 + (xdim * zdim);
					bool outer = particles[p0]->outer || particles[p1]->outer;
					springs.push_back(_createSpring(particles[p0], particles[p1], outer ? strucStiffness / outerMultiplier : strucStiffness));
				}
			}
		}
	}
	
	// Shear springs - (12 connections per node, unless surface node)
	// 3 passes - treat every dimension as a plane and take chunks of 2x2
	{
		//x-z plane
		for (int j = 0; j < ydim; j++) {
			for (int k = 0; k < zdim - 1; k++) {
				for (int i = 0; i < xdim - 1; i++) {
					// x - z
					int p00 = (k * xdim) + (j * xdim * zdim) + i;
					int p01 = p00 + xdim;
					int p10 = p00 + 1;
					int p11 = p01 + 1;
					bool outer = particles[p00]->outer || particles[p01]->outer || particles[p10]->outer || particles[p11]->outer;
					springs.push_back(_createSpring(particles[p00], particles[p11], outer ? shearStiffness / outerMultiplier : shearStiffness));
					springs.push_back(_createSpring(particles[p10], particles[p01], outer ? shearStiffness / outerMultiplier : shearStiffness));
				}
			}
		}
		//x-y plane
		for (int k = 0; k < zdim; k++) {
			for (int j = 0; j < ydim - 1; j++) {
				for (int i = 0; i < xdim - 1; i++) {
					// x - y
					int p00 = (k * xdim) + (j * xdim * zdim) + i;
					int p01 = p00 + (xdim * zdim);
					int p10 = p00 + 1;
					int p11 = p01 + 1;
					bool outer = particles[p00]->outer || particles[p01]->outer || particles[p10]->outer || particles[p11]->outer;
					springs.push_back(_createSpring(particles[p00], particles[p11], outer ? shearStiffness / outerMultiplier : shearStiffness));
					springs.push_back(_createSpring(particles[p10], particles[p01], outer ? shearStiffness / outerMultiplier : shearStiffness));
				}
			}
		}
		//z-y plane
		for (int i = 0; i < xdim; i++) {
			for (int j = 0; j < ydim - 1; j++) {
				for (int k = 0; k < zdim - 1; k++) {
					// z - y
					int p00 = (k * xdim) + (j * xdim * zdim) + i;
					int p01 = p00 + (xdim * zdim);
					int p10 = p00 + xdim;
					int p11 = p01 + xdim;
					bool outer = particles[p00]->outer || particles[p01]->outer || particles[p10]->outer || particles[p11]->outer;
					springs.push_back(_createSpring(particles[p00], particles[p11], outer ? shearStiffness / outerMultiplier : shearStiffness));
					springs.push_back(_createSpring(particles[p10], particles[p01], outer ? shearStiffness / outerMultiplier : shearStiffness));
				}
			}
		}
	}
	
	// Bending Springs - (6 connections per node, unless surface node)
	// Every node connected to its second neighbor in every direction
	{
		// x bend springs
		for (int j = 0; j < ydim; j++) {
			for (int k = 0; k < zdim; k++) {
				for (int i = 0; i < xdim - 2; i++) {
					int p0 = (k * xdim) + (j * xdim * zdim) + i;
					int p1 = p0 + 2;
					bool outer = particles[p0]->outer || particles[p1]->outer;
					springs.push_back(_createSpring(particles[p0], particles[p1], outer ? bendStiffness / outerMultiplier : bendStiffness));
				}
			}
		}
		// z bend springs
		for (int j = 0; j < ydim; j++) {
			for (int i = 0; i < xdim; i++) {
				for (int k = 0; k < zdim - 2; k++) {
					int p0 = (k * xdim) + (j * xdim * zdim) + i;
					int p1 = p0 + (2 * xdim);
					bool outer = particles[p0]->outer || particles[p1]->outer;
					springs.push_back(_createSpring(particles[p0], particles[p1], outer ? bendStiffness / outerMultiplier : bendStiffness));
				}
			}
		}
		// y bend springs
		for (int k = 0; k < zdim; k++) {
			for (int i = 0; i < xdim; i++) {
				for (int j = 0; j < ydim - 2; j++) {
					int p0 = (k * xdim) + (j * xdim * zdim) + i;
					int p1 = p0 + (2 * xdim * zdim);
					bool outer = particles[p0]->outer || particles[p1]->outer;
					springs.push_back(_createSpring(particles[p0], particles[p1], outer ? bendStiffness / outerMultiplier : bendStiffness));
				}
			}
		}
	}

	// edge springs
	{
		// x edge springs
		fix_springs.push_back(_createSpring(particle000, particle100, fixStiffness));
		fix_springs.push_back(_createSpring(particle010, particle110, fixStiffness));
		fix_springs.push_back(_createSpring(particle001, particle101, fixStiffness));
		fix_springs.push_back(_createSpring(particle011, particle111, fixStiffness));
		// y edge springs
		fix_springs.push_back(_createSpring(particle000, particle010, fixStiffness));
		fix_springs.push_back(_createSpring(particle100, particle110, fixStiffness));
		fix_springs.push_back(_createSpring(particle001, particle011, fixStiffness));
		fix_springs.push_back(_createSpring(particle101, particle111, fixStiffness));
		// z edge springs
		fix_springs.push_back(_createSpring(particle000, particle001, fixStiffness));
		fix_springs.push_back(_createSpring(particle010, particle011, fixStiffness));
		fix_springs.push_back(_createSpring(particle100, particle101, fixStiffness));
		fix_springs.push_back(_createSpring(particle110, particle111, fixStiffness));
	}

	// face shear springs
	{
		// // bottom springs
		// fix_springs.push_back(_createSpring(particle000, particle101, fixStiffness));
		// fix_springs.push_back(_createSpring(particle001, particle100, fixStiffness));
		// // top springs
		// fix_springs.push_back(_createSpring(particle010, particle111, fixStiffness));
		// fix_springs.push_back(_createSpring(particle011, particle110, fixStiffness));
		// // right springs
		// fix_springs.push_back(_createSpring(particle001, particle111, fixStiffness));
		// fix_springs.push_back(_createSpring(particle101, particle011, fixStiffness));
		// // left springs
		// fix_springs.push_back(_createSpring(particle000, particle110, fixStiffness));
		// fix_springs.push_back(_createSpring(particle100, particle010, fixStiffness));
		// // front springs
		// fix_springs.push_back(_createSpring(particle100, particle111, fixStiffness));
		// fix_springs.push_back(_createSpring(particle110, particle101, fixStiffness));
		// // back springs
		// fix_springs.push_back(_createSpring(particle000, particle011, fixStiffness));
		// fix_springs.push_back(_createSpring(particle010, particle001, fixStiffness));
		// // inner springs
		// fix_springs.push_back(_createSpring(particle000, particle111, fixStiffness));
		// fix_springs.push_back(_createSpring(particle101, particle010, fixStiffness));
		// fix_springs.push_back(_createSpring(particle110, particle001, fixStiffness));
		// fix_springs.push_back(_createSpring(particle011, particle100, fixStiffness));
	}

	// Build system matrices and vectors
	M.resize(n,n);
	K.resize(n,n);
	D.resize(n, n);
	v.resize(n);
	f.resize(n);
	
	// Build vertex buffers
	texBuf.clear();
	eleBuf.clear();
	texBuf2.clear();
	eleBuf2.clear();
	posBuf.clear();
	posBuf2.clear();
	posBuf.resize((xdim - 1)* (xdim - 1) * 6 * 3 * 6);
	posBuf2.resize((xdim - 3)* (xdim - 3) * 6 * 3 * 6);
	updatePosNor();

	_updateTexBuf(texBuf, Vector2d(0.0, 0.5), xdim, xdim);
	_updateTexBuf(texBuf2, Vector2d(0.0, 0.0), xdim - 2, xdim - 2);

	// Elements (don't change)
	for (int i = 0; i < posBuf.size() / 3; i++) {
		eleBuf.push_back(i);
	}
	for (int i = 0; i < posBuf2.size() / 3; i++) {
		eleBuf2.push_back(i);
	}

	D.setZero();
	vector<trip> Dt;
	for (int i = 0; i < n; i++) {
		Dt.push_back(trip(i, i, d));
	}
	D.setFromTriplets(Dt.begin(), Dt.end());
}

Slime::~Slime() {
}

void Slime::tare()
{
	for(int k = 0; k < (int)particles.size(); ++k) {
		particles[k]->tare();
	}
}

void Slime::reset()
{
	for(int k = 0; k < (int)particles.size(); ++k) {
		particles[k]->reset();
	}
	updatePosNor();
}

Vector3d Slime::_calculateNor(const int& i, const int& j, const int& k, const int& plane) {
	// Each particle has four face neighbors
	//
	//      v1
	//     /|\
	// u0 /_|_\ u1
	//    \ | /
	//     \|/
	//      v0
	//
	// Use these four triangles to compute the normal
	int u, v;
	int cols, rows, sign;
	int pu0, pu1, pv0, pv1;
	int p = (j * xdim) + (k * xdim * zdim) + i;
	if (plane == 0) { // z-y plane (front / back)
		u = k;
		v = j;
		rows = ydim;
		cols = zdim;
		if (i == 0) sign = -1;
		else sign = 1;
		pu0 = p - (xdim);
		pu1 = p + (xdim);
		pv0 = p - (xdim * zdim);
		pv1 = p + (xdim * zdim);
	}
	else if (plane == 1) { // x-z plane (top / bottom)
		u = i;
		v = k;
		rows = zdim;
		cols = xdim;
		if (j == 0) sign = -1;
		else sign = 1;
		pu0 = p - (xdim * zdim);
		pu1 = p + (xdim * zdim);
		pv0 = p - (1);
		pv1 = p + (1);
	}
	else { // x-y plane (left / right)
		u = i;
		v = j;
		rows = ydim;
		cols = xdim;
		if (k == 0) sign = -1;
		else sign = 1;
		pu0 = p - 1;
		pu1 = p + 1;
		pv0 = p - (xdim);
		pv1 = p + (xdim);
	}
	Vector3d x = particles[p]->x;
	Vector3d xu0, xu1, xv0, xv1, dx0, dx1, c;
	Vector3d nor(0.0, 0.0, 0.0);
	int count = 0;
	// Top-right triangle
	if(u != cols-1 && v != rows-1) {
		xu1 = particles[pu1]->x;
		xv1 = particles[pv1]->x;
		dx0 = xu1 - x;
		dx1 = xv1 - x;
		c = dx0.cross(dx1);
		nor += c.normalized();
		++count;
	}
	// Top-left triangle
	if(u != 0 && v != rows-1) {
		xu1 = particles[pv1]->x;
		xv1 = particles[pu0]->x;
		dx0 = xu1 - x;
		dx1 = xv1 - x;
		c = dx0.cross(dx1);
		nor += c.normalized();
		++count;
	}
	// Bottom-left triangle
	if(u != 0 && v != 0) {
		xu1 = particles[pu0]->x;
		xv1 = particles[pv0]->x;
		dx0 = xu1 - x;
		dx1 = xv1 - x;
		c = dx0.cross(dx1);
		nor += c.normalized();
		++count;
	}
	// Bottom-right triangle
	if(u != cols-1 && v != 0) {
		xu1 = particles[pv0]->x;
		xv1 = particles[pu1]->x;
		dx0 = xu1 - x;
		dx1 = xv1 - x;
		c = dx0.cross(dx1);
		nor += c.normalized();
		++count;
	}
	nor /= count;
	nor *= sign;
	nor.normalize();
	return nor;
}

void _updateNor(Vector3d& p0, Vector3d& p1, Vector3d& p2, vector<float>& norBuf) {
	Vector3d u = p1 - p0;
	Vector3d v = p2 - p1;
	Vector3d n = u.cross(v);
	n.normalize();
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++)
			norBuf.push_back(n[j]);
	}
}

void Slime::updatePosNor() {
	// outer mesh
	norBuf.clear();
	int n = 0;
	{
		// left
		int k = 0;
		for (int j = 0; j < ydim - 1; j++) {
			for (int i = 0; i < xdim - 1; i++) {
				int u = (j * xdim) + (k * xdim * zdim) + i;
				int u1 = u + 1;
				int v = u + (xdim);
				int v1 = v + 1;
				
				Vector3d p0 = particles[u]->x;
				Vector3d p1 = particles[u1]->x;
				Vector3d p2 = particles[v]->x;
				Vector3d p3 = particles[v1]->x;

				// u, u1, v
				posBuf[3 * n] = p0[0];
				posBuf[3 * n + 1] = (p0[1]);
				posBuf[3 * n + 2] = (p0[2]);
				n++;
				posBuf[3 * n] = (p1[0]);
				posBuf[3 * n + 1] = (p1[1]);
				posBuf[3 * n + 2] = (p1[2]);
				n++;
				posBuf[3 * n] = (p2[0]);
				posBuf[3 * n + 1] = (p2[1]);
				posBuf[3 * n + 2] = (p2[2]);
				n++;
				_updateNor(p0, p1, p2, norBuf);

				// u1, v1, v
				posBuf[3 * n] = (p1[0]);
				posBuf[3 * n + 1] = (p1[1]);
				posBuf[3 * n + 2] = (p1[2]);
				n++;
				posBuf[3 * n] = (p2[0]);
				posBuf[3 * n + 1] = (p2[1]);
				posBuf[3 * n + 2] = (p2[2]);
				n++;
				posBuf[3 * n] = (p3[0]);
				posBuf[3 * n + 1] = (p3[1]);
				posBuf[3 * n + 2] = (p3[2]);
				n++;
				_updateNor(p3, p2, p1, norBuf);
			}
		}

		// right
		k = zdim - 1;
		for (int j = 0; j < ydim - 1; j++) {
			for (int i = 0; i < xdim - 1; i++) {
				int u = (j * xdim) + (k * xdim * zdim) + i;
				int u1 = u + 1;
				int v = u + (xdim);
				int v1 = v + 1;

				Vector3d p0 = particles[u]->x;
				Vector3d p1 = particles[u1]->x;
				Vector3d p2 = particles[v]->x;
				Vector3d p3 = particles[v1]->x;

				// u, u1, v
				posBuf[3 * n] = p0[0];
				posBuf[3 * n + 1] = (p0[1]);
				posBuf[3 * n + 2] = (p0[2]);
				n++;
				posBuf[3 * n] = (p1[0]);
				posBuf[3 * n + 1] = (p1[1]);
				posBuf[3 * n + 2] = (p1[2]);
				n++;
				posBuf[3 * n] = (p2[0]);
				posBuf[3 * n + 1] = (p2[1]);
				posBuf[3 * n + 2] = (p2[2]);
				n++;

				_updateNor(p2, p1, p0, norBuf);

				// u1, v1, v
				posBuf[3 * n] = (p1[0]);
				posBuf[3 * n + 1] = (p1[1]);
				posBuf[3 * n + 2] = (p1[2]);
				n++;
				posBuf[3 * n] = (p2[0]);
				posBuf[3 * n + 1] = (p2[1]);
				posBuf[3 * n + 2] = (p2[2]);
				n++;
				posBuf[3 * n] = (p3[0]);
				posBuf[3 * n + 1] = (p3[1]);
				posBuf[3 * n + 2] = (p3[2]);
				n++;

				_updateNor(p1, p2, p3, norBuf);
			}
		}

		//back
		int i = 0;
		for (int k = 0; k < zdim - 1; k++) {
			for (int j = 0; j < ydim - 1; j++) {
				int u = (j * xdim) + (k * xdim * zdim) + i;
				int u1 = u + xdim;
				int v = u + (xdim * zdim);
				int v1 = v + xdim;

				Vector3d p0 = particles[u]->x;
				Vector3d p1 = particles[u1]->x;
				Vector3d p2 = particles[v]->x;
				Vector3d p3 = particles[v1]->x;

				// u, u1, v
				posBuf[3 * n] = p0[0];
				posBuf[3 * n + 1] = (p0[1]);
				posBuf[3 * n + 2] = (p0[2]);
				n++;
				posBuf[3 * n] = (p1[0]);
				posBuf[3 * n + 1] = (p1[1]);
				posBuf[3 * n + 2] = (p1[2]);
				n++;
				posBuf[3 * n] = (p2[0]);
				posBuf[3 * n + 1] = (p2[1]);
				posBuf[3 * n + 2] = (p2[2]);
				n++;

				_updateNor(p0, p1, p2, norBuf);

				// u1, v1, v
				posBuf[3 * n] = (p1[0]);
				posBuf[3 * n + 1] = (p1[1]);
				posBuf[3 * n + 2] = (p1[2]);
				n++;
				posBuf[3 * n] = (p2[0]);
				posBuf[3 * n + 1] = (p2[1]);
				posBuf[3 * n + 2] = (p2[2]);
				n++;
				posBuf[3 * n] = (p3[0]);
				posBuf[3 * n + 1] = (p3[1]);
				posBuf[3 * n + 2] = (p3[2]);
				n++;

				_updateNor(p3, p2, p1, norBuf);
			}
		}

		// front
		i = xdim - 1;
		for (int k = 0; k < zdim - 1; k++) {
			for (int j = 0; j < ydim - 1; j++) {
				int u = (j * xdim) + (k * xdim * zdim) + i;
				int u1 = u + xdim;
				int v = u + (xdim * zdim);
				int v1 = v + xdim;

				Vector3d p0 = particles[u]->x;
				Vector3d p1 = particles[u1]->x;
				Vector3d p2 = particles[v]->x;
				Vector3d p3 = particles[v1]->x;

				// u, u1, v
				posBuf[3 * n] = p0[0];
				posBuf[3 * n + 1] = (p0[1]);
				posBuf[3 * n + 2] = (p0[2]);
				n++;
				posBuf[3 * n] = (p1[0]);
				posBuf[3 * n + 1] = (p1[1]);
				posBuf[3 * n + 2] = (p1[2]);
				n++;
				posBuf[3 * n] = (p2[0]);
				posBuf[3 * n + 1] = (p2[1]);
				posBuf[3 * n + 2] = (p2[2]);
				n++;

				_updateNor(p2, p1, p0, norBuf);

				// u1, v1, v
				posBuf[3 * n] = (p1[0]);
				posBuf[3 * n + 1] = (p1[1]);
				posBuf[3 * n + 2] = (p1[2]);
				n++;
				posBuf[3 * n] = (p2[0]);
				posBuf[3 * n + 1] = (p2[1]);
				posBuf[3 * n + 2] = (p2[2]);
				n++;
				posBuf[3 * n] = (p3[0]);
				posBuf[3 * n + 1] = (p3[1]);
				posBuf[3 * n + 2] = (p3[2]);
				n++;

				_updateNor(p1, p2, p3, norBuf);
			}
		}

		// bottom
		int j = 0;
		for (int i = 0; i < xdim - 1; i++) {
			for (int k = 0; k < zdim - 1; k++) {
				int u = (j * xdim) + (k * xdim * zdim) + i;
				int u1 = u + (xdim * zdim);
				int v = u + 1;
				int v1 = v + (xdim * zdim);

				Vector3d p0 = particles[u]->x;
				Vector3d p1 = particles[u1]->x;
				Vector3d p2 = particles[v]->x;
				Vector3d p3 = particles[v1]->x;

				// u, u1, v
				posBuf[3 * n] = p0[0];
				posBuf[3 * n + 1] = (p0[1]);
				posBuf[3 * n + 2] = (p0[2]);
				n++;
				posBuf[3 * n] = (p1[0]);
				posBuf[3 * n + 1] = (p1[1]);
				posBuf[3 * n + 2] = (p1[2]);
				n++;
				posBuf[3 * n] = (p2[0]);
				posBuf[3 * n + 1] = (p2[1]);
				posBuf[3 * n + 2] = (p2[2]);
				n++;

				_updateNor(p0, p1, p2, norBuf);

				// u1, v1, v
				posBuf[3 * n] = (p1[0]);
				posBuf[3 * n + 1] = (p1[1]);
				posBuf[3 * n + 2] = (p1[2]);
				n++;
				posBuf[3 * n] = (p2[0]);
				posBuf[3 * n + 1] = (p2[1]);
				posBuf[3 * n + 2] = (p2[2]);
				n++;
				posBuf[3 * n] = (p3[0]);
				posBuf[3 * n + 1] = (p3[1]);
				posBuf[3 * n + 2] = (p3[2]);
				n++;

				_updateNor(p3, p2, p1, norBuf);
			}
		}

		// top
		j = ydim - 1;
		for (int i = 0; i < xdim - 1; i++) {
			for (int k = 0; k < zdim - 1; k++) {
				int u = (j * xdim) + (k * xdim * zdim) + i;
				int u1 = u + (xdim * zdim);
				int v = u + 1;
				int v1 = v + (xdim * zdim);

				Vector3d p0 = particles[u]->x;
				Vector3d p1 = particles[u1]->x;
				Vector3d p2 = particles[v]->x;
				Vector3d p3 = particles[v1]->x;

				// u, u1, v
				posBuf[3 * n] = p0[0];
				posBuf[3 * n + 1] = (p0[1]);
				posBuf[3 * n + 2] = (p0[2]);
				n++;
				posBuf[3 * n] = (p1[0]);
				posBuf[3 * n + 1] = (p1[1]);
				posBuf[3 * n + 2] = (p1[2]);
				n++;
				posBuf[3 * n] = (p2[0]);
				posBuf[3 * n + 1] = (p2[1]);
				posBuf[3 * n + 2] = (p2[2]);
				n++;

				_updateNor(p2, p1, p0, norBuf);

				// u1, v1, v
				posBuf[3 * n] = (p1[0]);
				posBuf[3 * n + 1] = (p1[1]);
				posBuf[3 * n + 2] = (p1[2]);
				n++;
				posBuf[3 * n] = (p2[0]);
				posBuf[3 * n + 1] = (p2[1]);
				posBuf[3 * n + 2] = (p2[2]);
				n++;
				posBuf[3 * n] = (p3[0]);
				posBuf[3 * n + 1] = (p3[1]);
				posBuf[3 * n + 2] = (p3[2]);
				n++;

				_updateNor(p1, p2, p3, norBuf);
			}
		}
	}
	// inner mesh
	n = 0;
	norBuf2.clear();
	{
		// left
		int k = 1;
		for (int j = 1; j < ydim - 2; j++) {
			for (int i = 1; i < xdim - 2; i++) {
				int u = (j * xdim) + (k * xdim * zdim) + i;
				int u1 = u + 1;
				int v = u + (xdim);
				int v1 = v + 1;

				Vector3d p0 = particles[u]->x;
				Vector3d p1 = particles[u1]->x;
				Vector3d p2 = particles[v]->x;
				Vector3d p3 = particles[v1]->x;

				// u, u1, v
				posBuf2[3 * n] = p0[0];
				posBuf2[3 * n + 1] = (p0[1]);
				posBuf2[3 * n + 2] = (p0[2]);
				n++;
				posBuf2[3 * n] = (p1[0]);
				posBuf2[3 * n + 1] = (p1[1]);
				posBuf2[3 * n + 2] = (p1[2]);
				n++;
				posBuf2[3 * n] = (p2[0]);
				posBuf2[3 * n + 1] = (p2[1]);
				posBuf2[3 * n + 2] = (p2[2]);
				n++;
				_updateNor(p0, p1, p2, norBuf2);

				// u1, v1, v
				posBuf2[3 * n] = (p1[0]);
				posBuf2[3 * n + 1] = (p1[1]);
				posBuf2[3 * n + 2] = (p1[2]);
				n++;
				posBuf2[3 * n] = (p2[0]);
				posBuf2[3 * n + 1] = (p2[1]);
				posBuf2[3 * n + 2] = (p2[2]);
				n++;
				posBuf2[3 * n] = (p3[0]);
				posBuf2[3 * n + 1] = (p3[1]);
				posBuf2[3 * n + 2] = (p3[2]);
				n++;

				_updateNor(p3, p2, p1, norBuf2);
			}
		}

		// right
		k = zdim - 2;
		for (int j = 1; j < ydim - 2; j++) {
			for (int i = 1; i < xdim - 2; i++) {
				int u = (j * xdim) + (k * xdim * zdim) + i;
				int u1 = u + 1;
				int v = u + (xdim);
				int v1 = v + 1;

				Vector3d p0 = particles[u]->x;
				Vector3d p1 = particles[u1]->x;
				Vector3d p2 = particles[v]->x;
				Vector3d p3 = particles[v1]->x;

				// u, u1, v
				posBuf2[3 * n] = p0[0];
				posBuf2[3 * n + 1] = (p0[1]);
				posBuf2[3 * n + 2] = (p0[2]);
				n++;
				posBuf2[3 * n] = (p1[0]);
				posBuf2[3 * n + 1] = (p1[1]);
				posBuf2[3 * n + 2] = (p1[2]);
				n++;
				posBuf2[3 * n] = (p2[0]);
				posBuf2[3 * n + 1] = (p2[1]);
				posBuf2[3 * n + 2] = (p2[2]);
				n++;
				_updateNor(p2, p1, p0, norBuf2);

				// u1, v1, v
				posBuf2[3 * n] = (p1[0]);
				posBuf2[3 * n + 1] = (p1[1]);
				posBuf2[3 * n + 2] = (p1[2]);
				n++;
				posBuf2[3 * n] = (p2[0]);
				posBuf2[3 * n + 1] = (p2[1]);
				posBuf2[3 * n + 2] = (p2[2]);
				n++;
				posBuf2[3 * n] = (p3[0]);
				posBuf2[3 * n + 1] = (p3[1]);
				posBuf2[3 * n + 2] = (p3[2]);
				n++;

				_updateNor(p1, p2, p3, norBuf2);
			}
		}

		//back
		int i = 1;
		for (int k = 1; k < zdim - 2; k++) {
			for (int j = 1; j < ydim - 2; j++) {
				int u = (j * xdim) + (k * xdim * zdim) + i;
				int u1 = u + xdim;
				int v = u + (xdim * zdim);
				int v1 = v + xdim;

				Vector3d p0 = particles[u]->x;
				Vector3d p1 = particles[u1]->x;
				Vector3d p2 = particles[v]->x;
				Vector3d p3 = particles[v1]->x;

				// u, u1, v
				posBuf2[3 * n] = p0[0];
				posBuf2[3 * n + 1] = (p0[1]);
				posBuf2[3 * n + 2] = (p0[2]);
				n++;
				posBuf2[3 * n] = (p1[0]);
				posBuf2[3 * n + 1] = (p1[1]);
				posBuf2[3 * n + 2] = (p1[2]);
				n++;
				posBuf2[3 * n] = (p2[0]);
				posBuf2[3 * n + 1] = (p2[1]);
				posBuf2[3 * n + 2] = (p2[2]);
				n++;
				_updateNor(p0, p1, p2, norBuf2);

				// u1, v1, v
				posBuf2[3 * n] = (p1[0]);
				posBuf2[3 * n + 1] = (p1[1]);
				posBuf2[3 * n + 2] = (p1[2]);
				n++;
				posBuf2[3 * n] = (p2[0]);
				posBuf2[3 * n + 1] = (p2[1]);
				posBuf2[3 * n + 2] = (p2[2]);
				n++;
				posBuf2[3 * n] = (p3[0]);
				posBuf2[3 * n + 1] = (p3[1]);
				posBuf2[3 * n + 2] = (p3[2]);
				n++;

				_updateNor(p3, p2, p1, norBuf2);
			}
		}

		// front
		i = xdim - 2;
		for (int k = 1; k < zdim - 2; k++) {
			for (int j = 1; j < ydim - 2; j++) {
				int u = (j * xdim) + (k * xdim * zdim) + i;
				int u1 = u + xdim;
				int v = u + (xdim * zdim);
				int v1 = v + xdim;

				Vector3d p0 = particles[u]->x;
				Vector3d p1 = particles[u1]->x;
				Vector3d p2 = particles[v]->x;
				Vector3d p3 = particles[v1]->x;

				// u, u1, v
				posBuf2[3 * n] = p0[0];
				posBuf2[3 * n + 1] = (p0[1]);
				posBuf2[3 * n + 2] = (p0[2]);
				n++;
				posBuf2[3 * n] = (p1[0]);
				posBuf2[3 * n + 1] = (p1[1]);
				posBuf2[3 * n + 2] = (p1[2]);
				n++;
				posBuf2[3 * n] = (p2[0]);
				posBuf2[3 * n + 1] = (p2[1]);
				posBuf2[3 * n + 2] = (p2[2]);
				n++;
				_updateNor(p2, p1, p0, norBuf2);

				// u1, v1, v
				posBuf2[3 * n] = (p1[0]);
				posBuf2[3 * n + 1] = (p1[1]);
				posBuf2[3 * n + 2] = (p1[2]);
				n++;
				posBuf2[3 * n] = (p2[0]);
				posBuf2[3 * n + 1] = (p2[1]);
				posBuf2[3 * n + 2] = (p2[2]);
				n++;
				posBuf2[3 * n] = (p3[0]);
				posBuf2[3 * n + 1] = (p3[1]);
				posBuf2[3 * n + 2] = (p3[2]);
				n++;

				_updateNor(p1, p2, p3, norBuf2);
			}
		}

		// bottom
		int j = 1;
		for (int i = 1; i < xdim - 2; i++) {
			for (int k = 1; k < zdim - 2; k++) {
				int u = (j * xdim) + (k * xdim * zdim) + i;
				int u1 = u + (xdim * zdim);
				int v = u + 1;
				int v1 = v + (xdim * zdim);

				Vector3d p0 = particles[u]->x;
				Vector3d p1 = particles[u1]->x;
				Vector3d p2 = particles[v]->x;
				Vector3d p3 = particles[v1]->x;

				// u, u1, v
				posBuf2[3 * n] = p0[0];
				posBuf2[3 * n + 1] = (p0[1]);
				posBuf2[3 * n + 2] = (p0[2]);
				n++;
				posBuf2[3 * n] = (p1[0]);
				posBuf2[3 * n + 1] = (p1[1]);
				posBuf2[3 * n + 2] = (p1[2]);
				n++;
				posBuf2[3 * n] = (p2[0]);
				posBuf2[3 * n + 1] = (p2[1]);
				posBuf2[3 * n + 2] = (p2[2]);
				n++;
				_updateNor(p0, p1, p2, norBuf2);

				// u1, v1, v
				posBuf2[3 * n] = (p1[0]);
				posBuf2[3 * n + 1] = (p1[1]);
				posBuf2[3 * n + 2] = (p1[2]);
				n++;
				posBuf2[3 * n] = (p2[0]);
				posBuf2[3 * n + 1] = (p2[1]);
				posBuf2[3 * n + 2] = (p2[2]);
				n++;
				posBuf2[3 * n] = (p3[0]);
				posBuf2[3 * n + 1] = (p3[1]);
				posBuf2[3 * n + 2] = (p3[2]);
				n++;

				_updateNor(p3, p2, p1, norBuf2);
			}
		}

		// top
		j = ydim - 2;
		for (int i = 1; i < xdim - 2; i++) {
			for (int k = 1; k < zdim - 2; k++) {
				int u = (j * xdim) + (k * xdim * zdim) + i;
				int u1 = u + (xdim * zdim);
				int v = u + 1;
				int v1 = v + (xdim * zdim);

				Vector3d p0 = particles[u]->x;
				Vector3d p1 = particles[u1]->x;
				Vector3d p2 = particles[v]->x;
				Vector3d p3 = particles[v1]->x;

				// u, u1, v
				posBuf2[3 * n] = p0[0];
				posBuf2[3 * n + 1] = (p0[1]);
				posBuf2[3 * n + 2] = (p0[2]);
				n++;
				posBuf2[3 * n] = (p1[0]);
				posBuf2[3 * n + 1] = (p1[1]);
				posBuf2[3 * n + 2] = (p1[2]);
				n++;
				posBuf2[3 * n] = (p2[0]);
				posBuf2[3 * n + 1] = (p2[1]);
				posBuf2[3 * n + 2] = (p2[2]);
				n++;

				_updateNor(p2, p1, p0, norBuf2);

				// u1, v1, v
				posBuf2[3 * n] = (p1[0]);
				posBuf2[3 * n + 1] = (p1[1]);
				posBuf2[3 * n + 2] = (p1[2]);
				n++;
				posBuf2[3 * n] = (p2[0]);
				posBuf2[3 * n + 1] = (p2[1]);
				posBuf2[3 * n + 2] = (p2[2]);
				n++;
				posBuf2[3 * n] = (p3[0]);
				posBuf2[3 * n + 1] = (p3[1]);
				posBuf2[3 * n + 2] = (p3[2]);
				n++;

				_updateNor(p1, p2, p3, norBuf2);
			}
		}
	}

	// forward
	Vector3d u = particle100->x - particle101->x;
	Vector3d v = particle111->x - particle101->x;
	nForward = u.cross(v);
	nForward.normalize();

	// right
	u = particle110->x - particle111->x;
	v = particle011->x - particle111->x;
	nRight = u.cross(v);
	nRight.normalize();

	// up
	u = particle101->x - particle001->x;
	v = particle011->x - particle001->x;
	nUp = u.cross(v);
	nUp.normalize();
}

void Slime::step(double h, const Vector3d &grav, const vector< shared_ptr<Particle> > spheres)
{
	M.setZero();
	K.setZero();
	v.setZero();
	f.setZero();

	vector<trip> Mt;
	vector<trip> Kt;

	Eigen::Matrix3d I;
	I.setIdentity();

	for (auto p : particles) {
		int i = p->i;
		if (i == -1) continue;
		Mt.push_back(trip(i, i, p->m));
		Mt.push_back(trip(i+1, i+1, p->m));
		Mt.push_back(trip(i+2, i+2, p->m));
		v.segment<3>(i) = p->v;
		f.segment<3>(i) = p->m * grav;
	}

	for (auto s : springs) {
		int i0 = s->p0->i;
		int i1 = s->p1->i;
		if (i0 < 0 && i1 < 0) continue;

		Eigen::Vector3d fs;
		Eigen::Matrix3d Ks;
		s->computeForceStiffness(fs, Ks);
		if (i0 >= 0) {
			f.segment<3>(i0) += fs;
			Kt.push_back(trip(i0,		i0,		-Ks(0, 0)));
			Kt.push_back(trip(i0+1,		i0,		-Ks(1, 0)));
			Kt.push_back(trip(i0+2,		i0,		-Ks(2, 0)));
			Kt.push_back(trip(i0,		i0 + 1, -Ks(0, 1)));
			Kt.push_back(trip(i0 + 1,	i0 + 1, -Ks(1, 1)));
			Kt.push_back(trip(i0 + 2,	i0 + 1, -Ks(2, 1)));
			Kt.push_back(trip(i0,		i0 + 2, -Ks(0, 2)));
			Kt.push_back(trip(i0 + 1,	i0 + 2, -Ks(1, 2)));
			Kt.push_back(trip(i0 + 2,	i0 + 2, -Ks(2, 2)));
		}
		if (i1 >= 0) {
			f.segment<3>(i1) -= fs;
			Kt.push_back(trip(i1, i1, -Ks(0, 0)));
			Kt.push_back(trip(i1 + 1, i1, -Ks(1, 0)));
			Kt.push_back(trip(i1 + 2, i1, -Ks(2, 0)));
			Kt.push_back(trip(i1, i1 + 1, -Ks(0, 1)));
			Kt.push_back(trip(i1 + 1, i1 + 1, -Ks(1, 1)));
			Kt.push_back(trip(i1 + 2, i1 + 1, -Ks(2, 1)));
			Kt.push_back(trip(i1, i1 + 2, -Ks(0, 2)));
			Kt.push_back(trip(i1 + 1, i1 + 2, -Ks(1, 2)));
			Kt.push_back(trip(i1 + 2, i1 + 2, -Ks(2, 2)));
		}
		if (i0 >= 0 && i1 >= 0) {
			Kt.push_back(trip(i0, i1, Ks(0, 0)));
			Kt.push_back(trip(i0 + 1, i1, Ks(1, 0)));
			Kt.push_back(trip(i0 + 2, i1, Ks(2, 0)));
			Kt.push_back(trip(i0, i1 + 1, Ks(0, 1)));
			Kt.push_back(trip(i0 + 1, i1 + 1, Ks(1, 1)));
			Kt.push_back(trip(i0 + 2, i1 + 1, Ks(2, 1)));
			Kt.push_back(trip(i0, i1 + 2, Ks(0, 2)));
			Kt.push_back(trip(i0 + 1, i1 + 2, Ks(1, 2)));
			Kt.push_back(trip(i0 + 2, i1 + 2, Ks(2, 2)));
			Kt.push_back(trip(i1, i0, Ks(0, 0)));
			Kt.push_back(trip(i1 + 1, i0, Ks(1, 0)));
			Kt.push_back(trip(i1 + 2, i0, Ks(2, 0)));
			Kt.push_back(trip(i1, i0 + 1, Ks(0, 1)));
			Kt.push_back(trip(i1 + 1, i0 + 1, Ks(1, 1)));
			Kt.push_back(trip(i1 + 2, i0 + 1, Ks(2, 1)));
			Kt.push_back(trip(i1, i0 + 2, Ks(0, 2)));
			Kt.push_back(trip(i1 + 1, i0 + 2, Ks(1, 2)));
			Kt.push_back(trip(i1 + 2, i0 + 2, Ks(2, 2)));
		}
	}

	if (fixInvertFrame) {
		cout << "Fixing inverted springs!" << endl;
		for (auto s : fix_springs) {
			int i0 = s->p0->i;
			int i1 = s->p1->i;
			if (i0 < 0 && i1 < 0) continue;

			Eigen::Vector3d fs;
			Eigen::Matrix3d Ks;
			s->computeForceStiffness(fs, Ks);
			if (i0 >= 0) {
				f.segment<3>(i0) += fs;
				Kt.push_back(trip(i0,		i0,		-Ks(0, 0)));
				Kt.push_back(trip(i0+1,		i0,		-Ks(1, 0)));
				Kt.push_back(trip(i0+2,		i0,		-Ks(2, 0)));
				Kt.push_back(trip(i0,		i0 + 1, -Ks(0, 1)));
				Kt.push_back(trip(i0 + 1,	i0 + 1, -Ks(1, 1)));
				Kt.push_back(trip(i0 + 2,	i0 + 1, -Ks(2, 1)));
				Kt.push_back(trip(i0,		i0 + 2, -Ks(0, 2)));
				Kt.push_back(trip(i0 + 1,	i0 + 2, -Ks(1, 2)));
				Kt.push_back(trip(i0 + 2,	i0 + 2, -Ks(2, 2)));
			}
			if (i1 >= 0) {
				f.segment<3>(i1) -= fs;
				Kt.push_back(trip(i1, i1, -Ks(0, 0)));
				Kt.push_back(trip(i1 + 1, i1, -Ks(1, 0)));
				Kt.push_back(trip(i1 + 2, i1, -Ks(2, 0)));
				Kt.push_back(trip(i1, i1 + 1, -Ks(0, 1)));
				Kt.push_back(trip(i1 + 1, i1 + 1, -Ks(1, 1)));
				Kt.push_back(trip(i1 + 2, i1 + 1, -Ks(2, 1)));
				Kt.push_back(trip(i1, i1 + 2, -Ks(0, 2)));
				Kt.push_back(trip(i1 + 1, i1 + 2, -Ks(1, 2)));
				Kt.push_back(trip(i1 + 2, i1 + 2, -Ks(2, 2)));
			}
			if (i0 >= 0 && i1 >= 0) {
				Kt.push_back(trip(i0, i1, Ks(0, 0)));
				Kt.push_back(trip(i0 + 1, i1, Ks(1, 0)));
				Kt.push_back(trip(i0 + 2, i1, Ks(2, 0)));
				Kt.push_back(trip(i0, i1 + 1, Ks(0, 1)));
				Kt.push_back(trip(i0 + 1, i1 + 1, Ks(1, 1)));
				Kt.push_back(trip(i0 + 2, i1 + 1, Ks(2, 1)));
				Kt.push_back(trip(i0, i1 + 2, Ks(0, 2)));
				Kt.push_back(trip(i0 + 1, i1 + 2, Ks(1, 2)));
				Kt.push_back(trip(i0 + 2, i1 + 2, Ks(2, 2)));
				Kt.push_back(trip(i1, i0, Ks(0, 0)));
				Kt.push_back(trip(i1 + 1, i0, Ks(1, 0)));
				Kt.push_back(trip(i1 + 2, i0, Ks(2, 0)));
				Kt.push_back(trip(i1, i0 + 1, Ks(0, 1)));
				Kt.push_back(trip(i1 + 1, i0 + 1, Ks(1, 1)));
				Kt.push_back(trip(i1 + 2, i0 + 1, Ks(2, 1)));
				Kt.push_back(trip(i1, i0 + 2, Ks(0, 2)));
				Kt.push_back(trip(i1 + 1, i0 + 2, Ks(1, 2)));
				Kt.push_back(trip(i1 + 2, i0 + 2, Ks(2, 2)));
			}
		}
	}

	for (auto s : spheres) {
		for (auto p : particles) {
			auto [d, nor] = s->collide(p);
			if (d > 0) {
				int i = p->i;
				if (i == -1) continue;
				Eigen::Matrix3d I;
				I.setIdentity();
				f.segment<3>(i) += c * d * nor;
				Eigen::Matrix3d Ks = c * d * I;
				Kt.push_back(trip(i, i, -Ks(0, 0)));
				Kt.push_back(trip(i + 1, i, -Ks(1, 0)));
				Kt.push_back(trip(i + 2, i, -Ks(2, 0)));
				Kt.push_back(trip(i, i + 1, -Ks(0, 1)));
				Kt.push_back(trip(i + 1, i + 1, -Ks(1, 1)));
				Kt.push_back(trip(i + 2, i + 1, -Ks(2, 1)));
				Kt.push_back(trip(i, i + 2, -Ks(0, 2)));
				Kt.push_back(trip(i + 1, i + 2, -Ks(1, 2)));
				Kt.push_back(trip(i + 2, i + 2, -Ks(2, 2)));
			}
		}
	}

	// collisions
	vector<int> spawnParticles;

	// set collision with floor
	bool floorCollide = false;
	double cf = 35.0;
	int ind = 0;
	for (auto p : particles) {
		Vector3d dx = Vector3d(0, p->x[1], 0);
		float l = dx.norm();
		Vector3d nor(0, 1, 0);
		float d = p->r - l;
		if (d > 0) {
			int i = p->i;
			if (i == -1) continue;
			floorCollide = true;
			Eigen::Matrix3d I;
			I.setIdentity();
			f.segment<3>(i) += cf * d * nor;
			Eigen::Matrix3d Ks = cf * d * I;
			Kt.push_back(trip(i, i, -Ks(0, 0)));
			Kt.push_back(trip(i + 1, i, -Ks(1, 0)));
			Kt.push_back(trip(i + 2, i, -Ks(2, 0)));
			Kt.push_back(trip(i, i + 1, -Ks(0, 1)));
			Kt.push_back(trip(i + 1, i + 1, -Ks(1, 1)));
			Kt.push_back(trip(i + 2, i + 1, -Ks(2, 1)));
			Kt.push_back(trip(i, i + 2, -Ks(0, 2)));
			Kt.push_back(trip(i + 1, i + 2, -Ks(1, 2)));
			Kt.push_back(trip(i + 2, i + 2, -Ks(2, 2)));

			// spawn particle
			spawnParticles.push_back(ind);
		}
		ind++;
	}

	{
		// collisions with bounding box
		double cb = 30.0;
		vector<pair<Vector3d, Vector3d>> box{
			{Vector3d(-1, 0, 0), Vector3d(1, 0, 0)},
			{Vector3d(1, 0, 0), Vector3d(-1, 0, 0)},
			{Vector3d(0, 0, -1), Vector3d(0, 0, 1)},
			{Vector3d(0, 0, 1), Vector3d(0, 0, -1)},
			{Vector3d(0, 2, 0), Vector3d(0, -1, 0)}
		};
		for (auto& plane : box) {
			Vector3d point = plane.first;
			Vector3d nor = plane.second;
			int ind = 0;
			for (auto p : particles) {
				// distance to plane
				Vector3d dx = p->x - point;
				float l = dx.dot(nor);
				float d = p->r + 0.01 - l;
				if (d > 0) {
					int i = p->i;
					if (i == -1) continue;
					Eigen::Matrix3d I;
					I.setIdentity();
					f.segment<3>(i) += cb * d * nor;
					Eigen::Matrix3d Ks = cb * d * I;
					Kt.push_back(trip(i, i, -Ks(0, 0)));
					Kt.push_back(trip(i + 1, i, -Ks(1, 0)));
					Kt.push_back(trip(i + 2, i, -Ks(2, 0)));
					Kt.push_back(trip(i, i + 1, -Ks(0, 1)));
					Kt.push_back(trip(i + 1, i + 1, -Ks(1, 1)));
					Kt.push_back(trip(i + 2, i + 1, -Ks(2, 1)));
					Kt.push_back(trip(i, i + 2, -Ks(0, 2)));
					Kt.push_back(trip(i + 1, i + 2, -Ks(1, 2)));
					Kt.push_back(trip(i + 2, i + 2, -Ks(2, 2)));

					// spawn particle
					spawnParticles.push_back(ind);
				}
				ind++;
			}
		}
	}

	if (jumpFrame && floorCollide) {
		for (auto p : particles) {
			//if (!p->bottom) continue;
			int i = p->i;
			if (i == -1) continue;
			f.segment<3>(i) += -10 * p->m * (4-jumpInt) * nUp * grav[1];
		}
		cout << "added " << -10 * 0.2 * nUp * grav[1] << " to the bottom" << endl;
		jumpInt++;
		if (jumpInt > 4) jumpInt = -1;
	}
	else {
		jumpInt = -1;
	}

	if (moveFrame) {
		for (auto p : particles) {
			//if (!p->bottom) continue;
			int i = p->i;
			if (i == -1) continue;
			f.segment<3>(i) += 50 * p->m * (4 - moveInt) * moveDir;
		}
		cout << "added " << 50 * 0.1 * (4 - moveInt) * moveDir << " to the bottom" << endl;
		moveInt++;
		if (moveInt > 3) moveInt = -1;
	}

	M.setFromTriplets(Mt.begin(), Mt.end());
	K.setFromTriplets(Kt.begin(), Kt.end());

	// Solve the linear system
	Eigen::SparseMatrix<double> A = M - (pow(h, 2) * K) + h * D;
	Eigen::VectorXd b = (M * v) + (h * f);

// #ifdef _DEBUG
// 	cout << "A matrix: " << A << endl;
// 	cout << "b vector: " << b << endl;
// #endif // _DEBUG

	ConjugateGradient< SparseMatrix<double> > cg;
	cg.setMaxIterations(25);
	cg.setTolerance(1e-6);
	cg.compute(A);
	VectorXd x = cg.solveWithGuess(b, v);

	for (auto p : particles) {
		int i = p->i;
		if (i == -1) continue;
		p->v = x.segment<3>(i);
		p->x += p->v * h;
	}

	if (collisionParticles.size() > 0) {

		collisionParticles_mtx.lock();
		for (auto it = collisionParticles.begin(); it != collisionParticles.end(); it++) {
			if (!*it) continue;
			(*it)->step(h, grav);
		}
		collisionParticles_mtx.unlock();
	}

	for (int i : spawnParticles) {
		bool spawn = (rand() % 100) < 2;
		Vector3d v = particles.at(i)->v;

		if(spawn && v.norm() > 0.5) collisionParticles.push_back(make_shared<SlimeParticle>(particles.at(i)->x, -v));
	}
	
	// Update position and normal buffers
	updatePosNor();
	jumpFrame = jumpInt > 0;
	moveFrame = moveInt > 0;
	fixInvertFrame = false;
}

void Slime::init()
{
	// outer mesh
	glGenBuffers(1, &posBufID);
	glBindBuffer(GL_ARRAY_BUFFER, posBufID);
	glBufferData(GL_ARRAY_BUFFER, posBuf.size()*sizeof(float), &posBuf[0], GL_DYNAMIC_DRAW);
	
	glGenBuffers(1, &norBufID);
	glBindBuffer(GL_ARRAY_BUFFER, norBufID);
	glBufferData(GL_ARRAY_BUFFER, norBuf.size()*sizeof(float), &norBuf[0], GL_DYNAMIC_DRAW);
	
	glGenBuffers(1, &texBufID);
	glBindBuffer(GL_ARRAY_BUFFER, texBufID);
	glBufferData(GL_ARRAY_BUFFER, texBuf.size()*sizeof(float), &texBuf[0], GL_STATIC_DRAW);
	
	glGenBuffers(1, &eleBufID);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, eleBufID);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, eleBuf.size()*sizeof(unsigned int), &eleBuf[0], GL_STATIC_DRAW);

	// inner mesh
	glGenBuffers(1, &posBufID2);
	glBindBuffer(GL_ARRAY_BUFFER, posBufID2);
	glBufferData(GL_ARRAY_BUFFER, posBuf2.size() * sizeof(float), &posBuf2[0], GL_DYNAMIC_DRAW);

	glGenBuffers(1, &norBufID2);
	glBindBuffer(GL_ARRAY_BUFFER, norBufID2);
	glBufferData(GL_ARRAY_BUFFER, norBuf2.size() * sizeof(float), &norBuf2[0], GL_DYNAMIC_DRAW);

	glGenBuffers(1, &texBufID2);
	glBindBuffer(GL_ARRAY_BUFFER, texBufID2);
	glBufferData(GL_ARRAY_BUFFER, texBuf2.size()*sizeof(float), &texBuf2[0], GL_STATIC_DRAW);

	glGenBuffers(1, &eleBufID2);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, eleBufID2);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, eleBuf2.size() * sizeof(unsigned int), &eleBuf2[0], GL_STATIC_DRAW);
	
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	
	assert(glGetError() == GL_NO_ERROR);
}

void Slime::setTexture(std::shared_ptr<Texture> tex) {
	texture = tex;
}

void Slime::draw(shared_ptr<MatrixStack> MV, const shared_ptr<Program> p, const std::shared_ptr<Program> p2, const std::shared_ptr<Program> pSimple)
{
	MV->pushMatrix();
	// draw inner mesh
	p->unbind();
	p2->bind();
	{
		if (texture) {
			texture->bind(p->getUniform("kdTex"));
		}
		glUniform3fv(p2->getUniform("kdFront"), 1, Vector3f(0.0, 1.0, 0.0).data());
		glUniform3fv(p2->getUniform("kdBack"), 1, Vector3f(1.0, 1.0, 0.0).data());
		glUniformMatrix4fv(p2->getUniform("MV"), 1, GL_FALSE, glm::value_ptr(MV->topMatrix()));

		int h_pos = p2->getAttribute("aPos");
		glEnableVertexAttribArray(h_pos);
		glBindBuffer(GL_ARRAY_BUFFER, posBufID2);
		glBufferData(GL_ARRAY_BUFFER, posBuf2.size() * sizeof(float), &posBuf2[0], GL_DYNAMIC_DRAW);
		glVertexAttribPointer(h_pos, 3, GL_FLOAT, GL_FALSE, 0, (const void*)0);
		int h_nor = p2->getAttribute("aNor");
		glEnableVertexAttribArray(h_nor);
		glBindBuffer(GL_ARRAY_BUFFER, norBufID2);
		glBufferData(GL_ARRAY_BUFFER, norBuf2.size() * sizeof(float), &norBuf2[0], GL_DYNAMIC_DRAW);
		glVertexAttribPointer(h_nor, 3, GL_FLOAT, GL_FALSE, 0, (const void*)0);
		int h_tex = p2->getAttribute("aTex");
		if (h_tex != -1) {
			GLSL::checkError(GET_FILE_LINE);
			glEnableVertexAttribArray(h_tex);
			GLSL::checkError(GET_FILE_LINE);
			glBindBuffer(GL_ARRAY_BUFFER, texBufID2);
			glBufferData(GL_ARRAY_BUFFER, texBuf2.size() * sizeof(float), &texBuf2[0], GL_DYNAMIC_DRAW);
			glVertexAttribPointer(h_tex, 2, GL_FLOAT, GL_FALSE, 0, (const void*)0);
		}
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, eleBufID2);
		glDrawElements(GL_TRIANGLES, eleBuf2.size(), GL_UNSIGNED_INT, 0);

		glDisableVertexAttribArray(h_nor);
		glDisableVertexAttribArray(h_pos);
		if (h_tex != -1) {
			glDisableVertexAttribArray(h_tex);
		}
		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	}
	p2->unbind();
	p->bind();
	// Draw outer mesh
	{
		if (texture) {
			texture->bind(p->getUniform("kdTex"));
		}
		//glUniform3fv(p->getUniform("kdFront"), 1, Vector3f(0.2, 1.0, 0.2).data());
		glUniformMatrix4fv(p->getUniform("MV"), 1, GL_FALSE, glm::value_ptr(MV->topMatrix()));

		int h_pos = p->getAttribute("aPos");
		glEnableVertexAttribArray(h_pos);
		glBindBuffer(GL_ARRAY_BUFFER, posBufID);
		glBufferData(GL_ARRAY_BUFFER, posBuf.size() * sizeof(float), &posBuf[0], GL_DYNAMIC_DRAW);
		glVertexAttribPointer(h_pos, 3, GL_FLOAT, GL_FALSE, 0, (const void*)0);
		int h_nor = p->getAttribute("aNor");
		glEnableVertexAttribArray(h_nor);
		glBindBuffer(GL_ARRAY_BUFFER, norBufID);
		glBufferData(GL_ARRAY_BUFFER, norBuf.size() * sizeof(float), &norBuf[0], GL_DYNAMIC_DRAW);
		glVertexAttribPointer(h_nor, 3, GL_FLOAT, GL_FALSE, 0, (const void*)0);
		int h_tex = p->getAttribute("aTex");
		if (h_tex != -1) {
			GLSL::checkError(GET_FILE_LINE);
			glEnableVertexAttribArray(h_tex);
			GLSL::checkError(GET_FILE_LINE);
			glBindBuffer(GL_ARRAY_BUFFER, texBufID);
			glBufferData(GL_ARRAY_BUFFER, texBuf.size() * sizeof(float), &texBuf[0], GL_DYNAMIC_DRAW);
			glVertexAttribPointer(h_tex, 2, GL_FLOAT, GL_FALSE, 0, (const void*)0);
		}
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, eleBufID);
		glDrawElements(GL_TRIANGLES, eleBuf.size(), GL_UNSIGNED_INT, 0);

		glDisableVertexAttribArray(h_nor);
		glDisableVertexAttribArray(h_pos);
		if (h_tex != -1) {
			glDisableVertexAttribArray(h_tex);
		}
		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
		GLSL::checkError(GET_FILE_LINE);
	}


//#ifdef _DEBUG
//	// debug
//	//glUniformMatrix4fv(pSimple->getUniform("P"), 1, GL_FALSE, glm::value_ptr(P->topMatrix()));
//	p->unbind();
//
//	pSimple->bind();
//	glUniformMatrix4fv(pSimple->getUniform("MV"), 1, GL_FALSE, glm::value_ptr(MV->topMatrix()));
//	glPointSize(3.0f);
//	glColor3f(0, 1, 0);
//	glBegin(GL_POINTS);
//	for (auto p : particles) {
//		if (p->outer) {
//			Vector3d pos = p->x;
//			glVertex3f(pos[0], p->x[1], p->x[2]);
//		}
//	}
//	glEnd();
//
//	glPointSize(1.0f);
//	glColor3f(1, 1, 1);
//	glBegin(GL_POINTS);
//	for (auto p : particles) {
//		if (!p->outer) {
//			Vector3d pos = p->x;
//			glVertex3f(pos[0], p->x[1], p->x[2]);
//		}
//	}
//	glEnd();
//
//	glColor3f(0.1f, 0.1f, 0.1f);
//	for (auto s : struc_springs) {
//		Vector3d p0 = s->p0->x;
//		Vector3d p1 = s->p1->x;
//		if (!(s->p0->outer && s->p1->outer)) continue;
//		glBegin(GL_LINES);
//		glVertex3f(p0[0], p0[1], p0[2]);
//		glVertex3f(p1[0], p1[1], p1[2]);
//		glEnd();
//	}
//	pSimple->unbind();
//	p->bind();
//#endif
	MV->popMatrix();

	for (auto particle : collisionParticles) {
		if (!particle) continue;
		if (particle->dead) {
			collisionParticles_mtx.lock();
			particle = nullptr;
			collisionParticles_mtx.unlock();
			continue;
		}
		particle->draw(MV, p);
	}

	collisionParticles_mtx.lock();
	collisionParticles.erase(std::remove(collisionParticles.begin(), collisionParticles.end(), nullptr), collisionParticles.end());
	collisionParticles_mtx.unlock();
}

void Slime::jump() {
	jumpFrame = true;
	jumpInt = 0;
}

void Slime::move() {
	moveFrame = true;
	moveInt = 0;
}

void Slime::fixInvert() {
	fixInvertFrame = true;
}