#pragma once
#ifndef Particle_H
#define Particle_H

#include <vector>
#include <memory>

#define EIGEN_DONT_ALIGN_STATICALLY
#include <Eigen/Dense>

class Shape;
class Program;
class MatrixStack;
class Texture;

class Particle
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	Particle();
	Particle(const std::shared_ptr<Shape> shape);
	Particle(const std::shared_ptr<Shape> shape, const std::shared_ptr<Texture> texture);
	virtual ~Particle();
	void tare();
	void reset();
	void draw(std::shared_ptr<MatrixStack> MV, const std::shared_ptr<Program> p) const;
	std::tuple<double, Eigen::Vector3d> collide(std::shared_ptr<Particle> p);
	
	double r; // radius
	double m; // mass
	int i;  // starting index
	Eigen::Vector3d x0; // initial position
	Eigen::Vector3d v0; // initial velocity
	Eigen::Vector3d x;  // position
	Eigen::Vector3d v;  // velocity
	// Eigen::Vector3d n0;  // initial normal
	bool fixed = false;
	bool outer = false;
	bool inner = false;
	bool bottom = false;
	
private:
	const std::shared_ptr<Shape> sphere;
	const std::shared_ptr<Texture> texture;
};

#endif
