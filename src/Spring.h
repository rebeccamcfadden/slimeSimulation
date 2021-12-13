#pragma once
#ifndef Spring_H
#define Spring_H

#include <memory>

#define EIGEN_DONT_ALIGN_STATICALLY
#include <Eigen/Dense>
#include <Eigen/Sparse>

class Particle;

class Spring
{
public:
	Spring(std::shared_ptr<Particle> p0, std::shared_ptr<Particle> p1);
	virtual ~Spring();
	void computeForce(Eigen::Vector3d& fs);
	void computeForceStiffness(Eigen::Vector3d& fs, Eigen::Matrix3d& K);
	
	std::shared_ptr<Particle> p0;
	std::shared_ptr<Particle> p1;
	double E;
	double L;
};

#endif
