#pragma once
#ifndef Slime_H
#define Slime_H

#include <vector>
#include <memory>
#include <mutex>

#define EIGEN_DONT_ALIGN_STATICALLY
#include <Eigen/Dense>
#include <Eigen/Sparse>

class Particle;
class SlimeParticle;
class Spring;
class MatrixStack;
class Program;
class Texture;


typedef Eigen::Triplet<double> trip;

class Slime
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	Slime(int x, int y, int z,
		  const Eigen::Vector3d &x000,
		  const Eigen::Vector3d &x010,
		  const Eigen::Vector3d &x100,
		  const Eigen::Vector3d &x110,
		  const Eigen::Vector3d &x001,
		  const Eigen::Vector3d &x011,
		  const Eigen::Vector3d &x101,
		  const Eigen::Vector3d &x111,
		  double mass,
		  double stiffness);
	virtual ~Slime();
	
	void tare();
	void reset();
	void updatePosNor();
	void step(double h, const Eigen::Vector3d &grav, const std::vector< std::shared_ptr<Particle> > spheres);
	
	void init();
	void setTexture(std::shared_ptr<Texture> tex);
	void draw(std::shared_ptr<MatrixStack> MV, const std::shared_ptr<Program> p, const std::shared_ptr<Program> p2, const std::shared_ptr<Program> pSimple);

	void jump();
	void move();
	void fixInvert();

    Eigen::Vector3d _calculateNor(const int& i, const int& j, const int& k, const int& plane);
    void _updateTexBuf(std::vector<float>& _texBuf, Eigen::Vector2d offset, float width, float height);
	void _interpolateTexCoords(std::vector<float>& _texBuf, Eigen::Vector2d p00, Eigen::Vector2d p01, Eigen::Vector2d p10, Eigen::Vector2d p11, float width, float height);
	
	// direction vectors
	Eigen::Vector3d nForward;
	Eigen::Vector3d nRight;
	Eigen::Vector3d nUp;
	Eigen::Vector3d moveDir;
private:
	std::shared_ptr<Particle> particle000;
	std::shared_ptr<Particle> particle001;
	std::shared_ptr<Particle> particle010;
	std::shared_ptr<Particle> particle011;
	std::shared_ptr<Particle> particle100;
	std::shared_ptr<Particle> particle101;
	std::shared_ptr<Particle> particle110;
	std::shared_ptr<Particle> particle111;

	int xdim;
	int ydim;
    int zdim;
	int n;
	double c; // collision constant
	double d; // damping constant
	std::vector< std::shared_ptr<Particle> > particles;
	std::vector< std::shared_ptr<Spring> > struc_springs;
	std::vector< std::shared_ptr<Spring> > springs;
	std::vector< std::shared_ptr<Spring> > fix_springs;
	std::shared_ptr<Texture> texture;
	
	Eigen::VectorXd v;
	Eigen::VectorXd f;
	Eigen::SparseMatrix<double> M;
	Eigen::SparseMatrix<double> K;
	Eigen::SparseMatrix<double> D;
	
	std::vector<unsigned int> eleBuf;
	std::vector<float> posBuf;
	std::vector<float> norBuf;
	std::vector<float> texBuf;
	unsigned eleBufID;
	unsigned posBufID;
	unsigned norBufID;
	unsigned texBufID;

	std::vector<unsigned int> eleBuf2;
	std::vector<float> posBuf2;
	std::vector<float> norBuf2;
	std::vector<float> texBuf2;
	unsigned eleBufID2;
	unsigned posBufID2;
	unsigned norBufID2;
	unsigned texBufID2;

	int jumpInt;
	bool jumpFrame;

	int moveInt;
	bool moveFrame;

	bool fixInvertFrame;

	std::vector< std::shared_ptr<SlimeParticle> > collisionParticles;
	std::mutex collisionParticles_mtx;
};

#endif
