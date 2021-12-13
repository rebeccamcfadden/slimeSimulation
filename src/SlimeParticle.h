#pragma once

#include "Particle.h"
class SlimeParticle : public Particle
{
public:
	SlimeParticle();
	SlimeParticle(Eigen::Vector3d x0, Eigen::Vector3d v0);
	//SlimeParticle(const std::shared_ptr<Shape> shape, const std::shared_ptr<Texture> texture);
	virtual ~SlimeParticle();
	void init();
	void tare();
	void reset();
	bool step(double h, const Eigen::Vector3d& grav);
	void draw(std::shared_ptr<MatrixStack> MV, const std::shared_ptr<Program> p) const;
	//std::tuple<double, Eigen::Vector3d> collide(std::shared_ptr<Particle> p);

	static std::shared_ptr<Shape> slimeShape;
	static std::shared_ptr<Texture> slimeTexture;
	bool dead;

private:
	std::shared_ptr<Texture> texture;
	float t;
	//std::vector<unsigned int> eleBuf;
	//std::vector<float> posBuf;
	//std::vector<float> norBuf;
	//std::vector<float> texBuf;
	//unsigned eleBufID;
	//unsigned posBufID;
	//unsigned norBufID;
	//unsigned texBufID;
};

