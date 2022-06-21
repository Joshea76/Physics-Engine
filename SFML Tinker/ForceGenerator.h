#pragma once

//#include "Shapes.h"
//#include <list>;
#include "Functions.h"


class ForceGenerator {
private:
	sf::Vector2f force;
public:

	ForceGenerator() {
		this->force = sf::Vector2f(0.f, 0.f);
	}
	ForceGenerator(sf::Vector2f force) {
		this->force = force;
	}

	void setForce(sf::Vector2f force) {
		this->force = force;
	}
	sf::Vector2f getForce() {
		return this->force;
	}
	/*
	virtual RigidBody updateForce(RigidBody body, float dt) {
		body.addForce(ScaleVector(force, body.getMass()));
		return body;
	};
	*/
	virtual sf::Vector2f updateForce(float Mass, float dt) {
		return ScaleVector(force, Mass);
	}
};
