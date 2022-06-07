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

/*
class Gravity : public ForceGenerator {
private:
	sf::Vector2f gravity;

public:
	Gravity() : ForceGenerator() {}
	Gravity(sf::Vector2f force) : ForceGenerator() {
		this->gravity = force;
	}

	virtual RigidBody updateForce(RigidBody body, float dt) override {
		body.addForce(ScaleVector(gravity, body.getMass()));
		return body;
	}
};

class ForceRegistration {
public:
	RigidBody rb;
	Gravity fg;
	sf::Vector2f id;

	bool operator == ( const ForceRegistration& fr) const { return (this->id == fr.id); }
	bool operator != ( const ForceRegistration& fr) const { return !operator==(fr); }

	ForceRegistration(Gravity fg, RigidBody rb) {
		this->fg = fg;
		this->rb = rb;
		this->id = rb.getPosition();
	}
};

class ForceRegistry {
private:
	std::list<ForceRegistration> registry;


public:
	ForceRegistry() {
		this->registry = std::list<ForceRegistration>();
	}

	void add(Gravity fg, RigidBody rb) {
		ForceRegistration fr = ForceRegistration(fg, rb);
		registry.push_back(fr);
	}



	void remove(Gravity fg, RigidBody rb) {
		ForceRegistration fr = ForceRegistration(fg, rb);
		registry.remove(fr);
		
	}
	
	void clear() {
		registry.clear();
	}

	void updateForces(float dt) { // idea by Millington 
		for (ForceRegistration fr : registry) {
			fr.fg.updateForce(fr.rb, dt);
		}
	}

	void zeroForces() {
		for (ForceRegistration fr : registry) {
			// TODO: FIXME
			//fr.rb.zeroForces();
		}
	}
};


*/