#pragma once

#include "ForceGenerator.h"
#include "Shapes.h"


/*
class PhysicsSystemGabe {
private:
	ForceRegistry forceRegistry;
	std::list<RigidBody> rigidbodies;
	Gravity gravity;
	float fixedUpdateTime;

public:
	PhysicsSystemGabe(float fixedUpdatedt, sf::Vector2f gravity) {
		this->forceRegistry = ForceRegistry();
		this->rigidbodies = std::list<RigidBody>();
		this->gravity = Gravity(gravity);
		this->fixedUpdateTime = fixedUpdatedt;
	}

	void update(float dt) {
		fixedUpdate();
	}
	void fixedUpdate() {
		forceRegistry.updateForces(fixedUpdateTime);

		// update Velcoites of all Bodies
		for (RigidBody rb : rigidbodies) {
			rb.physicsUpdate(fixedUpdateTime);
		}

	}

	void addRigidbody(RigidBody body) {
		this->rigidbodies.push_back(body);
		this->forceRegistry.add(gravity, body);
	}
};
*/