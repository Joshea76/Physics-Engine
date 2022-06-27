#pragma once
#include <SFML/Graphics.hpp>
#include <Windows.h>
#include <iostream>
#include <vector>
#include <math.h>

//#include "RectCollide.h"
#include "Raycast.h"
#include "Shapes.h"


class CollisionManifold {
private:
	bool iscolliding;
	sf::Vector2f normal;
	std::vector<sf::Vector2f> contactPoints;
	float depth;

public:

	bool operator == (const CollisionManifold& fr) const { return (this->normal == fr.normal); }
	bool operator != (const CollisionManifold& fr) const { return !operator==(fr); }

	CollisionManifold() {
		normal = sf::Vector2f();
		this->contactPoints = std::vector<sf::Vector2f>();
		depth = 0.0f;
		iscolliding = false;
	}

	CollisionManifold(sf::Vector2f normal, float depth) {
		this->normal = normal;
		this->contactPoints = std::vector<sf::Vector2f>();
		this->depth = depth;
		iscolliding = true;
	}

	void addContactPoint(sf::Vector2f contact) {
		this->contactPoints.push_back(contact);
	}

	sf::Vector2f getNormal() {
		return normal;
	}
	std::vector<sf::Vector2f> getContactPoints() {
		return contactPoints;
	}
	float getDepth() {
		return depth;
	}

	bool isColliding() {
		return this->iscolliding;
	}
};

class Collisions {
public: 

	static CollisionManifold findcollisionfeatures(Circle a, Circle b) {
		
		if (a.rigidbody.getPosition().x > b.rigidbody.getPosition().x || a.rigidbody.getPosition().y < b.rigidbody.getPosition().y) {
			Circle tmp = a;
			a = b;
			b = tmp;
		}
		
		CollisionManifold result = CollisionManifold();
		float sumRadii = a.getRadius() + b.getRadius();
		sf::Vector2f distance = sf::Vector2f(SubtractVectors(b.rigidbody.getPosition(), a.rigidbody.getPosition()));
		if (lengthofVectorSquared(distance) - (sumRadii * sumRadii) > 0) {
			return result; 
		}

		// Multiply by 0.5 to seperate circles the same
		float depth = (lengthofVector(distance) - sumRadii) * 0.5f;
		sf::Vector2f normal = distance;
		normal = normalise(normal);
		float distanceToPoint = a.getRadius() - depth; 
		sf::Vector2f contactPoint = sf::Vector2f(AddVectors(a.rigidbody.getPosition(), (MultiplyVectors(normal, distance))));

		result = CollisionManifold(normal, depth);
		result.addContactPoint(contactPoint);
		return result;
	}
};


class IntersectionDetection {
public:


	// point detection 
	static bool pointOnLine(sf::Vector2f point, sf::VertexArray line) {
		// y = mx + c
		float dy = line[0].position.y - line[1].position.y;
		float dx = line[0].position.x - line[1].position.x;
		float m = dy / dx;
		if (dx == 0) {
			return compare(point.x, line[0].position.x);
		}

		float c = line[1].position.y - (m * line[1].position.x);

		//check
		return point.y == m * point.x + c;
	}

	static bool pointinCircle(sf::Vector2f point, Circle circle) {
		sf::Vector2f circleCenter = circle.rigidbody.getPosition();
		sf::Vector2f centerPoint = SubtractVectors(point, circleCenter);

		return lengthofVectorSquared(centerPoint) <= circle.getRadius() * circle.getRadius();
		//return normalise(centerPoint);
	}


	// line detection
	static CollisionManifold lineandCircle(std::vector<sf::Vertex> line, Circle circle) {
		if (pointinCircle(line[0].position, circle) || pointinCircle(line[1].position, circle)) {
			return CollisionManifold();
		}
		sf::Vector2f ab = sf::Vector2f(SubtractVectors(line[1].position, line[0].position));

		// project circle pos onto line ab
		sf::Vector2f circleCenter = circle.rigidbody.getPosition();
		sf::Vector2f centertoLineStart = sf::Vector2f(SubtractVectors(circleCenter, line[0].position));
		float t = dotProduct(centertoLineStart, ab) / dotProduct(ab, ab);

		if (t < 0.0f || t > 1.0f) { 
			return CollisionManifold();
		}

		// find closest point on line ab
		sf::Vector2f closestPoint = sf::Vector2f(AddVectors(line[0].position, ScaleVector(ab, t)));
		sf::Vector2f normal = normalise(SubtractVectors(closestPoint, circleCenter));

		if(pointinCircle(closestPoint, circle)){

			float depth = lengthofVector(SubtractVectors(AddVectors(circleCenter, ScaleVector(normal, circle.getRadius())), closestPoint));
			//return pointinCircle(closestPoint, circle);
			CollisionManifold p = CollisionManifold(normal, depth);
			p.addContactPoint(closestPoint);
			return p;
		}
		else { return CollisionManifold(); }
	}

	static bool lineandCircle(std::vector<sf::Vector2f> line, Circle circle) {
		if (pointinCircle(line[0], circle) || pointinCircle(line[1], circle)) {
			return false;
		}
		sf::Vector2f ab = sf::Vector2f(SubtractVectors(line[1], line[0]));

		// project circle pos onto line ab
		sf::Vector2f circleCenter = circle.rigidbody.getPosition();
		sf::Vector2f centertoLineStart = sf::Vector2f(SubtractVectors(circleCenter, line[0]));
		float t = dotProduct(centertoLineStart, ab) / dotProduct(ab, ab);

		if (t < 0.0f || t > 1.0f) {
			return false;
		}

		// find closest point on line ab
		sf::Vector2f closestPoint = sf::Vector2f(AddVectors(line[0], ScaleVector(ab, t)));
		
		if (pointinCircle(closestPoint, circle)) {

			return true;
		}
		else { return false; }
	}

	static CollisionManifold boundCollide(std::vector<sf::Vertex> bounds, Circle c) {
		//if (c.rigidbody.getPosition().x >= bounds[2].position.x - c.getRadius()) {// near right side
			//return 
		//}
		CollisionManifold m = CollisionManifold();
		for (int i = 0; i < 4; i++) {
			std::vector<sf::Vertex> b = {};
			if (i != 3) {
				b.push_back( bounds[i+1]);
				b.push_back(bounds[i]);
			}
			else {
				b.push_back(bounds[i]);		
				b.push_back(bounds[0]);

			}
			m = lineandCircle(b, c);
			if (m != CollisionManifold()) { break; }
		}
		return m;
	}

	static CollisionManifold wallCollide(std::vector<sf::Vertex> wall, Circle c) {
		return lineandCircle(wall, c);
	}

	static bool circleAndCircle(Circle obj1, Circle obj2) {
		sf::Vector2f joiner = sf::Vector2f(SubtractVectors(obj2.rigidbody.getPosition(), obj1.rigidbody.getPosition()));
		if (lengthofVector(joiner) < (2 * (obj1.getRadius() * obj1.getRadius())) + (2 * (obj2.getRadius() * obj2.getRadius()))) {
			return true;
		}
		else { return false; }
	}
	
	static bool overlapAxis(Rect box1, Rect box2, sf::Vector2f axis) {
		sf::Vector2f interval1 = getInterval(box1, axis);
		sf::Vector2f interval2 = getInterval(box2, axis); 
		if ((interval2.x <= interval1.y) && (interval1.x <= interval2.y)) {
			return true;
		}
		else { return false; }
	}

	static sf::Vector2f getInterval(Rect box, sf::Vector2f axis) {
		sf::Vector2f result = sf::Vector2f(0.f, 0.f);

		sf::Vector2f min = box.getLocalMin();
		sf::Vector2f max = box.getLocalMax();

		sf::VertexArray vertices;
		vertices.append(sf::Vertex(sf::Vector2f(min.x, min.y)));
		vertices.append(sf::Vertex(sf::Vector2f(min.x, max.y)));
		vertices.append(sf::Vertex(sf::Vector2f(max.x, min.y)));
		vertices.append(sf::Vertex(sf::Vector2f(max.x, max.y)));
		

			result.x = dotProduct(axis, vertices[0].position);
			result.y = result.x;

			for (int i = 0; i < 3; i++) {
				float proj = dotProduct(axis, vertices[i].position);
				if (proj < result.x) {
					result.x = proj;
				}
				if (proj > result.y) {
					result.y = proj;
				}
			}
		return result;
	}
};


class PhysicsSystem {
private:
	std::vector<RigidBody> rigidbodies; // taken from primitives globally, same position in list as global primitives
	std::vector<ForceGenerator> globalforces;
	std::vector<ForceGenerator> localforces;
	std::vector<CollisionManifold> collisions = {};
	std::vector<RigidBody> bodies1 = {};
	std::vector<RigidBody> bodies2 = {};

public:
	PhysicsSystem() {
		this->rigidbodies = {};
		this->globalforces = {};
		this->localforces = {};
		this->bodies1 = {};
		this->bodies2 = {};
		this->collisions = {};
	}
	PhysicsSystem(std::vector<RigidBody> rigidbodies) {
		this->rigidbodies = rigidbodies;
		this->globalforces = {};
		this->localforces = {};
		this->bodies1 = {};
		this->bodies2 = {};
		this->collisions = {};
	}
	PhysicsSystem(std::vector<ForceGenerator> globalforces) {
		this->rigidbodies = {};
		this->globalforces = globalforces;
		this->localforces = {};
		this->bodies1 = {};
		this->bodies2 = {};
		this->collisions = {};
	}
	PhysicsSystem(std::vector<RigidBody> rigidbodies, std::vector<ForceGenerator> globalforces) {
		this->rigidbodies = rigidbodies;
		this->globalforces = globalforces;
		this->localforces = {};
		this->bodies1 = {};
		this->bodies2 = {};
		this->collisions = {};
	}

	void setRigidBodies(std::vector<RigidBody> rigidbodies) {
		this->rigidbodies = rigidbodies;
	}
	void setGlobalForces(std::vector<ForceGenerator> globalforces) {
		this->globalforces = globalforces;
	}
	void setLocalForces(std::vector<ForceGenerator> localforces) {
		this->localforces = localforces;
	}

	std::vector<RigidBody> applyForces(float dt) {
		std::vector<RigidBody>::iterator rb;
		for (rb = rigidbodies.begin(); rb != rigidbodies.end(); ++rb) {
			setLocalForces(rb->getLocalForces());
			for (ForceGenerator fg : globalforces) {
				rb->addForce(fg.updateForce(rb->getMass(), dt));
			}
			//for (ForceGenerator fg : localforces) {
			//	rb->addForce(fg.updateForce(1, dt));
			//}
			rb->physicsUpdate(dt);
		}
		return rigidbodies;
	}

	std::tuple<RigidBody, RigidBody>  applyImpulse(RigidBody a, RigidBody b, CollisionManifold m) {
		bool swapped = false;
		
		if ((a.getPosition().x > b.getPosition().x) || (a.getPosition().y < b.getPosition().y)) {
			RigidBody tmp = a;
			a = b;
			b = tmp;
			swapped = true; 
		}
		
		float invMass1 = a.getInverseMass();
		float invMass2 = b.getInverseMass();
		float invMassSum = invMass1 + invMass2;
		if (invMassSum == 0.f) {
			return  std::tuple<RigidBody, RigidBody>();
		}

		sf::Vector2f relativeVel = sf::Vector2f(SubtractVectors(b.getLinearVelocity(), a.getLinearVelocity()));
		sf::Vector2f relativeNormal = sf::Vector2f(normalise(m.getNormal()));
		
		//Moving away from eachother do nothing
		if (dotProduct(relativeVel, relativeNormal) > 0.f) { 
			return std::tuple<RigidBody, RigidBody>(); 
		}
		if (swapped) { relativeNormal = -relativeNormal; }

		float e = std::min(a.getCor(), b.getCor());
		sf::Vector2f DeltaVel = SubtractVectors(ScaleVector(-relativeVel, e), relativeVel);
		//float numerator = (-(1.0f + e) * dotProduct(relativeVel, relativeNormal));
		//float numerator = (-(1.0f + e) * dotProduct(DeltaVel, relativeNormal));
		//float j = numerator / invMassSum;
		float j = lengthofVector(DeltaVel) / invMassSum;
		//if (m.getContactPoints().size() > 0 && j != 0.0f) {
			//j = j / float(m.getContactPoints().size());
		//}

		sf::Vector2f impulse = sf::Vector2f(ScaleVector(relativeNormal, j));
		if (swapped) {
			a.addPosition(ScaleVector(-relativeNormal, (m.getDepth() / 2)));
			b.addPosition(ScaleVector(relativeNormal, (m.getDepth() / 2)));

			a.setLinearVelocity(sf::Vector2f(AddVectors(a.getLinearVelocity(), (ScaleVector(impulse, (invMass1))))));
			b.setLinearVelocity(sf::Vector2f(AddVectors(b.getLinearVelocity(), (ScaleVector(impulse, (-invMass2))))));
		}
		else {
			a.addPosition(ScaleVector(relativeNormal, (m.getDepth() / 2)));
			b.addPosition(ScaleVector(-relativeNormal, (m.getDepth() / 2)));

			a.setLinearVelocity(sf::Vector2f(AddVectors(a.getLinearVelocity(), (ScaleVector(impulse, (-invMass1))))));
			b.setLinearVelocity(sf::Vector2f(AddVectors(b.getLinearVelocity(), (ScaleVector(impulse, (invMass2))))));
		}

		//b.addLocalForce(ScaleVector(impulse, 1));
		return swapped == false ? std::tuple<RigidBody, RigidBody>(a, b) : std::tuple<RigidBody, RigidBody>(b, a);
	}
	RigidBody applyImpulse(RigidBody rb, CollisionManifold m, sf::Vector2f g, int sect) {
		//For collisions with bounds
		sf::Vector2f Vel = SubtractVectors(sf::Vector2f(0.f,0.f),rb.getLinearVelocity());
		sf::Vector2f relativeNormal = sf::Vector2f((normalise(m.getNormal())));
		//if (dotProduct(Vel, abs(relativeNormal)) > 0.f) { Vel = ScaleVector(Vel, -1.f); }


		float invMass = rb.getInverseMass();
		float e = rb.getCor();
		//float numerator = (-(1.f + e) * dotProduct(Vel, relativeNormal));
		sf::Vector2f deltaVel = SubtractVectors(ScaleVector((-abs(Vel)), e), abs(Vel));
		sf::Vector2f j = ScaleVector(deltaVel, 1 / invMass);
		
		
		switch (sect) {
		case 0:
			relativeNormal.y = (abs(relativeNormal.y) * 1.f);
			rb.setPosition(sf::Vector2f(rb.getPosition().x, rb.getPosition().y + m.getDepth()));
			j.y = abs(j.y);
			break;
		case 1:
			//if (abs(rb.getLinearVelocity().x) < 1.f) {
			relativeNormal.x = abs(relativeNormal.x) * -1.f;
			rb.setPosition(sf::Vector2f(rb.getPosition().x - m.getDepth(), rb.getPosition().y));
			j.x = abs(j.x);
			break;
		case 2:
			//if (abs(rb.getLinearVelocity().y) < 1.f) {
				//j = abs(1.f * ((lengthofVector(g)) * rb.getMass())) + abs(numerator / rb.getInverseMass()) + abs(forceAccum.y / rb.getInverseMass());
			relativeNormal.y = abs(relativeNormal.y) * -1.f;
			rb.setPosition(sf::Vector2f(rb.getPosition().x, rb.getPosition().y - m.getDepth()));
			j.y = abs(j.y);
			break;
		case 3:
			//if (abs(rb.getLinearVelocity().x) < 1.f) {
			relativeNormal.x = abs(relativeNormal.x);
			rb.setPosition(sf::Vector2f(rb.getPosition().x + m.getDepth(), rb.getPosition().y + m.getDepth()));
			j.x = abs(j.x);
			break;

		}
		
		
		
		//j = numerator / (rb.getInverseMass());
		sf::Vector2f impulse = sf::Vector2f(MultiplyVectors(relativeNormal, j));
		//rb.addLocalForce(ScaleVector(impulse, 1));
		rb.setLinearVelocity(sf::Vector2f(AddVectors(rb.getLinearVelocity(), (ScaleVector(impulse, (invMass))))));
		return rb;
	}
	RigidBody applyImpulse(RigidBody rb, CollisionManifold m) {
		//For collisions with bounds
		sf::Vector2f Vel = SubtractVectors(sf::Vector2f(0.f, 0.f), rb.getLinearVelocity());
		sf::Vector2f relativeNormal = sf::Vector2f((normalise(-m.getNormal())));
		//if (dotProduct(Vel, abs(relativeNormal)) > 0.f) { Vel = ScaleVector(Vel, -1.f); }


		float invMass = rb.getInverseMass();
		float e = rb.getCor();
		//float numerator = (-(1.f + e) * dotProduct(Vel, relativeNormal));
		sf::Vector2f deltaVel = SubtractVectors(ScaleVector((-abs(Vel)), e), abs(Vel));
		float j = lengthofVector(deltaVel) / invMass;



		//j = numerator / (rb.getInverseMass());
		sf::Vector2f impulse = sf::Vector2f(ScaleVector(relativeNormal, j));
		//rb.addLocalForce(ScaleVector(impulse, 1));
		rb.setLinearVelocity(sf::Vector2f(AddVectors(rb.getLinearVelocity(), (ScaleVector(impulse, (invMass))))));
		rb.setPosition(AddVectors(rb.getPosition(), ScaleVector(relativeNormal, m.getDepth())));
		return rb;
	}
};

// Sectoional Quadrant Class for walls
class Quadtree {
public:
	std::vector<sf::Vertex> bounds;
	std::vector<sf::Vector2f> vLine;
	std::vector<sf::Vector2f> hLine;

	std::vector<Circle> s1Rbs;
	std::vector<Circle> s2Rbs;
	std::vector<Circle> s3Rbs;
	std::vector<Circle> s4Rbs;


	Quadtree() {

	}
	Quadtree(std::vector<sf::Vertex> bounds) {
		this->bounds = bounds;
		vLine = {};
		hLine = {};
		createLines();
	}

	void createLines() {
		this->vLine.push_back(Midpoint(bounds[0].position, bounds[1].position));
		this->vLine.push_back(Midpoint(bounds[3].position, bounds[2].position));

		this->hLine.push_back(Midpoint(bounds[0].position, bounds[3].position));
		this->hLine.push_back(Midpoint(bounds[1].position, bounds[2].position));
	}

	std::vector<Circle> sortCircles(std::vector<Circle> circles) {
		for (int i = 0; i < circles.size(); i++) {
			bool vInt = IntersectionDetection().lineandCircle(vLine, circles[i]);
			bool hInt = IntersectionDetection().lineandCircle(hLine, circles[i]);

			if (vInt && hInt) { 
				circles[i].rigidbody.sections.setAllTrue(); }
			else {
				if (hInt) {
					if (circles[i].rigidbody.getPosition().x > vLine[0].x) {
						circles[i].rigidbody.sections.S2 = true;
						circles[i].rigidbody.sections.S3 = true;
					}
					else{
						circles[i].rigidbody.sections.S1 = true;
						circles[i].rigidbody.sections.S4 = true;
					}
				}
				else if (vInt) {
					if (circles[i].rigidbody.getPosition().y > hLine[0].y) {
						circles[i].rigidbody.sections.S4 = true;
						circles[i].rigidbody.sections.S3 = true;
					}
					else {
						circles[i].rigidbody.sections.S1 = true;
						circles[i].rigidbody.sections.S2 = true;
					}
				}
				else {
					if (circles[i].rigidbody.getPosition().x > vLine[0].x) {
						if (circles[i].rigidbody.getPosition().y > hLine[0].y) {
							circles[i].rigidbody.sections.S3 = true;
						}
						else {
							circles[i].rigidbody.sections.S2 = true;
						}
					}
					else {
						if (circles[i].rigidbody.getPosition().y > hLine[0].y) {
							circles[i].rigidbody.sections.S4 = true;
						}
						else {
							circles[i].rigidbody.sections.S1 = true;
						}
					}
				}
			}
			
			if (circles[i].rigidbody.getPosition().x > vLine[0].x) {
				circles[i].rigidbody.sections.S2 = true;
				circles[i].rigidbody.sections.S3 = true;
				if (!IntersectionDetection().lineandCircle(hLine, circles[i])) {
					if (circles[i].rigidbody.getPosition().y > hLine[0].y) {
						circles[i].rigidbody.sections.S3 = true;
						circles[i].rigidbody.sections.S2 = false;
						if (IntersectionDetection().lineandCircle(vLine, circles[i])) {
							circles[i].rigidbody.sections.S4 = true;
						}
					}
					else {
						circles[i].rigidbody.sections.S2 = true;
						circles[i].rigidbody.sections.S3 = false;
						if (IntersectionDetection().lineandCircle(vLine, circles[i])) {
							circles[i].rigidbody.sections.S1 = true;
						}
					}
				}
			}
			else {
				circles[i].rigidbody.sections.S1 = true;
				circles[i].rigidbody.sections.S4 = true;
				if (!IntersectionDetection().lineandCircle(hLine, circles[i])) {
					if (circles[i].rigidbody.getPosition().y > hLine[0].y) {
						circles[i].rigidbody.sections.S4 = true;
						circles[i].rigidbody.sections.S1 = false;
						if (IntersectionDetection().lineandCircle(vLine, circles[i])) {
							circles[i].rigidbody.sections.S3 = true;
						}
					}
					else {
						circles[i].rigidbody.sections.S4 = false;
						circles[i].rigidbody.sections.S1 = true;
						if (IntersectionDetection().lineandCircle(vLine, circles[i])) {
							circles[i].rigidbody.sections.S2 = true;
						}
					}
				}
			}
			
		}
		return circles;
	}

	

	void setrbsCircles(std::vector<Circle> circles) {
		for (int i = 0; i < circles.size(); i++) {
			if (circles[i].rigidbody.sections.S1) {
				s1Rbs.push_back(circles[i]);
			}
			if (circles[i].rigidbody.sections.S2) {
				s2Rbs.push_back(circles[i]);
			}
			if (circles[i].rigidbody.sections.S3) {
				s3Rbs.push_back(circles[i]);
			}
			if (circles[i].rigidbody.sections.S4) {
				s4Rbs.push_back(circles[i]);
			}
		}
	}


};
