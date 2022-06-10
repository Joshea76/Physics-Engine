#pragma once
#include <SFML/Graphics.hpp>
#include <Windows.h>
#include <iostream>
#include <vector>
#include <math.h>

//#include "RectCollide.h"
#include "Raycast.h"
#include "Shapes.h"



enum Axis {
	FACE_A_X,
	FACE_A_Y,
	FACE_B_X,
	FACE_B_Y
};

enum EdgeNumbers {			///		 e1
	NO_EDGE = 0,			///		_____
	EDGE1,					///	e2	|   |	e4
	EDGE2,					///		|___|
	EDGE3,					///		 e3
	EDGE4					///		
};



union FeaturePair {
	struct Edges {
		char inEdge1;
		char outEdge1;
		char inEdge2;
		char outEdge2;
	} e;
	int value;
};

struct ClipVertex {
	ClipVertex() { fp.value = 0; }
	sf::Vector2f v;
	FeaturePair fp;
};

struct Contact {
	Contact() : Pn(0.0f), Pt(0.0f), Pnb(0.0f) {}

	sf::Vector2f position;
	sf::Vector2f normal;
	sf::Vector2f r1, r2;
	float separation;
	float Pn;		// accumulated normal impulse
	float Pt;		// accumulated tangent impulse
	float Pnb;		// accumulated normal impulse for position bias
	float massNormal, massTangent;
	float bias;
	FeaturePair feature;
};

template<typename T> void Swap(T& a, T& b) {
	T tmp = a;
	a = b;
	b = tmp;
}

void Flip(FeaturePair& fp)
{
	Swap(fp.e.inEdge1, fp.e.inEdge2);
	Swap(fp.e.outEdge1, fp.e.outEdge2);
}

class MatrixTbyT {
public:
	sf::Vector2f col1, col2;


	MatrixTbyT() {}
	MatrixTbyT(float angle) {
		float c = cos(angle), s = sin(angle);
		col1.x = c; col2.x = -s;		//  | 1 , 0 |	//
		col1.y = s; col2.y = c;			//  | 0 , 1 |	identity matrix (angle = 0)
	}

	MatrixTbyT(sf::Vector2f col1, sf::Vector2f col2) {
		this->col1 = col1;
		this->col2 = col2;
	}

	MatrixTbyT Transpose() { // "reflect on the diagonal" - google
		return MatrixTbyT(sf::Vector2f(this->col1.x, this->col2.x), sf::Vector2f(this->col1.y, this->col2.y));
	}

	MatrixTbyT MultiplybyScalar(float scalar) {
		return MatrixTbyT(sf::Vector2f((this->col1.x * scalar), (this->col1.y * scalar)), sf::Vector2f((this->col2.x * scalar), (this->col2.y * scalar)));
	}

	sf::Vector2f MultiplyByVector(sf::Vector2f mul) {
		return sf::Vector2f(((col1.x * mul.x) + (col1.y * mul.x)), ((col2.x * mul.y) + (col2.y * mul.y)));
	}

	MatrixTbyT MultiplyByMatrix(MatrixTbyT mat2) {
		float a = ((this->col1.x * mat2.col1.x /*ae*/) + (this->col2.x * mat2.col1.y /*bg*/));
		float b = ((this->col1.x * mat2.col2.x /*af*/) + (this->col2.x * mat2.col2.y /*bh*/));


		float c = ((this->col1.y * mat2.col1.x /*ce*/) + (this->col2.y * mat2.col1.y /*dg*/));
		float d = ((this->col1.y * mat2.col2.x /*cf*/) + (this->col2.y * mat2.col2.y /*dh*/));

		return MatrixTbyT(sf::Vector2f(a, c), sf::Vector2f(b, d)); // col1 = ac, col2 = bd

		/*	| a b |		| e f |		~		| ae+bg af+bh |
		*	| c d |		| g h |		~		| ce+dg cf+dh |

			| col1.x  col2.x |
			| col1.y  col2.y |
		*/
	}

	MatrixTbyT absolute() {
		return MatrixTbyT(sf::Vector2f(abs(col1.x), abs(col1.y)), sf::Vector2f(abs(col2.x), abs(col2.y)));
	}

};

int ClipSegmentToLine(ClipVertex vOut[2], ClipVertex vIn[2], sf::Vector2f normal, float offset, char clipEdge) {

	// no output points to begin
	int numOut = 0;

	//Calc distance of endpoints to the line
	float distance0 = dotProduct(normal, vIn[0].v) - offset;
	float distance1 = dotProduct(normal, vIn[1].v) - offset;

	// if points are inside the box add to list of intersecting points
	if (distance0 <= 0.0f) vOut[numOut] = vIn[0];
	numOut += 1; // iterate counter
	if (distance1 <= 0.0f) vOut[numOut] = vIn[1];
	numOut += 1; // iterate counter

	if (distance0 * distance1 > 0.0f) { // if either is negative

		// find intersection point of edge and plane 
		float interp = distance0 / (distance0 - distance1);
		vOut[numOut].v = vIn[0].v + interp * (vIn[1].v - vIn[0].v);

		if (distance0 > 0.0f) {
			vOut[numOut].fp = vIn[0].fp;
			vOut[numOut].fp.e.inEdge1 = clipEdge;
			vOut[numOut].fp.e.inEdge2 = NO_EDGE;
		}
		else {
			vOut[numOut].fp = vIn[1].fp;
			vOut[numOut].fp.e.outEdge1 = clipEdge;
			vOut[numOut].fp.e.outEdge2 = NO_EDGE;
		}
		++numOut;
	}

	return numOut;

}

void ComputeIncidentEdge(ClipVertex c[2], sf::Vector2f h, sf::Vector2f pos, MatrixTbyT Rot, sf::Vector2f normal) {

	//rotate normal to indicent box's local space
	MatrixTbyT RotT = Rot.Transpose();
	sf::Vector2f n = -(RotT.MultiplyByVector(normal));
	sf::Vector2f nAbs = abs(n);

	// chooses the two vertices in the correct direction,
	// closest to the collision
	//Which axis has more leverage
	if (nAbs.x > nAbs.y) { // X Axis Bias
		//Pointing Right
		if (Sign(n.x) > 0.0f) {
			c[0].v = sf::Vector2f(h.x, -h.y);
			c[0].fp.e.inEdge2 = EDGE3;
			c[0].fp.e.outEdge2 = EDGE4;

			c[1].v = sf::Vector2f(h.x, h.y);
			c[1].fp.e.inEdge2 = EDGE4;
			c[1].fp.e.outEdge2 = EDGE1;
		}
		//Pointing Left
		else {
			c[0].v = sf::Vector2f(-h.x, h.y);
			c[0].fp.e.inEdge2 = EDGE1;
			c[0].fp.e.outEdge2 = EDGE2;

			c[1].v = sf::Vector2f(-h.x, -h.y);
			c[1].fp.e.inEdge2 = EDGE2;
			c[1].fp.e.outEdge2 = EDGE3;
		}
	}

	else { // Y Axis Bias
		//Pointing Up
		if (Sign(n.y) > 0.0f) {
			c[0].v = sf::Vector2f(h.x, h.y);
			c[0].fp.e.inEdge2 = EDGE4;
			c[0].fp.e.outEdge2 = EDGE1;

			c[1].v = sf::Vector2f(-h.x, h.y);
			c[1].fp.e.inEdge2 = EDGE1;
			c[1].fp.e.outEdge2 = EDGE2;
		}
		//Pointing Down
		else {
			c[0].v = sf::Vector2f(-h.x, -h.y);
			c[0].fp.e.inEdge2 = EDGE2;
			c[0].fp.e.outEdge2 = EDGE3;

			c[1].v = sf::Vector2f(h.x, -h.y);
			c[1].fp.e.inEdge2 = EDGE3;
			c[1].fp.e.outEdge2 = EDGE4;
		}
	}

	c[0].v = AddVectors(pos, Rot.MultiplyByVector(c[0].v));
	c[1].v = AddVectors(pos, Rot.MultiplyByVector(c[1].v));

}
/*
CollisionManifold collideErinCatto(Rect BodyA, Rect BodyB) {


	// get half width
	sf::Vector2f halfWidthA = BodyA.getHalfSize();
	sf::Vector2f halfWidthB = BodyB.getHalfSize();

	sf::Vector2f posA = BodyA.rigidbody.getPosition();
	sf::Vector2f posB = BodyB.rigidbody.getPosition();

	MatrixTbyT rotationA = MatrixTbyT(BodyA.rigidbody.getRotation());
	MatrixTbyT rotationB = MatrixTbyT(BodyB.rigidbody.getRotation());

	MatrixTbyT rotationAT = rotationA.Transpose();		// will rotate anything onto A's local space
	MatrixTbyT rotationBT = rotationB.Transpose();		// will rotate anything onto B's local space

	sf::Vector2f distancebetween = posA - posB;
	sf::Vector2f distbetwAT = rotationAT.MultiplyByVector(distancebetween);		//distancebetween in A's local Space
	sf::Vector2f distbetwBT = rotationBT.MultiplyByVector(distancebetween);		//distancebetween in B's local Space

	MatrixTbyT C = rotationAT.MultiplyByMatrix(rotationB);  //RotAT * RotB // Rotates anything in B's local space into A's Local Space
	MatrixTbyT absC = C.absolute();

	MatrixTbyT absCT = absC.Transpose();									// Rotates anything in A's local space into B's Local Space

	//SAT Separating Axis Theorem
	sf::Vector2f faceA = SubtractVectors(SubtractVectors(abs(distbetwAT), halfWidthA), (absCT.MultiplyByVector(halfWidthB))); // penetration distance
	if (faceA.x > 0.0f || faceA.y > 0.0f) { return CollisionManifold(); }

	//Vec2 faceB = Abs(dB) - absCT * hA - hB;
	sf::Vector2f faceB = SubtractVectors(SubtractVectors(abs(distbetwBT), (absCT.MultiplyByVector(halfWidthA))), halfWidthB);
	if (faceB.x > 0.0f || faceB.y > 0.0f) { return CollisionManifold(); }

	//Find Best Axis
	Axis axis;
	float separation;
	sf::Vector2f normal;

	//Box A Faces
	//Assume A's X is best axis first, unless another is better
	axis = FACE_A_X;
	separation = faceA.x;
	normal = distbetwAT.x > 0.0f ? rotationA.col1 : -rotationA.col1;

	//Tolerances
	float relativeTolerance = 0.95;
	float absoluteTolerance = 0.01;

	if (faceA.y > (relativeTolerance * separation) + (absoluteTolerance * halfWidthA.y)) {
		axis = FACE_A_Y;
		separation = faceA.y;
		normal = distbetwAT.y > 0.0f ? rotationA.col2 : -rotationA.col2;
	}


	//Box B Faces
	if (faceB.x > (relativeTolerance * separation) + (absoluteTolerance * halfWidthB.x)) {
		axis = FACE_B_X;
		separation = faceB.x;
		normal = distbetwBT.x > 0.0f ? rotationB.col1 : -rotationB.col1;
	}

	if (faceB.y > (relativeTolerance * separation) + (absoluteTolerance * halfWidthB.y)) {
		axis = FACE_B_Y;
		separation = faceB.y;
		normal = distbetwBT.y > 0.0f ? rotationB.col2 : -rotationB.col2;
	}

	sf::Vector2f frontNormal, sideNormal;
	ClipVertex incidentEdge[2];
	float front, negSide, posSide;
	char negEdge, posEdge;

	switch (axis) {
	case FACE_A_X: {
		frontNormal = normal;
		front = dotProduct(posA, frontNormal) + halfWidthA.x;
		sideNormal = rotationA.col2;
		float side = dotProduct(posA, sideNormal);
		negSide = -side + halfWidthA.y;
		posSide = side + halfWidthA.y;
		negEdge = EDGE3;
		posEdge = EDGE1;
		ComputeIncidentEdge(incidentEdge, halfWidthB, posB, rotationB, frontNormal);
	}
				 break;

	case FACE_A_Y: {
		frontNormal = normal;
		front = dotProduct(posA, frontNormal) + halfWidthA.y;
		sideNormal = rotationA.col1;
		float side = dotProduct(posA, sideNormal);
		negSide = -side + halfWidthA.x;
		posSide = side + halfWidthA.x;
		negEdge = EDGE2;
		posEdge = EDGE4;
		ComputeIncidentEdge(incidentEdge, halfWidthB, posB, rotationB, frontNormal);
	}
				 break;

	case FACE_B_X: {
		frontNormal = -normal;
		front = dotProduct(posB, frontNormal) + halfWidthB.x;
		sideNormal = rotationB.col2;
		float side = dotProduct(posB, sideNormal);
		negSide = -side + halfWidthB.y;
		posSide = side + halfWidthB.y;
		negEdge = EDGE3;
		posEdge = EDGE1;
		ComputeIncidentEdge(incidentEdge, halfWidthA, posA, rotationA, frontNormal);
	}
				 break;

	case FACE_B_Y: {
		frontNormal = -normal;
		front = dotProduct(posB, frontNormal) + halfWidthB.y;
		sideNormal = rotationB.col1;
		float side = dotProduct(posB, sideNormal);
		negSide = -side + halfWidthB.x;
		posSide = side + halfWidthB.x;
		negEdge = EDGE2;
		posEdge = EDGE4;
		ComputeIncidentEdge(incidentEdge, halfWidthA, posA, rotationA, frontNormal);
	}
				 break;
	}

	ClipVertex clipPoints1[2];
	ClipVertex clipPoints2[2];
	int np;

	//clip to box side 1
	np = ClipSegmentToLine(clipPoints1, incidentEdge, -sideNormal, negSide, negEdge);

	if (np < 2) { return CollisionManifold(); }

	//Clip to negative box side 1
	np = ClipSegmentToLine(clipPoints2, clipPoints1, sideNormal, posSide, posEdge);

	if (np < 2) { return CollisionManifold(); }

	//Now clipPoints2 contains the clipping points

	CollisionManifold CM;

	int numContacts = 0;
	//only include points that are actually intersecting
	for (int i = 0; i < 2; i++) {
		float separation = dotProduct(frontNormal, clipPoints2[i].v) - front;
		Contact contactt = Contact();
		//If separation is less than 0.0f // if intersecting
		if (separation < 0.0f) {
			//Add values to contacts
			CM = CollisionManifold(normal, separation);
			CM.addContactPoint(clipPoints2[i].v - separation * frontNormal);
			//contacts2[numContacts].separation = separation;
			//contacts2[numContacts].normal = normal;
			// slide contact point onto reference face (easy to cull)
			//contacts2[numContacts].position = clipPoints2[i].v - separation * frontNormal;
			//contacts2[numContacts].feature = clipPoints2[i].fp;
			//if (axis == FACE_B_X || axis == FACE_B_Y)
				//Flip(contacts2[numContacts].feature);
			++numContacts;
		}
	}
	return CM;
}

*/
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


	static CollisionManifold findcollisionfeatures(Rect BodyA, Rect BodyB) { 
		/*
		std::vector<Contact> Contacts = collideErinCatto(a, b);
		CollisionManifold x;
		if (!Contacts.empty()) {
			x = CollisionManifold(Contacts.front().normal, Contacts.front().separation);
			x.addContactPoint(Contacts.front().position);
			//delete contacts;
		}
		*/
		//return collideErinCatto(a, b);

		if (BodyA.rigidbody.getPosition().x > BodyB.rigidbody.getPosition().x || BodyA.rigidbody.getPosition().y > BodyB.rigidbody.getPosition().y) {
			Rect tmp = BodyA;
			BodyA = BodyB;
			BodyB = tmp;
		}


	// get half width
		sf::Vector2f halfWidthA = BodyA.getHalfSize();
		sf::Vector2f halfWidthB = BodyB.getHalfSize();

		sf::Vector2f posA = BodyA.rigidbody.getPosition();
		sf::Vector2f posB = BodyB.rigidbody.getPosition();

		MatrixTbyT rotationA = MatrixTbyT(BodyA.rigidbody.getRotation());
		MatrixTbyT rotationB = MatrixTbyT(BodyB.rigidbody.getRotation());

		MatrixTbyT rotationAT = rotationA.Transpose();		// will rotate anything onto A's local space
		MatrixTbyT rotationBT = rotationB.Transpose();		// will rotate anything onto B's local space

		sf::Vector2f distancebetween = posA - posB;
		sf::Vector2f distbetwAT = rotationAT.MultiplyByVector(distancebetween);		//distancebetween in A's local Space
		sf::Vector2f distbetwBT = rotationBT.MultiplyByVector(distancebetween);		//distancebetween in B's local Space

		MatrixTbyT C = rotationAT.MultiplyByMatrix(rotationB);  //RotAT * RotB // Rotates anything in B's local space into A's Local Space
		MatrixTbyT absC = C.absolute();

		MatrixTbyT absCT = absC.Transpose();									// Rotates anything in A's local space into B's Local Space

		//SAT Separating Axis Theorem
		sf::Vector2f faceA = SubtractVectors(SubtractVectors(abs(distbetwAT), halfWidthA), (absCT.MultiplyByVector(halfWidthB))); // penetration distance
		if (faceA.x > 0.0f || faceA.y > 0.0f) { return CollisionManifold(); }

		//Vec2 faceB = Abs(dB) - absCT * hA - hB;
		sf::Vector2f faceB = SubtractVectors(SubtractVectors(abs(distbetwBT), (absCT.MultiplyByVector(halfWidthA))), halfWidthB);
		if (faceB.x > 0.0f || faceB.y > 0.0f) { return CollisionManifold(); }

		//Find Best Axis
		Axis axis;
		float separation;
		sf::Vector2f normal;

		//Box A Faces
		//Assume A's X is best axis first, unless another is better
		axis = FACE_A_X;
		separation = faceA.x;
		normal = distbetwAT.x > 0.0f ? rotationA.col1 : -rotationA.col1;

		//Tolerances
		float relativeTolerance = 0.95;
		float absoluteTolerance = 0.01;

		if (faceA.y > (relativeTolerance * separation) + (absoluteTolerance * halfWidthA.y)) {
			axis = FACE_A_Y;
			separation = faceA.y;
			normal = distbetwAT.y > 0.0f ? rotationA.col2 : -rotationA.col2;
		}


		//Box B Faces
		if (faceB.x > (relativeTolerance * separation) + (absoluteTolerance * halfWidthB.x)) {
			axis = FACE_B_X;
			separation = faceB.x;
			normal = distbetwBT.x > 0.0f ? rotationB.col1 : -rotationB.col1;
		}

		if (faceB.y > (relativeTolerance * separation) + (absoluteTolerance * halfWidthB.y)) {
			axis = FACE_B_Y;
			separation = faceB.y;
			normal = distbetwBT.y > 0.0f ? rotationB.col2 : -rotationB.col2;
		}

		sf::Vector2f frontNormal, sideNormal;
		ClipVertex incidentEdge[2];
		float front, negSide, posSide;
		char negEdge, posEdge;

		switch (axis) {
		case FACE_A_X: {
			frontNormal = normal;
			front = dotProduct(posA, frontNormal) + halfWidthA.x;
			sideNormal = rotationA.col2;
			float side = dotProduct(posA, sideNormal);
			negSide = -side + halfWidthA.y;
			posSide = side + halfWidthA.y;
			negEdge = EDGE3;
			posEdge = EDGE1;
			ComputeIncidentEdge(incidentEdge, halfWidthB, posB, rotationB, frontNormal);
		}
					 break;

		case FACE_A_Y: {
			frontNormal = normal;
			front = dotProduct(posA, frontNormal) + halfWidthA.y;
			sideNormal = rotationA.col1;
			float side = dotProduct(posA, sideNormal);
			negSide = -side + halfWidthA.x;
			posSide = side + halfWidthA.x;
			negEdge = EDGE2;
			posEdge = EDGE4;
			ComputeIncidentEdge(incidentEdge, halfWidthB, posB, rotationB, frontNormal);
		}
					 break;

		case FACE_B_X: {
			frontNormal = -normal;
			front = dotProduct(posB, frontNormal) + halfWidthB.x;
			sideNormal = rotationB.col2;
			float side = dotProduct(posB, sideNormal);
			negSide = -side + halfWidthB.y;
			posSide = side + halfWidthB.y;
			negEdge = EDGE3;
			posEdge = EDGE1;
			ComputeIncidentEdge(incidentEdge, halfWidthA, posA, rotationA, frontNormal);
		}
					 break;

		case FACE_B_Y: {
			frontNormal = -normal;
			front = dotProduct(posB, frontNormal) + halfWidthB.y;
			sideNormal = rotationB.col1;
			float side = dotProduct(posB, sideNormal);
			negSide = -side + halfWidthB.x;
			posSide = side + halfWidthB.x;
			negEdge = EDGE2;
			posEdge = EDGE4;
			ComputeIncidentEdge(incidentEdge, halfWidthA, posA, rotationA, frontNormal);
		}
					 break;
		}

		ClipVertex clipPoints1[2];
		ClipVertex clipPoints2[2];
		int np;

		//clip to box side 1
		np = ClipSegmentToLine(clipPoints1, incidentEdge, -sideNormal, negSide, negEdge);

		if (np < 2) { return CollisionManifold(); }

		//Clip to negative box side 1
		np = ClipSegmentToLine(clipPoints2, clipPoints1, sideNormal, posSide, posEdge);

		if (np < 2) { return CollisionManifold(); }

		//Now clipPoints2 contains the clipping points

		CollisionManifold CM;

		int numContacts = 0;
		//only include points that are actually intersecting
		for (int i = 0; i < 2; i++) {
			float separation = dotProduct(frontNormal, clipPoints2[i].v) - front;
			Contact contactt = Contact();
			//If separation is less than 0.0f // if intersecting
			if (separation < 0.0f) {
				//Add values to contacts
				CM = CollisionManifold(normal, separation);
				CM.addContactPoint(clipPoints2[i].v - separation * frontNormal);
				//contacts2[numContacts].separation = separation;
				//contacts2[numContacts].normal = normal;
				// slide contact point onto reference face (easy to cull)
				//contacts2[numContacts].position = clipPoints2[i].v - separation * frontNormal;
				//contacts2[numContacts].feature = clipPoints2[i].fp;
				//if (axis == FACE_B_X || axis == FACE_B_Y)
					//Flip(contacts2[numContacts].feature);
				++numContacts;
			}
		}
		return CM;
	}
	static CollisionManifold findcollisionfeatures(Rect a, Circle b) { return CollisionManifold(); }
	static CollisionManifold findcollisionfeatures(Circle a, Rect b) { return CollisionManifold(); }

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
		sf::Vector2f normal = sf::Vector2f(distance);
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

	sf::VertexArray Line;

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

	static bool pointinQuadrilateral(sf::Vector2f point, Rect box) {

		sf::Vector2f pointBoxOriented = sf::Vector2f(point);
		rotateVector(pointBoxOriented, box.outputShape().getRotation(), box.rigidbody.getPosition());

		sf::Vector2f min = box.getLocalMin();
		sf::Vector2f max = box.getLocalMax();

		return pointBoxOriented.x <= max.x && min.x <= pointBoxOriented.x &&
			pointBoxOriented.y <= max.y && min.y <= pointBoxOriented.y;
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

	static boolean lineinBoxUnrotated(sf::VertexArray line, Rect box) {
		if (pointinQuadrilateral(line[0].position, box) || pointinQuadrilateral(line[1].position, box)) {
			return true;
		}

		sf::Vector2f unitVector = sf::Vector2f(SubtractVectors(line[0].position, line[1].position));
		unitVector = normalise(unitVector);
		unitVector.x = (unitVector.x != 0) ? 1.0f / unitVector.x : 0.0f;
		unitVector.y = (unitVector.y != 0) ? 1.0f / unitVector.y : 0.0f;

		sf::Vector2f min = box.getLocalMin();
		min = MultiplyVectors(SubtractVectors(min, line[0].position), unitVector);
		sf::Vector2f max = box.getLocalMax();
		max = MultiplyVectors(SubtractVectors(max, line[0].position), unitVector);

		float tmin = std::max(std::min(min.x, max.x), std::min(min.y, max.y));
		float tmax = std::min(std::max(min.x, max.x), std::max(min.y, max.y));
		if (tmax < 0 || tmin > tmax) {
			return false;
		}

		float t = (tmin < 0.0f) ? tmax : tmin;
		return t > 0.0f && t * t < lengthofVectorSquared(SubtractVectors(line[0].position, line[1].position));
	}

	static bool lineInBox(sf::VertexArray line, Rect box) {
		float theta = box.outputShape().getRotation();
		sf::Vector2f center = box.rigidbody.getPosition();
		sf::Vector2f localStart = sf::Vector2f(line[0].position);
		sf::Vector2f localEnd = sf::Vector2f(line[1].position);
		rotateVector(localStart, theta, center);
		rotateVector(localEnd, theta, center);

		sf::VertexArray localLine = sf::VertexArray(line);
		Rect localBox = box;

		return lineinBoxUnrotated(localLine, localBox);

	}

	// Raycasting
	static bool raycast(Circle circle, Ray ray, RaycastResult result) {
		RaycastResult().reset(result);

		sf::Vector2f originToCircle = sf::Vector2f(SubtractVectors(circle.rigidbody.getPosition(), ray.getOrigin()));
		float radiusSquared = circle.getRadius() * circle.getRadius();
		float originToCircleLengthSquared = lengthofVectorSquared(originToCircle);

		float a = dotProduct(originToCircle, ray.getDirection());
		float bSq = originToCircleLengthSquared - (a * a);
		if (radiusSquared - bSq < 0.0f) {
			return false;
		}

		float f = sqrtf(radiusSquared - bSq);
		float t = 0;
		if (originToCircleLengthSquared < radiusSquared) {
			// ray start point is inside circle
			t = a + f;
		}
		else {
			t = a - f;
		}

		sf::Vector2f point = sf::Vector2f(ScaleVector(AddVectors(ray.getOrigin(), ray.getDirection()), t));
		sf::Vector2f normal = sf::Vector2f(SubtractVectors(point, circle.rigidbody.getPosition()));
		normal = normalise(normal);

		result.init(point, normal, t, true);
		return true;
	}

	//unrotated
	static bool raycastUnrotated(Rect box, Ray ray, RaycastResult result) {
		RaycastResult().reset(result);

		sf::Vector2f unitVector = ray.getDirection();
		unitVector = normalise(unitVector);
		unitVector.x = (unitVector.x != 0) ? 1.0f / unitVector.x : 0.0f;
		unitVector.y = (unitVector.y != 0) ? 1.0f / unitVector.y : 0.0f;

		sf::Vector2f min = box.getLocalMin();
		min = MultiplyVectors(SubtractVectors(min, ray.getOrigin()), unitVector);
		sf::Vector2f max = box.getLocalMax();
		max = MultiplyVectors(SubtractVectors(max, ray.getOrigin()), unitVector);

		float tmin = std::max(std::min(min.x, max.x), std::min(min.y, max.y));
		float tmax = std::min(std::max(min.x, max.x), std::max(min.y, max.y));
		if (tmax < 0 || tmin > tmax) {
			return false;
		}

		float t = (tmin < 0.0f) ? tmax : tmin;
		bool hit = t > 0.f; 
		if(!hit){
			return false;
		}

		sf::Vector2f point = sf::Vector2f(ScaleVector(AddVectors(ray.getOrigin(),ray.getDirection()),t));
		sf::Vector2f normal = sf::Vector2f(SubtractVectors(ray.getOrigin(), point));
		normal = normalise(normal);

		result.init(point, normal, t, true);

		return true; 
	}

	static bool raycast(Rect box, Ray ray, RaycastResult result) {
		RaycastResult().reset(result);

		sf::Vector2f size = ScaleVector(box.getSize(), 0.5); // half size
		sf::Vector2f xAxis = sf::Vector2f(1.f, 0.f);
		sf::Vector2f yAxis = sf::Vector2f(0.f, 1.f);
		xAxis = rotateVector(xAxis, -box.rigidbody.getRotation(), sf::Vector2f(0.f,0.f));
		yAxis = rotateVector(yAxis, -box.rigidbody.getRotation(), sf::Vector2f(0.f, 0.f));

		sf::Vector2f p = sf::Vector2f(SubtractVectors(box.rigidbody.getPosition(), ray.getOrigin()));
		sf::Vector2f f = sf::Vector2f(
			dotProduct(xAxis, ray.getDirection()), 
			dotProduct(yAxis, ray.getDirection())
		);

		sf::Vector2f e = sf::Vector2f(
			dotProduct(xAxis, p),
			dotProduct(yAxis, p)
		);
		float tArray[4] = { 0,0,0,0 };
			if (compare(f.x, 0)) {
				if (-e.x - size.x > 0 || -e.x + size.x < 0) {
					return false;
				}
			}
			if (compare(f.y, 0)) {
				if (-e.y - size.y > 0 || -e.y + size.y < 0) {
					return false;
				}
			}
			f.x = 0.00001f;
			f.y = 0.00001f;

			tArray[0] = ((e.x + size.x) / f.x);
			tArray[1] = ((e.x - size.x) / f.x);
			tArray[2] = ((e.y + size.y) / f.y);
			tArray[3] = ((e.x - size.x) / f.x);

			float tmin = std::max(std::min(tArray[0], tArray[1]), std::min(tArray[2], tArray[3]));
			float tmax = std::min(std::max(tArray[0], tArray[1]), std::max(tArray[2], tArray[3]));

			float t = (tmin < 0.0f) ? tmax : tmin;
			bool hit = t > 0.f;
			if (!hit) {
				return false;
			}

			sf::Vector2f point = sf::Vector2f(ScaleVector(AddVectors(ray.getOrigin(), ray.getDirection()), t));
			sf::Vector2f normal = sf::Vector2f(SubtractVectors(ray.getOrigin(), point));
			normal = normalise(normal);

			result.init(point, normal, t, true);

			return true;
	}

	static bool circleAndCircle(Circle obj1, Circle obj2) {
		sf::Vector2f joiner = sf::Vector2f(SubtractVectors(obj2.rigidbody.getPosition(), obj1.rigidbody.getPosition()));
		if (lengthofVector(joiner) < (2 * (obj1.getRadius() * obj1.getRadius())) + (2 * (obj2.getRadius() * obj2.getRadius()))) {
			return true;
		}
		else { return false; }
	}
	
	static bool circleAndRectUnrotated(Circle circle, Rect box) {
		sf::Vector2f min = box.getLocalMin();
		sf::Vector2f max = box.getLocalMax();

		sf::Vector2f closestPointToCircle = sf::Vector2f(circle.rigidbody.getPosition());
		if (closestPointToCircle.x < min.x) {
			closestPointToCircle.x = min.x;
		}
		else if (closestPointToCircle.x > max.x) {
			closestPointToCircle.x = max.x;
		}
		if (closestPointToCircle.y < min.y) {
			closestPointToCircle.y = min.y;
		}
		else if (closestPointToCircle.y > max.y) {
			closestPointToCircle.y = max.y;
		}

		sf::Vector2f circletoBox = sf::Vector2f(SubtractVectors(circle.rigidbody.getPosition(), closestPointToCircle));
		return lengthofVectorSquared(circletoBox) <= circle.getRadius() * circle.getRadius();
	}

	static bool circleAndRect(Circle circle, Rect box) {
		// treat min as 0,0
		sf::Vector2f min = sf::Vector2f();
		sf::Vector2f max = sf::Vector2f(box.getSize());

		// visualise cirlce locally
		sf::Vector2f r = sf::Vector2f(SubtractVectors(circle.rigidbody.getPosition(), box.rigidbody.getPosition()));
		r = rotateVector(r, -box.rigidbody.getRotation(), sf::Vector2f(0.f, 0.f));
		sf::Vector2f virtualCirclePos = sf::Vector2f(AddVectors(r, ScaleVector(box.getSize(), 0.5f)));

		sf::Vector2f closestPointToCircle = sf::Vector2f(virtualCirclePos);
		if (closestPointToCircle.x < min.x) {
			closestPointToCircle.x = min.x;
		}
		else if (closestPointToCircle.x > max.x) {
			closestPointToCircle.x = max.x;
		}
		if (closestPointToCircle.y < min.y) {
			closestPointToCircle.y = min.y;
		}
		else if (closestPointToCircle.y > max.y) {
			closestPointToCircle.y = max.y;
		}

		sf::Vector2f circletoBox = sf::Vector2f(SubtractVectors(virtualCirclePos, closestPointToCircle));
		return lengthofVectorSquared(circletoBox) <= circle.getRadius() * circle.getRadius();
	}


	static bool RectandRectUnrotated(Rect box1, Rect box2) {
		sf::Vector2f testAxis[] = { sf::Vector2f(0.f,1.f), sf::Vector2f(1.f,0.f) };
		for (int i = 0; i < (sizeof(testAxis) / sizeof(*testAxis)); i++) {
			if (!overlapAxis(box1, box2, testAxis[i])) {
				return false;
			}
		}
		return true;
	}

	static bool RectandRect(Rect box1, Rect box2) {
		sf::Vector2f testAxis[] = {
			sf::Vector2f(0.f,1.f),
			sf::Vector2f(1.f,0.f),
			sf::Vector2f(0.f,1.f),
			sf::Vector2f(1.f,0.f)
		};

		testAxis[0] = rotateVector(testAxis[0], box1.rigidbody.getRotation(), sf::Vector2f(0.f, 0.f));
		testAxis[1] = rotateVector(testAxis[1], box1.rigidbody.getRotation(), sf::Vector2f(0.f, 0.f));
		testAxis[2] = rotateVector(testAxis[2], box2.rigidbody.getRotation(), sf::Vector2f(0.f, 0.f));
		testAxis[3] = rotateVector(testAxis[3], box2.rigidbody.getRotation(), sf::Vector2f(0.f, 0.f));

		for (int i = 0; i < (sizeof(testAxis) / sizeof(*testAxis)); i++) {
			if (!overlapAxis(box1, box2, testAxis[i])) {
				return false;
			}
		}
		return true;
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
		
		if (a.getPosition().x > b.getPosition().x || a.getPosition().y < b.getPosition().y) {
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
		sf::Vector2f j = ScaleVector(DeltaVel, 1 / invMassSum);
		//if (m.getContactPoints().size() > 0 && j != 0.0f) {
			//j = j / float(m.getContactPoints().size());
		//}

		sf::Vector2f impulse = sf::Vector2f(MultiplyVectors(relativeNormal, j));
		a.setLinearVelocity(sf::Vector2f(AddVectors(a.getLinearVelocity(), (ScaleVector(impulse, (invMass1))))));
		//a.addLocalForce(ScaleVector(-impulse, 1));
		b.setLinearVelocity(sf::Vector2f(AddVectors(b.getLinearVelocity(), (ScaleVector(impulse, (-invMass2))))));
		if (swapped) {
			a.addPosition(ScaleVector(-relativeNormal, (m.getDepth() / 2)));
			b.addPosition(ScaleVector(relativeNormal, (m.getDepth() / 2)));
		}
		else {
			a.addPosition(ScaleVector(relativeNormal, (m.getDepth() / 2)));
			b.addPosition(ScaleVector(-relativeNormal, (m.getDepth() / 2)));
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
		//sf::Vector2f forceAccum = rb.getForceAccum();
		/*
		int k = 4; 
		if (abs(rb.getLinearVelocity().y) < 1.f && abs(relativeNormal.x) < abs(relativeNormal.y)) {
			
			if (rb.getPosition().y > 400) {
				k = 2;
				//j = abs(1.f * ((lengthofVector(g)) * rb.getMass()) + abs(numerator / rb.getInverseMass())) + abs(forceAccum.y);
				//rb.setForceAccumYZero();
				//j = abs(1.f * ((lengthofVector(g)) * rb.getMass())) + abs(numerator / rb.getInverseMass());
				//relativeNormal.y = abs(relativeNormal.y) * -1.f;
			}
			else {
				k = 0;
				//j = numerator / (rb.getInverseMass());
			}
		}
		if (abs(rb.getLinearVelocity().x) < 1.f && abs(relativeNormal.x) > abs(relativeNormal.y)) {
			
			if (rb.getPosition().x > 400) {
				k = 1;
				//j = numerator / (rb.getInverseMass());
				//relativeNormal.x = abs(relativeNormal.x) * -1.f;
			}
			else{
				k = 3;
				//j = numerator / (rb.getInverseMass());
				//relativeNormal.x = abs(relativeNormal.x);
			}
		}
		//else {
			//j = (numerator) / (rb.getInverseMass());
			/*
			switch (sect) {
			case 0: // top
				relativeNormal.y = abs(relativeNormal.y);
				break;
			case 1: // right
				relativeNormal.x = abs(relativeNormal.x) * -1.f;
				break;
			case 2: // bottom
				relativeNormal.y = abs(relativeNormal.y) * -1.f;
				break;
			case 3: // left
				relativeNormal.x = abs(relativeNormal.x);
				break;
			}
			
		//}
		switch (k) {
		case 0 :
			j = numerator / (rb.getInverseMass());
			break;
		case 1:
			j = numerator / (rb.getInverseMass());
			relativeNormal.x = abs(relativeNormal.x) * -1.f;
			break;
		case 2:
			rb.setForceAccumYZero();
			j = abs(1.f * ((lengthofVector(g)) * rb.getMass())) + abs(numerator / rb.getInverseMass());
			relativeNormal.y = abs(relativeNormal.y) * -1.f;
			break;
		case 3:
			j = numerator / (rb.getInverseMass());
			relativeNormal.x = abs(relativeNormal.x);
			break;
		default:
			j = (numerator) / (rb.getInverseMass());
		}
		*/
		
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
};


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
	/*
	void QuadtreeWall(std::vector<Circle> circles, std::vector<sf::Vertex> bounds) {
		for (int i = 0; i < s1Rbs.size(); i++) {

			CollisionManifold m = CollisionManifold();
			//0,1 & 0,4
			std::vector<sf::Vertex> a = {};
			a.push_back(bounds[0]);
			a.push_back(bounds[1]);
			m = IntersectionDetection().lineandCircle(a, s1Rbs[i]);
			if (m != CollisionManifold() && m.isColliding()) {
				s1Rbs
			}

			std::vector<sf::Vertex> b = {};
			b.push_back(bounds[0]);
			b.push_back(bounds[1]);
			m = IntersectionDetection().lineandCircle(b, s1Rbs[i]);

		}
		for (int i = 0; i < circles.size(); i++) {
			if (circles[i].rigidbody.sections.S1 == true || circles[i].rigidbody.sections.S2 == true) {
			}
		}
	}
	*/

};