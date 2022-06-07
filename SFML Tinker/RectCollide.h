#pragma once
#include <SFML/Graphics.hpp>
#include <Windows.h>
#include <iostream>
#include <vector>
#include <math.h>

#include "Shapes.h"

/*
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
		//float a = ((this->col1.x * mat2.col1.x /*ae*///) + (this->col2.x * mat2.col1.y /*bg*/));
		/*float b = ((this->col1.x * mat2.col2.x /*af*///) + (this->col2.x * mat2.col2.y /*bh*/));


		//float c = ((this->col1.y * mat2.col1.x /*ce*/) + (this->col2.y * mat2.col1.y /*dg*/));
		//float d = ((this->col1.y * mat2.col2.x /*cf*/) + (this->col2.y * mat2.col2.y /*dh*/));

		//return MatrixTbyT(sf::Vector2f(a, c), sf::Vector2f(b, d)); // col1 = ac, col2 = bd

		/*	| a b |		| e f |		~		| ae+bg af+bh |
		*	| c d |		| g h |		~		| ce+dg cf+dh |

			| col1.x  col2.x |
			| col1.y  col2.y |
		*/
	//}
	/*
	MatrixTbyT absolute() {
		return MatrixTbyT(sf::Vector2f(abs(col1.x), abs(col1.y)), sf::Vector2f(abs(col2.x), abs(col2.y)));
	}

};
/*
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

 collideErinCatto(Rect BodyA, Rect BodyB) {


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