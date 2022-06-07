#pragma once
#include <SFML/Graphics.hpp>
#include <Windows.h>
#include <iostream>
#include <vector>
#include <math.h>
/*
#include "Functions.h"	
#include "Shapes.h"
#include "CollisionManifold.h"


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
		createLines();
	}

	void createLines() {
		this->vLine[0] = Midpoint(bounds[0].position, bounds[1].position);
		this->vLine[1] = Midpoint(bounds[4].position, bounds[3].position);

		this->hLine[0] = Midpoint(bounds[0].position, bounds[4].position);
		this->hLine[1] = Midpoint(bounds[1].position, bounds[3].position);
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

	void QuadtreeWall(std::vector<Circle> circles, std::vector<sf::Vertex> bounds) {
		for (int i = 0; i < s1Rbs.size(); i++) {

			CollisionManifold m = CollisionManifold();
			//0,1 & 0,4
			std::vector<sf::Vertex> a = {};
			a.push_back(bounds[0]);
			a.push_back(bounds[1]);
			m = IntersectionDetection().lineandCircle(a,s1Rbs[i]);
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

};
*/