#pragma once
#include <SFML/Graphics.hpp>
#include <Windows.h>
#include <iostream>
#include <list>
#include <math.h>
#include "Functions.h"

class Ray {
private:
	sf::Vector2f origin;
	sf::Vector2f direction;

public:
	Ray(sf::Vector2f origin, sf::Vector2f direction) {
		this->origin = origin;
		this->direction = direction;
		this->direction = normalise(direction);		
	}

	sf::Vector2f getOrigin() {
		return this->origin;
	}
	sf::Vector2f getDirection() {
		return this->direction;
	}
};

class RaycastResult {
private:
	sf::Vector2f point;
	sf::Vector2f normal;
	float t;
	bool hit;

public:
	RaycastResult() {
		this->point = sf::Vector2f();
		this->normal = sf::Vector2f()	;
		this->t = -1;
		this->hit = false;
	}

	void init(sf::Vector2f point, sf::Vector2f normal, float t, bool hit) {
		this->point = point;
		this->normal = normal;
		this->t = t;
		this->hit = hit;
	}

	static void reset(RaycastResult result) {
		result.point = sf::Vector2f(0, 0);
		result.normal = sf::Vector2f(0, 0);
		result.t = -1;
		result.hit = false;
	}

};