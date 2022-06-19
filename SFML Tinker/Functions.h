#pragma once
#include <SFML/Graphics.hpp>
#include <Windows.h>
#include <iostream>
#include <list>
#include <math.h>


#define _USE_MATH_DEFINES

# define M_PI           3.14159265358979323846  /* pi */


sf::Vector2f SubtractVectors(sf::Vector2f a, sf::Vector2f b) {
	return sf::Vector2f((a.x - b.x), (a.y - b.y));
}
sf::Vector2f AddVectors(sf::Vector2f a, sf::Vector2f b) {
    return sf::Vector2f((a.x + b.x), (a.y + b.y));
}
sf::Vector2f MultiplyVectors(sf::Vector2f a, sf::Vector2f b) {
    return sf::Vector2f((a.x * b.x), (a.y * b.y));
}
sf::Vector2f ScaleVector(sf::Vector2f v, float s) {
    return sf::Vector2f((v.x * s), (v.y * s));
}
sf::Vector2f AddToVector(sf::Vector2f v, float a) {
    return sf::Vector2f(v.x + a, v.y + a);
}

sf::Vector2f resultForce(sf::Vector2f V1, sf::Vector2f V2) {
	return sf::Vector2f(V1.x + V2.x, V1.y + V2.y);
}
sf::Vector2f inverse(sf::Vector2f v) { return sf::Vector2f(-v.x, -v.y); }

float dotProduct(sf::Vector2f v1, sf::Vector2f v2) {
    return v1.x * v2.x + v1.y * v2.y;
}

float lengthofVector(sf::Vector2f v) {
    //std::cout << ((v.x * v.x) + (v.y * v.y)) << std::endl;
    return sqrt((v.x * v.x) + (v.y * v.y));
}
float lengthofVectorSquared(sf::Vector2f v) {
    //std::cout << ((v.x * v.x) + (v.y * v.y)) << std::endl;
    return (v.x * v.x) + (v.y * v.y);
}

sf::Vector2f perpendicularVector(sf::Vector2f& vector)
{
    return { -vector.y, vector.x };
}

sf::Vector2f normalise(sf::Vector2f v) {
    float m = lengthofVector(v);
    return sf::Vector2f((v.x / m), (v.y / m));
}

float DegtoRadians(float angle) {
    return angle * (M_PI / 180);
}

sf::Vector2f abs(sf::Vector2f v) { return sf::Vector2f(std::abs(v.x), std::abs(v.y)); }

float Sign(float x) {
    return x < 0.0f ? -1.0f : 1.0f;
}

sf::Vector2f rotateVector(sf::Vector2f vec, float angle, sf::Vector2f origin) {
    float x = vec.x - origin.x;
    float y = vec.y - origin.y;

    float cosine = cos(DegtoRadians(angle));
    float sine = sin(DegtoRadians(angle));

    float newx = (x * cosine) - (y * sine);
    float newy = (x * sine) + (y * cosine);

    newx += origin.x;
    newy += origin.y;

    vec.x = newx;
    vec.y = newy;

    return vec;
}

static bool compare(float x, float y, float decimalplaces) {
    return abs(x - y) <= decimalplaces;
}

static bool compare(sf::Vector2f vec1, sf::Vector2f vec2, float decimalplaces) {
    return compare(vec1.x, vec2.x, decimalplaces) && compare(vec1.y, vec2.y, decimalplaces);
}

static bool compare(float x, float y) {
    return abs(x - y) <= 0.0001;
}

static bool compare(sf::Vector2f vec1, sf::Vector2f vec2) {
    return compare(vec1.x, vec2.x) && compare(vec1.y, vec2.y);
}

static sf::Vector2f pow(sf::Vector2f x, float exp) {
    return sf::Vector2f(pow(x.x, exp), pow(x.y, exp));
}

sf::Vector2f Midpoint(sf::Vector2f a, sf::Vector2f b) {
    return sf::Vector2f((a.x + b.x) / 2, (a.y + b.y) / 2);
}


class Transform {
public: 
    sf::Vector2f position;
    sf::Vector2f scale;

    Transform() {
        this->position = sf::Vector2f(0.f, 0.f);
        this->scale = sf::Vector2f(0.f, 0.f);
    }
    Transform(sf::Vector2f position) {
        this->position = position;
        this->scale = sf::Vector2f(0.f, 0.f);
    }
    Transform(sf::Vector2f position, sf::Vector2f scale) {
        this->position = position;
        this->scale = scale;
    }

    Transform copy() {
        return Transform(sf::Vector2f(this->position), sf::Vector2f(this->scale));
    }

    void copy(Transform to) {
        to.position = this->position;
        to.scale = this->scale;
    }
};