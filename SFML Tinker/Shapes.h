#pragma onceLinearVelocity
#include <SFML/Graphics.hpp>
#include <Windows.h>
#include <iostream>
#include <vector>
#include <math.h>

#include "Functions.h"
#include "ForceGenerator.h"
#include "Quadtree.h"

class Section
{
public:
    bool S1, S2, S3, S4;
    Section() {
        S1 = false;
        S2 = false;
        S3 = false;
        S4 = false;
    }

    void reset() {
        S1 = false;S2 = false;S3 = false;S4 = false;
    }

    void setAllTrue() {
        S1 = true;S2 = true;S3 = true;S4 = true;
    }

};



class RigidBody {
private:
    sf::Vector2f position, linearVelocity, forceAccum;
    sf::Vector2f linearDamping = sf::Vector2f(0.999999f, 0.999999f);
    float rotation, angularVeclocity, angularDamping, mass, inversemass;
    float coefficientRestitution = 0.8f;
    bool fixedRotation;

    std::vector<ForceGenerator> localForces = {};
    
    //Collider collider;

public:
    Section sections;
    Node* node = new Node;
    int id;
    bool operator == (const RigidBody& fr) const { return (this->id == fr.id); }
    bool operator != (const RigidBody& fr) const { return !operator==(fr); }


    RigidBody() {
        this->id = 0;
        node->id = this->id;
    }
    // Create Constructors
    RigidBody(sf::Vector2f position, float mass, float rotation) {
        this->position = position;
        this->node->pos = position;
        this->setMass(mass);
        this->rotation = rotation;
        sections.reset();
    }
    RigidBody(sf::Vector2f position, float mass){
        this->position = position;
        this->node->pos = position;
        this->setMass(mass);
        sections.reset();
    }
    RigidBody(sf::Vector2f position) {
        this->position = position;
        this->node->pos = position;
        sections.reset();
    }

    void setID(int id) {
        this->id = id;
        this->node->id = id;
    }
    int getID() {
        return this->id;
    }

    void setPosition(sf::Vector2f v) { position = v; }
    sf::Vector2f getPosition() { return position; }
    void addPosition(sf::Vector2f v) { this->position = AddVectors(this->position, v); }

    void setRotation(float rotation) { this->rotation = rotation; }
    float getRotation() { return rotation; }

    void setMass(float mass) {
        this->mass = mass;
        if (mass != 0.0f) {
            this->inversemass = 1.0f / mass;
        }
    }
    float getMass() { return mass; }

    std::vector<ForceGenerator> getLocalForces() {
        return this->localForces;
    }

    bool hasInfinteMass() {
        return this->mass == 0.0f;
    }

    void setTransform(sf::Vector2f position, float rotation) {
        this->position = position;
        this->rotation = rotation;
    }
    void setTransform(sf::Vector2f position) {
        this->position = position;
    }

    float getInverseMass() {
        return this->inversemass;
    }

    void physicsUpdate(float dt) {
        if (this->mass == 0.0f) { return; }

        //Calc Linear Velocity
        sf::Vector2f acceleration = sf::Vector2f(ScaleVector(forceAccum, this->inversemass));
        this->linearVelocity += ScaleVector(acceleration, dt);
        linearVelocity = MultiplyVectors(linearVelocity,(pow(linearDamping, dt)));

        // Update Linear Pos
        this->position = AddVectors(this->position, ScaleVector(linearVelocity, dt));

        this->clearAccumulators();
    }

    void setCor(float r) { this->coefficientRestitution = r; }
    float getCor() { return this->coefficientRestitution; }

    void clearAccumulators() {
        this->forceAccum = sf::Vector2f(0.f, 0.f);
    }

    sf::Vector2f addForce(sf::Vector2f force) {
        this->forceAccum += AddVectors(forceAccum, force);
        return this->forceAccum;
    }

    void addLocalForce(sf::Vector2f f) {
        this->localForces.push_back(f);
        this->addForce(f);
    }
    void removeLocalForces() { this->localForces = {}; }

    sf::Vector2f getLinearVelocity() { return this->linearVelocity; }
    void setLinearVelocity(sf::Vector2f v) { this->linearVelocity = v; }
    void addLinearVelocity(sf::Vector2f v) { this->linearVelocity = AddVectors(this->linearVelocity, v); }

    float getAngularVelocty() { return this->angularVeclocity; }
    void setAngularVelocity(float v) { this->angularVeclocity = v; }
    sf::Vector2f getForceAccum() { return this->forceAccum; }
    void setForceAccumYZero() { this->forceAccum.y = 0; }
    void setForceAccumXZero() { this->forceAccum.x = 0; }

    Node* getNode() {
        this->node->pos = this->position;
        return node;
    }
};

class Circle {
private:
    sf::Vector2f size;
    sf::CircleShape self;
    float mass, radius;
    bool pickedUp = false;
    int id;
public:

    RigidBody rigidbody;

    bool operator == (const RigidBody& fr) const { return (this->id == fr.id); }
    bool operator != (const RigidBody& fr) const { return !operator==(fr); }
    bool operator == (const Circle& fr) const { return (this->id == fr.id); }
    bool operator != (const Circle& fr) const { return !operator==(fr); }

    Circle() {
        this->id = 0;
        self.setOrigin(sf::Vector2f(0, 0));
    }

    Circle(int id, sf::Vector2f position, float radius, float rotation, sf::Color color) {
        this->id = id;
        this->rigidbody.setID(id);

        this->rigidbody.setPosition(position);
        this->self.setPosition(position);
        this->self.setOrigin(radius,radius);

        this->radius = radius;
        this->self.setRadius(radius);

        this->rigidbody.setRotation(rotation);
        this->self.setRotation(rotation);

        this->self.setFillColor(color);

    }
    Circle(int id, sf::Vector2f position, float radius, float rotation) {
        this->id = id;
        this->rigidbody.setID(id);

        this->rigidbody.setPosition(position);
        this->self.setPosition(position);
        this->self.setOrigin(radius, radius);

        this->radius = radius;
        this->self.setRadius(radius);

        this->rigidbody.setRotation(rotation);
        this->self.setRotation(rotation);

    }
    Circle(int id, sf::Vector2f position, float radius) {
        this->id = id;
        this->rigidbody.setID(id);

        this->rigidbody.setPosition(position);
        this->self.setPosition(position);

        this->radius = radius;
        this->self.setRadius(radius);
        this->self.setOrigin(radius, radius);

        this->rigidbody.setRotation(0);
        this->self.setRotation(0);

    }
    Circle(int id, sf::Vector2f position, float radius, sf::Color color) {
        this->id = id;
        this->rigidbody.setID(id);

        this->rigidbody.setPosition(position);
        this->self.setPosition(position);

        this->radius = radius;
        this->self.setRadius(radius);
        this->self.setOrigin(radius, radius);
        
        this->rigidbody.setRotation(0);
        this->self.setRotation(0);

        this->self.setFillColor(color);

    }

    void setRigidbodyToSelf(RigidBody rb) {
        this->self.setPosition(rb.getPosition());
        this->self.setRotation(rb.getRotation());
    }

    int isPickedUp() {
        if (pickedUp == true) { return 1; }
        else { return 0; }
    }
    void picked() {
        pickedUp = true;
    }
    void dropped() {
        pickedUp = false;
    }

    void setRigidbody(RigidBody rb) {
        this->rigidbody = rb;
    }

    void setID(int id) {
        this->id = id;
    }
    int getID() {
        return this->id;
    }

    void setColor(sf::Color col) { self.setFillColor(col); }
    void setSize(sf::Vector2f s) {
        size = s;
        //self.setSize(size);
        self.setRadius(s.x / 2);
        self.setOrigin(s.x / 2, s.y / 2);
        setRadius(s.x / 2);
    }
    

    void setRadius(float r) { radius = r; self.setRadius(r); }
    float getRadius() { 
        return this->radius;
    }

    

    sf::CircleShape outputShape() { 
        self.setPosition(rigidbody.getPosition());
        self.setRotation(rigidbody.getRotation());
        return self; 
    }


};



class Rect {
private:
    sf::Vector2f size, acceleration;
    sf::RectangleShape self;
    float mass;
    bool pickedUp = false;
    int id;

public:
    RigidBody rigidbody;

    bool operator == (const RigidBody& fr) const { return (this->id == fr.id); }
    bool operator != (const RigidBody& fr) const { return !operator==(fr); }
    bool operator == (const Rect& fr) const { return (this->id == fr.id); }
    bool operator != (const Rect& fr) const { return !operator==(fr); }

    Rect(int id, sf::Vector2f position, sf::Vector2f size, float rotation, sf::Color color) {
        this->id = id;
        this->rigidbody.setID(id);

        this->rigidbody.setPosition(position);
        this->self.setPosition(position);

        this->size = size;
        this->self.setSize(size);

        this->rigidbody.setRotation(rotation);
        this->self.setRotation(rotation);

        this->self.setFillColor(color);
    }
    Rect(int id, sf::Vector2f position, sf::Vector2f size, float rotation) {
        this->id = id;
        this->rigidbody.setID(id);

        this->rigidbody.setPosition(position);
        this->self.setPosition(position);

        this->size = size;
        this->self.setSize(size);

        this->rigidbody.setRotation(rotation);
        this->self.setRotation(rotation);
    }
    Rect(int id, sf::Vector2f position, sf::Vector2f size) {
        this->id = id;
        this->rigidbody.setID(id);

        this->rigidbody.setPosition(position);
        this->self.setPosition(position);

        this->size = size;
        this->self.setSize(size);

        this->rigidbody.setRotation(0);
        this->self.setRotation(0);
    }
    Rect(int id, RigidBody rb, sf::Vector2f size) {
        this->id = id;
        this->rigidbody.setID(id);

        this->rigidbody = rb;
        setRigidbodyToSelf(rb);

        this->size = size;
        this->self.setSize(size);

        this->rigidbody.setRotation(0);
        this->self.setRotation(0);
    }
    Rect(int id, RigidBody rb, sf::Vector2f size, sf::Color color) {
        this->id = id;
        this->rigidbody.setID(id);

        this->rigidbody = rb;
        setRigidbodyToSelf(rb);

        this->size = size;
        this->self.setSize(size);

        this->rigidbody.setRotation(0);
        this->self.setRotation(0);

        this->self.setFillColor(color);
    }

    void setRigidbodyToSelf(RigidBody rb) {
        this->self.setPosition(rb.getPosition());
        this->self.setRotation(rb.getRotation());
    }

    void setID(int id) {
        this->id = id;
    }
    int getID() {
        return this->id;
    }

    int isPickedUp() {
        if (pickedUp == true) { return 1; }
        else { return 0; }
    }
    void picked() {
        pickedUp = true;
    }
    void dropped() {
        pickedUp = false;
    }

    void setColor(sf::Color col) { self.setFillColor(col); }

    void setRigidbody(RigidBody rb) {
        this->rigidbody = rb;
    }
    void setAccX(float x) { acceleration.x = x; }
    void setAccY(float y) { acceleration.y = y; }


    sf::Vector2f getLocalMin() {
        return sf::Vector2f(rigidbody.getPosition().x - size.x / 2, rigidbody.getPosition().y - size.y / 2);
    }
    sf::Vector2f getLocalMax() {
        return sf::Vector2f(rigidbody.getPosition().x + size.x / 2, rigidbody.getPosition().y + size.y / 2);
    }


    sf::Vector2f getSize() {
        //self.setRadius(Radius);
        self.setSize(size);
        return self.getSize();
    }
    sf::Vector2f getHalfSize() {
        self.setSize(size);
        return ScaleVector(size, 0.5f);
    }


    sf::RectangleShape outputShape() { 
        self.setPosition(rigidbody.getPosition());
        self.setRotation(rigidbody.getRotation());
        return self; 
    }
};

Circle getCircleByID(std::vector<Circle> circles, int rbID) {
    for (Circle c : circles) {
        if (c.getID() == rbID) {
            return c;
        }
    }
}

int getRBPositionbyID(std::vector<RigidBody> rbs, int rbID) {
    for (int i = 0; i < rbs.size(); i++) {
        if (rbs[i].id == rbID) {
            return i;
        }
    }
}

Rect getRectByID(std::vector<Rect> circles, int rbID) {
    for (Rect c : circles) {
        if (c.getID() == rbID) {
            return c;
        }
    }
}


class Sorter {
public:
    std::vector<Circle> circleCols;

    Sorter() {
        circleCols = {};
    }
    Sorter(std::vector<Circle> c) {
        circleCols = c;
    }

    void sort(Quad* q, std::vector<Circle> circles) {

        if (q->topLeftTree != NULL) {
            if (q->topLeftTree->n->id != -842150451) {
                circleCols.push_back(getCircleByID(circles, q->topLeftTree->n->id));
            }
            else {
                sort(q->topLeftTree, circles);
            }
        }
        if (q->topRightTree != NULL) {
            if (q->topRightTree->n->id != -842150451) {
                circleCols.push_back(getCircleByID(circles, q->topRightTree->n->id));
            }
            else {
                sort(q->topRightTree, circles);
            }
        }
        if (q->btmLeftTree != NULL) {
            if (q->btmLeftTree->n->id != -842150451) {
                circleCols.push_back(getCircleByID(circles, q->btmLeftTree->n->id));
            }
            else {
                sort(q->btmLeftTree, circles);
            }
        }
        if (q->btmRightTree != NULL) {
            if (q->btmRightTree->n->id != -842150451) {
                circleCols.push_back(getCircleByID(circles, q->btmRightTree->n->id));
            }
            else {
                sort(q->btmRightTree, circles);
            }
        }
    }
};