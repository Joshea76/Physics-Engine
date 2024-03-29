#include <SFML/Graphics.hpp>
#include <Windows.h>
#include <iostream>
#include <list>
#include <vector>
#include <math.h>

#define _USE_MATH_DEFINES

# define M_PI           3.14159265358979323846  /* pi */

//
#include "CollisionManifold.h"
//#include "Quadtree.h"
//#include "Shapes.h"
//#include "Functions.h"
//#include "PhysicsSystem.h"
//#include "ForceGenerator.h"
//#include "Raycast.h"



void testScreen() {
    int bounds[] = { 800,800 };
    std::vector<sf::Vertex> Bounds;
    Bounds.push_back(sf::Vertex(sf::Vector2f(0.f, 0.f)));     //top left
    Bounds.push_back(sf::Vertex(sf::Vector2f(800.f, 0.f)));   //top right
    Bounds.push_back(sf::Vertex(sf::Vector2f(800.f, 800.f))); //bottom right
    Bounds.push_back(sf::Vertex(sf::Vector2f(0.f, 800.f)));   //bottom left

    Quadtree quadtree = Quadtree(Bounds);

    float dt = 0.16;

    sf::RenderWindow window(sf::VideoMode(bounds[0], bounds[1]), "Window");


    ForceGenerator gravity = ForceGenerator(sf::Vector2f(0.f, 1.95f));
    std::vector<ForceGenerator> globalforces = { gravity };
    PhysicsSystem physics = PhysicsSystem(globalforces);

    std::vector<int> objIDs = { 0 };

    std::vector<Circle> circles{};
    std::vector<RigidBody> Objects;


    //Shit tonne of circles
    for (int i = 1; i < 2; i++) {
        for (int j = 1; j < 5; j++){
        Circle circ(((objIDs.back()) + 1), sf::Vector2f((i*51.f) + 150.f, (j*55.f)), 25.f, 0.0f, sf::Color(0.f, 0.f, 255.f));
        objIDs.push_back(objIDs.back() + 1); // create id
        circ.rigidbody.setMass(10.f);
        if (i % 2 == 0) {
            circ.rigidbody.addLocalForce(sf::Vector2f(i * 200.f, -i * 100.f));
        }
        else {
            circ.rigidbody.addLocalForce(sf::Vector2f(-i * 100.f, i * 200.f));

        }
        circles.push_back(circ);
        Objects.push_back(circ.rigidbody);
        }
    }


    // TRIANGLE
    sf::VertexArray triangle(sf::Triangles, 3);

    // define the position of the triangle's points
    triangle[0].position = sf::Vector2f(0.f, 500.f);
    triangle[1].position = sf::Vector2f(0.f, 800.f);
    triangle[2].position = sf::Vector2f(300.f, 800.f);

    // define the color of the triangle's points
    triangle[0].color = sf::Color::Black;
    triangle[1].color = sf::Color::Black;
    triangle[2].color = sf::Color::Black;

    
    // TRIANGLE
    sf::VertexArray triangle2(sf::Triangles, 3);

    // define the position of the triangle2's points
    triangle2[0].position = sf::Vector2f(800.f, 500.f);
    triangle2[1].position = sf::Vector2f(800.f, 800.f);
    triangle2[2].position = sf::Vector2f(500.f, 800.f);

    // define the color of the triangle2's points
    triangle2[0].color = sf::Color::Black;
    triangle2[1].color = sf::Color::Black;
    triangle2[2].color = sf::Color::Black;
    
   
    std::vector<Rect> rectangles{};

    physics.setRigidBodies(Objects);

    std::vector<CollisionManifold> collisions = {};
    std::vector<RigidBody> bodies1 = {};
    std::vector<RigidBody> bodies2 = {};
    
    std::vector<Circle> circleCols = {};

    while (window.isOpen()) {
        Sleep(10);
        sf::Event evnt;
        while (window.pollEvent(evnt)) {
            if (evnt.type == sf::Event::Closed) {
                window.close();
            }
        }

        bodies1.clear();
        bodies2.clear();
        collisions.clear();
        circleCols.clear();


        Quad root = Quad(sf::Vector2f(0, 0), sf::Vector2f(bounds[0], bounds[1]));


        CollisionManifold result = CollisionManifold();

        for (int i = 0; i < circles.size(); i++) {
            Node* n = circles[i].rigidbody.getNode();
            root.insert(n);
        }
        std::vector<Quad*> quads = {};
        root.getLowestQuads(&root, &quads);

        for (int i = 0; i < quads.size(); i++) {

            Sorter sorter = Sorter();
            sorter.sort(quads[i], circles);
            circleCols = sorter.circleCols;


            
            if (circleCols.size() >= 2) {
                for (int c1 = 0; c1 < circleCols.size(); c1++) {
                    for (int c2 = 0; c2 < circleCols.size(); c2++) {
                        if (c1 != c2) {
                            result = Collisions().findcollisionfeatures(circleCols[c1], circleCols[c2]);
                            if (result != CollisionManifold() && result.isColliding()) {
                                bodies1.push_back(circleCols[c1].rigidbody);
                                bodies2.push_back(circleCols[c2].rigidbody);
                                collisions.push_back(result);
                            }
                        }
                    }
                }
            }
        }

        int impulseIterations = 1;
        for (int k = 0; k < impulseIterations; k++) {
            for (int l = 0; l < collisions.size(); l++) {
                int jSize = collisions[l].getContactPoints().size();
                for (int h = 0; h < jSize; h++) {
                    RigidBody r1 = bodies1[l];
                    RigidBody r2 = bodies2[l];
                    std::tuple<RigidBody, RigidBody> rbs = physics.applyImpulse(r1, r2, collisions[l]);
                    if (std::get<0>(rbs).getID() != 0 && std::get<1>(rbs).getID() != 0) {
                        r1 = std::get<0>(rbs);
                        r2 = std::get<1>(rbs);
                        Objects[getRBPositionbyID(Objects, r1.id)] = r1;
                        Objects[getRBPositionbyID(Objects, r2.id)] = r2;
                    }
                    //reassign rigidbodies post impulse
                }
            }
        }

        bodies1.clear();
        bodies2.clear();
        collisions.clear();

        // find Collisions
        circles = quadtree.sortCircles(circles);

            for (int i = 0; i < Objects.size(); ++i) {
                
            if (circles[i].rigidbody.sections.S1 || circles[i].rigidbody.sections.S2) { // Top wall
                CollisionManifold wallCollide = IntersectionDetection().wallCollide(std::vector<sf::Vertex>{Bounds[0], Bounds[1]}, circles[i]);
                if (wallCollide != CollisionManifold() && wallCollide.isColliding()) {
                    Objects[i] = physics.applyImpulse(Objects[i], wallCollide,0);
                }
            }
            if (circles[i].rigidbody.sections.S2 || circles[i].rigidbody.sections.S3) { // Right wall
                CollisionManifold wallCollide = IntersectionDetection().wallCollide(std::vector<sf::Vertex>{Bounds[1], Bounds[2]}, circles[i]);
                if (wallCollide != CollisionManifold() && wallCollide.isColliding()) {
                    Objects[i] = physics.applyImpulse(Objects[i], wallCollide,1);
                }
            }
            if (circles[i].rigidbody.sections.S3 || circles[i].rigidbody.sections.S4) { // Bottom wall
                CollisionManifold wallCollide = IntersectionDetection().wallCollide(std::vector<sf::Vertex>{Bounds[3], Bounds[2]}, circles[i]);
                if (wallCollide != CollisionManifold() && wallCollide.isColliding()) {
                    Objects[i] = physics.applyImpulse(Objects[i], wallCollide,2);
                }
                CollisionManifold tCollide = IntersectionDetection().wallCollide(std::vector<sf::Vertex>{triangle[0],triangle[2]}, circles[i]);
                if (tCollide != CollisionManifold() && tCollide.isColliding()) {
                    Objects[i] = physics.applyImpulse(Objects[i], tCollide);
                }

                CollisionManifold tCollide2 = IntersectionDetection().wallCollide(std::vector<sf::Vertex>{triangle2[0], triangle2[2]}, circles[i]);
                if (tCollide2 != CollisionManifold() && tCollide2.isColliding()) {
                    Objects[i] = physics.applyImpulse(Objects[i], tCollide2);
                }
            }
            if (circles[i].rigidbody.sections.S4 || circles[i].rigidbody.sections.S1) { // Left wall
                CollisionManifold wallCollide = IntersectionDetection().wallCollide(std::vector<sf::Vertex>{Bounds[0], Bounds[3]}, circles[i]);
                if (wallCollide != CollisionManifold() && wallCollide.isColliding()) {
                    Objects[i] = physics.applyImpulse(Objects[i], wallCollide,3);
                }

                
            }
        }
        
        
        

        // apply forces
        physics.setRigidBodies(Objects);
        Objects = physics.applyForces(dt);
        for (int i = 0; i < Objects.size(); i++) {
            Objects[i].removeLocalForces();
        }
        for (int i = 0; i < circles.size(); i++) {
            circles[i].setRigidbody(Objects[i]);
            circles[i].rigidbody.sections.reset();
        }
        
        // Draw
        window.clear();
        for (int i = 0; i < circles.size(); i++) { window.draw(circles[i].outputShape()); }

        window.draw(triangle);
        window.draw(triangle2);
        window.display();
    }
}



int main() {
    testScreen();
    return 0;
}
