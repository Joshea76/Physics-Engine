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
#include "PhaseCollision.h"
//#include "Shapes.h"
//#include "Functions.h"
//#include "PhysicsSystem.h"
//#include "ForceGenerator.h"
//#include "Raycast.h"




/*
bool collide(Circle obj1, Circle obj2) {
    sf::Vector2f joiner = sf::Vector2f((obj2.Position.x - obj1.Position.x), (obj2.Position.y - obj1.Position.y));
    if (lengthofVector(joiner) < (2*(obj1.Radius * obj1.Radius)) + (2*(obj2.Radius* obj2.Radius))) {
        std::cout << "HIT" << std::endl;
        return true;
    }
    else { return false; }
}


int wallCollide(Circle obj, int bounds[]) {
    if (obj.Position.x >= bounds[0] - (obj.Radius)) {
        return 1;
    }
    if (obj.Position.x <= 0 + (obj.Radius)) {
        return 1;
    }
    if (obj.Position.y >= bounds[1] - (obj.Radius)) {
        return 2;
    }
    if (obj.Position.y <= 0 + (obj.Radius)) {
        return 2;
    }
}


bool doNotIntersect(Circle obj1, Circle obj2){
    if (int(obj1.Position.x) == int(obj2.Position.x) && int(obj1.Position.y) == int(obj2.Position.y)) {
        std::cout << "Balls" << std::endl;
        return true;
    }
    else { return false; }
}

int ClickClass() {
    int bounds[] = { 800,800 };
    float dampner = 0.5;

    sf::RenderWindow window(sf::VideoMode(bounds[0], bounds[1]), "Window");
    sf::Vector2f vec(25.f, 25.f);
    sf::Color col(255, 0, 0);

    Circle shape;
    shape.setSize(vec);
    shape.setPosition(sf::Vector2f(50.f, 50.f));
    shape.setVelx(1.f);
    shape.setColor(col);

    Circle box;
    box.setSize(vec);
    box.setPosition(sf::Vector2f(150.f, 150.f));
    box.setColor(sf::Color(0, 255, 0));
    box.setVelocity(sf::Vector2f(1.f,1.f));

    Circle sqaure;
    sqaure.setSize(vec);
    sqaure.setPosition(sf::Vector2f(100.f, 50.f));
    sqaure.setVelx(-1.f);
    sqaure.setColor(sf::Color(0, 0, 255));

    sf::Vector2i mousepos(0, 0);

    bool mouse_selection = false;

    //v = u+at;
float t = 0.07;
float g = 9.8f;
bool pickable = false;

//Setup For ALL Variables /// 
Circle Objects[] = { shape,box,sqaure };
for (int i = 0; i < (sizeof(Objects) / sizeof(*Objects)); ++i) {
    Objects[i].setAccY((g * (t * t)) / 2);
    Objects[i].setMass(i + 1);
    //Objects[i].setVelx(i + 1);
    Objects[i].setDamping(sf::Vector2f(0.995, 0.995));
}

while (window.isOpen()) {
    Sleep(10);
    sf::Event evnt;
    while (window.pollEvent(evnt)) {
        if (evnt.type == sf::Event::Closed) {
            window.close();
        }
        if (evnt.type == (sf::Event::KeyPressed)) {
            float angle;
            std::cin >> angle;
            Objects[2].applyForce(angle, 50);
        }
    }
    mousepos = sf::Mouse::getPosition(window);
    for (int i = 0; i < (sizeof(Objects) / sizeof(*Objects)); ++i) {
        if (IntersectionDetection().pointinCircle(sf::Vector2f(mousepos.x,mousepos.y),Objects[i]) && sf::Mouse::isButtonPressed(sf::Mouse::Left)) {
            Objects[i].picked();

        }
        else {
            Objects[i].dropped();
        }
        if (Objects[i].isPickedUp()) {
            Objects[i].setPosition(sf::Vector2f(mousepos.x, mousepos.y));
            Objects[i].setVelx(0);
            Objects[i].setVely(0);
            Objects[i].outputPosition();
        }
        else {
            //Objects[i].isInbounds(bounds);
            if (wallCollide(Objects[i], bounds) == 1) {
                Objects[i].setVelx(-Objects[i].LinearVelocity.x * dampner);
            }
            if (wallCollide(Objects[i], bounds) == 2) {
                Objects[i].setVely(-Objects[i].LinearVelocity.y * dampner);
                Objects[i].changeVely((-Objects[i].Acceleration.y));

            }
            Objects[i].gravityDragLoop();
            for (int j = 0; j < (sizeof(Objects) / sizeof(*Objects)); ++j) {
                if (i != j) {
                    if (collide(Objects[i], Objects[j])) {

                        sf::Vector2f joiner = sf::Vector2f((Objects[j].Position.x - Objects[i].Position.x), (Objects[j].Position.y - Objects[i].Position.y));
                        /*
                        sf::Vector2f v = perpendicularVector(resultForce(Objects[i].Velocity, Objects[j].Velocity));
                        sf::Vector2f joiner = sf::Vector2f((Objects[j].Position.x - Objects[i].Position.x), (Objects[j].Position.y - Objects[i].Position.y));

                        if (int(Objects[i].Velocity.x) == 0 && int(Objects[j].Velocity.x) == 0) {
                            v.x = 0;
                        }

                            Objects[i].setVelocity(sf::Vector2f(v.x * dampner, v.y * dampner));
                            Objects[j].setVelocity(inverse(sf::Vector2f(v.x * dampner, v.y * dampner)));

                            if (Objects[i].Position.y < Objects[j].Position.y) {
                                Objects[i].changeVely(-0.5 * (Objects[i].Acceleration.y));
                            }
                            else if (Objects[j].Position.y < Objects[i].Position.y){
                                Objects[j].changeVely(-0.5 * (Objects[j].Acceleration.y));
                            }
                            if (Objects[i].Position.x < Objects[j].Position.x) {
                                Objects[i].changeVelx(-0.3 * (Objects[i].Damping.x));
                                Objects[j].changeVelx(0.3 * (Objects[j].Damping.x));

                            }
                            else if (Objects[j].Position.x < Objects[i].Position.x) {
                                Objects[i].changeVelx(0.3 * (Objects[i].Damping.x));
                                Objects[j].changeVelx(-0.3 * (Objects[j].Damping.x));
                            }

                            


                        if (Objects[i].Position.y < Objects[j].Position.y || Objects[i].Position.y > Objects[j].Position.y) { // i below j
                            Objects[i].setVely(-Objects[i].LinearVelocity.y * dampner);
                            Objects[j].setVely(-Objects[j].LinearVelocity.y * dampner);
                        }
                        if (Objects[i].Position.x < Objects[j].Position.x || Objects[i].Position.x > Objects[j].Position.x) { // i left of j
                            Objects[i].setVelx(-Objects[i].LinearVelocity.x * dampner);
                            Objects[j].setVelx(-Objects[j].LinearVelocity.x * dampner);
                        }
                        
                        sf::Vector2f v = resultForce(Objects[i].Velocity, Objects[j].Velocity);

                        Objects[i].setVely(v.y);
                        Objects[i].setVelx(v.x);

                        Objects[j].setVely(-v.y);
                        Objects[j].setVelx(-v.x);
                        

                        //Objects[i].penetrate(Objects[j]);

                    }
                    
                        if (doNotIntersect(Objects[i], Objects[j])) {
                            Objects[i].setPosition(sf::Vector2f(Objects[i].Position.x - Objects[i].Size.x, Objects[i].Position.y));
                            Objects[j].setPosition(sf::Vector2f(Objects[i].Position.x + Objects[i].Size.x, Objects[i].Position.y));
                        }
                    }
                }
                Objects[i].applyVelocity();
                //std::cout << Objects[i].Velocity.y << std::endl;
                Objects[i].outputPosition();
            }
        }
        window.clear();
        for (int i = 0; i < (sizeof(Objects) / sizeof(*Objects)); ++i) {
        window.draw(Objects[i].outputShape());
        }

        sf::VertexArray line;
        line.append(sf::Vertex(sf::Vector2f(10.f, 10.f)));
        line.append(sf::Vertex(sf::Vector2f(20.f, 20.f)));

        window.draw(line);
        window.display();
    }

    return 0;
}

*/

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



    //PhysicsSystem physics = PhysicsSystem(1.0f / 60.f, sf::Vector2f(0.f, -10.f));
    //Transform obj1, obj2;
    //RigidBody rb1, rb2;

    /*Gabes Shit
    obj1 = Transform(sf::Vector2f(100.f, 300.f));
    obj2 = Transform(sf::Vector2f(200.f, 300.f));
    rb1 = RigidBody();
    rb2 = RigidBody();
    rb1.setRawTransform(obj1);
    rb2.setRawTransform(obj2);
    rb1.setMass(100.f);
    rb2.setMass(200.f);

    physics.addRigidbody(rb1);
    physics.addRigidbody(rb2);
    */


    ForceGenerator gravity = ForceGenerator(sf::Vector2f(0.f, 0.95f));
    std::vector<ForceGenerator> globalforces = { gravity };
    PhysicsSystem physics = PhysicsSystem(globalforces);

    std::vector<int> objIDs = { 0 };

    Circle circ1(((objIDs.back()) + 1), sf::Vector2f(700.f, 100.f), 25.f, 0.0f, sf::Color(255.f, 0.f, 0.f));
    objIDs.push_back(objIDs.back() + 1); // create id
    circ1.rigidbody.setMass(10.f);
    //circ1.rigidbody.addLocalForce(sf::Vector2f(-100.f, -100.f));
    Circle circ2(((objIDs.back()) + 1), sf::Vector2f(150.f, 505.f), 25.f, 0.0f, sf::Color(0.f, 0.f, 255.f));
    objIDs.push_back(objIDs.back() + 1); // create id
    circ2.rigidbody.setMass(10.f);
    
    Circle circ3(((objIDs.back()) + 1), sf::Vector2f(250.f, 380.f), 25.f, 0.0f, sf::Color(0.f, 255.f, 0.f));
    objIDs.push_back(objIDs.back() + 1); // create id
    circ3.rigidbody.setMass(10.f);
    
    Circle circ4(((objIDs.back()) + 1), sf::Vector2f(225.f, 200.f), 25.f, 0.0f, sf::Color(125.f, 125.f, 0.f));
    objIDs.push_back(objIDs.back() + 1); // create id
    circ4.rigidbody.setMass(10.f);
    Circle circ5(((objIDs.back()) + 1), sf::Vector2f(60.f, 200.f), 25.f, 0.0f, sf::Color(0.f, 125.f, 125.f));
    objIDs.push_back(objIDs.back() + 1); // create id
    circ5.rigidbody.setMass(10.f);

    Circle circ6(((objIDs.back()) + 1), sf::Vector2f(50.f, 50.f), 25.f, 0.0f, sf::Color(125.f, 0.f, 125.f));
    objIDs.push_back(objIDs.back() + 1); // create id
    circ6.rigidbody.setMass(10.f);

    Circle circ7(((objIDs.back()) + 1), sf::Vector2f(380.f, 250.f), 25.f, 0.0f, sf::Color(200.f, 200.f, 200.f));
    objIDs.push_back(objIDs.back() + 1); // create id
    circ7.rigidbody.setMass(10.f);

    circ5.rigidbody.addLocalForce(sf::Vector2f(-400.f, 00.f));
    circ1.rigidbody.addLocalForce(sf::Vector2f(400.f, 00.f));

    
    std::vector<Circle> circles{};
    std::vector<Rect> rectangles{};

    circles.push_back(circ1);
    circles.push_back(circ2);
    circles.push_back(circ3);
    circles.push_back(circ4);
    circles.push_back(circ5);
    circles.push_back(circ6);
    circles.push_back(circ7);




    std::vector<RigidBody> Objects;
    Objects.push_back(circ1.rigidbody);
    Objects.push_back(circ2.rigidbody);
    Objects.push_back(circ3.rigidbody);
    Objects.push_back(circ4.rigidbody);
    Objects.push_back(circ5.rigidbody);
    Objects.push_back(circ6.rigidbody);
    Objects.push_back(circ7.rigidbody);

    physics.setRigidBodies(Objects);

    std::vector<CollisionManifold> collisions = {};
    std::vector<RigidBody> bodies1 = {};
    std::vector<RigidBody> bodies2 = {};
    
    
    //UI
    //sf::Sprite sprite = sf::Sprite(sf::Texture());

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

        
        // find Collisions
        
            for (int i = 0; i < Objects.size(); ++i) {
                if (Objects.size() > 1) {
                for (int j = i; j < Objects.size(); ++j) {
                    if (i == j) { continue; }



                    CollisionManifold result = CollisionManifold();
                    RigidBody r1 = Objects[i];
                    RigidBody r2 = Objects[j];


                    //if c1 and c2 are NOT NULL  AND Not infinte Mass ->
                    try {
                        Circle o1 = getCircleByID(circles, Objects[i].getID());
                        Circle o2 = getCircleByID(circles, Objects[j].getID());
                        if (o1 != Circle() && o2 != Circle()) {
                            //o1.rigidbody = r1;
                            //o2.rigidbody = r2;
                            result = Collisions().findcollisionfeatures(o1, o2);
                        }
                    }
                    catch (std::exception e) {}
                    /*
                    try {
                        Rect o1 = getRectByID(rectangles, Objects[i].getID());
                        Rect o2 = getRectByID(rectangles, Objects[j].getID());
                        result = Collisions().findcollisionfeatures(o1, o2);
                    }
                    catch (std::exception e) {}
                    try {
                        Rect o1 = getRectByID(rectangles, Objects[i].getID());
                        Circle o2 = getCircleByID(circles, Objects[j].getID());
                        result = Collisions().findcollisionfeatures(o1, o2);
                    }
                    catch (std::exception e) {}
                    try {
                        Circle o1 = getCircleByID(circles, Objects[i].getID());
                        Rect o2 = getRectByID(rectangles, Objects[j].getID());
                        result = Collisions().findcollisionfeatures(o1, o2);
                    }
                    catch (std::exception e) {}
                    */
                    /*
                    try {
                        Circle o1 = Objects[i].getCircleByID(circles);
                        Circle o2 = Objects[j].getCircleByID(circles);
                        result = Collisions().findcollisionfeatures(o1, o2);
                    }
                    catch(std::exception e){}
                    try {
                        Rect o1 = Objects[i].getRectByID(rectangles);
                        Rect o2 = Objects[j].getRectByID(rectangles);
                        result = Collisions().findcollisionfeatures(o1, o2);
                    }
                    catch (std::exception e) {}
                    try {
                        Rect o1 = Objects[i].getRectByID(rectangles);
                        Circle o2 = Objects[j].getCircleByID(circles);
                        result = Collisions().findcollisionfeatures(o1, o2);
                    }
                    catch (std::exception e) {}
                    try {
                        Circle o1 = Objects[i].getCircleByID(circles);
                        Rect o2 = Objects[j].getRectByID(rectangles);
                        result = Collisions().findcollisionfeatures(o1, o2);
                    }
                    catch (std::exception e) {}
                    */

                    if (result != CollisionManifold() && result.isColliding()) {
                        bodies1.push_back(r1);
                        bodies2.push_back(r2);
                        collisions.push_back(result);
                        std::cout << "HIT" << std::endl;
                        


                    }
                    //std::cout << "B1 " << Objects[i].getLinearVelocity().x << " , " << Objects[i].getLinearVelocity().y << "  " << Objects[i].getForceAccum().y << " ||  ";
                    //std::cout << "B2 " << Objects[j].getLinearVelocity().x << " , " << Objects[j].getLinearVelocity().y << "  " << Objects[j].getForceAccum().y << std::endl;

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
                                    Objects[i] = r1;
                                    Objects[j] = r2;
                                }
                                //reassign rigidbodies post impulse
                            }
                        }
                    }

                    bodies1.clear();
                    bodies2.clear();
                    collisions.clear();


                }
            }
            //std::cout << "B" << i << " " << Objects[i].getLinearVelocity().x << " , " << Objects[i].getLinearVelocity().y << "  " << Objects[i].getForceAccum().y << std::endl;
            circles = quadtree.sortCircles(circles);
            if (circles[i].rigidbody.sections.S1 || circles[i].rigidbody.sections.S2) { // Top wall
                CollisionManifold wallCollide = IntersectionDetection().wallCollide(std::vector<sf::Vertex>{Bounds[0], Bounds[1]}, circles[i]);
                if (wallCollide != CollisionManifold() && wallCollide.isColliding()) {
                    Objects[i] = physics.applyImpulse(Objects[i], wallCollide, gravity.getForce(),0);
                }
            }
            if (circles[i].rigidbody.sections.S2 || circles[i].rigidbody.sections.S3) { // Right wall
                CollisionManifold wallCollide = IntersectionDetection().wallCollide(std::vector<sf::Vertex>{Bounds[1], Bounds[2]}, circles[i]);
                if (wallCollide != CollisionManifold() && wallCollide.isColliding()) {
                    Objects[i] = physics.applyImpulse(Objects[i], wallCollide, gravity.getForce(),1);
                    //std::cout << " Force for " << i << " : " << Objects[i].getForceAccum().x << std::endl;
                }
            }
            if (circles[i].rigidbody.sections.S3 || circles[i].rigidbody.sections.S4) { // Bottom wall
                CollisionManifold wallCollide = IntersectionDetection().wallCollide(std::vector<sf::Vertex>{Bounds[3], Bounds[2]}, circles[i]);
                if (wallCollide != CollisionManifold() && wallCollide.isColliding()) {
                    Objects[i] = physics.applyImpulse(Objects[i], wallCollide, gravity.getForce(),2);
                }
            }
            if (circles[i].rigidbody.sections.S4 || circles[i].rigidbody.sections.S1) { // Left wall
                CollisionManifold wallCollide = IntersectionDetection().wallCollide(std::vector<sf::Vertex>{Bounds[0], Bounds[3]}, circles[i]);
                if (wallCollide != CollisionManifold() && wallCollide.isColliding()) {
                    Objects[i] = physics.applyImpulse(Objects[i], wallCollide, gravity.getForce(),3);
                    //std::cout << " Force for " << i << " : " << Objects[i].getForceAccum().x << std::endl;
                }
            }
            circles[i].rigidbody.sections.reset();
            /*
            CollisionManifold wallCol = IntersectionDetection().boundCollide(Bounds, circles[i]);
            if (wallCol != CollisionManifold() && wallCol.isColliding()) {
                Objects[i] = physics.applyImpulse(Objects[i], wallCol, gravity.getForce());
                std::cout << "Force: " << i << Objects[i].getLinearVelocity().x << ", " << Objects[i].getLinearVelocity().y << std::endl;
            }
            */
        }
        
        

        // apply forces
        physics.setRigidBodies(Objects);
        Objects = physics.applyForces(dt);
        for (int i = 0; i < Objects.size(); i++) {
            Objects[i].removeLocalForces();
        }
        for (int i = 0; i < circles.size(); i++) {
            circles[i].setRigidbody(Objects[i]);
        }
        //circ1.setRigidbody(Objects[0]);
        //circ2.setRigidbody(Objects[1]);
        //circ3.setRigidbody(Objects[2]);
        
        // Draw
        window.clear();
        for (int i = 0; i < circles.size(); i++) { window.draw(circles[i].outputShape()); }
        //window.draw(circ1.outputShape());
        //window.draw(circ2.outputShape());
        //window.draw(circ3.outputShape());
        window.display();
    }
}

void BoxesTesting() {
    int bounds[] = { 800,800 };
    float dt = 0.16;

    sf::RenderWindow window(sf::VideoMode(bounds[0], bounds[1]), "Window");

    ForceGenerator gravity = ForceGenerator(sf::Vector2f(0.f, 0.f));
    std::vector<ForceGenerator> globalforces = { gravity };
    PhysicsSystem physics = PhysicsSystem(globalforces);

    std::vector<int> objIDs = { 0 };

    //Rect(int id, sf::Vector2f position, sf::Vector2f size, float rotation, sf::Color color) {
        

    Rect box1(((objIDs.back()) + 1), sf::Vector2f(210.f, 0.f), sf::Vector2f(25.f,25.f), 0.0f, sf::Color(255.f, 0.f, 0.f));
    objIDs.push_back(objIDs.back() + 1); // create id
    box1.rigidbody.setMass(1.f);
    box1.rigidbody.addLocalForce(sf::Vector2f(0.f, 1.f));
    Rect box2(((objIDs.back()) + 1), sf::Vector2f(220.f, 100.f), sf::Vector2f(25.f, 25.f), 0.0f, sf::Color(0.f, 0.f, 255.f));
    objIDs.push_back(objIDs.back() + 1); // create id
    box2.rigidbody.setMass(1.f);

    std::vector<Rect> rectangles{};
    rectangles.push_back(box1);
    rectangles.push_back(box2);
#

    std::vector<RigidBody> Objects;
    Objects.push_back(box1.rigidbody);
    Objects.push_back(box2.rigidbody);


    physics.setRigidBodies(Objects);

    std::vector<CollisionManifold> collisions = {};
    std::vector<RigidBody> bodies1 = {};
    std::vector<RigidBody> bodies2 = {};

    while (window.isOpen()) {
        Sleep(10);
        sf::Event evnt;
        while (window.pollEvent(evnt)) {
            if (evnt.type == sf::Event::Closed) {
                window.close();
            }
        }

        for (int i = 0; i < Objects.size(); ++i) {
            for (int j = i; j < Objects.size(); ++j) {
                if (i == j) { continue; }

                CollisionManifold result = CollisionManifold();
                RigidBody r1 = Objects[i];
                RigidBody r2 = Objects[j];

                try {
                    Rect o1 = getRectByID(rectangles, Objects[i].getID());
                    Rect o2 = getRectByID(rectangles, Objects[j].getID());
                    result = Collisions().findcollisionfeatures(o1, o2);
                }
                catch (std::exception e) {}


                if (result != CollisionManifold() && result.isColliding()) {
                    bodies1.push_back(r1);
                    bodies2.push_back(r2);
                    collisions.push_back(result);
                    std::cout << "HIT" << std::endl;
                }

                int impulseIterations = 6;
                for (int k = 0; k < impulseIterations; k++) {
                    for (int l = 0; l < collisions.size(); l++) {
                        int jSize = collisions[l].getContactPoints().size();
                        for (int h = 0; h < jSize; h++) {
                            RigidBody r1 = bodies1[l];
                            RigidBody r2 = bodies2[l];
                            std::tuple<RigidBody, RigidBody> rbs = physics.applyImpulse(r1, r2, collisions[l]);
                            if (rbs != std::tuple<RigidBody, RigidBody>()) {
                                r1 = std::get<0>(rbs);
                                r2 = std::get<1>(rbs);
                                Objects[i] = r1;
                                Objects[j] = r2;
                            }
                            //reassign rigidbodies post impulse
                        }
                    }
                }
                bodies1.clear();
                bodies2.clear();
                collisions.clear();
            }


        }

        physics.setRigidBodies(Objects);
        Objects = physics.applyForces(dt);
        for (int i = 0; i < rectangles.size(); i++) {
            rectangles[i].setRigidbody(Objects[i]);
        }

        window.clear();
        for (int i = 0; i < rectangles.size(); i++) { window.draw(rectangles[i].outputShape()); }
        window.display();
    }
}

int main() {
    //movingBox();
    //Click();
    //ClickClass();
    //CollisionTesting();
    testScreen();
    //BoxesTesting();
    return 0;
}
