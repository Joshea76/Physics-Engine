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
//#include "Quadtree.h"
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

    //Quadtree quadtree = Quadtree(Bounds);

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


    ForceGenerator gravity = ForceGenerator(sf::Vector2f(0.f, 1.95f));
    std::vector<ForceGenerator> globalforces = { gravity };
    PhysicsSystem physics = PhysicsSystem(globalforces);

    std::vector<int> objIDs = { 0 };
    /*
    Circle circ1(((objIDs.back()) + 1), sf::Vector2f(700.f, 100.f), 25.f, 0.0f, sf::Color(255.f, 0.f, 0.f));
    objIDs.push_back(objIDs.back() + 1); // create id
    circ1.rigidbody.setMass(100.f);
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
    */


    std::vector<Circle> circles{};
    std::vector<RigidBody> Objects;


    //Shit tonne of circles
    for (int i = 1; i < 5; i++) {
        for (int j = 1; j < 5; j++){
        Circle circ(((objIDs.back()) + 1), sf::Vector2f((i*101.f) + 150.f, (j*201.f)), 35.f, 0.0f, sf::Color(0.f, 0.f, 255.f));
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
    triangle[0].color = sf::Color::Red;
    triangle[1].color = sf::Color::Green;
    triangle[2].color = sf::Color::Red;

    /*
    // TRIANGLE
    sf::VertexArray triangle2(sf::Triangles, 3);

    // define the position of the triangle2's points
    triangle2[0].position = sf::Vector2f(800.f, 500.f);
    triangle2[1].position = sf::Vector2f(800.f, 800.f);
    triangle2[2].position = sf::Vector2f(500.f, 800.f);

    // define the color of the triangle2's points
    triangle2[0].color = sf::Color::Red;
    triangle2[1].color = sf::Color::Green;
    triangle2[2].color = sf::Color::Red;
    */
   
    std::vector<Rect> rectangles{};
    /*
    circles.push_back(circ1);
    circles.push_back(circ2);
    circles.push_back(circ3);
    circles.push_back(circ4);
    circles.push_back(circ5);
    circles.push_back(circ6);
    circles.push_back(circ7);
    */
    /*
    Objects.push_back(circ1.rigidbody);
    Objects.push_back(circ2.rigidbody);
    Objects.push_back(circ3.rigidbody);
    Objects.push_back(circ4.rigidbody);
    Objects.push_back(circ5.rigidbody);
    Objects.push_back(circ6.rigidbody);
    Objects.push_back(circ7.rigidbody);
    */

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
        circleCols.clear();


        Quad root = Quad(sf::Vector2f(0, 0), sf::Vector2f(bounds[0], bounds[1]));


        CollisionManifold result = CollisionManifold();

        for (int i = 0; i < circles.size(); i++) {
            Node* n = circles[i].rigidbody.getNode();
            root.insert(n);
        }
        std::vector<Quad*> quads = {};
        root.getLowestQuads(&root, &quads);

        std::vector<Quad*>::iterator it;
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

        
        // find Collisions
        //circles = quadtree.sortCircles(circles);

            for (int i = 0; i < Objects.size(); ++i) {
                
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
                CollisionManifold tCollide = IntersectionDetection().wallCollide(std::vector<sf::Vertex>{triangle[0],triangle[2]}, circles[i]);
                if (tCollide != CollisionManifold() && tCollide.isColliding()) {
                    Objects[i] = physics.applyImpulse(Objects[i], tCollide);
                }
            }
            if (circles[i].rigidbody.sections.S4 || circles[i].rigidbody.sections.S1) { // Left wall
                CollisionManifold wallCollide = IntersectionDetection().wallCollide(std::vector<sf::Vertex>{Bounds[0], Bounds[3]}, circles[i]);
                if (wallCollide != CollisionManifold() && wallCollide.isColliding()) {
                    Objects[i] = physics.applyImpulse(Objects[i], wallCollide, gravity.getForce(),3);
                    //std::cout << " Force for " << i << " : " << Objects[i].getForceAccum().x << std::endl;
                }
                /*
                CollisionManifold tCollide = IntersectionDetection().wallCollide(std::vector<sf::Vertex>{triangle2[0], triangle2[2]}, circles[i]);
                if (tCollide != CollisionManifold() && tCollide.isColliding()) {
                    Objects[i] = physics.applyImpulse(Objects[i], tCollide);
                }
                */
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
        //window.draw(triangle2);
        window.display();
    }
}

void Balls() {
    sf::RenderWindow window(sf::VideoMode(800.f,800.f), "Window");

    sf::VertexArray triangle(sf::Triangles, 3);

    // define the position of the triangle's points
    triangle[0].position = sf::Vector2f(10.f, 10.f);
    triangle[1].position = sf::Vector2f(100.f, 10.f);
    triangle[2].position = sf::Vector2f(100.f, 100.f);

    // define the color of the triangle's points
    triangle[0].color = sf::Color::Red;
    triangle[1].color = sf::Color::Blue;
    triangle[2].color = sf::Color::Green;

    while (window.isOpen()) {
        sf::Event evnt;
        while (window.pollEvent(evnt)) {
            if (evnt.type == sf::Event::Closed) {
                window.close();
            }
        }
        window.clear();
        window.draw(triangle);
        window.display();
    }
}

void treeTesting() {
    sf::RenderWindow window(sf::VideoMode(800, 800), "Window");

    PhysicsSystem physics = PhysicsSystem();

    std::vector<int> objIDs = { 0 };

    std::vector<Circle> circles{};

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            Circle circ(((objIDs.back()) + 1), sf::Vector2f((i * 200.f) + 100.f, (j * 200.f) + 100.f), 35.f, 0.0f, sf::Color(0.f, 0.f, 255.f));
            objIDs.push_back(objIDs.back() + 1); // create id
            circ.rigidbody.setMass(10.f);
            circles.push_back(circ);
        }
    }


    Quad root = Quad(sf::Vector2f(0,0),sf::Vector2f(800,800));


    while (window.isOpen()) {
        sf::Event evnt;
        while (window.pollEvent(evnt)) {
            if (evnt.type == sf::Event::Closed) {
                window.close();
            }
        }
        window.clear();
        for (int i = 0; i < circles.size(); i++) {
            Node* n = circles[i].rigidbody.getNode();
            n->pos = circles[i].rigidbody.getPosition();
            root.insert(n);
            window.draw(circles[i].outputShape());

        }
        std::list<Quad*>* quads = new std::list<Quad*>();
        quads = root.getLowestQuads(&root, quads);
        window.display();
    }
}

int main() {
    //movingBox();
    //Click();
    //ClickClass();
    //CollisionTesting();
    //testScreen();
    //BoxesTesting();
    //Balls();
    treeTesting();
    return 0;
}
