/*
int movingBox()
{

    int bounds[] = { 800,800 };

    sf::RenderWindow window(sf::VideoMode(bounds[0], bounds[1]), "Window");
    //sf::CircleShape shape(100.f);
    sf::Vector2f vec(10.f, 10.f);
    Cube shape;
    shape.setSize(vec);
    shape.setPosition(vec);
    shape.setVely(-1.f);
    sf::Color col(255, 0, 0);
    shape.setColor(col);

    Cube box;
    box.setSize(vec);
    box.setPosition(sf::Vector2f(20.f, 20.f));
    box.setColor(sf::Color(0, 255, 0));
    box.setVely(-2.f);

    //sf::Shape jim(sf::RectangleShape(0,0,10,10));

     //v = u+at;
    float t = 0.01;
    float g = 4.9f;
    while (window.isOpen())
    {
        Sleep(t * 1000);
        sf::Vector2f wall(bounds[0], bounds[1]);
        sf::Vector2f obj = shape.outputPosition();
        sf::Vector2f shapesize = shape.outputSize();
        shape.isInbounds(bounds);
        shape.applyGravity();
        box.isInbounds(bounds);
        box.applyGravity();



        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        window.clear();
        window.draw(shape.outputShape());
        window.draw(box.outputShape());
        window.display();
        std::cout << obj.x << " " << obj.y << std::endl;
    }

    return 0;
}
*/
/*
int ClickClass() {
    int bounds[] = { 800,800 };

    sf::RenderWindow window(sf::VideoMode(bounds[0], bounds[1]), "Window");
    sf::Vector2f vec(10.f, 10.f);
    sf::Color col(255, 0, 0);

    Cube shape;
    shape.setSize(vec);
    shape.setPosition(vec);
    shape.setVely(-1.f);

    shape.setColor(col);

    Cube box;
    box.setSize(vec);
    box.setPosition(sf::Vector2f(20.f, 20.f));
    box.setColor(sf::Color(0, 255, 0));
    box.setVely(-2.f);

    sf::Vector2i mousepos(0, 0);

    bool mouse_selection = false;

    //v = u+at;
    float t = 0.01;
    float g = 4.9f;
    bool pickable = false;


    std::list<Cube> Objects = { shape,box };

    while (window.isOpen()) {
        Sleep(10);
        sf::Event evnt;
        while (window.pollEvent(evnt)) {
            if (evnt.type == sf::Event::Closed) {
                window.close();
            }
        }
        mousepos = sf::Mouse::getPosition(window);
        for (int i = 0; i < Objects.size(); ++i) {

        }

        pickable = false;
        if (shape.insideMe(mousepos)) {
            if (sf::Mouse::isButtonPressed(sf::Mouse::Left)) {
                pickable = true;
            }
        }


        if (pickable == true) {
            shape.setPositionCentre(mousepos);
            shape.setVelx(0);
            shape.setVely(0);
            //shape.outputPosition();
        }
        else {
            shape.isInbounds(bounds);
            shape.applyGravity(g, t);

            std::cout << shape.outputPosition().x << " " << shape.outputPosition().y << std::endl;
        }


        window.clear();
        window.draw(shape.outputShape());
        window.display();
    }

    return 0;
}
*/


/*
class Sphere {
public:
    sf::Vector2f  Position, Velocity, Acceleration, Damping;
    //sf::Shape self(sf::RectangleShape);
    sf::CircleShape self;
    float Mass, Size;
    bool PickedUp = false;



    int isPickedUp() {
        if (PickedUp == true) { return 1; }
        else { return 0; }
    }
    void picked() {
        PickedUp = true;
    }
    void dropped() {
        PickedUp = false;
    }

    void setPosition(sf::Vector2f pos) {
        Position.x = pos.x;
        Position.y = pos.y;
        setVertices();
    }
    void setSize(float size) {
        Size = size;
        self.setRadius(size);
    }
    void setColor(sf::Color col) { self.setFillColor(col); }

    void setVelx(float x) { Velocity.x = x; }
    void setVely(float y) { Velocity.y = y; }
    void setAccX(float x) { Acceleration.x = x; }
    void setAccY(float y) { Acceleration.y = y; }
    void setMass(float m) { Mass = m; }
    void setDamping(sf::Vector2f d) { Damping = d; }
    void setDampingX(float x) { Damping.x = x; }
    void setDampingY(float y) { Damping.y = y; }

    void isInbounds(int bounds[]) {

        if (Position.x > bounds[0] - (Size)) {
            Position.x = bounds[0] - (Size);
        }
        if (Position.x < 0 + (Size)) {
            Position.x = 0 + (Size);
        }
        if (Position.y > bounds[1] - (Size)) {
            Position.y = bounds[1] - (Size);
            Velocity.y = 0;
            Damping.x = 0.8;
        }
        if (Position.y < 0 + (Size)) {
            Position.y = Size;
        }
        setPosition(Position);
    }
    void applyGravity() {
        Velocity.y += Acceleration.y;
    }
    float applyDampingX(float d) {
        return d * Damping.x;
    }
    float applyDampingY(float d) {
        return d * Damping.y;
    }

    void gravityDragLoop() {
        applyGravity();
        Velocity.x = applyDampingX(Velocity.x);
        Velocity.y = applyDampingY(Velocity.y);
        applyVelocity();
    }
    int insideMe(sf::Vector2i mousepos) {
        if (mousepos.x > Position.x && mousepos.x < Vertices[3].x && mousepos.y > Vertices[0].y && mousepos.y < Vertices[3].y) {

            return 1;
        }
        else { return 0; }
    }

    void collision(Cube obj) {
        for (int i = 0; i < (sizeof(Vertices) / sizeof(*Vertices)); i++) {
            if (Vertices[i].x > obj.Position.x && Vertices[i].x < (obj.Position.x + obj.Size.x) && Vertices[i].y > obj.Position.y && Vertices[i].y < (obj.Position.y + obj.Size.y)) {
                //Position.x += 1.f;
                //Velocity.y = -1.f;

                Position.y -= obj.Size.y / 2;

                //Position += Velocity;
                std::cout << "Hit" << std::endl;
            }
            else {}
        }

    }

    void applyForce(float angle, float force) {
        //x = SinOH, y = CosAH
        angle = angle * (M_PI / 180);
        float x = 0, y = 0;

        if (angle <= 90) {
            x = sin(angle) * force;
            y = cos(angle) * force;
            applyFtoPosition(sf::Vector2f(x, y));
        }
        else if (angle <= 180) {
            x = sin(angle) * force;
            y = cos(angle) * force;
            applyFtoPosition(sf::Vector2f(x, y));
        }
        else if (angle <= 270) {
            x = sin(angle) * force;
            y = cos(angle) * force;
            applyFtoPosition(sf::Vector2f(x, y));
        }
        else {
            x = sin(angle) * force;
            y = cos(angle) * force;
            applyFtoPosition(sf::Vector2f(x, y));
        }
    }
    void applyFtoPosition(sf::Vector2f vec) {
        Position.x += vec.x;
        Position.y += vec.y;
        outputPosition();
    }


    void CornerCollisions(Cube Objects[]) {
        for (int i = 0; i < (sizeof(Objects) / sizeof(*Objects)); ++i) {
            if (Objects[i].outputShape().getOrigin() != self.getOrigin()) {

            }
        }
    }

    sf::Vector2f outputSize() {
        self.setSize(Size);
        return self.getSize();
    }
    void applyVelocity() {
        Position.x += Velocity.x;
        Position.y += Velocity.y;
    }
    sf::Vector2f outputPosition() {
        self.setPosition(Position);
        setVertices();
        return self.getPosition();
    }

    sf::RectangleShape outputShape() { return self; }
};
*/

/*

void CollisionTesting() {
    int bounds[] = { 800,800 };

    sf::RenderWindow window(sf::VideoMode(bounds[0], bounds[1]), "Window");
    sf::Vector2f vec(25.f, 25.f);
    sf::Color col(255, 0, 0);

    Cube shape;
    shape.setSize(vec);
    shape.setPosition(sf::Vector2f(50.f, 50.f));
    shape.setVelocity(sf::Vector2f(1.f,1.f));
    shape.setColor(col);
    shape.setRadius(12.5);

    Cube box;
    box.setSize(vec);
    box.setPosition(sf::Vector2f(150.f, 200.f));
    box.setColor(sf::Color(0, 255, 0));
    box.setVelocity(sf::Vector2f((-1.f), (-2.f)));
    box.setRadius(12.5);

    Cube square;
    square.setSize(vec);
    square.setPosition(sf::Vector2f(150.f, 200.f));
    square.setColor(sf::Color(0, 0, 255));
    square.setRadius(12.5);

    sf::Vector2i mousepos(0, 0);

    bool mouse_selection = false;

    //v = u+at;
    float t = 0.07;
    float g = 9.8f;
    bool pickable = false;

    //Setup For ALL Variables ///
    Cube Objects[] = { shape,box };

    for (int i = 0; i < (sizeof(Objects) / sizeof(*Objects)); ++i) {
        Objects[i].setAccY((g * (t * t)) / 2);
        Objects[i].setMass(i + 1);
        Objects[i].setDamping(sf::Vector2f(0.995, 0.995));
        Objects[i].outputPosition();
    }

    while (window.isOpen()) {
        Sleep(1);
        sf::Event evnt;
        while (window.pollEvent(evnt)) {
            if (evnt.type == sf::Event::Closed) {
                window.close();
            }
            if (evnt.type == (sf::Event::KeyPressed)) {
                float angle;
                std::cin >> angle;
                //Objects[2].applyForce(angle, 50);
            }
        }
        mousepos = sf::Mouse::getPosition(window);
        for (int i = 0; i < (sizeof(Objects) / sizeof(*Objects)); ++i) {
            //Objects[2].setPosition(sf::Vector2f(Objects[1].Position.x + Objects[2].Size.x/2 + Objects[1].Size.x/2, Objects[1].Position.y)); //Join Objects Together
            /*
            if (Objects[i].insideMe(mousepos) && sf::Mouse::isButtonPressed(sf::Mouse::Left)) {
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
                std::cout << "Clicked" << std::endl;
            }
            
            //else {
for (int j = 0; j < (sizeof(Objects) / sizeof(*Objects)); ++j) {
    if (i != j) {
        if (collide(Objects[i], Objects[j])) {
            sf::Vector2f v = perpendicularVector(resultForce(Objects[i].Velocity, Objects[j].Velocity));
            if (v != sf::Vector2f(0, 0)) {
                Objects[i].setVelocity(sf::Vector2f(v.x * 0.5, v.y * 0.5));
                Objects[j].setVelocity(inverse(sf::Vector2f(v.x * 0.5, v.y * 0.5)));
            }
            else {
                Objects[i].setVelocity(sf::Vector2f(0, 0));
                Objects[j].setVelocity(sf::Vector2f(0, 0));
            }
        }


        /*
        if (intersect(Objects[i].outputShape(), Objects[j].outputShape())) {
            sf::Vector2f v = perpendicularVector(resultForce(Objects[i].Velocity,Objects[j].Velocity));
            Objects[i].setVelocity(inverse(v));
            Objects[j].setVelocity(v);
            std::cout << "HIT" << std::endl;
        }
        

    }
}
Objects[i].applyVelocity();
Objects[i].outputPosition();




//std::cout << Objects[i].outputPosition().x << " " << Objects[i].outputPosition().y << std::endl;
//}



        }





        window.clear();
        for (int i = 0; i < (sizeof(Objects) / sizeof(*Objects)); ++i) {
            window.draw(Objects[i].outputShape());
        }
        window.display();
    }
}
*/



/*
sf::Vector2f resultForce(sf::Vector2f V1, sf::Vector2f V2) {
    return sf::Vector2f(V1.x + V2.x, V1.y + V2.y);
}
sf::Vector2f inverse(sf::Vector2f v) { return sf::Vector2f(-v.x, -v.y); }


class Cube {
public:
    sf::Vector2f Size, Position, Velocity, Acceleration, Damping;
    //sf::Shape self(sf::RectangleShape);
    sf::CircleShape self;
    sf::Vector2f Vertices[4] = {};
    sf::Vector2f Faces[4] = {};
    float Mass, Radius;
    bool PickedUp = false;
    bool quadDetection = false;
    sf::Vector2f Colliders[4] = {};



    int isPickedUp() {
        if (PickedUp == true) { return 1; }
        else { return 0; }
    }
    void picked() {
        PickedUp = true;
    }
    void dropped() {
        PickedUp = false;
    }

    void setPosition(sf::Vector2f pos) {
        Position.x = pos.x;
        Position.y = pos.y;
        setVertices();
    }
    void setSize(sf::Vector2f size) {
        Size = size;
        //self.setSize(size);
        self.setRadius(size.x/2);
        self.setOrigin(size.x / 2, size.y / 2);
        setRadius(size.x/2);
    }
    void setVertices() {
        Vertices[0] = sf::Vector2f(Position.x-(Size.x/2),Position.y-(Size.y/2));
        Vertices[1] = sf::Vector2f(Position.x - (Size.x / 2), Position.y + (Size.y / 2)); //(sf::Vector2f(Position.x + Size.x, Position.y));
        Vertices[2] = sf::Vector2f(Position.x + (Size.x / 2), Position.y - (Size.y / 2)); //(sf::Vector2f(Position.x, Position.y + Size.y));
        Vertices[3] = sf::Vector2f(Position.x + (Size.x / 2), Position.y + (Size.y / 2)); //(sf::Vector2f(Position.x + Size.x, Position.y + Size.y));
    }
    void setFaces() {
        Faces[0] = sf::Vector2f((Position.x), (Position.y - (Size.y / 2)));
        Faces[1] = sf::Vector2f((Position.x + (Size.x / 2)), (Position.y));
        Faces[2] = sf::Vector2f((Position.x), (Position.y + (Size.y / 2)));
        Faces[3] = sf::Vector2f((Position.x - (Size.x / 2)), (Position.y));
    }
    void setColor(sf::Color col) { self.setFillColor(col); }

    void setVelx(float x) { Velocity.x = x; }
    void setVely(float y) { Velocity.y = y; }
    void setVelocity(sf::Vector2f v) { Velocity = v; }
    void changeVelx(float x) { Velocity.x += x; }
    void changeVely(float y) { Velocity.y += y; }
    void setAccX(float x) { Acceleration.x = x; }
    void setAccY(float y) { Acceleration.y = y; }
    void setMass(float m) { Mass = m; }
    void setDamping(sf::Vector2f d) { Damping = d; }
    void setDampingX(float x) { Damping.x = x; }
    void setDampingY(float y) { Damping.y = y; }

    void setRadius(float r) { Radius = r; }

    void isInbounds(int bounds[]) {

        if (Position.x > bounds[0] - (Size.x/2)) {
            Position.x = bounds[0] - (Size.x/2);
        }
        if (Position.x < 0 + (Size.x/2)) {
            Position.x = 0 + (Size.x / 2);
        }
        if (Position.y > bounds[1] - (Size.y/2)) {
            Position.y = bounds[1] - (Size.y/2);
            Velocity.y = 0;
            Damping.x = 0.8;
        }
        if (Position.y < 0 + (Size.y/2)) {
            Position.y = Size.y / 2;
        }
        setPosition(Position);
    }
    void applyGravity() {
        Velocity.y += Acceleration.y;
    }
    float applyDampingX(float d) {
        return d * Damping.x;
    }
    float applyDampingY(float d) {
        return d * Damping.y;
    }

    void gravityDragLoop() {
        applyGravity();
        Velocity.x = applyDampingX(Velocity.x);
        Velocity.y = applyDampingY(Velocity.y);
        applyVelocity();
    }
    int insideMe(sf::Vector2i mousepos) {
        if (mousepos.x > Vertices[0].x && mousepos.x < Vertices[3].x && mousepos.y > Vertices[0].y && mousepos.y < Vertices[3].y) {
                return 1;
            }
        else{ return 0;}
    }

    void applyForce(float angle, float force) {
        //x = SinOH, y = CosAH
        angle = angle * (M_PI / 180);
        float x= 0, y = 0;

        if (angle <= 90) {
            x = sin(angle) * force;
            y = cos(angle) * force;
            applyFtoPosition(sf::Vector2f(x, y));
        }
        else if (angle <= 180) {
            x = sin(angle) * force;
            y = cos(angle) * force;
            applyFtoPosition(sf::Vector2f(x, y));
        }
        else if (angle <= 270) {
            x = sin(angle) * force;
            y = cos(angle) * force;
            applyFtoPosition(sf::Vector2f(x, y));
        }
        else {
            x = sin(angle) * force;
            y = cos(angle) * force;
            applyFtoPosition(sf::Vector2f(x, y));
        }
    }
    void applyFtoPosition(sf::Vector2f vec) {
        Position.x += vec.x;
        Position.y += vec.y;
        outputPosition();
    }

    Cube penetrate(Cube obj) {
        float dif;
        // for x
        if (obj.Position.x + obj.Radius > Position.x - Radius) { // obj is LEFT
            dif = (obj.Position.x - Position.x - Radius + obj.Radius);
            obj.Position.x -= dif;
            std::cout << "X" << std::endl;
        }
        if (obj.Position.x - obj.Radius < Position.x + Radius) { // obj is RIGHT
            dif = (obj.Position.x - Position.x + Radius - obj.Radius);
            Position.x -= dif;
            std::cout << "X" << std::endl;
        }

        //for y
        if (int(obj.Position.y + obj.Radius) > int(Position.y - Radius)) { // obj is BELOW
            dif = (obj.Position.y + obj.Radius - Position.y - Radius);
            Position.y -= dif;
            std::cout << "Y" << std::endl;
        }
        if (int(obj.Position.y - obj.Radius) > int(Position.y + Radius)) { // Obj is ABOVE
            dif = (obj.Position.y - obj.Radius - Position.y + Radius);
            obj.Position.y -= dif;
            std::cout << "Y" << std::endl;
        }
        return obj;
    }

    void CornerCollisions(Cube Objects[]) {
        for (int i = 0; i < (sizeof(Objects) / sizeof(*Objects)); ++i) {
            if (Objects[i].outputShape().getOrigin() != self.getOrigin()) {

            }
        }
    }

    sf::Vector2f outputSize() {
        self.setRadius(Radius);
        //self.setSize(Size);
        //return self.getSize();
    }
    void applyVelocity() {
        Position.x += Velocity.x;
        Position.y += Velocity.y;
    }
    sf::Vector2f outputPosition() {
        self.setPosition(Position);
        setVertices();
        return self.getPosition();
    }

    sf::CircleShape outputShape() { return self; }
};


    void applyForce(float angle, float force) {
        //x = SinOH, y = CosAH
        angle = angle * (M_PI / 180);
        float x = 0, y = 0;

        if (angle <= 90) {
            x = sin(angle) * force;
            y = cos(angle) * force;
            applyFtoPosition(sf::Vector2f(x, y));
        }
        else if (angle <= 180) {
            x = sin(angle) * force;
            y = cos(angle) * force;
            applyFtoPosition(sf::Vector2f(x, y));
        }
        else if (angle <= 270) {
            x = sin(angle) * force;
            y = cos(angle) * force;
            applyFtoPosition(sf::Vector2f(x, y));
        }
        else {
            x = sin(angle) * force;
            y = cos(angle) * force;
            applyFtoPosition(sf::Vector2f(x, y));
        }
    }
    void applyFtoPosition(sf::Vector2f vec) {
        Position.x += vec.x;
        Position.y += vec.y;
        outputPosition();
    }

    Rect penetrate(Rect obj) {
        float dif;
        // for x
        if (obj.Position.x + obj.Radius > Position.x - Radius) { // obj is LEFT
            dif = (obj.Position.x - Position.x - Radius + obj.Radius);
            obj.Position.x -= dif;
            std::cout << "X" << std::endl;
        }
        if (obj.Position.x - obj.Radius < Position.x + Radius) { // obj is RIGHT
            dif = (obj.Position.x - Position.x + Radius - obj.Radius);
            Position.x -= dif;
            std::cout << "X" << std::endl;
        }

        //for y
        if (int(obj.Position.y + obj.Radius) > int(Position.y - Radius)) { // obj is BELOW
            dif = (obj.Position.y + obj.Radius - Position.y - Radius);
            Position.y -= dif;
            std::cout << "Y" << std::endl;
        }
        if (int(obj.Position.y - obj.Radius) > int(Position.y + Radius)) { // Obj is ABOVE
            dif = (obj.Position.y - obj.Radius - Position.y + Radius);
            obj.Position.y -= dif;
            std::cout << "Y" << std::endl;
        }
        return obj;
    }

    void CornerCollisions(Rect Objects[]) {
        for (int i = 0; i < (sizeof(Objects) / sizeof(*Objects)); ++i) {
            if (Objects[i].outputShape().getOrigin() != self.getOrigin()) {

            }
        }
    }

*/