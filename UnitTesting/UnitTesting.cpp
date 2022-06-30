#include "../SFML Tinker/CollisionManifold.h" 
#include <SFML/Graphics.hpp>

#define BOOST_TEST_MODULE MyTest
#include <boost/test/included/unit_test.hpp> //single-header
// project being tested

BOOST_AUTO_TEST_SUITE(RigidBody_Tests);

    BOOST_AUTO_TEST_CASE(Rigidbody_ID_Set)
    {
        int expected_value = 1;

        RigidBody rb;
        rb.setID(1);

        BOOST_CHECK(expected_value == rb.getID());
    }
    
    RigidBody rb = RigidBody(sf::Vector2f(1.f, 1.f), 3.14, 12.07);
    BOOST_AUTO_TEST_CASE(Rigidbody_Contsructor_Test1) {
        float expected_value = 3.14;


        RigidBody rb0 = RigidBody(sf::Vector2f(1.f, 1.f), 3.14, 12.07);

        BOOST_CHECK(expected_value == rb0.getMass());
    }
    BOOST_AUTO_TEST_CASE(Rigidbody_Contsructor_Test2) {
        float expected_value = 12.07;

        RigidBody rb1 = RigidBody(sf::Vector2f(1.f, 1.f), 3.14, 12.07);

        BOOST_CHECK(expected_value == rb1.getRotation());
    }
    BOOST_AUTO_TEST_CASE(Rigidbody_Contsructor_Test3) {
        sf::Vector2f expected_value = sf::Vector2f(1.f,1.f);

        RigidBody rb2 = RigidBody(sf::Vector2f(1.f, 1.f), 3.14, 12.07);

        BOOST_CHECK(expected_value == rb2.getPosition());
    }

    BOOST_AUTO_TEST_CASE(Rigidbody_SetID_Test) {
        int expected_value = 7;
        rb.setID(7);
        BOOST_CHECK(expected_value == rb.getID());
    }
    BOOST_AUTO_TEST_CASE(Rigidbody_SetPosition_Test1) {
        sf::Vector2f expected_value = sf::Vector2f(64.f, 64.f);
        rb.setPosition(sf::Vector2f(64.f, 64.f));
        BOOST_CHECK(expected_value == rb.getPosition());
    }
    BOOST_AUTO_TEST_CASE(Rigidbody_SetPosition_Test2) {
        sf::Vector2f expected_value = sf::Vector2f(128.f, 128.f);
        rb.setPosition(sf::Vector2f(0.f, 0.f));
        rb.setPosition(sf::Vector2f(64.f, 64.f));
        rb.addPosition(sf::Vector2f(64.f, 64.f));
        BOOST_CHECK(expected_value == rb.getPosition());
    }
    BOOST_AUTO_TEST_CASE(Rigidbody_SetRotation_Test) {
        float expected_value = 12;
        rb.setRotation(12.f);
        BOOST_CHECK(expected_value == rb.getRotation());
    }
    BOOST_AUTO_TEST_CASE(Rigidbody_SetMass_Test) {
        float expected_value = 12;
        rb.setMass(12.f);
        BOOST_CHECK(expected_value == rb.getMass());
    }
    BOOST_AUTO_TEST_CASE(Rigidbody_InfiniteMass_Test_T) {
        float expected_value = true;
        rb.setMass(0.0f);
        BOOST_CHECK(expected_value == rb.hasInfinteMass());
    }
    BOOST_AUTO_TEST_CASE(Rigidbody_InfiniteMass_Test_F) {
        float expected_value = false;
        rb.setMass(1.0f);
        BOOST_CHECK(expected_value == rb.hasInfinteMass());
    }
    BOOST_AUTO_TEST_CASE(Rigidbody_SetTransform_Test_Pos1) {
        sf::Vector2f expected_value = sf::Vector2f(20.f, 20.f);
        rb.setTransform(sf::Vector2f(20.f, 20.f));
        BOOST_CHECK(expected_value == rb.getPosition());
    }
    BOOST_AUTO_TEST_CASE(Rigidbody_SetTransform_Test_Pos2) {
        sf::Vector2f expected_value = sf::Vector2f(20.f, 20.f);
        rb.setTransform(sf::Vector2f(20.f, 20.f), 127);
        BOOST_CHECK(expected_value == rb.getPosition());
    }
    BOOST_AUTO_TEST_CASE(Rigidbody_SetTransform_Test_Rotation) {
        float expected_value = 127;
        rb.setTransform(sf::Vector2f(20.f, 20.f), 127);
        BOOST_CHECK(expected_value == rb.getRotation());
    }
    BOOST_AUTO_TEST_CASE(Rigidbody_SetCor_Test) {
        float expected_value = 127;
        rb.setTransform(sf::Vector2f(20.f, 20.f), 127);
        BOOST_CHECK(expected_value == rb.getRotation());
    }
    BOOST_AUTO_TEST_CASE(Rigidbody_AddForce_Test1) {
        sf::Vector2f expected_value = sf::Vector2f(10.f, 10.f);
        rb.addForce(sf::Vector2f(10.f, 10.f));
        BOOST_CHECK(expected_value == rb.getForceAccum());
    }
    BOOST_AUTO_TEST_CASE(Rigidbody_clearAccumulators_Test) {
        sf::Vector2f expected_value = sf::Vector2f(0.f,0.f);
        rb.addForce(sf::Vector2f(690.f, 13475.f));
        rb.clearAccumulators();
        BOOST_CHECK(expected_value == rb.getForceAccum());
    }
    BOOST_AUTO_TEST_CASE(Rigidbody_addLocalForce_Test) {
        float expected_value = 0;
        rb.addLocalForce(sf::Vector2f(10.f,10.f));
        BOOST_CHECK(expected_value < rb.getLocalForces().size());
    }
    BOOST_AUTO_TEST_CASE(Rigidbody_removceLocalForces_Test) {
        float expected_value = 0;
        rb.addLocalForce(sf::Vector2f(10.f, 10.f));
        rb.removeLocalForces();
        BOOST_CHECK(expected_value == rb.getLocalForces().size());
    }
    BOOST_AUTO_TEST_CASE(Rigidbody_addLinearVelocity_Test) {
        sf::Vector2f expected_value = sf::Vector2f(10.f, 10.f);
        rb.addLinearVelocity(sf::Vector2f(10.f, 10.f));
        BOOST_CHECK(expected_value == rb.getLinearVelocity());
    }
    BOOST_AUTO_TEST_CASE(Rigidbody_setlinearVelocity_Test) {
        sf::Vector2f expected_value = sf::Vector2f(10.f, 10.f);
        rb.setLinearVelocity(sf::Vector2f(10.f, 10.f));
        BOOST_CHECK(expected_value == rb.getLinearVelocity());
    }
    BOOST_AUTO_TEST_CASE(Rigidbody_getNode_Test) {
        float expected_value = 11;
        rb.node->id = 11;
        BOOST_CHECK(expected_value == rb.getNode()->id);
    }
    BOOST_AUTO_TEST_CASE(Rigidbody_setCor_Test) {
        float expected_value = 11;
        rb.setCor(11);
        BOOST_CHECK(expected_value == rb.getCor());
    }
    BOOST_AUTO_TEST_CASE(Rigidbody_physicsUpdate_Test1) {
        sf::Vector2f expected_value = sf::Vector2f(0.0256f, 0.0256f);
        rb.setMass(10);
        rb.setPosition(sf::Vector2f(0.f, 0.f));
        rb.setLinearVelocity(sf::Vector2f(0.f, 0.f));
        rb.removeLocalForces();
        rb.addForce(sf::Vector2f(10.f, 10.f));
        rb.physicsUpdate(0.16);
        BOOST_CHECK(floor(expected_value.x * 1000) == (floor(rb.getPosition().x * 1000)) && floor(expected_value.y * 1000) == (floor(rb.getPosition().y * 1000)));
    }
    BOOST_AUTO_TEST_CASE(Rigidbody_physicsUpdate_Test2) {
        sf::Vector2f expected_value = sf::Vector2f(0.8256f, 0.8256f);
        rb.setMass(10);
        rb.setPosition(sf::Vector2f(0.f, 0.f));
        rb.setLinearVelocity(sf::Vector2f(5.f, 5.f));
        rb.removeLocalForces();
        rb.addForce(sf::Vector2f(10.f, 10.f));
        rb.physicsUpdate(0.16);
        BOOST_CHECK(floor(expected_value.x * 1000) == (floor(rb.getPosition().x * 1000)) && floor(expected_value.y * 1000) == (floor(rb.getPosition().y * 1000)));
    }
    BOOST_AUTO_TEST_CASE(Rigidbody_physicsUpdate_Test_Neg) {
        sf::Vector2f expected_value = sf::Vector2f(-0.0256f, -0.0256f);
        rb.setMass(10);
        rb.setPosition(sf::Vector2f(0.f, 0.f));
        rb.setLinearVelocity(sf::Vector2f(0.f, 0.f));
        rb.removeLocalForces();
        rb.addForce(sf::Vector2f(-10.f, -10.f));
        rb.physicsUpdate(0.16);
        BOOST_CHECK(floor(expected_value.x * 1000) == (floor(rb.getPosition().x * 1000)) && floor(expected_value.y * 1000) == (floor(rb.getPosition().y * 1000)));
    }


BOOST_AUTO_TEST_SUITE_END();



BOOST_AUTO_TEST_SUITE(Circle_Tests);

    
    
    BOOST_AUTO_TEST_CASE(Circle_Constructor_Test1) {
        float expected_value = 0;
        Circle circ = Circle();
        BOOST_CHECK(expected_value == circ.getID());
    }
    BOOST_AUTO_TEST_CASE(Circle_Constructor_Test2) {
        float expected_value = 0;
        Circle circ = Circle(0, sf::Vector2f(100.f, 100.f), 10.f, 0.f, sf::Color(255, 0, 0));
        BOOST_CHECK(expected_value == circ.getID());
    }
    BOOST_AUTO_TEST_CASE(Circle_Constructor_Test3) {
        float expected_value = 0;
        Circle circ = Circle(0, sf::Vector2f(100.f, 100.f), 10.f, 0.f);
        BOOST_CHECK(expected_value == circ.getID());
    }
    BOOST_AUTO_TEST_CASE(Circle_Constructor_Test4) {
        float expected_value = 0;
        Circle circ = Circle(0, sf::Vector2f(100.f, 100.f), 10.f);
        BOOST_CHECK(expected_value == circ.getID());
    }
    BOOST_AUTO_TEST_CASE(Circle_Constructor_Test5) {
        float expected_value = 0;
        Circle circ = Circle(0, sf::Vector2f(100.f, 100.f), 10.f, sf::Color(255, 0, 0));
        BOOST_CHECK(expected_value == circ.getID());
    }

    Circle circ = Circle();
    BOOST_AUTO_TEST_CASE(Circle_setID_Test) {
        float expected_value = 11;
        circ.setID(11);
        BOOST_CHECK(expected_value == circ.getID());
    }
    BOOST_AUTO_TEST_CASE(Circle_setRigidodyToSelf_Test) {
        circ.setRigidbodyToSelf(circ.rigidbody);
        BOOST_CHECK(circ.rigidbody.getPosition() == circ.outputShape().getPosition());
    }
    BOOST_AUTO_TEST_CASE(Circle_setRigidody_Test) {
        circ.setRigidbody(circ.rigidbody);
        BOOST_CHECK(circ.rigidbody.getPosition() == circ.outputShape().getPosition());
    }
    BOOST_AUTO_TEST_CASE(Circle_setColor_Test) {
        sf::Color expected_value = sf::Color(100.f,100.f,100.f);
        circ.setColor(sf::Color(100.f, 100.f, 100.f));
        BOOST_CHECK(expected_value == circ.outputShape().getFillColor());
    }
    BOOST_AUTO_TEST_CASE(Circle_setSize_Test) {
        float expected_value = 27.f;
        circ.setSize(sf::Vector2f(54.f,54.f));
        BOOST_CHECK(expected_value == circ.outputShape().getRadius());
    }
    BOOST_AUTO_TEST_CASE(Circle_setRadius_Test1) {
        float expected_value = 27.f;
        circ.setRadius(27.f);
        BOOST_CHECK(expected_value == circ.getRadius());
    }
    BOOST_AUTO_TEST_CASE(Circle_setRadius_Test2) {
        float expected_value = 27.f;
        circ.setRadius(27.f);
        BOOST_CHECK(expected_value == circ.outputShape().getRadius());
    }

BOOST_AUTO_TEST_SUITE_END();

BOOST_AUTO_TEST_SUITE(Functions_Tests);

BOOST_AUTO_TEST_CASE(Functions_SubtractVectors_Test1) {
    sf::Vector2f expected_value = sf::Vector2f(12.f, 6.f);
    sf::Vector2f a = sf::Vector2f(24.f, 12.f);
    sf::Vector2f b = sf::Vector2f(12.f, 6.f);
    BOOST_CHECK(expected_value == SubtractVectors(a, b));
}
BOOST_AUTO_TEST_CASE(Functions_SubtractVectors_Test2) {
    sf::Vector2f expected_value = sf::Vector2f(12.f, 6.f);
    sf::Vector2f a = sf::Vector2f(24.f, 12.f);
    sf::Vector2f b = sf::Vector2f(0.f, 0.f);
    BOOST_CHECK(expected_value != SubtractVectors(a, b));
}

BOOST_AUTO_TEST_CASE(Functions_AddVectors_Test1) {
    sf::Vector2f expected_value = sf::Vector2f(24.f, 12.f);
    sf::Vector2f a = sf::Vector2f(8.f, 8.f);
    sf::Vector2f b = sf::Vector2f(16.f, 4.f);
    BOOST_CHECK(expected_value == AddVectors(a, b));
}
BOOST_AUTO_TEST_CASE(Functions_AddVectors_Test2) {
    sf::Vector2f expected_value = sf::Vector2f(12.f, 6.f);
    sf::Vector2f a = sf::Vector2f(8.f, 6.f);
    sf::Vector2f b = sf::Vector2f(0.f, 0.f);
    BOOST_CHECK(expected_value != AddVectors(a, b));
}

BOOST_AUTO_TEST_CASE(Functions_MultiplyVectors_Test1) {
    sf::Vector2f expected_value = sf::Vector2f(24.f, 12.f);
    sf::Vector2f a = sf::Vector2f(8.f, 6.f);
    sf::Vector2f b = sf::Vector2f(3.f, 2.f);
    BOOST_CHECK(expected_value == MultiplyVectors(a, b));
}
BOOST_AUTO_TEST_CASE(Functions_MultiplyVectors_Test2) {
    sf::Vector2f expected_value = sf::Vector2f(12.f, 6.f);
    sf::Vector2f a = sf::Vector2f(8.f, 6.f);
    sf::Vector2f b = sf::Vector2f(99.f, 99.f);
    BOOST_CHECK(expected_value != MultiplyVectors(a, b));
}
BOOST_AUTO_TEST_CASE(Functions_MultiplyVectors_Test_Zero) {
    sf::Vector2f expected_value = sf::Vector2f(0.f,0.f);
    sf::Vector2f a = sf::Vector2f(24.f, 12.f);
    sf::Vector2f b = sf::Vector2f(0.f, 0.f);
    BOOST_CHECK(expected_value == MultiplyVectors(a, b));
}

BOOST_AUTO_TEST_CASE(Functions_ScaleVector_Test_Pos) {
    sf::Vector2f expected_value = sf::Vector2f(240.f, 120.f);
    sf::Vector2f a = sf::Vector2f(24.f, 12.f);
    float b = 10;
    BOOST_CHECK(expected_value == ScaleVector(a, b));
}
BOOST_AUTO_TEST_CASE(Functions_ScaleVector_Test_Neg) {
    sf::Vector2f expected_value = sf::Vector2f(-240.f, -120.f);
    sf::Vector2f a = sf::Vector2f(24.f, 12.f);
    float b = -10;
    BOOST_CHECK(expected_value == ScaleVector(a, b));
}
BOOST_AUTO_TEST_CASE(Functions_ScaleVector_Test_Zero) {
    sf::Vector2f expected_value = sf::Vector2f(0.f, 0.f);
    sf::Vector2f a = sf::Vector2f(24.f, 12.f);
    float b = 0;
    BOOST_CHECK(expected_value == ScaleVector(a, b));
}

BOOST_AUTO_TEST_CASE(Functions_AddToVector_Test_Pos) {
    sf::Vector2f expected_value = sf::Vector2f(20.f, 20.f);
    sf::Vector2f a = sf::Vector2f(10.f, 10.f);
    float b = 10;
    BOOST_CHECK(expected_value == AddToVector(a, b));
}
BOOST_AUTO_TEST_CASE(Functions_AddToVector_Test_Neg) {
    sf::Vector2f expected_value = sf::Vector2f(5.f, 5.f);
    sf::Vector2f a = sf::Vector2f(10.f, 10.f);
    float b = -5;
    BOOST_CHECK(expected_value == AddToVector(a, b));
}
BOOST_AUTO_TEST_CASE(Functions_AddToVector_Test_Zero) {
    sf::Vector2f expected_value = sf::Vector2f(0.f, 0.f);
    sf::Vector2f a = sf::Vector2f(0.f, 0.f);
    float b = 0;
    BOOST_CHECK(expected_value == AddToVector(a, b));
}

BOOST_AUTO_TEST_CASE(Functions_resultForce_Test1) {
    sf::Vector2f expected_value = sf::Vector2f(24.f, 12.f);
    sf::Vector2f a = sf::Vector2f(8.f, 8.f);
    sf::Vector2f b = sf::Vector2f(16.f, 4.f);
    BOOST_CHECK(expected_value == resultForce(a, b));
}
BOOST_AUTO_TEST_CASE(Functions_resultForce_Test2) {
    sf::Vector2f expected_value = sf::Vector2f(12.f, 6.f);
    sf::Vector2f a = sf::Vector2f(8.f, 6.f);
    sf::Vector2f b = sf::Vector2f(0.f, 0.f);
    BOOST_CHECK(expected_value != resultForce(a, b));
}

BOOST_AUTO_TEST_CASE(Functions_Inverse_Test1) {
    sf::Vector2f expected_value = sf::Vector2f(-12.f, -6.f);
    sf::Vector2f a = sf::Vector2f(12.f, 6.f);
    BOOST_CHECK(expected_value == inverse(a));
}
BOOST_AUTO_TEST_CASE(Functions_Inverse_Test2) {
    sf::Vector2f expected_value = sf::Vector2f(12.f, 6.f);
    sf::Vector2f a = sf::Vector2f(-12.f, -6.f);
    BOOST_CHECK(expected_value == inverse(a));
}

BOOST_AUTO_TEST_CASE(Functions_DotProduct_Test1) {
    float expected_value = 36.f;
    sf::Vector2f a = sf::Vector2f(8.f, 6.f);
    sf::Vector2f b = sf::Vector2f(3.f, 2.f);
    BOOST_CHECK(expected_value == dotProduct(a,b));
}
BOOST_AUTO_TEST_CASE(Functions_DotProduct_Test2) {
    float expected_value = 12.f;
    sf::Vector2f a = sf::Vector2f(8.f, 6.f);
    sf::Vector2f b = sf::Vector2f(3.f, -2.f);
    BOOST_CHECK(expected_value == dotProduct(a, b));
}
BOOST_AUTO_TEST_CASE(Functions_DotProduct_Test_Zero) {
    float expected_value = 0.f;
    sf::Vector2f a = sf::Vector2f(8.f, 6.f);
    sf::Vector2f b = sf::Vector2f(0.f, 0.f);
    BOOST_CHECK(expected_value == dotProduct(a, b));
}

BOOST_AUTO_TEST_CASE(Functions_LengthOfVector_Test1) {
    float expected_value = 5.f;
    sf::Vector2f a = sf::Vector2f(3.f, 4.f);
    BOOST_CHECK(expected_value == lengthofVector(a));
}
BOOST_AUTO_TEST_CASE(Functions_LengthOfVector_Test_Zero) {
    float expected_value = 3.f;
    sf::Vector2f a = sf::Vector2f(3.f, 0.f);
    BOOST_CHECK(expected_value == lengthofVector(a));
}

BOOST_AUTO_TEST_CASE(Functions_LengthOfVectorSquared_Test1) {
    float expected_value = 25.f;
    sf::Vector2f a = sf::Vector2f(3.f, 4.f);
    BOOST_CHECK(expected_value == lengthofVectorSquared(a));
}
BOOST_AUTO_TEST_CASE(Functions_LengthOfVectorSquared_Test_Zero) {
    float expected_value = 9.f;
    sf::Vector2f a = sf::Vector2f(3.f, 0.f);
    BOOST_CHECK(expected_value == lengthofVectorSquared(a));
}

BOOST_AUTO_TEST_CASE(Functions_PerpendicularVector_Test1) {
    sf::Vector2f expected_value = sf::Vector2f(-1.f,1.f);
    sf::Vector2f a = sf::Vector2f(1.f, 1.f);
    BOOST_CHECK(expected_value == perpendicularVector(a));
}
BOOST_AUTO_TEST_CASE(Functions_PerpendicularVector_Test_Zero) {
    sf::Vector2f expected_value = sf::Vector2f(0.f, 1.f);
    sf::Vector2f a = sf::Vector2f(1.f, 0.f);
    BOOST_CHECK(expected_value == perpendicularVector(a));
}

//they are the same number wtf
BOOST_AUTO_TEST_CASE(Functions_Normalise_Test1) {
    sf::Vector2f expected_value = sf::Vector2f(0.707107f, 0.707107f);
    sf::Vector2f a = sf::Vector2f(5.f, 5.f);
    BOOST_CHECK(floor(expected_value.x * 1000) == (floor(normalise(a).x * 1000)) && floor(expected_value.y * 1000) == (floor(normalise(a).y * 1000)));
}

BOOST_AUTO_TEST_CASE(Functions_DegToRadians_Test1) {
    float expected_value = M_PI;
    float a = 180.f;
    BOOST_CHECK(expected_value == DegtoRadians(a));
}

BOOST_AUTO_TEST_CASE(Functions_abs_Test1) {
    sf::Vector2f expected_value = sf::Vector2f(10.f,10.f);
    sf::Vector2f a = sf::Vector2f(-10.f,-10.f);
    BOOST_CHECK(expected_value == abs(a));
}

BOOST_AUTO_TEST_CASE(Functions_RotateVector_Test1) {
    sf::Vector2f expected_value = sf::Vector2f(0.f,1.41421f);
    sf::Vector2f vec = sf::Vector2f(1.f,1.f);
    float angle = 45;
    sf::Vector2f origin = sf::Vector2f(0.f,0.f);
    std::cout << std::fixed <<  std::setprecision(5) << rotateVector(vec, angle, origin).y - 1.41421f;
    BOOST_CHECK(floor(expected_value.x * 1000) == (floor(rotateVector(vec, angle, origin).x * 1000)) && floor(expected_value.y * 1000) == (floor(rotateVector(vec, angle, origin).y * 1000)));
}

//compare decimal values
BOOST_AUTO_TEST_CASE(Functions_compare_Test_ffd1) {
    float expected_value = true;
    float x = 1.1122;
    float y = 1.1199;
    float dcpl = 0.01;
    BOOST_CHECK(expected_value == compare(x,y,dcpl));
}
BOOST_AUTO_TEST_CASE(Functions_compare_Test_ffd2) {
    float expected_value = false;
    float x = 1.1122;
    float y = -1.1199;
    float dcpl = 0.01;
    BOOST_CHECK(expected_value == compare(x, y, dcpl));
}
BOOST_AUTO_TEST_CASE(Functions_compare_Test_ff1) {
    float expected_value = true;
    float x = 1.11992324;
    float y = 1.1199534;
    BOOST_CHECK(expected_value == compare(x, y));
}
BOOST_AUTO_TEST_CASE(Functions_compare_Test_ff2) {
    float expected_value = false;
    float x = 1.1122;
    float y = -1.1199;
    BOOST_CHECK(expected_value == compare(x, y));
}
BOOST_AUTO_TEST_CASE(Functions_compare_Test_vvd1) {
    float expected_value = true;
    sf::Vector2f v1 = sf::Vector2f(1.1122f,1.1199);
    sf::Vector2f v2 = sf::Vector2f(1.1199f,1.1122);
    float dcpl = 0.01;
    BOOST_CHECK(expected_value == compare(v1,v2,dcpl));
}
BOOST_AUTO_TEST_CASE(Functions_compare_Test_vvd2) {
    float expected_value = false;
    sf::Vector2f v1 = sf::Vector2f(1.1221f, 1.1991);
    sf::Vector2f v2 = sf::Vector2f(1.1199f, 1.1122);
    float dcpl = 0.01;
    BOOST_CHECK(expected_value == compare(v1, v2, dcpl));
}
BOOST_AUTO_TEST_CASE(Functions_compare_Test_vv1) {
    float expected_value = true;
    sf::Vector2f v1 = sf::Vector2f(1.11229, 1.11221);
    sf::Vector2f v2 = sf::Vector2f(1.11220, 1.11222);
    BOOST_CHECK(expected_value == compare(v1, v2));
}
BOOST_AUTO_TEST_CASE(Functions_compare_Test_vv2) {
    float expected_value = false;
    sf::Vector2f v1 = sf::Vector2f(1.1221f, 1.1991);
    sf::Vector2f v2 = sf::Vector2f(1.1199f, 1.1122);
    BOOST_CHECK(expected_value == compare(v1, v2));
}

BOOST_AUTO_TEST_CASE(Functions_MidPoint_Test1) {
    sf::Vector2f expected_value = sf::Vector2f(5.f,5.f);
    sf::Vector2f a = sf::Vector2f(10.f, 10.f);
    sf::Vector2f b = sf::Vector2f(0.f, 0.f);
    BOOST_CHECK(expected_value == Midpoint(a, b));
}
BOOST_AUTO_TEST_CASE(Functions_MidPoint_Test_Neg) {
    sf::Vector2f expected_value = sf::Vector2f(-5.f, -5.f);
    sf::Vector2f a = sf::Vector2f(-10.f, -10.f);
    sf::Vector2f b = sf::Vector2f(0.f, 0.f);
    BOOST_CHECK(expected_value == Midpoint(a, b));
}


BOOST_AUTO_TEST_SUITE_END();


BOOST_AUTO_TEST_SUITE(CollisionManifold_Tests);

BOOST_AUTO_TEST_CASE(CollisionManifold_Constructor_Test1) {
    float expected_value = 0.0f;
    sf::Vector2f expected_value2 = sf::Vector2f();
    bool expected_value3 = false;
    CollisionManifold m = CollisionManifold();
    BOOST_CHECK(expected_value == m.getDepth() && expected_value2 == m.getNormal() && expected_value3 == m.isColliding());
}
BOOST_AUTO_TEST_CASE(CollisionManifold_Constructor_Test2) {
    float expected_value = 12.f;
    sf::Vector2f expected_value2 = sf::Vector2f(14.f,14.f);
    bool expected_value3 = true;
    CollisionManifold m = CollisionManifold(sf::Vector2f(14.f,14.f),12.f);
    BOOST_CHECK(expected_value == m.getDepth() && expected_value2 == m.getNormal() && expected_value3 == m.isColliding());
}

CollisionManifold m = CollisionManifold();
BOOST_AUTO_TEST_CASE(CollisionManifold_addContactPoint_Test) {
    float expected_value = 0;
    m.addContactPoint(sf::Vector2f());
    BOOST_CHECK(expected_value < m.getContactPoints().size());
}

BOOST_AUTO_TEST_SUITE_END();


BOOST_AUTO_TEST_SUITE(Collisions_Tests);

    // X No Collide 
    Circle a_nch = Circle(0, sf::Vector2f(100.f, 100.f), 25.f, sf::Color(255, 0, 0));
    Circle b_nch = Circle(0, sf::Vector2f(151.f, 100.f), 25.f, sf::Color(255, 0, 0));
    CollisionManifold m_nch = Collisions().findcollisionfeatures(a_nch, b_nch);
    BOOST_AUTO_TEST_CASE(Collisions_findCollisionFeatures_NoCollide_Horizontal_Test) {
        bool expected_value = false;
        BOOST_CHECK(expected_value == m_nch.isColliding());
    }
    // Y No Collide
    Circle a_ncv = Circle(0, sf::Vector2f(100.f, 100.f), 25.f, sf::Color(255, 0, 0));
    Circle b_ncv = Circle(0, sf::Vector2f(100.f, 151.f), 25.f, sf::Color(255, 0, 0));
    CollisionManifold m_ncv = Collisions().findcollisionfeatures(a_ncv, b_ncv);
    BOOST_AUTO_TEST_CASE(Collisions_findCollisionFeatures_NoCollide_Vertical_Test) {
        bool expected_value = false;
        BOOST_CHECK(expected_value == m_ncv.isColliding());
    }
    // Diagonal No Collide
    Circle a_ncd = Circle(0, sf::Vector2f(100.f, 100.f), 25.f, sf::Color(255, 0, 0));
    Circle b_ncd = Circle(0, sf::Vector2f(150.f, 150.f), 25.f, sf::Color(255, 0, 0));
    CollisionManifold m_ncd = Collisions().findcollisionfeatures(a_ncd, b_ncd);
    BOOST_AUTO_TEST_CASE(Collisions_findCollisionFeatures_NoCollide_Diagonal_Test) {
        bool expected_value = false;
        BOOST_CHECK(expected_value == m_ncd.isColliding());
    }
    // Extreme Value No Collide
    Circle a_ncext = Circle(0, sf::Vector2f(1000.f, 1000.f), 25.f, sf::Color(255, 0, 0));
    Circle b_ncext = Circle(0, sf::Vector2f(0.f, 0.f), 25.f, sf::Color(255, 0, 0));
    CollisionManifold m_ncext = Collisions().findcollisionfeatures(a_ncext, b_ncext);
    BOOST_AUTO_TEST_CASE(Collisions_findCollisionFeatures_NoCollide_Extreme_Test) {
        bool expected_value = false;
        BOOST_CHECK(expected_value == m_ncext.isColliding());
    }

    // X intersect - no Swap
    Circle a_x1 = Circle(0, sf::Vector2f(100.f, 100.f), 25.f, sf::Color(255, 0, 0));
    Circle b_x1 = Circle(0, sf::Vector2f(149.f, 100.f), 25.f, sf::Color(255, 0, 0));
    CollisionManifold m_x1 = Collisions().findcollisionfeatures(a_x1, b_x1);
    BOOST_AUTO_TEST_CASE(Collisions_findCollisionFeatures_Horizontal_Test_NS_Colliding) {
        bool expected_value = true;
        BOOST_CHECK(expected_value == m_x1.isColliding());
    }
    BOOST_AUTO_TEST_CASE(Collisions_findCollisionFeatures_Horizontal_Test_NS_Depth) {
        float expected_value = -0.5f;
        BOOST_CHECK(expected_value == m_x1.getDepth());
    }
    BOOST_AUTO_TEST_CASE(Collisions_findCollisionFeatures_Horizontal_Test_NS_Normal) {
        sf::Vector2f expected_value = sf::Vector2f(1.f,0.f);
        BOOST_CHECK(expected_value == m_x1.getNormal());
    }
    BOOST_AUTO_TEST_CASE(Collisions_findCollisionFeatures_Horizontal_Test_NS_ContactPoint) {
        sf::Vector2f expected_value = sf::Vector2f(149.f, 100.f);
        BOOST_CHECK(expected_value == m_x1.getContactPoints()[0]);
    }

    // Y intersect - no Swap
    Circle a_y1 = Circle(0, sf::Vector2f(100.f, 100.f), 25.f, sf::Color(255, 0, 0));
    Circle b_y1 = Circle(0, sf::Vector2f(100.f, 149.f), 25.f, sf::Color(255, 0, 0));
    CollisionManifold m_y1 = Collisions().findcollisionfeatures(a_y1, b_y1);
    BOOST_AUTO_TEST_CASE(Collisions_findCollisionFeatures_Vertical_Test_NS_Colliding) {
        bool expected_value = true;
        BOOST_CHECK(expected_value == m_y1.isColliding());
    }
    BOOST_AUTO_TEST_CASE(Collisions_findCollisionFeatures_Vertical_Test_NS_Depth) {
        float expected_value = -0.5f;
        BOOST_CHECK(expected_value == m_y1.getDepth());
    }
    BOOST_AUTO_TEST_CASE(Collisions_findCollisionFeatures_Vertical_Test_NS_Normal) {
        sf::Vector2f expected_value = sf::Vector2f(0.f, -1.f);
        BOOST_CHECK(expected_value == m_y1.getNormal());
    }
    BOOST_AUTO_TEST_CASE(Collisions_findCollisionFeatures_Vertical_Test_NS_ContactPoint) {
        sf::Vector2f expected_value = sf::Vector2f(100.f, 198.f);
        std::cout << m_y1.getContactPoints()[0].y;
        BOOST_CHECK(expected_value == m_y1.getContactPoints()[0]);
    }

    // X intersect - Swap
    Circle a_x2 = Circle(0, sf::Vector2f(149.f, 100.f), 25.f, sf::Color(255, 0, 0));
    Circle b_x2 = Circle(0, sf::Vector2f(100.f, 100.f), 25.f, sf::Color(255, 0, 0));
    CollisionManifold m_x2 = Collisions().findcollisionfeatures(a_x2, b_x2);
    BOOST_AUTO_TEST_CASE(Collisions_findCollisionFeatures_Horizontal_Test_S_Colliding) {
        bool expected_value = true;
        BOOST_CHECK(expected_value == m_x2.isColliding());
    }
    BOOST_AUTO_TEST_CASE(Collisions_findCollisionFeatures_Horizontal_Test_S_Depth) {
        float expected_value = -0.5f;
        BOOST_CHECK(expected_value == m_x2.getDepth());
    }
    BOOST_AUTO_TEST_CASE(Collisions_findCollisionFeatures_Horizontal_Test_S_Normal) {
        sf::Vector2f expected_value = sf::Vector2f(1.f, 0.f);
        BOOST_CHECK(expected_value == m_x2.getNormal());
    }
    BOOST_AUTO_TEST_CASE(Collisions_findCollisionFeatures_Horizontal_Test_S_ContactPoint) {
        sf::Vector2f expected_value = sf::Vector2f(149.f, 100.f);
        BOOST_CHECK(expected_value == m_x2.getContactPoints()[0]);
    }

    // Y intersect - Swap
    Circle a_y2 = Circle(0, sf::Vector2f(100.f, 149.f), 25.f, sf::Color(255, 0, 0));
    Circle b_y2 = Circle(0, sf::Vector2f(100.f, 100.f), 25.f, sf::Color(255, 0, 0));
    CollisionManifold m_y2 = Collisions().findcollisionfeatures(a_y2, b_y2);
    BOOST_AUTO_TEST_CASE(Collisions_findCollisionFeatures_Vertical_Test_S_Colliding) {
        bool expected_value = true;
        BOOST_CHECK(expected_value == m_y2.isColliding());
    }
    BOOST_AUTO_TEST_CASE(Collisions_findCollisionFeatures_Vertical_Test_S_Depth) {
        float expected_value = -0.5f;
        BOOST_CHECK(expected_value == m_y2.getDepth());
    }
    BOOST_AUTO_TEST_CASE(Collisions_findCollisionFeatures_Vertical_Test_S_Normal) {
        sf::Vector2f expected_value = sf::Vector2f(0.f, -1.f);
        BOOST_CHECK(expected_value == m_y2.getNormal());
    }
    BOOST_AUTO_TEST_CASE(Collisions_findCollisionFeatures_Vertical_Test_S_ContactPoint) {
        sf::Vector2f expected_value = sf::Vector2f(100.f, 198.f);
        BOOST_CHECK(expected_value == m_y2.getContactPoints()[0]);
    }

    // Diagonal - no Swap
    Circle a_d1 = Circle(0, sf::Vector2f(100.f, 100.f), 25.f, sf::Color(255, 0, 0));
    Circle b_d1 = Circle(0, sf::Vector2f(124.5f, 124.5f), 25.f, sf::Color(255, 0, 0));
    CollisionManifold m_d1 = Collisions().findcollisionfeatures(a_d1, b_d1);
    BOOST_AUTO_TEST_CASE(Collisions_findCollisionFeatures_Diagonal_Test_NS_Colliding) {
        bool expected_value = true;
        BOOST_CHECK(expected_value == m_d1.isColliding());
    }
    BOOST_AUTO_TEST_CASE(Collisions_findCollisionFeatures_Diagonal_Test_NS_Depth) {
        float expected_value = -7.67588f;
        BOOST_CHECK(floor(expected_value * 1000) == floor(m_d1.getDepth() * 1000));
    }
    BOOST_AUTO_TEST_CASE(Collisions_findCollisionFeatures_Diagonal_Test_NS_Normal) {
        sf::Vector2f expected_value = sf::Vector2f(-0.707107f, -0.707107f);
        BOOST_CHECK(floor(expected_value.x * 1000) == floor(m_d1.getNormal().x * 1000) && floor(expected_value.y * 1000) == floor(m_d1.getNormal().y * 1000));
    }
    BOOST_AUTO_TEST_CASE(Collisions_findCollisionFeatures_Diagonal_Test_NS_ContactPoint) {
        sf::Vector2f expected_value = sf::Vector2f(141.824f, 141.824f);
        BOOST_CHECK(floor(expected_value.x * 1000) == floor(m_d1.getContactPoints()[0].x * 1000) && floor(expected_value.y * 1000) == floor(m_d1.getContactPoints()[0].y * 1000));
    }


    // Y intersect - Swap
    Circle a_d2 = Circle(0, sf::Vector2f(124.5f, 124.5f), 25.f, sf::Color(255, 0, 0));
    Circle b_d2 = Circle(0, sf::Vector2f(100.f, 100.f), 25.f, sf::Color(255, 0, 0));
    CollisionManifold m_d2 = Collisions().findcollisionfeatures(a_d2, b_d2);
    BOOST_AUTO_TEST_CASE(Collisions_findCollisionFeatures_Diagonal_Test_S_Colliding) {
        bool expected_value = true;
        BOOST_CHECK(expected_value == m_d2.isColliding());
    }
    BOOST_AUTO_TEST_CASE(Collisions_findCollisionFeatures_Diagonal_Test_S_Depth) {
        float expected_value = -7.67588f;
        BOOST_CHECK(floor(expected_value * 1000) == floor(m_d2.getDepth() * 1000));
    }
    BOOST_AUTO_TEST_CASE(Collisions_findCollisionFeatures_Diagonal_Test_S_Normal) {
        sf::Vector2f expected_value = sf::Vector2f(0.707107f, 0.707107f);
        BOOST_CHECK(floor(expected_value.x * 1000) == floor(m_d2.getNormal().x * 1000) && floor(expected_value.y * 1000) == floor(m_d2.getNormal().y * 1000));

    }
    BOOST_AUTO_TEST_CASE(Collisions_findCollisionFeatures_Diagonal_Test_S_ContactPoint) {
        sf::Vector2f expected_value = sf::Vector2f(117.324f, 117.324f);
        BOOST_CHECK(floor(expected_value.x * 1000) == floor(m_d2.getContactPoints()[0].x * 1000) && floor(expected_value.y * 1000) == floor(m_d2.getContactPoints()[0].y * 1000));
    }

BOOST_AUTO_TEST_SUITE_END();


BOOST_AUTO_TEST_SUITE(ForceGenerator_Tests)

    BOOST_AUTO_TEST_CASE(ForceGenerator_Constructor_Test1) {
        sf::Vector2f expected_value = sf::Vector2f(0.f,0.f);
        ForceGenerator fg = ForceGenerator();
        BOOST_CHECK(expected_value == fg.getForce());
    }
    BOOST_AUTO_TEST_CASE(ForceGenerator_Constructor_Test2) {
        sf::Vector2f expected_value = sf::Vector2f(10.f, 10.f);
        ForceGenerator fg = ForceGenerator(sf::Vector2f(10.f, 10.f));
        BOOST_CHECK(expected_value == fg.getForce());
    }

    ForceGenerator fg = ForceGenerator();
    BOOST_AUTO_TEST_CASE(ForceGenerator_setForce_Test) {
        sf::Vector2f expected_value = sf::Vector2f(10.f, 10.f);
        fg.setForce(sf::Vector2f(10.f, 10.f));
        BOOST_CHECK(expected_value == fg.getForce());
    }
    BOOST_AUTO_TEST_CASE(ForceGenerator_updateForce_Test) {
        sf::Vector2f expected_value = sf::Vector2f(20.f, 20.f);
        fg.setForce(sf::Vector2f(10.f, 10.f));
        BOOST_CHECK(expected_value == fg.updateForce(2.f, 0.16f));
    }

BOOST_AUTO_TEST_SUITE_END();



BOOST_AUTO_TEST_SUITE(Section_Tests)

    BOOST_AUTO_TEST_CASE(Section_Constructor_Test) {
        bool expected_value = false;
        Section s = Section();
        BOOST_CHECK(s.S1 == expected_value && s.S2 == expected_value && s.S3 == expected_value && s.S4 == expected_value);
    }
    
    Section s = Section();

    BOOST_AUTO_TEST_CASE(Section_S1_Test) {
        bool expected_value = true;
        s.S1 = true;
        BOOST_CHECK(expected_value == s.S1);
    }
    BOOST_AUTO_TEST_CASE(Section_S2_Test) {
        bool expected_value = true;
        s.S2 = true;
        BOOST_CHECK(expected_value == s.S2);
    }
    BOOST_AUTO_TEST_CASE(Section_S3_Test) {
        bool expected_value = true;
        s.S3 = true;
        BOOST_CHECK(expected_value == s.S3);
    }
    BOOST_AUTO_TEST_CASE(Section_S4_Test) {
        bool expected_value = true;
        s.S4 = true;
        BOOST_CHECK(expected_value == s.S4);
    }

    BOOST_AUTO_TEST_CASE(Section_reset_Test) {
        bool expected_value = false;
        s.S1, s.S2, s.S3, s.S4 = false, true, false, true;
        s.reset();
        BOOST_CHECK(s.S1 == expected_value && s.S2 == expected_value && s.S3 == expected_value && s.S4 == expected_value);
    }
    BOOST_AUTO_TEST_CASE(Section_setAllTrue_Test) {
        bool expected_value = true;
        s.S1, s.S2, s.S3, s.S4 = false, true, false, true;
        s.setAllTrue();
        BOOST_CHECK(s.S1 == expected_value && s.S2 == expected_value && s.S3 == expected_value && s.S4 == expected_value);
    }

BOOST_AUTO_TEST_SUITE_END();


BOOST_AUTO_TEST_SUITE(PhysicsSystem_Tests);
// All variables are private adn there are no getters as there are no need
// Therefore testing of constructors cannot occur
BOOST_AUTO_TEST_CASE(PhysicsSystem_applyForces_Test_Pos) {
    sf::Vector2f expected_value = sf::Vector2f(0.256f, 0.256f);
    RigidBody r = RigidBody(sf::Vector2f(0.f, 0.f), 3.14, 12.07);
    r.setMass(10);
    std::vector<RigidBody> rb = { r };
    std::vector<ForceGenerator> fg = { ForceGenerator(sf::Vector2f(10.f,10.f)) };
    PhysicsSystem physics = PhysicsSystem(rb, fg);
    rb = physics.applyForces(0.16);    
    BOOST_CHECK(floor(expected_value.x * 100) == (floor(rb[0].getPosition().x * 100)) && floor(expected_value.y * 100) == (floor(rb[0].getPosition().y * 100)));
}
BOOST_AUTO_TEST_CASE(PhysicsSystem_applyForces_Test_Neg) {
    sf::Vector2f expected_value = sf::Vector2f(-0.256f, -0.256f);
    RigidBody r = RigidBody(sf::Vector2f(0.f, 0.f), 3.14, 12.07);
    r.setMass(10);
    std::vector<RigidBody> rb = { r };
    std::vector<ForceGenerator> fg = { ForceGenerator(sf::Vector2f(-10.f,-10.f)) };
    PhysicsSystem physics = PhysicsSystem(rb, fg);
    rb = physics.applyForces(0.16);
    BOOST_CHECK(floor(expected_value.x * 100) == (floor(rb[0].getPosition().x * 100)) && floor(expected_value.y * 100) == (floor(rb[0].getPosition().y * 100)));
}

BOOST_AUTO_TEST_CASE(PhysicsSystem_ApplyImpulse_TwoObj_Test1) {
    //Cor Value of 0.5
    sf::Vector2f expected_value1 = sf::Vector2f(15.1568, 29.3712);
    sf::Vector2f expected_value2 = sf::Vector2f(7.24298, 4.50082);
    sf::Vector2f expected_value3 = sf::Vector2f(595.933, 52.3431);
    sf::Vector2f expected_value4 = sf::Vector2f(581.819, 63.616);
    RigidBody a = RigidBody(sf::Vector2f(595.176086f, 52.9475098f), 10);
    a.setLinearVelocity(sf::Vector2f(-28.7999821f, 58.5359688f));
    RigidBody b = RigidBody(sf::Vector2f(582.576050f, 63.0115242f), 10);
    b.setLinearVelocity(sf::Vector2f(51.199741f, -24.6639881f));
    CollisionManifold m = CollisionManifold(sf::Vector2f(0.781352997f, -0.624089301), -1.93703938);
    PhysicsSystem physics = PhysicsSystem();
    std::tuple<RigidBody, RigidBody> rbs = physics.applyImpulse(a, b, m);

    // Check Velocity Resolution
    BOOST_CHECK(floor(expected_value1.x * 1000) == (floor(std::get<0>(rbs).getLinearVelocity().x * 1000)) && floor(expected_value1.y * 1000) == (floor(std::get<0>(rbs).getLinearVelocity().y * 1000)));
    BOOST_CHECK(floor(expected_value2.x * 1000) == (floor(std::get<1>(rbs).getLinearVelocity().x * 1000)) && floor(expected_value2.y * 1000) == (floor(std::get<1>(rbs).getLinearVelocity().y * 1000)));
    // Check Depth Resolution
    BOOST_CHECK(floor(expected_value3.x * 100) == (floor(std::get<0>(rbs).getPosition().x * 100)) && floor(expected_value3.y * 100) == (floor(std::get<0>(rbs).getPosition().y * 100)));
    BOOST_CHECK(floor(expected_value4.x * 100) == (floor(std::get<1>(rbs).getPosition().x * 100)) && floor(expected_value4.y * 100) == (floor(std::get<1>(rbs).getPosition().y * 100)));

}

BOOST_AUTO_TEST_CASE(PhysicsSystem_ApplyImpulse_WallObj_Test_TopWall0) {
    //Cor Value of 0.5
    sf::Vector2f expected_value1 = sf::Vector2f(0.f, 4.f);
    sf::Vector2f expected_value2 = sf::Vector2f(150.f, 10.f);
    RigidBody a = RigidBody(sf::Vector2f(150.f, 9.f), 10);
    a.setLinearVelocity(sf::Vector2f(0.f, -5.f));
    CollisionManifold m = CollisionManifold(sf::Vector2f(0.f, -1.f), 1);
    PhysicsSystem physics = PhysicsSystem();
    a = physics.applyImpulse(a, m, 0);


    std::cout << a.getLinearVelocity().x << " , " << a.getLinearVelocity().y << std::endl;
    std::cout << a.getPosition().x << " , " << a.getPosition().y << std::endl;

    // Check Velocity Resolution
    BOOST_CHECK(floor(expected_value1.x * 1000) == (floor(a.getLinearVelocity().x * 1000)) && floor(expected_value1.y * 1000) == (floor(a.getLinearVelocity().y * 1000)));
    // Check Depth Resolution
    BOOST_CHECK(floor(expected_value2.x * 100) == (floor(a.getPosition().x * 100)) && floor(expected_value2.y * 100) == (floor(a.getPosition().y * 100)));
}
BOOST_AUTO_TEST_CASE(PhysicsSystem_ApplyImpulse_WallObj_Test_RightWall1) {
    //Cor Value of 0.5
    sf::Vector2f expected_value1 = sf::Vector2f(-4.f, 0.f);
    sf::Vector2f expected_value2 = sf::Vector2f(790.f, 150.f);
    RigidBody a = RigidBody(sf::Vector2f(791.f, 150.f), 10);
    a.setLinearVelocity(sf::Vector2f(5.f,0.f));
    CollisionManifold m = CollisionManifold(sf::Vector2f(-1.f, 0.f), 1);
    PhysicsSystem physics = PhysicsSystem();
    a = physics.applyImpulse(a, m, 1);

    std::cout << a.getLinearVelocity().x << " , " << a.getLinearVelocity().y << std::endl;
    std::cout << a.getPosition().x << " , " << a.getPosition().y << std::endl;

    // Check Velocity Resolution
    BOOST_CHECK(floor(expected_value1.x * 1000) == (floor(a.getLinearVelocity().x * 1000)) && floor(expected_value1.y * 1000) == (floor(a.getLinearVelocity().y * 1000)));
    // Check Depth Resolution
    BOOST_CHECK(floor(expected_value2.x * 100) == (floor(a.getPosition().x * 100)) && floor(expected_value2.y * 100) == (floor(a.getPosition().y * 100)));
}
BOOST_AUTO_TEST_CASE(PhysicsSystem_ApplyImpulse_WallObj_Test_BtmWall2) {
    //Cor Value of 0.5
    sf::Vector2f expected_value1 = sf::Vector2f(0.f, -4.f);
    sf::Vector2f expected_value2 = sf::Vector2f(150.f, 790.f);
    RigidBody a = RigidBody(sf::Vector2f(150.f, 791.f), 10);
    a.setLinearVelocity(sf::Vector2f(0.f, 5.f));
    CollisionManifold m = CollisionManifold(sf::Vector2f(0.f, -1.f), 1);
    PhysicsSystem physics = PhysicsSystem();
    a = physics.applyImpulse(a, m, 2);

    std::cout << a.getLinearVelocity().x << " , " << a.getLinearVelocity().y << std::endl;
    std::cout << a.getPosition().x << " , " << a.getPosition().y << std::endl;

    // Check Velocity Resolution
    BOOST_CHECK(floor(expected_value1.x * 1000) == (floor(a.getLinearVelocity().x * 1000)) && floor(expected_value1.y * 1000) == (floor(a.getLinearVelocity().y * 1000)));
    // Check Depth Resolution
    BOOST_CHECK(floor(expected_value2.x * 100) == (floor(a.getPosition().x * 100)) && floor(expected_value2.y * 100) == (floor(a.getPosition().y * 100)));
}
BOOST_AUTO_TEST_CASE(PhysicsSystem_ApplyImpulse_WallObj_Test_LeftWall3) {
    //Cor Value of 0.5
    sf::Vector2f expected_value1 = sf::Vector2f(4.f, 0.f);
    sf::Vector2f expected_value2 = sf::Vector2f(10.f, 150.f);
    RigidBody a = RigidBody(sf::Vector2f(9.f, 150.f), 10);
    a.setLinearVelocity(sf::Vector2f(-5.f, 0.f));
    CollisionManifold m = CollisionManifold(sf::Vector2f(1.f, 0.f), 1);
    PhysicsSystem physics = PhysicsSystem();
    a = physics.applyImpulse(a, m, 3);

    std::cout << a.getLinearVelocity().x << " , " << a.getLinearVelocity().y << std::endl;
    std::cout << a.getPosition().x << " , " << a.getPosition().y << std::endl;

    // Check Velocity Resolution
    BOOST_CHECK(floor(expected_value1.x * 1000) == (floor(a.getLinearVelocity().x * 1000)) && floor(expected_value1.y * 1000) == (floor(a.getLinearVelocity().y * 1000)));
    // Check Depth Resolution
    BOOST_CHECK(floor(expected_value2.x * 100) == (floor(a.getPosition().x * 100)) && floor(expected_value2.y * 100) == (floor(a.getPosition().y * 100)));
}

BOOST_AUTO_TEST_CASE(PhysicsSystem_ApplyImpulse_LineObj_Test1) {
    //Cor Value of 0.5
    sf::Vector2f expected_value1 = sf::Vector2f(-11.364, 11.364);
    sf::Vector2f expected_value2 = sf::Vector2f(99.2929, 100.707);
    RigidBody a = RigidBody(sf::Vector2f(100.f, 100.f), 10);
    a.setLinearVelocity(sf::Vector2f(-5.f, 5.f));
    CollisionManifold m = CollisionManifold(sf::Vector2f(0.707107f, -0.707107f), 1);
    PhysicsSystem physics = PhysicsSystem();
    a = physics.applyImpulse(a, m);

    std::cout << a.getLinearVelocity().x << " " << a.getLinearVelocity().y << std::endl;
    std::cout << a.getPosition().x << " " << a.getPosition().y << std::endl;

    // Check Velocity Resolution
    BOOST_CHECK(floor(expected_value1.x * 100) == (floor(a.getLinearVelocity().x * 100)) && floor(expected_value1.y * 100) == (floor(a.getLinearVelocity().y * 100)));
    // Check Depth Resolution
    BOOST_CHECK(floor(expected_value2.x * 100) == (floor(a.getPosition().x * 100)) && floor(expected_value2.y * 100) == (floor(a.getPosition().y * 100)));
}


BOOST_AUTO_TEST_SUITE_END();



BOOST_AUTO_TEST_SUITE(QuadTree_Tests);


BOOST_AUTO_TEST_CASE(QuadTree_Create_Test) {

    // Ids
    std::vector<int> objIDs = { 0 };
    //Objects:
    std::vector<Circle> circles{};
    //Root Node
    Quad root = Quad(sf::Vector2f(0, 0), sf::Vector2f(800, 800));

    //Sorting Circles
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            Circle circ(((objIDs.back()) + 1), sf::Vector2f((i * 200.f) + 100.f, (j * 200.f) + 100.f), 35.f, 0.0f, sf::Color(0.f, 0.f, 255.f));
            objIDs.push_back(objIDs.back() + 1); // create id
            circ.rigidbody.setMass(10.f);
            circles.push_back(circ);
        }
    }
    // Creating Tree
    for (int i = 0; i < circles.size(); i++) {
        Node* n = circles[i].rigidbody.getNode();
        root.insert(n);
    }
    /* 
    // Sorting Algorithm:
        std::vector<Quad*> quads = {};
        root.getLowestQuads(&root, &quads);
    */
    int expected_value = -842150451;

    // Root Node is null
    BOOST_CHECK(root.n->id == expected_value);
    // Children of Root is Node Null
    BOOST_CHECK(root.topLeftTree->n->id == expected_value);
    BOOST_CHECK(root.topRightTree->n->id == expected_value);
    BOOST_CHECK(root.btmLeftTree->n->id == expected_value);
    BOOST_CHECK(root.btmRightTree->n->id == expected_value);
    // All chidlren of Root Children are Nodes
    //      Top Left Tree:
    BOOST_CHECK(root.topLeftTree->topLeftTree->n->id == 1);
    BOOST_CHECK(root.topLeftTree->topRightTree->n->id == 5);
    BOOST_CHECK(root.topLeftTree->btmLeftTree->n->id == 2);
    BOOST_CHECK(root.topLeftTree->btmRightTree->n->id == 6);
    //      Top Right Tree:
    BOOST_CHECK(root.topRightTree->topLeftTree->n->id == 9);
    BOOST_CHECK(root.topRightTree->topRightTree->n->id == 13);
    BOOST_CHECK(root.topRightTree->btmLeftTree->n->id == 10);
    BOOST_CHECK(root.topRightTree->btmRightTree->n->id == 14);
    //      Bottom Left Tree:
    BOOST_CHECK(root.btmLeftTree->topLeftTree->n->id == 3);
    BOOST_CHECK(root.btmLeftTree->topRightTree->n->id == 7);
    BOOST_CHECK(root.btmLeftTree->btmLeftTree->n->id == 4);
    BOOST_CHECK(root.btmLeftTree->btmRightTree->n->id == 8);
    //      Bottom Right Tree:
    BOOST_CHECK(root.btmRightTree->topLeftTree->n->id == 11);
    BOOST_CHECK(root.btmRightTree->topRightTree->n->id == 15);
    BOOST_CHECK(root.btmRightTree->btmLeftTree->n->id == 12);
    BOOST_CHECK(root.btmRightTree->btmRightTree->n->id == 16);
}

BOOST_AUTO_TEST_CASE(Quadtree_OrganiseQuads_Test) {

    // Ids
    std::vector<int> objIDs = { 0 };
    //Objects:
    std::vector<Circle> circles{};
    //Root Node
    Quad root = Quad(sf::Vector2f(0, 0), sf::Vector2f(800, 800));

    //Sorting Circles
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            Circle circ(((objIDs.back()) + 1), sf::Vector2f((i * 200.f) + 100.f, (j * 200.f) + 100.f), 35.f, 0.0f, sf::Color(0.f, 0.f, 255.f));
            objIDs.push_back(objIDs.back() + 1); // create id
            circ.rigidbody.setMass(10.f);
            circles.push_back(circ);
        }
    }
    // Creating Tree
    for (int i = 0; i < circles.size(); i++) {
        Node* n = circles[i].rigidbody.getNode();
        root.insert(n);
    }
    // Sorting Algorithm:
    std::vector<Quad*> quads = {};
    root.getLowestQuads(&root, &quads);
    
    int expected_value = -842150451;

    // Check Size of List
    BOOST_CHECK(quads.size() == 4);
    // Check Nodes in list are Null
    BOOST_CHECK(quads[0]->n->id == expected_value);
    BOOST_CHECK(quads[1]->n->id == expected_value);
    BOOST_CHECK(quads[2]->n->id == expected_value);
    BOOST_CHECK(quads[3]->n->id == expected_value);
    // All chidlren of Quad Nodes are Nodes
    //      Top Left Tree:
    BOOST_CHECK(quads[0]->topLeftTree->n->id == 1);
    BOOST_CHECK(quads[0]->topRightTree->n->id == 5);
    BOOST_CHECK(quads[0]->btmLeftTree->n->id == 2);
    BOOST_CHECK(quads[0]->btmRightTree->n->id == 6);
    //      Top Right Tree:
    BOOST_CHECK(quads[1]->topLeftTree->n->id == 9);
    BOOST_CHECK(quads[1]->topRightTree->n->id == 13);
    BOOST_CHECK(quads[1]->btmLeftTree->n->id == 10);
    BOOST_CHECK(quads[1]->btmRightTree->n->id == 14);
    //      Bottom Left Tree:
    BOOST_CHECK(quads[2]->topLeftTree->n->id == 3);
    BOOST_CHECK(quads[2]->topRightTree->n->id == 7);
    BOOST_CHECK(quads[2]->btmLeftTree->n->id == 4);
    BOOST_CHECK(quads[2]->btmRightTree->n->id == 8);
    //      Bottom Right Tree:
    BOOST_CHECK(quads[3]->topLeftTree->n->id == 11);
    BOOST_CHECK(quads[3]->topRightTree->n->id == 15);
    BOOST_CHECK(quads[3]->btmLeftTree->n->id == 12);
    BOOST_CHECK(quads[3]->btmRightTree->n->id == 16);
}

BOOST_AUTO_TEST_CASE(Quadtree_Sorter_Test1) {

    // Ids
    std::vector<int> objIDs = { 0 };
    //Objects:
    std::vector<Circle> circles{};
    //Objects in potential Collision
    std::vector<Circle> circleCollisions = {};
    //Root Node
    Quad root = Quad(sf::Vector2f(0, 0), sf::Vector2f(800, 800));

    //Sorting Circles
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            Circle circ(((objIDs.back()) + 1), sf::Vector2f((i * 200.f) + 100.f, (j * 200.f) + 100.f), 35.f, 0.0f, sf::Color(0.f, 0.f, 255.f));
            objIDs.push_back(objIDs.back() + 1); // create id
            circ.rigidbody.setMass(10.f);
            circles.push_back(circ);
        }
    }
    // Creating Tree
    for (int i = 0; i < circles.size(); i++) {
        Node* n = circles[i].rigidbody.getNode();
        root.insert(n);
    }
    // Sorting Algorithm:
    std::vector<Quad*> quads = {};
    root.getLowestQuads(&root, &quads);

    int expected_value = 4;

    for (int i = 0; i < quads.size(); i++) {

        Sorter sorter = Sorter();
        sorter.sort(quads[i], circles);
        circleCollisions = sorter.circleCols;

        BOOST_CHECK(circleCollisions.size() == expected_value);
    }

}
BOOST_AUTO_TEST_CASE(Quadtree_Sorter_Test2) {

    // Ids
    std::vector<int> objIDs = { 0 };
    //Objects:
    std::vector<Circle> circles{};
    //Objects in potential Collision
    std::vector<Circle> circleCollisions = {};
    // Collsision Manifolds for collisions
    std::vector<CollisionManifold> collisions = {};
    // Objects in collsion according to the Collision Manifolds above
    std::vector<RigidBody> bodies1 = {};
    std::vector<RigidBody> bodies2 = {};
    // Measure Potential collisions between bodies
    CollisionManifold result = CollisionManifold();
    //Root Node
    Quad root = Quad(sf::Vector2f(0, 0), sf::Vector2f(800, 800));

    //Sorting Circles
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            Circle circ(((objIDs.back()) + 1), sf::Vector2f((i * 200.f) + 100.f, (j * 200.f) + 100.f), 35.f, 0.0f, sf::Color(0.f, 0.f, 255.f));
            objIDs.push_back(objIDs.back() + 1); // create id
            circ.rigidbody.setMass(10.f);
            circles.push_back(circ);
        }
    }
    // Creating Tree
    for (int i = 0; i < circles.size(); i++) {
        Node* n = circles[i].rigidbody.getNode();
        root.insert(n);
    }
    // Sorting Algorithm:
    std::vector<Quad*> quads = {};
    root.getLowestQuads(&root, &quads);

    int expected_value = 4;

    for (int i = 0; i < quads.size(); i++) {

        Sorter sorter = Sorter();
        sorter.sort(quads[i], circles);
        circleCollisions = sorter.circleCols;


        if (circleCollisions.size() >= 2) {
            for (int c1 = 0; c1 < circleCollisions.size(); c1++) {
                for (int c2 = 0; c2 < circleCollisions.size(); c2++) {
                    if (c1 != c2) {
                        result = Collisions().findcollisionfeatures(circleCollisions[c1], circleCollisions[c2]);
                        if (result != CollisionManifold() && result.isColliding()) {
                            bodies1.push_back(circleCollisions[c1].rigidbody);
                            bodies2.push_back(circleCollisions[c2].rigidbody);
                            collisions.push_back(result);
                        }
                    }
                }
            }
        }
    }
    BOOST_CHECK(bodies1.empty());
    BOOST_CHECK(bodies2.empty());
    BOOST_CHECK(collisions.empty());

}

BOOST_AUTO_TEST_SUITE_END();

BOOST_AUTO_TEST_SUITE(SectionQuadtree_Tests);

BOOST_AUTO_TEST_CASE(SectionQuadtree_Constructor_Test1) {
    std::vector<sf::Vertex> Bounds; 
    Bounds.push_back(sf::Vertex(sf::Vector2f(0.f, 0.f)));     //top left
    Bounds.push_back(sf::Vertex(sf::Vector2f(800.f, 0.f)));   //top right
    Bounds.push_back(sf::Vertex(sf::Vector2f(800.f, 800.f))); //bottom right
    Bounds.push_back(sf::Vertex(sf::Vector2f(0.f, 800.f)));   //bottom left

    Quadtree q;
    q = Quadtree(Bounds);
    BOOST_CHECK(q.bounds.size() > 0);

    std::vector<sf::Vector2f> expected_value_Vline = { sf::Vector2f(400.f,0.f), sf::Vector2f(400.f,800.f) };
    std::vector<sf::Vector2f> expected_value_Hline = { sf::Vector2f(0.f,400.f), sf::Vector2f(800.f,400.f) };

    BOOST_CHECK(q.vLine[0] == expected_value_Vline[0] && q.vLine[1] == expected_value_Vline[1]);
    BOOST_CHECK(q.hLine[0] == expected_value_Hline[0] && q.hLine[1] == expected_value_Hline[1]);
    
}

BOOST_AUTO_TEST_CASE(SectionQuadtree_sortCircles_Test) {
    std::vector<sf::Vertex> Bounds;
    Bounds.push_back(sf::Vertex(sf::Vector2f(0.f, 0.f)));     //top left
    Bounds.push_back(sf::Vertex(sf::Vector2f(800.f, 0.f)));   //top right
    Bounds.push_back(sf::Vertex(sf::Vector2f(800.f, 800.f))); //bottom right
    Bounds.push_back(sf::Vertex(sf::Vector2f(0.f, 800.f)));   //bottom left
    Quadtree q = Quadtree(Bounds);
    Circle a = Circle(1, sf::Vector2f(200.f, 200.f), 10);
    Circle b = Circle(2, sf::Vector2f(600.f, 200.f), 10);
    Circle c = Circle(3, sf::Vector2f(600.f, 600.f), 10);
    Circle d = Circle(4, sf::Vector2f(200.f, 600.f), 10);
    std::vector<Circle> circles = { a,b,c,d };
    circles = q.sortCircles(circles);

    BOOST_CHECK(circles[0].rigidbody.sections.S1 == true && circles[0].rigidbody.sections.S2 == false &&
        circles[0].rigidbody.sections.S3 == false && circles[0].rigidbody.sections.S4 == false);
    BOOST_CHECK(circles[1].rigidbody.sections.S1 == false && circles[1].rigidbody.sections.S2 == true &&
        circles[1].rigidbody.sections.S3 == false && circles[1].rigidbody.sections.S4 == false);
    BOOST_CHECK(circles[2].rigidbody.sections.S1 == false && circles[2].rigidbody.sections.S2 == false &&
        circles[2].rigidbody.sections.S3 == true && circles[2].rigidbody.sections.S4 == false);
    BOOST_CHECK(circles[3].rigidbody.sections.S1 == false && circles[3].rigidbody.sections.S2 == false &&
        circles[3].rigidbody.sections.S3 == false && circles[3].rigidbody.sections.S4 == true);

}

BOOST_AUTO_TEST_SUITE_END();