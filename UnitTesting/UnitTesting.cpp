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

        

        BOOST_CHECK(expected_value == rb.getMass());
    }
    BOOST_AUTO_TEST_CASE(Rigidbody_Contsructor_Test2) {
        float expected_value = 12.07;

        //RigidBody rb = RigidBody(sf::Vector2f(1.f, 1.f), 3.14, 12.07);

        BOOST_CHECK(expected_value == rb.getRotation());
    }
    BOOST_AUTO_TEST_CASE(Rigidbody_Contsructor_Test3) {
        sf::Vector2f expected_value = sf::Vector2f(1.f,1.f);

        //RigidBody rb = RigidBody(sf::Vector2f(1.f, 1.f), 3.14, 12.07);

        BOOST_CHECK(expected_value == rb.getPosition());
    }

    BOOST_AUTO_TEST_CASE(Rigidbody_SetPosition_Test) {
        sf::Vector2f expected_value = sf::Vector2f(64.f, 64.f);
        rb.setPosition(sf::Vector2f(64.f, 64.f));
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
    //Need Force Accum TESTs / not implemented yet

    BOOST_AUTO_TEST_CASE(Test) {
        BOOST_CHECK(false);
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

//same  number grow up ffs
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



BOOST_AUTO_TEST_SUITE_END();


