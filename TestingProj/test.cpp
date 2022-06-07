#include "pch.h"

#include <SFML/Graphics.hpp>
#include <Windows.h>
#include <iostream>
#include <list>
#include <math.h>

#include "../SFML Tinker/CollisionManifold.h"


TEST(TestCaseName, TestName) {
  EXPECT_EQ(1, 1);
  EXPECT_TRUE(true);
}

//IntersectionDetection ( point, line )
TEST(InterSectionDetection, PointOnLineShouldReturnTrue) {
	sf::VertexArray line;
	line.append(sf::Vector2f(0.0f, 0.0f));
	line.append(sf::Vector2f(10.0f, 10.0f));

	sf::Vector2f point = sf::Vector2f(0.0f, 0.0f);

	EXPECT_TRUE(IntersectionDetection().pointOnLine(point, line));
}
TEST(IntersectionDetection, PointOnLineShouldReturnTrueTwo) {
	EXPECT_FLOAT_EQ(1, 1);
}

//C:\Users\joshe\source\repos\SFML Tinker\packages\Microsoft.googletest.v140.windesktop.msvcstl.dyn.rt-dyn.1.8.1.4\lib\native\v140\windesktop\msvcstl\dyn\rt-dyn\x64\Debug