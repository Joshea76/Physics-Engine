#include "pch.h"
#include "CppUnitTest.h"
#include "../SFML Tinker/CollisionManifold.h"
#include "../SFML Tinker/Functions.h"
#include "../SFML Tinker/Shapes.h"

#include "../SFML Tinker/main.cpp"

#include <SFML/Graphics.hpp>
#include <Windows.h>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace Testing
{
	TEST_CLASS(Testing)
	{
	public:
		
		TEST_METHOD(Tester)
		{
			Assert::IsTrue(true);
		}

		TEST_METHOD(PointOnLineShouldReturnTrue) {
			sf::VertexArray line;
			line.append(sf::Vertex(sf::Vector2f(0.0f, 0.0f)));
			line.append(sf::Vertex(sf::Vector2f(10.0f, 10.0f)));

			sf::Vector2f point = sf::Vector2f(0.0f, 0.0f);

			Assert::IsTrue(IntersectionDetection().pointOnLine(point, line));

			

		}
	};
}
