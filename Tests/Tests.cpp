#include "pch.h"
#include "CppUnitTest.h"
#include "../SFML Tinker/CollisionManifold.h"
#include "../SFML Tinker/Functions.h"
#include "../SFML Tinker/Shapes.h"

#include <SFML/Graphics.hpp>
#include <Windows.h>

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace Tests
{
	TEST_CLASS(CollisionManifoldTests)
	{
	public:
		
		TEST_METHOD(PointOnLineShouldReturnTrueTest)
		{
			sf::VertexArray line = {};
			line.append(sf::Vector2f(0.0f, 0.0f));
			line.append(sf::Vector2f(10.0f, 10.0f));

			sf::Vector2f point = sf::Vector2f(0, 0);

			Assert::IsTrue(IntersectionDetection().pointOnLine(point, line));
		}

		TEST_METHOD(John) {
			Assert::IsTrue(true);
		}
	};
}
