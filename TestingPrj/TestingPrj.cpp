#include "pch.h"
#include "CppUnitTest.h"

#include ""


#include <SFML/Graphics.hpp>
#include <Windows.h>
#include <iostream>
#include <vector>
#include <math.h>

#include "../SFML Tinker/main.cpp"



using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace TestingPrj
{
	TEST_CLASS(TestingPrj)
	{
	public:
		
		TEST_METHOD(TestMethod1)
		{
			Assert::IsTrue(true);
		}
		
		TEST_METHOD(TestMethod2) {
			sf::Vector2f x = sf::Vector2f(AddVectors(sf::Vector2f(1.f, 1.f), sf::Vector2f(1.f, 1.f)));
			Assert::AreEqual(1,1);
		}
	};
}
