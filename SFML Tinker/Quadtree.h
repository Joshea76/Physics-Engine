#pragma once
#include <SFML/Graphics.hpp>
#include <Windows.h>
#include <iostream>
#include <list>
#include <math.h>
#include <cassert>
#include <algorithm>
#include <array>
#include <memory>
#include <type_traits>
#include <vector>





struct Node {
public:
	int id;
	sf::Vector2f pos;
	//Simple obj;
};
class Quad {
private:

	//Boundary of Node
	sf::Vector2f topLeft;
	sf::Vector2f btmRight;



public:


	//Details of Node
	Node* n;

	//Children of Tree
	Quad* topLeftTree;
	Quad* topRightTree;
	Quad* btmLeftTree;
	Quad* btmRightTree;

	Quad() {
		this->topLeft = sf::Vector2f(0.f, 0.f);
		this->btmRight = sf::Vector2f(0.f, 0.f);
		this->n = NULL;
		this->topLeftTree = NULL;
		this->topRightTree = NULL;
		this->btmLeftTree = NULL;
		this->btmRightTree = NULL;
	}
	Quad(sf::Vector2f tl, sf::Vector2f br) {
		this->topLeft = tl;
		this->btmRight = br;
		this->n = NULL;
		this->topLeftTree = NULL;
		this->topRightTree = NULL;
		this->btmLeftTree = NULL;
		this->btmRightTree = NULL;
	}
	void insert(Node*);
	bool inBoundary(sf::Vector2f);
	std::vector<Quad*>* getLowestQuads(Quad* q, std::vector<Quad*>* lowestQuads ) {
		//Quad* localQuad = q;
		if (q != nullptr) {
			if (q->topLeftTree != NULL) {
				if (q->topLeftTree->topLeftTree == NULL && q->topLeftTree->topRightTree == NULL && q->topLeftTree->btmLeftTree == NULL && q->topLeftTree->btmRightTree == NULL) {
					lowestQuads->push_back(q);
					return lowestQuads;
				}
				else {
					getLowestQuads(q->topLeftTree, lowestQuads);
				}
			}
			if (q->topRightTree != NULL) {
				if (q->topRightTree->topLeftTree == NULL && q->topRightTree->topRightTree == NULL && q->topRightTree->btmLeftTree == NULL && q->topRightTree->btmRightTree == NULL) {
					lowestQuads->push_back(q);
					return lowestQuads;
				}
				else {
					getLowestQuads(q->topRightTree, lowestQuads);
				}
			}
			if (q->btmLeftTree != NULL) {
				if (q->btmLeftTree->topLeftTree == NULL && q->btmLeftTree->topRightTree == NULL && q->btmLeftTree->btmLeftTree == NULL && q->btmLeftTree->btmRightTree == NULL) {
					lowestQuads->push_back(q);
					return lowestQuads;
				}
				else {
					getLowestQuads(q->btmLeftTree, lowestQuads);
				}
			}
			if (q->btmRightTree != NULL) {
				if (q->btmRightTree->topLeftTree == NULL && q->btmRightTree->topRightTree == NULL && q->btmRightTree->btmLeftTree == NULL && q->btmRightTree->btmRightTree == NULL) {
					lowestQuads->push_back(q);
					return lowestQuads;
				}
				else {
					getLowestQuads(q->btmRightTree, lowestQuads);
				}
			}
		}
	}
	Node* search(sf::Vector2f v) {
		//Cannot Contain it
		if (!inBoundary(v)) { return NULL; }

		if (n != NULL) { return n; }

		
		if ((topLeft.x + btmRight.x) / 2 >= v.x) {
			//Indicates Top Left Tree
			if ((topLeft.y + btmRight.y) / 2 >= v.y) {
				if (topLeftTree == NULL) { return NULL; }
				else { return topLeftTree->search(v); }
			}
			//Indicates Btm Left Tree
			else {
				if (btmLeftTree == NULL) { return NULL; }
				else { return btmLeftTree->search(v); }
			}
		}
		else {
			//Indicates Top Right Tree
			if ((topLeft.y + btmRight.y) / 2 >= v.y) {
				if (topRightTree == NULL) { return NULL; }
				else { return topRightTree->search(v); }
			}
			//Indicates Btm Right Tree
			else {
				if (btmRightTree == NULL) { return NULL; }
				else { return btmRightTree->search(v); }
			}
		}
		
	}
};

bool Quad::inBoundary(sf::Vector2f v) {
	return (v.x >= topLeft.x &&
		v.x <= btmRight.x &&
		v.y >= topLeft.y &&
		v.y <= btmRight.y);
}

void Quad::insert(Node* node) {

	if (node == NULL) { return; }

	//Current Quad Does Not Contain
	if (!inBoundary(node->pos)) { return; }

	if (n == NULL) {
		n = node;
		return;
	}

	if ((topLeft.x + btmRight.x) / 2 >= node->pos.x) {
		//Indicates Top Left Tree
		if ((topLeft.y + btmRight.y) / 2 >= node->pos.y) {
			if (topLeftTree == NULL) {
				topLeftTree = new Quad(	topLeft, sf::Vector2f((topLeft.x + btmRight.x) / 2, (topLeft.y + btmRight.y) / 2));
				if (n->id != node->id && n->id != -842150451 /* default Value */ ) {
					this->insert(n);
					this->n = new Node;
				}
			}
			topLeftTree->insert(node);			
		}
		//Indicates Btm Left Tree
		else {
			if (btmLeftTree == NULL) {
				btmLeftTree = new Quad(sf::Vector2f(topLeft.x, (topLeft.y + btmRight.y) / 2), sf::Vector2f((topLeft.x + btmRight.x) / 2, btmRight.y));
				if (n->id != node->id && n->id != -842150451 /* default Value */) {
					this->insert(n);
					this->n = new Node;
				}
			}
			btmLeftTree->insert(node);			
		}
	}
	else {
		//Indicates Top Right Tree
		if ((topLeft.y + btmRight.y) / 2 >= node->pos.y) {
			if (topRightTree == NULL) {
				topRightTree = new Quad(sf::Vector2f((topLeft.x + btmRight.x) / 2, topLeft.y), sf::Vector2f(btmRight.x, (topLeft.y + btmRight.y) / 2));
				if (n->id != node->id && n->id != -842150451 /* default Value */) {
					this->insert(n);
					this->n = new Node;
				}
			}
			topRightTree->insert(node);
		}
		//Indicates Btm Right Tree
		else {
			if (btmRightTree == NULL) {
				btmRightTree = new Quad(sf::Vector2f((topLeft.x + btmRight.x) / 2, (topLeft.y + btmRight.y) / 2),btmRight);
				if (n->id != node->id && n->id != -842150451 /* default Value */) {
					this->insert(n);
					this->n = new Node;
				}
			}
			btmRightTree->insert(node);
		}
	}
}

