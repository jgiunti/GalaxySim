// BarnesHut.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <list>
#include <iostream>
#include <sstream>
#include <string>
#include <fstream>
#include <math.h>
#include <stdlib.h>
#include <ctime>

using namespace std;

class Body {
public:
	double x;
	double y;
	double vx;
	double vy;
	double mass;
	int r, g, b;

	Body(double xIn, double yIn, double vxIn, double xyIn, double massIn, int rIn, int gIn, int bIn) {
		x = xIn;
		y = yIn;
		vx = vxIn;
		vy = xyIn;
		mass = massIn;
		r = rIn;
		g = gIn;
		b = bIn;
	}

};

class Node {
public:
	float xmin;
	float xmax;
	float ymin;
	float ymax;
	double mass;
	double cMass[2];
	Node* nextNode[4];
	list<Body> bodyList;

	Node(float xminIn, float xmaxIn, float yminIn, float ymaxIn, list<Body> bList) {
		xmin = xminIn;
		xmax = xmaxIn;
		ymin = yminIn;
		ymax = ymaxIn;
		bodyList = bList;

		for each (Body b in bList)
		{
			cMass[0] += b.mass * b.x;
			cMass[1] += b.mass * b.y;
			mass += b.mass;
		}

		cMass[0] /= mass;
		cMass[1] /= mass;
	}
};

const double GLOBAL_XMIN = pow(10, 35);
const double GLOBAL_XMAX = pow(10, 35) * -1;
const double GLOBAL_YMIN = pow(10, 35);
const double GLOBAL_YMAX = pow(10, 35) * -1;

bool isFirstQuad(Body b, float xmin, float xmax, float ymin, float ymax) {
	if (b.x == 0 && b.y == 0) { return true; }
	if (b.x < GLOBAL_XMAX && b.x > GLOBAL_XMIN && b.y < GLOBAL_YMAX && b.y > GLOBAL_YMIN) {
		if ((b.x > ((xmin + xmax) / 2)) && (b.y < ((ymin + ymax) / 2))) {
			return true;
		}
		else {
			return false;
		}
	}
	return false;
}

bool isSecondQuad(Body b, float xmin, float xmax, float ymin, float ymax) {
	if (b.x < GLOBAL_XMAX && b.x > GLOBAL_XMIN && b.y < GLOBAL_YMAX && b.y > GLOBAL_YMIN) {
		if ((b.x > ((xmin + xmax) / 2)) && (b.y > ((ymin + ymax) / 2))) {
			return true;
		}
		else {
			return false;
		}
	}
	return false;
}

bool isThirdQuad(Body b, float xmin, float xmax, float ymin, float ymax) {
	if (b.x < GLOBAL_XMAX && b.x > GLOBAL_XMIN && b.y < GLOBAL_YMAX && b.y > GLOBAL_YMIN) {
		if ((b.x < ((xmin + xmax) / 2)) && (b.y >((ymin + ymax) / 2))) {
			return true;
		}
		else {
			return false;
		}
	}
	return false;
}

bool isFourthQuad(Body b, float xmin, float xmax, float ymin, float ymax) {
	if (b.x < GLOBAL_XMAX && b.x > GLOBAL_XMIN && b.y < GLOBAL_YMAX && b.y > GLOBAL_YMIN) {
		if ((b.x < ((xmin + xmax) / 2)) && (b.y < ((ymin + ymax) / 2))) {
			return true;
		}
		else {
			return false;
		}
	}
	return false;
}

class Force {
public:
	double forceXY[2];

	Force() {
		forceXY[0] = 0;
		forceXY[1] = 0;
	}
};

const double G = 6.64300 * pow(10, -11);
const double THETA = .5;
const double dt = .3;

Force calcForce(Body b, Node* n) {
	double dist;
	Force fxy = Force();

	if (n->bodyList.size() == 1) {
		dist = sqrt(pow((b.x - n->bodyList.front().x), 2) + pow((b.y - n->bodyList.front().y), 2));
		if (dist != 0) {
			fxy.forceXY[0] = ((G * b.mass * n->bodyList.front().mass * (n->bodyList.front().x - b.x)) / pow(dist, 3));
			fxy.forceXY[1] = ((G * b.mass * n->bodyList.front().mass * (n->bodyList.front().y - b.y)) / pow(dist, 3));
		}
	}
	else {
		dist = sqrt(pow((n->cMass[0] - b.x), 2) + pow((n->cMass[1] - b.y), 2));
		if (dist != 0 && abs(n->xmax - n->xmin) / dist < THETA) {
			fxy.forceXY[0] = ((G * b.mass * n->mass * (n->cMass[0] - b.x)) / pow(dist, 3));
			fxy.forceXY[1] = ((G * b.mass * n->mass * (n->cMass[1] - b.y)) / pow(dist, 3));
		}
		else
		{
			Force f0 = Force();
			Force f1 = Force();
			Force f2 = Force();
			Force f3 = Force();

			if (n->nextNode[0] != NULL) {
				f0 = calcForce(b, n->nextNode[0]);
			}
			if (n->nextNode[1] != NULL) {
				f0 = calcForce(b, n->nextNode[1]);
			}
			if (n->nextNode[2] != NULL) {
				f0 = calcForce(b, n->nextNode[2]);
			}
			if (n->nextNode[3] != NULL) {
				f0 = calcForce(b, n->nextNode[3]);
			}
			fxy.forceXY[0] = f0.forceXY[0] + f1.forceXY[0] + f2.forceXY[0] + f3.forceXY[0];
			fxy.forceXY[1] = f0.forceXY[1] + f1.forceXY[1] + f2.forceXY[1] + f3.forceXY[1];
		}
	}
	return fxy;
}
int nodeCount = 0;

class BHTree {
public:
	Node* root;

	Node* buildTree(float xmin, float xmax, float ymin, float ymax, list<Body> *bList) {
		if ((*bList).empty()) {
			return NULL;
		}
		if ((*bList).size() == 1) {
			return new Node(xmin, xmax, ymin, ymax, (*bList));
		}
		else {
			list<Body> l0;
			list<Body> l1;
			list<Body> l2;
			list<Body> l3;

			list<Body>::iterator iBody;
			int i = 0;
			for (iBody = (*bList).begin(); iBody != (*bList).end(); ++iBody) {
				if (isFirstQuad(*iBody, xmin, xmax, ymin, ymax)) {
					l0.push_back(*iBody);
				}
				if (isSecondQuad(*iBody, xmin, xmax, ymin, ymax)) {
					l1.push_back(*iBody);
				}
				if (isThirdQuad(*iBody, xmin, xmax, ymin, ymax)) {
					l2.push_back(*iBody);
				}
				if (isFourthQuad(*iBody, xmin, xmax, ymin, ymax)) {
					l3.push_back(*iBody);
				}
			}
			Node* temp0 = buildTree((xmin + xmax) / 2, xmax, ymin, (ymin + ymax) / 2, &l0);
			Node* temp1 = buildTree((xmin + xmax) / 2, xmax, (ymin + ymax) / 2, ymax, &l1);
			Node* temp2 = buildTree(xmin, (xmin + xmax) / 2, (ymin + ymax) / 2, ymax, &l2);
			Node* temp3 = buildTree(xmin, (xmin + xmax) / 2, ymin, (ymin + ymax) / 2, &l3);

			Node* tempNode = new Node(xmin, xmax, ymin, ymax, *bList);

			tempNode->nextNode[0] = temp0;
			tempNode->nextNode[1] = temp1;
			tempNode->nextNode[2] = temp2;
			tempNode->nextNode[3] = temp3;

			return tempNode;
		}
	}

	void reposition(list<Body> *blist) {
		double x[5];
		double y[5];

		list<Body>::iterator iBody;
		int i = 0;
		for (iBody = (*blist).begin(); iBody != (*blist).end(); ++iBody)
		{
			Force F = calcForce(*iBody, root);
			double vx = iBody->vx + ((dt * F.forceXY[0]) / iBody->mass);
			double vy = iBody->vy + ((dt * F.forceXY[1]) / iBody->mass);
			x[i] = iBody->x + (vx * dt);
			y[i] = iBody->y + (vy * dt);
			iBody->vx = vx;
			iBody->vy = vy;
			i++;
		}
		i = 0;
		for (iBody = (*blist).begin(); iBody != (*blist).end(); ++iBody)
		{
			iBody->x = x[i];
			iBody->y = y[i];
			i++;
		}
	}
};





int _tmain(int argc, _TCHAR* argv[])
{
	ifstream infile("G:/Documents/Computer Science/Parallel/proj 1/Project1_input_files/planets.in");
	string line;
	int numBodies;
	double radius;

	getline(infile, line);
	numBodies = stoi(line);

	getline(infile, line);
	radius = stod(line);

	list<Body> initList;

	while (getline(infile, line))
	{
		istringstream iss(line);
		double x, y, vx, vy, mass;
		int r, g, b;
		if (!(iss >> x >> y >> vx >> vy >> mass >> r >> g >> b)) { break; } // error

		Body bod = Body(x, y, vx, vy, mass, r, g, b);

		initList.push_back(bod);
	}
	infile.close();

	BHTree bh;

	ofstream outfile;
	outfile.open("G:/Documents/Computer Science/Parallel/bhout.out");
	outfile << numBodies << '\n';
	outfile << 100 << '\n';
	outfile << radius << '\n';

	list<Body>::iterator iBody;
	for (iBody = initList.begin(); iBody != initList.end(); ++iBody) {
		outfile << (*iBody).r << ' ' << (*iBody).g << ' ' << (*iBody).b << '\n';
	}
	for (iBody = initList.begin(); iBody != initList.end(); ++iBody) {
		outfile << (*iBody).x << ' ' << (*iBody).y << '\n';
	}

	int s = 0;
	while (s < 100) {
		bh.root = bh.buildTree(GLOBAL_XMIN, GLOBAL_XMAX, GLOBAL_YMIN, GLOBAL_YMAX, &initList);
		bh.reposition(&initList);
		for (iBody = initList.begin(); iBody != initList.end(); ++iBody) {
			outfile << (*iBody).x << ' ' << (*iBody).y << '\n';
		}
		s++;
	}

	return 0;
}

