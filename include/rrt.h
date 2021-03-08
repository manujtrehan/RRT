#ifndef RRT_H
#define RRT_H

#include <iostream>
#include <vector>
#include <random>
#include <cstdlib>

static const int map_x = 800; //global map co-ords
static const int map_y = 800;

struct Point { //2D point struct
	float x, y;

	Point(float x, float y) {
		this->x = x;
		this->y = y;
	}
};

struct Obs { //rectangular obstacle struct, stores top-left (x1, y1) and bottom-right (x2, y2) co-ords
	float x1, y1, x2, y2;

	Obs(float x1, float y1, float x2, float y2) {
		this->x1 = x1;
		this->y1 = y1;
		this->x2 = x2;
		this->y2 = y2;
	}
};

class Node {
	public:
		float x;
		float y;
		double parent_dist;
		int parent;
		
		Node(float x, float y, int p);

		Node();
};

class RRT {
	public:

		Node randomNode();
		int nearestNode(Node& rand_node, std::vector<Node> list);
		Node threshCheck(Node rand_node, int near_ind, std::vector<Node> list, double thresh);
		bool collisionCheck(Node rand_node, int near_ind, std::vector<Node> list, std::vector<Obs> ob);
		std::vector<Node> rrt(Point start_p, Point goal_p, std::vector<Obs> ob, int num_itr, double step_thresh, double goal_thresh);
		//bool line_intersect(Point a1, Point a2, double m1, Point b1, Point b2, double m2);

};

#endif