#include "rrt.h"

Node::Node(float x, float y, int p) {
	this->x = x;
	this->y = y;
	this->parent = p;
	this->parent_dist = -1;
}

Node::Node() {
	this->x = -1;
	this->y = -1;
	this->parent = -1;
	this->parent_dist = -1;
}

std::random_device rd; //random number generation

Node RRT::randomNode() { //random node sampling from map
	Point r(0, 0);
	std::mt19937 gen(rd());
	std::uniform_int_distribution<> x_dist(1, 50000);
	std::uniform_int_distribution<> y_dist(1, 50000);
	r.x = (float) (x_dist(gen) % map_x); //uniform distribution - random point sample between map limits
	r.y = (float) (y_dist(gen) % map_y);
	std::cout << r.x << " " << r.y << std::endl;
	Node rNode(r.x, r.y, -1);
	return rNode;
}

int RRT::nearestNode(Node &rand_node, std::vector<Node> list) { //find closest node to random node
	double min_dist = sqrt(pow((rand_node.x - list.at(0).x), 2) + pow((rand_node.y - list.at(0).y), 2)), dist = 0;
	int ind = 0;
	for (int i = 1; i < list.size(); i++) {
		dist = sqrt(pow((rand_node.x - list.at(i).x), 2) + pow((rand_node.y - list.at(i).y), 2));
		if (dist < min_dist) {
			min_dist = dist;
			ind = i;
		}
	}
	rand_node.parent_dist = min_dist;
	return ind;
}

Node RRT::threshCheck(Node rand_node, int near_ind, std::vector<Node> list, double thresh) {
	if (rand_node.parent_dist > thresh) { //thresholding required
		float t = thresh / rand_node.parent_dist;
		rand_node.x = (1 - t) * list.at(near_ind).x + t * rand_node.x;
		rand_node.y = (1 - t) * list.at(near_ind).y + t * rand_node.y;
		rand_node.parent_dist = thresh;
	}
	rand_node.parent = near_ind;
	return rand_node;
}

bool RRT::collisionCheck(Node rand_node, int near_ind, std::vector<Node> list, std::vector<Obs> ob) { //check collision with all obs
	float m, d1, d2, d3, d4;
	Point p1(0, 0), p2(0, 0);
	if (rand_node.x == list.at(near_ind).x) { //define slope of line segment
		m = std::numeric_limits<float>::infinity();
	}
	else {
		m = (rand_node.y - list.at(near_ind).y) / (rand_node.x - list.at(near_ind).x);
	}

	for (int i = 0; i < ob.size(); i++) {
		//check if node lies inside obstacle
		if (rand_node.x >= ob.at(i).x1 && rand_node.x <= ob.at(i).x2 && rand_node.y >= ob.at(i).y2 && rand_node.y <= ob.at(i).y1) {
			return true;
		}

		//check if mid-point of line-segment lies inside obstacle - quick check
		p1.x = (rand_node.x + list.at(near_ind).x) / 2;
		p1.y = (rand_node.y + list.at(near_ind).y) / 2;
		if (p1.x >= ob.at(i).x1 && p1.x <= ob.at(i).x2 && p1.y >= ob.at(i).y2 && p1.y <= ob.at(i).y1) {
			return true;
		}
		
		//define other two vertices of obstacle - bottom-left and top-right
		p1.x = ob.at(i).x1;
		p1.y = ob.at(i).y2;
		p2.x = ob.at(i).x2;
		p2.y = ob.at(i).y1;
		
		//input each obstacle vertex into line equation
		d1 = m * (ob.at(i).x1 - list.at(near_ind).x) + list.at(near_ind).y - ob.at(i).y1;
		d2 = m * (p1.x - list.at(near_ind).x) + list.at(near_ind).y - p1.y;
		d3 = m * (ob.at(i).x2 - list.at(near_ind).x) + list.at(near_ind).y - ob.at(i).y2;
		d4 = m * (p2.x - list.at(near_ind).x) + list.at(near_ind).y - p2.y;

		if (d1*d2 > 0 && d2*d3 > 0 && d3*d4 > 0) { //no collision - all 4 rect vertices lie on same side of segment
			if (abs(d1) > 3 && abs(d2) > 3 && abs(d3) > 3 && abs(d4) > 3) {
				continue;
			}
		}

		//no collision - both line segment points are completely to the left or right of the obstacle
		else if (std::max(rand_node.x, list.at(near_ind).x) < ob.at(i).x1 - 3 || std::min(rand_node.x, list.at(near_ind).x) > ob.at(i).x2 + 3) {
			continue;
		}

		//no collision - both line segment points are completely above or below the obstacle
		else if (std::max(rand_node.y, list.at(near_ind).y) < ob.at(i).y2 - 3 || std::min(rand_node.y, list.at(near_ind).y) > ob.at(i).y1 + 3) {
			continue;
		}
		else { //collision occurs
			return true;
		}		
	}
	return false;


	/*p1.x = list.at(near_ind).x;
	p1.y = list.at(near_ind).y;
	p2.x = rand_node.x;
	p2.y = rand_node.y;
	Point a(0, 0), b(0, 0), c(0, 0), d(0, 0);
	for (int i = 0; i < ob.size(); i++) {
		//check if node lies inside obstacle
		if (rand_node.x >= ob.at(i).x1 && rand_node.x <= ob.at(i).x2 && rand_node.y >= ob.at(i).y2 && rand_node.y <= ob.at(i).y1) {
			return true;
		}

		a.x = ob.at(i).x1;
		a.y = ob.at(i).y1;
		b.x = ob.at(i).x2;
		b.y = ob.at(i).y1;
		c.x = ob.at(i).x2;
		c.y = ob.at(i).y2;
		d.x = ob.at(i).x1;
		d.y = ob.at(i).y2;

		if (line_intersect(p1, p2, m, a, b, 0)) {
			return true;
		}
		else if (line_intersect(p1, p2, m, b, c, std::numeric_limits<double>::infinity())) {
			return true;
		}
		else if (line_intersect(p1, p2, m, c, d, 0)) {
			return true;
		}
		else if (line_intersect(p1, p2, m, d, a, std::numeric_limits<double>::infinity())) {
			return true;
		}
	}
	return false;*/
}

std::vector<Node> RRT::rrt(Point start_p, Point goal_p, std::vector<Obs> ob, int num_itr, double step_thresh, double goal_thresh) {
	RRT new_rrt;

	std::vector<Node> nodeList;
	Node root(start_p.x, start_p.y, -1), goal(goal_p.x, goal_p.y, -1), rand;
	nodeList.push_back(root);
	double goal_dist;
	int near_ind;

	for (int i = 0; i < num_itr; i++) {

		//sample a random node
		rand = new_rrt.randomNode();

		//find index of nearest node in nodeList
		near_ind = new_rrt.nearestNode(rand, nodeList);
		
		//calculate new node based on step_thresh, along path joining nearest and rand
		rand = new_rrt.threshCheck(rand, near_ind, nodeList, step_thresh);

		//check obstacle collision on path between nearest and new node
		if (new_rrt.collisionCheck(rand, near_ind, nodeList, ob)) {
			continue;
		}

		//push new node into nodeList
		nodeList.push_back(rand);

		//check goal dist threshold
		goal_dist = sqrt(pow((rand.x - goal.x), 2) + pow((rand.y - goal.y), 2));
		if (goal_dist <= goal_thresh) {
			std::cout << "found goal" << std::endl;
			goal.parent = nodeList.size() - 1;
			nodeList.push_back(goal);
			return nodeList;
		}
	}
	std::cout << "goal not found" << std::endl;
	return nodeList;
}


/*bool RRT::line_intersect(Point a1, Point a2, double m1, Point b1, Point b2, double m2) {
	double c1, c2;
	Point in(0, 0);
	c1 = a1.y - m1 * a1.x;
	c2 = b1.y - m2 * b1.x;
	if (m1 == m2) {
		if (c1 == c2) {
			if (a1.x >= std::min(b1.x, b2.x) && a1.x <= std::max(b1.x, b2.x) && a1.y >= std::min(b1.y, b2.y) && a1.y <= std::max(b1.y, b2.y)) {
				return true;
			}
			else if (a2.x >= std::min(b1.x, b2.x) && a2.x <= std::max(b1.x, b2.x) && a2.y >= std::min(b1.y, b2.y) && a2.y <= std::max(b1.y, b2.y)) {
				return true;
			}
			else {
				return false;
			}
		}
		else {
			return false;
		}
	}
	else {
		in.x = (c2 - c1) / (m1 - m2);
		in.y = (m1 * c1 - m2 * c2) / (m1 - m2);
		if (abs(m1 * ((double)in.x - a1.x) + a1.y - in.y) < 20 && abs(m2*((double)in.x - b1.x) + b1.y - in.y) < 20) {
			return true;
		}
		else {
			return false;
		}
	}
}*/