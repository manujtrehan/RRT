#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "rrt.h"

class DrawRRT {
	public:

		void drawRRT(std::vector<Node> n_list, Point g, std::vector<Obs> ob_vec) {
			cv::Mat img(map_y, map_x, CV_8UC3, cv::Scalar(255, 255, 255)), flipped;
			cv::Point start_cen, goal_cen, p1, p2;
			start_cen.x = n_list.at(0).x;
			start_cen.y = n_list.at(0).y;
			goal_cen.x = g.x;
			goal_cen.y = g.y;
			cv::circle(img, start_cen, 5, cv::Scalar(255, 0, 0), cv::FILLED);
			cv::circle(img, goal_cen, 5, cv::Scalar(0, 255, 0), cv::FILLED);

			for (int i = 0; i < ob_vec.size(); i++) {
				p1.x = ob_vec.at(i).x1;
				p1.y = ob_vec.at(i).y2;
				p2.x = ob_vec.at(i).x2;
				p2.y = ob_vec.at(i).y1;
				cv::rectangle(img, p1, p2, cv::Scalar(0, 0, 0), cv::FILLED);
			}

			cv::flip(img, flipped, 0);
			cv::imwrite("output/map.jpg", flipped);
			std::cout << n_list.size() << std::endl;

			for (int i = 1; i < n_list.size(); i++) {
				p1.x = n_list.at(i).x;
				p1.y = n_list.at(i).y;
				p2.x = n_list.at(n_list.at(i).parent).x;
				p2.y = n_list.at(n_list.at(i).parent).y;
				cv::circle(img, p1, 2, cv::Scalar(0, 0, 255), cv::FILLED);
				cv::line(img, p1, p2, cv::Scalar(0, 255, 255), 1);
			}
			Node l1, l2;
			l1 = n_list.back();
			l2 = n_list.at(l1.parent);
			while (true) {
				if (l2.parent == -1) {
					p1.x = l1.x;
					p1.y = l1.y;
					p2.x = l2.x;
					p2.y = l2.y;
					cv::line(img, p1, p2, cv::Scalar(255, 0, 0), 1);
					break;
				}
				p1.x = l1.x;
				p1.y = l1.y;
				p2.x = l2.x;
				p2.y = l2.y;
				cv::line(img, p1, p2, cv::Scalar(255, 0, 0), 1);
				cv::circle(img, p2, 2, cv::Scalar(0, 165, 255), cv::FILLED);
				l1 = l2;
				l2 = n_list.at(l2.parent);
			}
			cv::flip(img, flipped, 0);
			cv::imwrite("output/rrt.jpg", flipped);
		}
};

int main() {

	Point s(40, 40); //define start point

	Point g(750, 750); //define goal point

	int n = 5000; //max number of iterations

	double step = 45; //step size threshold

	double goal = 20; //goal threshold distance

	std::vector<Node> list;

	//define obstacles - rectangles/squares only - (x1, y1, x2, y2)
	// top-left - (x1, y1); bottom-right (x2, y2)
	Obs o1(100, 600, 200, 0);
	Obs o2(300, 800, 400, 200);
	Obs o3(500, 700, 700, 500);

	//push all obstacles into vector
	std::vector<Obs> ob_vec;
	ob_vec.push_back(o1);
	ob_vec.push_back(o2);
	ob_vec.push_back(o3);

	RRT my_rrt;
	list = my_rrt.rrt(s, g, ob_vec, n, step, goal);
	DrawRRT draw;
	draw.drawRRT(list, g, ob_vec);
}