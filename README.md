# Output
![Image](https://github.com/manujtrehan/RRT/blob/main/output/rrt.jpg)

# RRT
An implementation of Rapidly-exploring Random Trees in 2D

## Packages used
OpenCV (tested with 3.4.11) to visualize the output

## Input
To be given in 'draw_rrt.cpp' inside the main() function  
Start location: Point s(x, y)  
Goal location: Point g(x, y)  

## Obstacle definition
Obstacles: Rectangular/square obstacles - defined by top-left and bottom-right vertices  
Format of obstacle definition: Obs o1(x1, y1, x2, y2) where:  
Top-left corner - (x1, y1)  
Bottom-right corner - (x2, y2)  
So, x1 < x2 and y1 > y2 should hold true  

## Environment size
Environment (map) size is defined in 'rrt.h' - (map_x, map_y)
