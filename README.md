# Laboratory of Applied Robotics Student Interface
Package used by student to complete the assignment of the course. The objective of this package is to recognize the position of several object in an arena and subsequently plan a path for a robot to take. In this Readme the general gist of each phase is described, for a more in dept description of the api(s) provided, refer to  `src/Readme.md`.

## Computer vision
### From a raw image to an ideal image
The first part of the pipeline is fed a raw image from a camera that is suspended above an arena. The first objective is to translate the image into an "ideal" representation of the arena (id_im). The result should be a prospective fixed translation of the field, where all objects are scaled to the correct relative size and the artifacts like lens curvature are removed. 
### Object recognition
Once this preliminary work is done, the new objective is to see where the robot and the obstacles and victims are. To do this a mixture of various techniques are used. The common ground among all the following is that the color of each group of objects described is fixed, this is the main way that gets used to discriminate and find a rough estimation of where they are
#### Robot
The robot is marked above with a blue triangle that points in the direction where the robot is facing. Given this fact first all blue objects are isolated, after this operation the bigger one is taken as the robot. This greedy operation heavily relies on a half decent calibration of the color recognition parameters. Once this is done the contour of the object approximated to a triangle and, even if this operation does not give a perfect estimation of the size of the object, it is good enough having a nice property. The operation can not underestimate the size of the object, and that is the only error that could also make the mission fail. 
#### Obstacles
The obstacles are red and of various shapes.  They are isolated using their color and filtered by size to remove any noise still present in the image. Once this is done their contour is approximated.
#### Victims and the Gate
This two targets are grouped together because both are green. 
Once a preliminary work of filtering is done, as usual by size, the contour with the lowest number of edges is selected as the gate, the rationale behind that is that the circles, even if roughly approximated, are basically obliged to have more edges than a rectangle. This is true because the same factor is used for both approximations, so even if the parameter "guts" the circles, it is going to have even a larger impact on the square. Once the square is identified, it is approximated to a rectangle.
The victims are slightly more complicated to detect because this operation requires two steps. The first is carried out as above, filtering by size and approximating to a circle. The additional problem is that the victims all have a tag on them, identifying them with an integer number. To detect the exact value of this tag the victim is isolated and sent to a template matching procedure that compares the input against a set of templates.

## Planning
The planning algorithm is sub-divided into two parts: Graphing the plane and finding the best path through it.

### Plane graphing
Given a set of areas of interest, a graph is constructed linking together each visitable area. This graph is constructed with a subdivision function and a linking function that create a tiled space of explorable states. The algorithm in this implementation is not very scalable, but the concepts at hands are.

### Path exploration
The exploration of the graph is done through a priority queue, sorted according to a priority function. The objective is to get to the end of the path having the shortest travel time possible. Given that the travel time is influenced not only by the distance, but also from the victims collected, this methodology (+ node marking) allows to carry out both tasks contemporarely, with an efficient hysteresis function to avoid dept first research.

## Installation
The project comes preconfigured with a `CMakeList.txt`, to have a sample of the computer vision there are three possible routes. 
#### Python 3 script
In the src directory a `cv_main.py` script is provided, this script requires the templates and an input image in the `cwd` in `.jpg` format. To execute it install the opencv library and simply type `python3 cv_main.py` in the terminal. This script is just an example and as such should be seen, but it gives a rough estimation of the final product. This script was written from `python 3.9`.
#### C++ executable version
A compiled version of the computer vision api developed with an example is provided along this project, to produce it:

 - Go in the project directory
 - Create a `build` directory and get into it
 - Execute `cmake .. && make`

Now a version of the `see` executable should be in the directory. This executable requires the templates to be in `/tmp/templates` and the `img.jpg` of the arena in the `cwd`. 
#### Using the ROS pipeline
All the blocks described above are built into the `libstudent.so`, refer to the ROS api and the repository from whom this process is forked from for additional info.

