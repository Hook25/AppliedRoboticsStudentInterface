# Source
The source is separated in the computer vision and the planning part. Additionally in this directory are included most tools used for the development and examples.
## The map watcher
The map watcher namespace contains all the functions used to identify the objects in the arena. This namespace assumes that the preprocessing has already made the image ideal, in the sense that it is free from prospective and distortion artifacts. All the images placed in input **must** be in the `HSV` format.
### Usage
Although an example usage is provided along with the library, a more "reference like" text is here proposed as well.

 - **Configuration**: This is a preliminary step, to use the library, the parameters to discriminate the colors, defined on top of the source file, **should** be fine tuned to the application at hand. The values there are *ok* defaults, for an ideal scenario with perfect studio lighting, *but your milage might vary*.
 - **Initialization**: First thing first the namespace must be initialized to point to the correct directory for the templates used in the template matching, this step is by no mean necessary, but if it does not get done the templates are searched in `cwd/templates`, that might not be the correct path.
 - **Finding objects**: Each object group has its own `findGroupName` method in the API, for example the robot has a `findRobot` method. Please note that all of them do not modify the input image, so you can refrain from backing it up before passing it to any of them. The returns are usually carried out by reference using the last arguments of each functions. Please note that all of the coordinates returned by the methods are relative to the reference system that is the input image.

### In dept description of each method
#### findVictimsGate(const Mat &img, vector\<Points\> &gate, vector\<victim\> &victims)
The following is the pseudo code of the operations that a method does and the rationale behind them, many might coincide, so they are omitted in the following descriptions for the sake of brevity. It was decided to write them as pseudo code to avoid all the c++ verbosity that does not add to the clarity of the algorithm described
```javascript
function getGate(contours){
  //get the contours with the lower edge count
  gate_contour = getMinEdges(contours);
  //incapsulate the gate_contour in an actual rectangle
  return makeBox(gate_contour);
}

function indentifyVictim(img, victim){
  //firstly mask the image so that only victim is visible
  img.mask(victim);
  //then cut out the victim, to make the template
  //  matching faster
  tmp = img.cutRect(victim);
  //then match the cutted out image against the templates
  return templateMatching(tmp);
}

function getVictims(img, victims){
  for(victim of victims){
    //make the victim into a proper circle with
    //  a centre and a radius
    circle = makeCircle(victim);
    id = identifyVictim(img, circle);
  }
}

function findVictims(...){ 
  //firstly search for all the green contours
  contoursByColor = findContByColor(img, GREEN);
  //remove all the contours who's area is lower then
  //  factor * max(area(contoursByColor))
  contoursFiltered = filterBySize(contoursByColor, factor);
  gate = getGate(contoursFiltered);
  contoursFiltered.remove(gate);
  victims = getVictims(contoursFiltered);
}
```
#### findObstacles(const Mat &img, vector\<vector<Points\>\> &obstacles)
```javascript
function findObstacles(...){
  obstacles = findContByColor(img, RED);
  //clean the edges from noise
  obstacles = approximate(obstacles);
  //remove random noise from the actual obstacles
  return filterBySize(obstacles, factor);
}
```
#### findRobot(const cv::Mat &img, robot &r)
```javascript
function findRobot(...){
  bots = findContByColor(img, BLUE);
  //the biggest blue spot should be the robot
  bot_cont = biggest(bots);
  //make a triangle around the points
  bot = makeTriangle(bot_cont);
  //the tail point is the further from the centre
  centre_point = getCentre(bot);
  tail_point = furtherCentre(bot, centre_point);
  //the angle is the atan2 of the 2 points (like
  //  a line going between them
  angle = atan2(centre_point, tail_point);
  return ...;
}
```

