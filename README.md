# emc-examples

## Description
### Full example
This example shows suggestions on how to set up your project and structure your code. It is still far from perfect, but it can be a nice starting point. At its core, in the main file of this project, there is a state machine. Different other functionalities are separated into different files and classes. The current behavior is as follows: the robot starts driving forward using its drive forward skill. In the meantime, it continuously monitors its distance to the wall, using its detection skill. When a wall is detected, the robot cannot drive forward anymore and needs to turn. The state thus switches, and another set of skills is called: drive backward and rotate.

### Feature extraction example
This example implements a basic split and merge algorithm to extract line and corner features. It starts with all data points, and calculates the perpendicular distacne between each data point and a line through the first and last data point. If the largest calculated distance exceeds some threshold, the data segment is splitted at this point. Now, the same is recursively repeated on the two new segments, until no segment is splittable anymore. At this point the split indices can be used to contruct the lines and corners. This algorithm, including visualization, is implemented in the FeatureExtraction class provided in featureExtraction.cpp, and can be readily used in your main file. To run it standalone, the provided main.cpp file can be used. 

The input of this example is the LRF data, in the EMC environment format. The output consists of the detected lines and corners. Some important parameters that influence the output a lot are stored in the config.h file: the threshold at which a line is split (SEGEMENT_DIST_THRESHOLD) and the number of points required for one segment (SEGMENT_MIN_NR_POINTS). The latter makes it more robust against small holes in a wall or when there are outliers in the measurement data.

Note that it is not mandatory to use this code, it is even encouraged to use any algorithm that works best for you. If you _do_ decide to use this code, it is still very much advised to at the very least better tweak the parameters, but ideally also improve other parts of the code.

### Localization example
This example implements an often used, and easily implementable, algorithm for localisation: the particle filter. Particle filters rely on sampling to approximate the distribution that describes the probabilities of the current robot location given the obtained measurments and the (provided) map. The example implementation (among other) assumes the following:

* The particle filter receives **odometry information** from the robot, the received odometry information is not perfect and **contains noise**
* The particle filter receives **enviroment measurements** from the robot, the enviroment measurements are not perfect and **contain noise**


More importantly,

* The enviroment measurements contain the distance and angle to **features**[^1] on the map,
* Every measurement **does**[^2] result from a feature on the map 
* The particle filter assumes **known correspondence**[^3] between the measurement i and feature j,
* The enviroment measurements have **infinite range**, and can not be **occluded**[^4].

[^1]: With features (or landmarks) we mean recognizable objects in the physical world. For instance, corners, door frames, or permanent objects.
[^2]: In the limited simulation enviroment in this example, spurious measurements are not considered. In the real world the robot might detect features that are not really there, either because of (dynamic) obstacles or through a spurious detection in your feature detection components.
[^3]: With known correspondence we mean that we know the mapping between a feature measurement and the corresponding feature. For instance, we could imagine that the robot detecting **a** corner, doesn't tell us with certainty **which** corner it detected.
[^4]: With infinite range we mean that the robot is able to all the features on the map, no matter if it is close or far away.
 By occlusion we mean that a feature will be detected, even if there is another featured that blocks the view. The combination of these two assumptions means that the robot will **always** measure the distance and orientation to **all** features.

Solution strategies exsist for relaxing these last three assumptions, however, they will require a bit of creativity/research from your part. You **are** allowed to use the core part of the code in your project, however you're also encouraged to come up with your own solutions. 

Useful pointers or starting points to understanding the localisation problem can be:
* Probabilistic Robotics by Sebastian Thrun, Wolfram Burgard and Dieter Fox,[^6]
* [Course: Robot Mapping - WS 2013/14 by Cyrill Stachniss](http://ais.informatik.uni-freiburg.de/teaching/ws13/mapping/)[^7]


[^6]: Especially, CH 2-9 can be useful, however, it is likely not necessary to read all of it.
[^7]: Especially, lectures 1 (Introduction to Robot Mapping) and 11(Short Introduction to the Particle Filter and PF Localization). However, you might need some prior knowledge for the latter.


## Using the Examples
### Full example and Feature extraction example
These examples require the MRC environment to be installed on your machine. If you haven't already, download the install script:
```bash
wget https://raw.githubusercontent.com/tue-robotics/emc-env/master/install.bash
```
and run it:
```bash
source install.bash
```

For compiling and using these examples, the reader is referred to the README files inside the project folders.

### Localization example

The Localisation example (currently) uses matplotlibcpp and requires python3 and matplotlib to function.

To install the preqrequisites:

```bash
sudo apt-get install python3-matplotlib python3-numpy
```

## Credit
### Full example
Taken from: http://cstwiki.wtb.tue.nl/index.php?title=MRC/FullExample

### Localization example

The localization example is based on the python-code in:
[https://github.com/jelfring/particle-filter-tutorial](https://github.com/jelfring/particle-filter-tutorial)

Users of the code who want to better understand the background, uses, and configuration of the particle filters implemented in this repository are suggested to read the corresponding paper:

    Elfring J, Torta E, van de Molengraft R. Particle Filters: A Hands-On Tutorial. Sensors. 2021; 21(2):438.

In order to create a visualization the localization example makes use of:
[https://github.com/lava/matplotlib-cpp](https://github.com/lava/matplotlib-cpp)

The library used for parsing the .json file is taken from:
[https://github.com/nlohmann/json](https://github.com/nlohmann/json)
