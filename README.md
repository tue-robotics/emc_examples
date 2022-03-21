# emc-examples

## Description
### Full example
This example shows suggestions on how to set up your project and structure your code. It is still far from perfect, but it can be a nice starting point. At its core, in the main file of this project, there is a state machine. Different other functionalities are separated into different files and classes. The current behavior is as follows: the robot starts driving forward using its drive forward skill. In the meantime, it continuously monitors its distance to the wall, using its detection skill. When a wall is detected, the robot cannot drive forward anymore and needs to turn. The state thus switches, and another set of skills is called: drive backward and rotate.

### Feature extraction example
This example implements a basic split and merge algorithm to extract line and corner features. Note that it is not mandatory to use this code, it is even encouraged to use any algorithm that works best for you. If you _do_ decide to use this code, it is still very much advised to at the very least better tweak the parameters, but ideally also improve other parts of the code.

### Localization example


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
  
### Feature extraction example
The feature extraction example requires the MRC environment to be installed on you machine. Please follow the instructions on <TODO>.
For compiling and using this example, the reader is referred to the README file inside the project folder.

### Localization example

The Localisation example (currently) uses matplotlibcpp and requires python3 and matplotlib to function.

To install the preqrequisites:

```bash
sudo apt-get install python3-matplotlib python3-numpy
```

## Credit
### Full example
Taken from: http://cstwiki.wtb.tue.nl/index.php?title=MRC/FullExample

### Feature extraction example


### Localization example

The localization example is based on the python-code in:

[https://github.com/jelfring/particle-filter-tutorial](https://github.com/jelfring/particle-filter-tutorial)

Users of the code who want to better understand the background, uses, and configuration of the particle filters implemented in this repository are suggested to read the corresponding paper:

Elfring J, Torta E, van de Molengraft R. Particle Filters: A Hands-On Tutorial. Sensors. 2021; 21(2):438.

In order to create a visualization the localization example makes use of:

[https://github.com/lava/matplotlib-cpp](https://github.com/lava/matplotlib-cpp)
