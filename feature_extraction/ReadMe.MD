#### Terminal 1: build the project ####  
```bash
mkdir build
cd build
cmake ..
make
``` 

#### Terminal 2: start a roscore ####  
```bash
roscore
```

#### Terminal 3: start the simulator ####
```bash
mrc-sim
```

#### (optional) Terminal 4: visualize the sensor data  ####
```bash
mrc-viz
```

#### Terminal 1: run the program ####  
```bash
../bin/main
```

#### (optional) Terminal 5: Manually control the robot  ####
```bash
hero-teleop
```

#### Troubleshooting  ####
If you encounter an error "Could not find a configuation file for package "OpenCV" that is compatible with the requested version ...", please open the *CMakeLists.txt* file, and check whether the version # in **find_package(OpenCV # REQUIRED)** coincides with the OpenCV version installed on your machine.
You can find the OpenCV version installed by typing the following command in your terminal:
```bash
python3 -c "import cv2; print(cv2.__version__)"
```
