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
emc-sim
```

#### (optional) Terminal 4: visualize the sensor data  ####
```bash
emc-viz
```

#### Terminal 1: run the program ####  
```bash
../bin/main
```

#### (optional) Terminal 5: Manually control the robot  ####
```bash
pico-teleop
```

#### Troubleshooting  ####
If you encounter an error "Could not find a configuation file for package "OpenCV" that is compatible with the requested version ...", please open the *CMakeLists.txt* file, and check whether the version # in **find_package(OpenCV # REQUIRED)** coincides with the OpenCV version installed on your machine.
