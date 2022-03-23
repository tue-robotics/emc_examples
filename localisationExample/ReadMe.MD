
## Building and running the Code
Make sure the prerequisites are installed
```bash
sudo apt-get update
sudo apt-get install python3-matplotlib python3-numpy
```
To build the code
```bash
mkdir build
cd build
cmake ..
make 
cd ..
```

To run the code
```bash
./build/main
```

## Localization example parameters
Name          | Value        | Value                     | Value |Explanation
------------- | -------------|---------------------------|-------| ---------------------------
World         | Landmarks    | _Array of Coordinate-arrays_|&nbsp; | Location of each landmark
&nbsp;        | WorldSize    | _Array of 2x1_              |&nbsp; |Size of the simulated world 
&nbsp;        | &nbsp;        |  &nbsp;                    |&nbsp; | &nbsp;
Robot         | InitialPosition| _Array of 3x1_            |&nbsp; | Initial location of the robot
&nbsp;        | NoiseParameters| \_std\_forward          |_scalar_| stand. dev. of noise on forward movement
&nbsp;        | &nbsp;         | \_std\_turn             |_scalar_| stand. dev. of noise on angular movement
&nbsp;        | &nbsp;         | \_std\_meas\_dist       |_scalar_| stand. dev. of noise on angular movement
&nbsp;        | &nbsp;         | \_std\_meas\_angl       |_scalar_| stand. dev. of noise on angular movement
&nbsp;        | &nbsp;        |  &nbsp;                  |&nbsp;| &nbsp;
Control       | DesiredForwardVelocity | _scalar_          |&nbsp;| approx. distance driven every timestep
&nbsp;        | DesiredAngleVelocity	 | _scalar_          |&nbsp;| approx. angle turned every timestep
&nbsp;        | &nbsp;        |  &nbsp;                  |&nbsp;| &nbsp;
ParticleFilter| Particles     |  _scalar_                  |&nbsp;| number of Particles at t = 0
&nbsp;        | PropagationParameters| motion_forward_std|_scalar_| stand. dev. on forward movement in propagation
&nbsp;        | &nbsp;               | motion_turn_std   |_scalar_| stand. dev. on turn movement in propagation
&nbsp;        | &nbsp;               | meas_dist_std	   |_scalar_| stand. dev. on distance measurement in likelihood computation
&nbsp;        | &nbsp;               | meas_angl_std		 |_scalar_| stand. dev. on angular measurement in likelihood computation
&nbsp;        | ParticleFilterFlavor | _string_          |&nbsp;|   Choose from {"Conventional", "Adaptive"} 
&nbsp;        | ResamplingScheme		 | _string_          |&nbsp;|   Choose from {"Always", "effectiveParticleThreshold"}
&nbsp;        | ResamplingAlgorithm	 | _string_          |&nbsp;|   Choose from {"Multinomial", "Stratified"}
&nbsp;        | ResamplingThreshold	 | _scalar_          |&nbsp;|   Paramater in effectiveParticleThreshold resampling scheme. Set between 0 and 1.
