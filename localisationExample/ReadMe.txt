Parameters
	- World						-	LandMarks 			(locations of LandMarks)			
								-  	WorldSize 
	- Robot					
								- 	InitialPosition
								-	NoiseParameters
									- _std_forward	 	(stand. dev. of noise on forward movement)
									- _std_turn			(stand. dev. of noise on angular movement)
									- _std_meas_dist	(stand. dev. of noise on measurement dist.)
									- _std_meas_angl	(stand. dev. of noise on measurement angle)

	- Control
								- 	DesiredForwardVelocity			(approx. distance driven every timestep)
								- 	DesiredAngleVelocity			(approx. angle turned every timestep)

	- ParticleFilter
								-	Particles 	(number of Particles initialised at t = 0)

								-	"PropagationParameters"
									- motion_forward_std	(stand. dev. of noise on forward movement in propagation of particles)
									- motion_turn_std		(stand. dev. of noise on turn movement in propagation of particles)
									- meas_dist_std			(stand. dev. of noise on distance measurement in likelihood computation of particles)
									- meas_angl_std			(stand. dev. of noise on angular measurement in likelihood computation of particles)

								-	ParticleFilterFlavor	(Determines wheter number of particles is constant)
									("Conventional" or "Adaptive")

								- 	ResamplingScheme		(Determines when to resample. When deemed necessary or every timestep)
									("Always" or "effectiveParticleThreshold" )
									---> When "Adaptive" is selected this parameter is (currently) ignored

								- 	ResamplingThreshold		(Parameter used in effectiveParticleThreshold ResamplingScheme)

								-	ResamplingAlgorithm		(Determines how the resampled particles are chosen)
									("Multinomial" or "Stratified")
									---> When "Adaptive" is selected this parameter is ignored
