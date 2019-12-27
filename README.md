## 1. Overview
This repository contains the codes for the Localization course, *kidnapped vehicle*. 
 
 
## 2. Project Introduction
The aim of this project is to localize the vehicle or robot using the provided map of the location, a (noisy) GPS estimate of the initial location of the vehicle, and lots of (noisy) sensor and control data. To do so, a 2-dimensional particle filter was developed in C++. The particle filter algorithm will be given a map and some initial localization information (analogous to what a GPS would provide). At each time step of the simulation, the filter will also get observation and control data. The localization error is measured against the ground-truth values in the simulation. 
 
The passing criteria for the success of the project are: 
 
1. **Accuracy**: The particle filter should localize vehicle position and yaw to within the values specified in the parameters `max_translation_error` and `max_yaw_error` in `src/main.cpp`. 

2. **Performance**: The particle filter should complete execution within the time of 100 seconds. 

## 3. Running the Code

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and install uWebSocketIO for either Linux or Mac systems. For windows you can use either Docker, VMware, or even Windows 10 Bash on Ubuntu to install uWebSocketIO. 

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./particle_filter

Alternatively some scripts have been included to streamline this process, these can be leveraged by executing the following in the top directory of the project:

1. ./clean.sh
2. ./build.sh
3. ./run.sh

Here is the main protocol that main.cpp uses for uWebSocketIO in communicating with the simulator.

**INPUT:** values provided by the simulator to the c++ program 

- sense noisy position data from the simulator: 
  ["sense_x"] 
  ["sense_y"] 
  ["sense_theta"] 
   
- get the previous velocity and yaw rate to predict the particle's transitioned state:
  ["previous_velocity"]
  ["previous_yawrate"]
 
- receive noisy observation data from the simulator, in a respective list of x/y values: 
  ["sense_observations_x"]
  ["sense_observations_y"]
 

**OUTPUT:** values provided by the c++ program to the simulator

- best particle values used for calculating the error evaluation
  ["best_particle_x"]
  ["best_particle_y"]
  ["best_particle_theta"]
 
- Optional message data used for debugging particle's sensing and associations

- for respective (x,y) sensed positions ID label
  ["best_particle_associations"]
 
- for respective (x,y) sensed positions
  ["best_particle_sense_x"] <= list of sensed x positions
  ["best_particle_sense_y"] <= list of sensed y positions

## 4. Implementation of the Particle Filter 

### 4-1. The directory structure of this repository is as follows: 

```
root
|   build.sh
|   clean.sh
|   CMakeLists.txt
|   README.md
|   run.sh
|
|___data
|   |   
|   |   map_data.txt
|   
|   
|___src
    |   helper_functions.h
    |   main.cpp
    |   map.h
    |   particle_filter.cpp
    |   particle_filter.h
```

## 4-2. The Key Funcions 
The key functionalities are included in `ParticleFilter` class, defined in `particle_filter.cpp` with the following member fucntions (please refer to the source code and the comments in the code for more details): 

- `ParticleFilter::init`: This function is defined to: 
  - 1. Set the number of particles. 
  - 2. Initialize all particles to the first position (based on estimates of x, y, theta and their uncertainties from GPS) and all weights to 1.0.  
  
  The number of particles is set to be 800 at least. Based on my exercises, a particle count between 800 and 1200 shoud be a good trade off between accuracy and performance. 

- `ParticleFilter::prediction`: This function predicts the state for the next time step using the process model. There are two ways to add random Gaussian noise to the particles' predicted states. In the first approach, we can define distributions with mean zero (0.0) and the specified standard deviations for each state. Then we will add this Gaussian noise to the previous particle states and then we will procees with the rest of the calculation (prediction process). In the second approach, we can first update the states using the previous states and then add random Gaussian noise to it, by setting the mean to the updated predicted state. For particle filter, it should not matter much which approach to take, but it seems that the first approach makes more sense from process or motion model point of view. Therefore, I chose the first approach to add random Gaussian noise to the particles' states. 

- `ParticleFilter::dataAssociation`: This method, which is called in `updateWeights` method, finds the predicted measurement that is closest to each observed measurement and assign the observed measurement to this particular landmark. This is done by setting the id of each predicted observations. 

- `ParticleFilter::updateWeights`: This method updates the weights of each particle using a mult-variate Gaussian distribution as follows: 
  -  Selecting the most possible map landmark observations: This step selets the landmark observations that need to be considered based on the the location of the best particle estimate from the previous state and the provided sensor range. 
  - Finding the predicted measurement that is closest to each observed measurement using `dataAssociation` method explained above. It should be noted that the observations are given in the vehicles coordinate system; However, the particles are located according to the MAP'S coordinate system. Therefore, I transformed between the two systems. 
  - Updating the weight of each particle through combining the probabilities of each observation for each particle. Based on the scales of the measurements and the particles' locations, I had to divide the components that goes in the `exp` function by `1.0e5` to avoid overflowing the numbers. 
  
- `ParticleFilter::resample`: This method is used to resample particles with replacement with probability proportional to their weight. 


## 5. Inputs to the Particle Filter
You can find the inputs to the particle filter in the `data` directory.

### 5-1. The Map*
`map_data.txt` includes the position of landmarks (in meters) on an arbitrary Cartesian coordinate system. Each row has three columns
1. x position
2. y position
3. landmark id

### 5-2. All other data the simulator provides, such as observations and controls.

> * Map data provided by 3D Mapping Solutions GmbH.
