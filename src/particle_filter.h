/**
 * particle_filter.h
 * 2D particle filter class.
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 * 
 * Modified on Nov 2019
 * by HamidReza Mirkhani
 */

#ifndef PARTICLE_FILTER_H_
#define PARTICLE_FILTER_H_

#include <string>
#include <vector>
#include "helper_functions.h"

struct Particle
{
public:
  int id;
  double x;
  double y;
  double theta;
  double weight;
  std::vector<int> associations; // The landmark ids that goes along with each listed association
  std::vector<double> sense_x;
  std::vector<double> sense_y;
};

class ParticleFilter
{
private:
  // Number of particles to draw
  int particles_count_;

  // Flag, if filter is initialized
  bool is_initialized_;

  // Vector of weights of all particles
  std::vector<double> weights_;

  int update_step_counter_ ;

public:
  // Set of current particles
  std::vector<Particle> particles_;
  Particle best_particle_;

  // Constructor
  // @param particles_count_ Number of particles
  ParticleFilter() : particles_count_(0), is_initialized_(false)
  {

  }

  // Destructor
  ~ParticleFilter()
  {

  }

  /**
   * init Initializes particle filter by initializing particles to Gaussian
   *   distribution around first position and all the weights to 1.
   * @param x Initial x position [m] (simulated estimate from GPS)
   * @param y Initial y position [m]
   * @param theta Initial orientation [rad]
   * @param std[] Array of dimension 3 [standard deviation of x [m], 
   *   standard deviation of y [m], standard deviation of yaw [rad]]
   */
  void init(const double &x, const double &y, const double &theta, const double std[]);

  /**
   * prediction Predicts the state for the next time step
   *   using the process model.
   * @param delta_t Time between time step t and t+1 in measurements [s]
   * @param std_pos[] Array of dimension 3 [standard deviation of x [m], 
   *   standard deviation of y [m], standard deviation of yaw [rad]]
   * @param velocity Velocity of car from t to t+1 [m/s]
   * @param yaw_rate Yaw rate of car from t to t+1 [rad/s]
   */
  void prediction(double delta_t, double std_devs[], double velocity, double yaw_rate);

  /**
   * dataAssociation Finds which observations correspond to which landmarks 
   *   (likely by using a nearest-neighbors data association).
   * @param predicted_observations Vector of predicted landmark observations
   * predicted[i]: the predicted measurement for the map landmark corresponding to the ith measurement
   * @param landmark_observations Vector of landmark observations
   */
  void dataAssociation(std::vector<LandmarkObservation> &predicted_observations, const std::vector<LandmarkObservation> &landmark_observations);
  
  // void dataAssociation(std::vector<LandmarkObs> &predicted, const std::vector<LandmarkObs> &observations);

  /**
   * updateWeights Updates the weights for each particle based on the likelihood
   *   of the observed measurements. 
   * @param sensor_range Range [m] of sensor
   * @param std_landmark[] Array of dimension 2
   *   [Landmark measurement uncertainty [x [m], y [m]]]
   * @param observations Vector of landmark observations
   * @param map Map class containing map landmarks
   */
  void updateWeights(double sensor_range, double landmark_devs[], const std::vector<LandmarkObservation> &observations, const Map &map);

  /**
   * resample Resamples from the updated set of particles to form
   *   the new set of particles.
   */
  void resample();

  /**
   * Set a particles list of associations, along with the associations'
   *   calculated world x,y coordinates
   * This can be a very useful debugging tool to make sure transformations 
   *   are correct and assocations correctly connected
   */
  void setAssociations(Particle &particle, const std::vector<int> &associations,
                       const std::vector<double> &sense_x,
                       const std::vector<double> &sense_y);

  /**
   * initialized Returns whether particle filter is initialized yet or not.
   */
  const bool initialized() const
  {
    return is_initialized_;
  }

  /**
   * Used for obtaining debugging information related to particles.
   */
  std::string getAssociations(Particle best);
  std::string getSenseCoord(Particle best, std::string coord);

};

#endif // PARTICLE_FILTER_H_