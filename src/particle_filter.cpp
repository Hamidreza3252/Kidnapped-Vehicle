/**
 * particle_filter.cpp 
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 * 
 * Modified on Nov 2019
 * by HamidReza Mirkhani
*/

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "helper_functions.h"

using std::string;
using std::vector;

void ParticleFilter::init(const double &x, const double &y, const double &theta, const double std_devs[])
{
  /**
   * 1. Set the number of particles. 
   * 2. Initialize all particles to the first position (based on estimates of x, y, theta and their uncertainties from GPS) and all weights to 1. 
   */

  // particles_count_ = *(&std_devs + 1) - std_devs;  // in case an input array is given

  Particle *particle;

  std::default_random_engine random_generator;

  std::normal_distribution<double> distribution_x(x, std_devs[0]);
  std::normal_distribution<double> distribution_y(y, std_devs[1]);
  std::normal_distribution<double> distribution_theta(theta, std_devs[2]);

  particles_count_ = 1000;

  particles_.reserve(particles_count_);

  for (int i = 0; i < particles_count_; ++i)
  {
    // particle = &particles_[i] = Particle();
    particles_[i] = *(particle = new Particle());

    particle->id = i;
    particle->x = distribution_x(random_generator);
    particle->y = distribution_y(random_generator);
    particle->theta = distribution_theta(random_generator);
    particle->weight = 1.0;
  }
}

void ParticleFilter::prediction(double delta_t, double std_devs[], double velocity, double yaw_rate)
{
  /**
   * Adding measurements to each particle and add random Gaussian noise.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */

  Particle *particle;
  double velocity_ratio;
  double theta_increment;

  std::default_random_engine random_generator;
  velocity_ratio = velocity / yaw_rate;

  for (int i = 0; i < particles_count_; ++i)
  {
    particle = &particles_[i];
    theta_increment = particle->theta + yaw_rate * delta_t;

    particle->x += velocity_ratio * (std::sin(theta_increment) - std::sin(particle->theta));
    particle->y += velocity_ratio * (std::cos(particle->theta) - std::cos(theta_increment));
    particle->theta = theta_increment;

    std::normal_distribution<double> distribution_x(particle->x, std_devs[0]);
    std::normal_distribution<double> distribution_y(particle->y, std_devs[1]);
    std::normal_distribution<double> distribution_theta(particle->theta, std_devs[2]);

    particle->x = distribution_x(random_generator);
    particle->y = distribution_y(random_generator);
    particle->theta = distribution_theta(random_generator);
  }
}

// void ParticleFilter::dataAssociation(vector<LandmarkObs> &predicted, const vector<LandmarkObs> &observations)
void ParticleFilter::dataAssociation(vector<LandmarkObservation> &predicted_observations, const vector<LandmarkObservation> &landmark_observations)
{
  /**
   * TODO: Find the predicted measurement that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will 
   *   probably find it useful to implement this method and use it as a helper 
   *   during the updateWeights phase.
   * 
   * set the id of each predicted observations
   */

  double obtained_distance;
  double shortest_distance;
  int landmark_id = -1;
  // LandmarkObservation *selected_landmark;

  for (int i = 0; i < predicted_observations.size(); ++i)
  {
    if (predicted_observations[i].id != -1)
    {
      continue;
    }

    shortest_distance = std::numeric_limits<double>::max();

    for (int j = 0; j < landmark_observations.size(); ++j)
    {
      obtained_distance = dist(predicted_observations[i].x, predicted_observations[i].y, 
          landmark_observations[j].x, landmark_observations[j].y);
      
      if (obtained_distance < shortest_distance)
      {
        shortest_distance = obtained_distance;
        landmark_id = landmark_observations[j].id;
      }
    }

    predicted_observations[i].id = landmark_id;
  }

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
                                   const vector<LandmarkObservation> &observations,
                                   const Map &map_landmarks)
{
  /**
   * TODO: Update the weights of each particle using a mult-variate Gaussian 
   *   distribution. You can read more about this distribution here: 
   *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
   * NOTE: The observations are given in the VEHICLE'S coordinate system. 
   *   Your particles are located according to the MAP'S coordinate system. 
   *   You will need to transform between the two systems. Keep in mind that
   *   this transformation requires both rotation AND translation (but no scaling).
   *   The following is a good resource for the theory:
   *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
   *   and the following is a good resource for the actual equation to implement
   *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
   */
}

void ParticleFilter::resample()
{
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */
}

void ParticleFilter::setAssociations(Particle &particle,
                                     const vector<int> &associations,
                                     const vector<double> &sense_x,
                                     const vector<double> &sense_y)
{
  // particle: the particle to which assign each listed association,
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations = associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best)
{
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length() - 1); // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord)
{
  vector<double> v;

  if (coord == "X")
  {
    v = best.sense_x;
  }
  else
  {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length() - 1); // get rid of the trailing space
  return s;
}