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
#include <list>

#include <iterator>

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

  particles_count_ = 600;

  particles_.reserve(particles_count_);
  weights_.reserve(particles_count_);

  for (int i = 0; i < particles_count_; ++i)
  {
    // particle = &particles_[i] = Particle();
    particle = new Particle();

    particle->id = i;
    particle->x = distribution_x(random_generator);
    particle->y = distribution_y(random_generator);
    particle->theta = distribution_theta(random_generator);

    weights_.push_back(particle->weight = 1.0);
    particles_.push_back(*particle);

    delete particle;

    // particles_[i] = *particle;
  }

  is_initialized_ = true;
}

void ParticleFilter::prediction(double delta_t, double std_devs[], double velocity, double yaw_rate)
{
  /**
   * Adding measurements to each particle and add random Gaussian noise.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */

  // Particle *particle;
  double velocity_ratio;
  double theta_increment;

  std::default_random_engine random_generator;
  velocity_ratio = velocity / yaw_rate;

  for (auto &particle : particles_)
  {
    particle.weight = 1.0;

    std::normal_distribution<double> distribution_x(particle.x, std_devs[0]);
    std::normal_distribution<double> distribution_y(particle.y, std_devs[1]);
    std::normal_distribution<double> distribution_theta(particle.theta, std_devs[2]);

    particle.x = distribution_x(random_generator);
    particle.y = distribution_y(random_generator);
    particle.theta = distribution_theta(random_generator);

    theta_increment = particle.theta + yaw_rate * delta_t;

    particle.x += velocity_ratio * (std::sin(theta_increment) - std::sin(particle.theta));
    particle.y += velocity_ratio * (std::cos(particle.theta) - std::cos(theta_increment));
    particle.theta = theta_increment;

    /*
    std::normal_distribution<double> distribution_x(particle.x, std_devs[0]);
    std::normal_distribution<double> distribution_y(particle.y, std_devs[1]);
    std::normal_distribution<double> distribution_theta(particle.theta, std_devs[2]);

    particle.x = distribution_x(random_generator);
    particle.y = distribution_y(random_generator);
    particle.theta = distribution_theta(random_generator);
    */
  }
}

// void ParticleFilter::dataAssociation(vector<LandmarkObs> &predicted, const vector<LandmarkObs> &observations)
void ParticleFilter::dataAssociation(std::vector<LandmarkObservation> &predicted_observations, const std::vector<LandmarkObservation> &landmark_observations)
{
  /**
   * This method finds the predicted measurement that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark. This is done by setting the id of each predicted observations. 
   * NOTE: this method will NOT be called by the grading code. But you will 
   *   probably find it useful to implement this method and use it as a helper 
   *   during the updateWeights phase.
   */

  double obtained_distance;
  double shortest_distance;
  int landmark_id = -1;
  // int id_counter = 0;
  // std::list<int> selected_ids;
  std::vector<int> selected_ids;
  // LandmarkObservation *selected_landmark;
  // LandmarkObservation *predicted_observation;
  // const LandmarkObservation *landmark_observation;

  selected_ids.reserve(landmark_observations.size());

  for (auto &predicted_observation : predicted_observations)
  {
    // predicted_observation = &predicted_observations[i];

    if (predicted_observation.id != -1)
    {
      continue;
    }

    shortest_distance = std::numeric_limits<double>::max();
    // int index = 0;

    for (const auto &landmark_observation : landmark_observations)
    {
      // landmark_observation = &landmark_observations[j];

      bool found = (std::find(selected_ids.begin(), selected_ids.end(), landmark_observation.id) != selected_ids.end());

      if(found)
      {
        continue;
      }

      obtained_distance = dist(predicted_observation.x, predicted_observation.y, 
          landmark_observation.x, landmark_observation.y);
      
      if (obtained_distance < shortest_distance)
      {
        shortest_distance = obtained_distance;
        landmark_id = landmark_observation.id;
        // landmark_id = index++;
      }
    }

    selected_ids.push_back(predicted_observation.id = landmark_id);

    // std::cout << "hamid: " << landmark_id << std::endl;

    // predicted_observation.id = selected_ids[id_counter++] = landmark_id;
    // selected_ids.push_back(landmark_id);
  }
}

void ParticleFilter::updateWeights(double sensor_range, double landmark_devs[],
                                   const vector<LandmarkObservation> &observations,
                                   const Map &map)
{
  /**
   * const Map &map_landmarks
   * 
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


  vector<LandmarkObservation> predicted_observations;
  // LandmarkObservation *predicted_observation;
  vector<LandmarkObservation> landmark_observations;
  // LandmarkObservation *landmark_observation;
  // Particle *particle;
  std::vector<int> associations;
  std::vector<int> landmark_indexes;
  std::vector<double> x_senses;
  std::vector<double> y_senses;

  double sigma_inv_11 = 1.0 / landmark_devs[0];
  double sigma_inv_22 = 1.0 / landmark_devs[1];
  double denom = std::sqrt(2*M_PI*(landmark_devs[0] * landmark_devs[1]));
  double x_mu_x;
  double y_mu_y;

  predicted_observations.reserve(observations.size());
  landmark_indexes.reserve(map.landmark_list.size());
  // predicted_observations = observations;
  landmark_observations.reserve(map.landmark_list.size());
  // associations.reserve(observations.size());
  // x_senses.reserve(observations.size());
  // y_senses.reserve(observations.size());

  int index = 0;

  // std::vector<std::pair<LandmarkObservation, Map::LandmarkData>> zipped_data = {landmark_observations, map.landmark_list};
  // std::pair<aaa, Map::LandmarkData> zipped_data(landmark_observations, map.landmark_list);

  for (const auto &map_landmark : map.landmark_list)
  {
    landmark_observations.push_back(LandmarkObservation());

    landmark_indexes.push_back(landmark_observations[index].id = map_landmark.id);
    landmark_observations[index].x = map_landmark.x;
    landmark_observations[index].y = map_landmark.y;

    ++index;
  }

  index = 0;

  LandmarkObservation predicted_observation = LandmarkObservation();

  for (auto &particle : particles_)
  {
    // particle = &particles_[i];

    predicted_observations.clear();
    predicted_observations.reserve(observations.size());

    associations.clear();
    associations.reserve(observations.size());

    x_senses.clear();
    x_senses.reserve(observations.size());

    y_senses.clear();
    y_senses.reserve(observations.size());

    // int obs_index = 0;
    for (auto &observation : observations)
    {
      // predicted_observation = &predicted_observations[j];
      predicted_observation.x = transform_x_l2g(observation.x, observation.y, particle.x, particle.theta);
      predicted_observation.y = transform_y_l2g(observation.x, observation.y, particle.y, particle.theta);

      predicted_observations.push_back(predicted_observation);
    }

    dataAssociation(predicted_observations, landmark_observations);

    for (unsigned long int k = 0; k < predicted_observations.size(); ++k)
    {
      long unsigned int landmark_id = predicted_observations[k].id;

      // ptrdiff_t pos_index = std::find(landmark_observations.begin(), landmark_observations.end(), landmark_id) - landmark_observations.begin();
      long unsigned int pos_index = std::find(landmark_indexes.begin(), landmark_indexes.end(), landmark_id) - landmark_indexes.begin();

      if (pos_index >= landmark_indexes.size())
      {
        std::cout << "Warning: pos_index >= landmark_indexes.size()";
      }

      associations.push_back(landmark_id);
      x_senses.push_back(predicted_observations[k].x);
      y_senses.push_back(predicted_observations[k].y);

      x_mu_x = landmark_observations[pos_index].x - predicted_observations[k].x;
      y_mu_y = landmark_observations[pos_index].y - predicted_observations[k].y;

      double exp_temp = ((x_mu_x*sigma_inv_11) * x_mu_x + (y_mu_y*sigma_inv_22) * y_mu_y) / 1.0e5;

      particle.weight *= std::exp(-0.5 * exp_temp) / denom;
    }

    weights_[index++] = particle.weight;

    setAssociations(particle, associations, x_senses, y_senses);
  }
}

void ParticleFilter::resample()
{
  /**
   * TODO: Resample particles with replacement with probability proportional to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */

  std::random_device rd;
  std::mt19937 generator(rd());
  std::discrete_distribution<> dd(weights_.begin(), weights_.end());
  std::vector<Particle> temp_particles;

  temp_particles.reserve(particles_count_);

  for (int i = 0; i < particles_count_; ++i)
  {
    int random_int = dd(generator);
    // std::cout << "rnd: " << random_int << std::endl;
    temp_particles.push_back(particles_[random_int]);
  }

  // particles_.swap(temp_particles);

  particles_ = temp_particles;
  
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