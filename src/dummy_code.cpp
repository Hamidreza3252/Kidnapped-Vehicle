/**
 * print_samples.cpp
 * 
 * Print out to the terminal 3 samples from a normal distribution with
 * mean equal to the GPS position and IMU heading measurements and
 * standard deviation of 2 m for the x and y position and 0.05 radians
 * for the heading of the car. 
 *
 * Author: Tiffany Huang
 */

#include <iostream>
#include <random> // Need this for sampling from distributions

using std::normal_distribution;

/**
 * Prints samples of x, y and theta from a normal distribution
 * @param gps_x   GPS provided x position
 * @param gps_y   GPS provided y position
 * @param theta   GPS provided yaw
 */
void printSamples(double gps_x, double gps_y, double theta);

int main()
{
  // Set GPS provided state of the car.

  double gpsX = 4983;
  double gpsY = 5029;
  double theta = 1.201;
  // Sample from the GPS provided position.

  printSamples(gpsX, gpsY, theta);

  return 0;
}

void printSamples(double gpsX, double gpsY, double theta)
{
  std::default_random_engine gen;
  // define and set standard deviations for x, y, and theta
  double stdX = 2.0;
  double stdY = 2.0;
  double stdTheta = 0.05; // Standard deviations for x, y, and theta
  // Create normal distributions for x, y, and theta

  normal_distribution<double> distX(gpsX, stdX);
  normal_distribution<double> distY(gpsY, stdX);
  normal_distribution<double> distTheta(theta, stdX);

  for (int i = 0; i < 3; ++i)
  {
    double sampleX, sampleY, sampleTheta;
    // Sample from these normal distributions, where "gen" is the random engine initialized earlier 
    sampleX = distX(gen);
    sampleY = distY(gen);
    sampleTheta = distTheta(gen);
    // Print your samples to the terminal.
    std::cout << "Sample " << i + 1 << " " << sampleX << " " << sampleY << " " << sampleTheta << std::endl;
  }
  return;
}
