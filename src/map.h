/**
 * map.h
 *
 * Created on: Dec 12, 2016
 * Author: mufferm
 */

#ifndef MAP_H_
#define MAP_H_

#include <vector>

class Map
{
public:
  struct LandmarkData
  {
    int id;  // Landmark ID
    float x; // Landmark x-position in the map (global coordinates)
    float y; // Landmark y-position in the map (global coordinates)
  };

  std::vector<LandmarkData> landmark_list; // List of landmarks in the map
};

#endif // MAP_H_
