#include <iostream>
#include <vector>
#include <utility> 
#include <obstacle_detection/LidarUtility.h>

// Constructor implementation
LidarUtility::LidarUtility(float* angles, float* distances) : scanAngles(angles), scanDistances(distances) {}

// pair used as the scan data has angle and corresponding distance. The scan data consists of all angles from 0-360 but we are only interested in certain sections.
//minAngle is the lower angle in a section and max is the higher angle. ARRAY_SIZE is fixed for rplidar s2 scan data
std::vector<std::pair<float, float>> LidarUtility::getScanDataInRange(float minAngle, float maxAngle, int ARRAY_SIZE) {
    
    std::vector<std::pair<float, float>> result;
    for (size_t i = 0; i < ARRAY_SIZE; i++) {
        if (scanAngles[i] >= minAngle && scanAngles[i] <= maxAngle) {
            result.emplace_back(scanAngles[i], scanDistances[i]);
        }
    }
    return result;
}
// Checks for obstacle in the scan area, not used. also not tested is it workes
bool LidarUtility::checkForObstacle(float minAngle, float maxAngle, float threshold, int ARRAY_SIZE)
{
    for (size_t i = 0; i < ARRAY_SIZE; i++) {
        if (scanAngles[i] >= minAngle && scanAngles[i] <= maxAngle) {
            if (scanDistances[i] < threshold) {
                return true; 
            }
        }
    }
    return false;
}
// Checks for obstacle in the filtered data which is output of the getScanDataInRange method. This method is currently used.
bool LidarUtility::checkObstacleInFilteredData(const std::vector<std::pair<float, float>>& filteredData, float threshold) {
    for (const auto& data : filteredData) {
        if (data.second < threshold) {
            return true;  
        }
    }
    return false;  
}
