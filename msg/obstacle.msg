float64 x #x coordinate of obstacle
float64 y #y coordinate of obstacle
float64 distance #distance from robot to obstacle
float64 heading #TODO: populate with angle from robot heading vector to relative obstacle position vector
bool isAMovingObstacle #true is obstacle is moving, otherwise false
int32 objStartIndex #index of the first point that is grouped in the obstacle in the laser scan
int32 objEndIndex #index of the last point that is grouped in the obstacle in the laser scan
float64 obsRoughSize #diameter of obstacle (may not be populated)
float64 obsLineSize #width of detected obstacle
lidar_point startPoint #x y coordinate of start index
lidar_point endPoint #x y coordinate of end index