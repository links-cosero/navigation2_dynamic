# Special types:
# Polygon with 1 vertex: Point obstacle (you might also specify a non-zero value for radius)
# Polygon with 2 vertices: Line obstacle
# Polygon with more than 2 vertices: First and last points are assumed to be connected

std_msgs/Header header

# Obstacle footprint (polygon descriptions)
geometry_msgs/Polygon polygon

# Specify the radius for circular/point obstacles
float64 radius

# Obstacle ID
# Specify IDs in order to provide (temporal) relationships
# between obstacles among multiple messages.
unique_identifier_msgs/UUID id

# Individual orientation (centroid)
geometry_msgs/Quaternion orientation

# Bounding box information
geometry_msgs/Vector3 size

# Individual velocities (centroid)
geometry_msgs/Vector3 velocity

# Individual position (centroid)
geometry_msgs/Point position 

# Confidence score
int32 score