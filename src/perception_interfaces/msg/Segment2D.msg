string name
string color

float64 centroid_u
float64 centroid_v

# Original-image coordinates: x, y, width, height
uint32[4] bbox_xywh
uint32 pixel_count

# Full-resolution binary image: 0 background, 255 object
sensor_msgs/Image mask