std_msgs/Header header # Header timestamp
uint32 height # Data height, that is, number of rows
uint32 width # Data width, that is, number of columns
uint32 step # Full row length in bytes
float32[] data # actual matrix data, size is (step * rows)
