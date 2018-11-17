## Calculate area of triangle in 3 dimensions 

This package calculates the area of a 3D triangle via C++ client/server nodes: 

1. The "find_area_triangle_3d_client" uses the data type float64 Point found in the package "geometry_msgs/Point.h" to send 3 points in 3D space (x, y, z) to the node "find_area_triangle_3d_server" 

2. The server calculates the area and returns the answer to the client. The client publishes the triangle's area to the "area_triangle_3d" topic

3. The node "find_area_triangle_3d_sub" subscribes to the "area_triangle_3d" topic created by the client and outputs the triangle's area to the console

## Usage

1. Modify the point values in "find_area_triangle_3d_client.cpp"
2. Use the following command to launch the nodes: 
    roslaunch area_triangle_3d area_triangle_3d.launch
3. The 3 nodes will execute in separate consoles

## Dependencies

* geometry/msgs 
* std_msgs