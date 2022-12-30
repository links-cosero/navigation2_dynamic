# ros2_costmap_to_dynamic_obstacles

A ROS2 package that includes nodes to extrapolate dynamic obstacles from occupied costmap2d.

This work has been based on: https://github.com/rst-tu-dortmund/costmap_converter/tree/ros2

A detailed report is available here:
https://webthesis.biblio.polito.it/21253/

Starting from the costmap representation of the environment, dynamic obstacles are identified and separated from static ones, applying image processing algorithms and running average filters.

The operation of separating the static obstacles from dynamic ones in the costmap is referenced as background subtraction and is realized adopting two thresholding step.
The output of thess thresholding operations is a binary map: if a cell is compliant to both thresholding operations, it is marked with one, while free space and static obstacles are labeled with zeros. Thus, a binary map marking all the dynamic obstacles is obtained.

The following step leverages an heuristic blob detection and clustering algorithm, based on the OpenCV library, to compute the each obstacle centroid and contour.