# nav2_gradient_costmap_plugin

This plugin inflates the region around a detected dynamic obstacle with a two-dimensional Gaussian shape. In particular, the intuition suggested to associate the magnitude of the obstacle velocity with the peak of the Gaussian; in this way, faster obstacles are inflated more than slower ones. This has the aim to make the local planning aware of the obstacle with a sufficient heads-up for replanning. 
Furthermore, the orientation information is used to inflate more the cells along the moving direction of the obstacles. This is actually obtained blending two 2D Gaussian shapes, one inflating the cells in front of the obstacle and the other inflating the cells on its back region.

## Requirements
Currently the code is compatible with a customized version of Nav2 (Humble release) available here:

https://github.com/links-cosero/navigation2 (dynamic-obstacles-dev branch)

A more detailed description of the implementation is available here:
https://webthesis.biblio.polito.it/21253/

## Videos
![plot](./doc/DWB%2BDOL.gif)