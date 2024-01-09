# FaSTTUBe_planner

The algorithm requires the following inputs:

- The car's current position and orientation in the slam map
- The position of the (optionally colored) cones in the slam map

The algorithm outputs:

- Samples of a parameterized b-spline with the x,y and curvature of the samples

The algorithm is completely stateless. Every time it is called no previous results are used. The only aspect that can be used again is the path that was previously generated. It is only used if the path calculation has failed.

On the publish and subscription replace with the actual topic name

## Dependencies
- ros2
- [lart_msgs](https://github.com/FSLART/lart_msgs)
- numpy
- transformations
- [fsd_path_planning](https://github.com/papalotis/ft-fsd-path-planning)

## Installation
- pip install "fsd-path-planning[demo] @ git+https://git@github.com/papalotis/ft-fsd-path-planning.git"

## Building
- Install the dependencies.
- root worskpace run: `colcon build`.
- run package: `ros2 run path_planner my_node`

