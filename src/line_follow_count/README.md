# line-following and object counting robot
* This project is intended for competition #2 in CMPUT 412 at University of Alberta. All rights reserved.

## Purpose
This project is a warehouse robot prototype using vision signal.
A race track shaped loop with a solid white line marks the path to follow in our use case.
When travelling in the counterclockwise (CCW) direction, red stop lines indicate where the robot should stop.
The robot will be initially place in full stop at the first stop line.
The robot must stay on the line while looping around the track.
There are four stop lines, once you have done a full loop and reach the start stop line your run is done.
There are 3 locations for your robot to get extra points.
Location 1 is marked by a short red line adjacent to the track, your robot has the chance to earn points by counting a number of objects (one, two, or three) at this location.
Location 2 is down a detour, marked by a red line adjacent to the track, again your robot has the chance to earn points by counting the number of objects (one, two, or three).
Location 3 has three separate red line markers, one for each shape (square, circle, triangle).


![](header.png)

## Prerequisite
- This project is developed and tested under [Ubuntu 16.4](https://www.ubuntu.com/download/alternative-downloads) and [ROS Kinetic](http://wiki.ros.org/kinetic) platform.
- This project is to be executed on [kobuki turtlebot](https://www.turtlebot.com/turtlebot2/).
- [openni2 camera](http://wiki.ros.org/openni_camera) and [launch](http://wiki.ros.org/openni2_launch) packages.

## Installation
Open a terminal and type the following bash commands in order

First, clone the release branch of this workspace repoisitory to the directory of your choice
```sh
git clone --single-branch --branch release https://github.com/stwklu/CMPUT_412_code.git

```

Make the packages
```sh
cd CMPUT_412_code
catkin_make
```

## Usage example
Open three terminals, and type each the following command in one of the terminal windows

Before doing so, you should source the setup file in each of the terminals by typing
```sh
source devel/setup.bash
```

In the first terminal, invoke roscore
```sh
roscore
```
In the second terminal, bring up the [turtlebot](https://www.turtlebot.com/turtlebot2/)
```sh
roslaunch turtlebot_bringup minimal.launch
```
In the third terminal, bring up the sensors
```sh
roslaunch turtlebot_bringup 3dsensor.launch
```

## Meta

Your Name – [@YourTwitter](https://twitter.com/dbader_org) – YourEmail@example.com

Distributed under the XYZ license. See ``LICENSE`` for more information.

[https://github.com/yourname/github-link](https://github.com/dbader/)

## Contributing

1. Fork it (<https://github.com/yourname/yourproject/fork>)
2. Create your feature branch (`git checkout -b feature/fooBar`)
3. Commit your changes (`git commit -am 'Add some fooBar'`)
4. Push to the branch (`git push origin feature/fooBar`)
5. Create a new Pull Request

<!-- Markdown link & img dfn's -->
[npm-image]: https://img.shields.io/npm/v/datadog-metrics.svg?style=flat-square
[npm-url]: https://npmjs.org/package/datadog-metrics
[npm-downloads]: https://img.shields.io/npm/dm/datadog-metrics.svg?style=flat-square
[travis-image]: https://img.shields.io/travis/dbader/node-datadog-metrics/master.svg?style=flat-square
[travis-url]: https://travis-ci.org/dbader/node-datadog-metrics
[wiki]: https://github.com/yourname/yourproject/wiki
