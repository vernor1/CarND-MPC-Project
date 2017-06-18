# Model Predictive Control Project

The goals / steps of this project are the following:

* Complete the MPC algorithm in C++.
* Ensure that the project compiles.
* Test the MPC with the simulator.

## [Rubric](https://review.udacity.com/#!/rubrics/896/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.

---
### Compilation
#### 1. Your code should compile.

The code compiles without errors with cmake-3.8.2 and make-3.81 on macOS-10.12.5.

---
### Implementation
#### 1. The Model.

The Kinematic Model of the vehicle is described by the equations: <p align="center"><img src="pic/state_update.png" alt="State Update" width="200"/></p>
The state variables and actuators are:

* **x**,**y**: vehicle coordinates;
* **v**: speed;
* **psi**: yaw;
* **Lf**: distance between the front of the vehicle and its center of gravity (measured 2.67m for the sumulator);
* **delta**: steering angle;
* **a**: acceleration.

For optimization purpose, the vehicle state also includes the Cross Track Error (CTE) and error of psi:

* <img src="pic/cte_update.png" alt="CTE Update" width="300"/>, where f(x) is the reference path polinomial evaluated at the vehicle coordinate x;
* <img src="pic/epsi_update.png" alt="ePsi Update" width="250"/>, where <img src="pic/psides.png" alt="PsiDes" width="45"/> is equal to <img src="pic/psides_value.png" alt="PsiDes Value" width="90"/>.

The initial state of the vehicle at any moment is:

* **x=0**, **y=0**, **psi=0** (assuming that the reference waypoints are converted to the vehicle coordinate system)
* v is taken from the vehicle telemetry
* <p align="left"><img src="pic/cte.png" alt="CTE" width="120"/></p>
* <p align="left"><img src="pic/epsi.png" alt="CTE" width="120"/></p>

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions


1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.
