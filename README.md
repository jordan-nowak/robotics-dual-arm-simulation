# robotics-dual-arm-simulation
Development of a 2D simulator for a dual arm articulated robot (T-shaped structure) capable of reaching targets within its workspace. The project covers direct and inverse kinematics, multitasking control and graphical visualisation. It is based on a V-cycle approach, with automated testing and comprehensive technical documentation.

---
## Installation
Clone the source code locally:
```bash
git clone https://github.com/jordan-nowak/robotics-dual-arm-simulation.git
```

The development was carried out on version:
- gcc.exe (Rev8, Built by MSYS2 project) 15.2.0
- g++.exe (Rev8, Built by MSYS2 project) 15.2.0
- cmake 4.2.1
- ninja 1.13.2
- sfml 3.0.2
- eigen 5.0.0

**Note:** 
    Eigen is a header-only dependency used for matrix and vector computations. 
    To install it, create a third_party folder at the root of the project and place the Eigen directory inside (e.g. third_party/eigen). 
    Download Eigen from: https://libeigen.gitlab.io/

---
## Development

If you develop, please create a new 'feature' branch on the 'develop' branch, following these steps:
```bash
cd robotics-dual-arm-simulation
git checkout develop
git checkout -b feature/xxxx-SimpleTitleToDescribeTheFeature
```

Nb: 'xxxx' represent the number incremented of features developped

---
## Test

Don't forget to develop tests for each development made. It is necessary to add them to CMakeList.txt and run all tests with the following command: 
```bash
ctest --verbose
```

---
## Usage

Configure the compiler and build the library to obtain executable:
```bash
cd robotics-dual-arm-simulation/build
cmake -G Ninja -DCMAKE_BUILD_TYPE=Release ..
cmake --build .
```

---
### [WORK IN PROGRESS] Offline API Documentation
With [Doxygen](https://www.doxygen.nl) installed, the documentation can be built locally by...

The resulting documentation can be accessed by opening `robotics-dual-arm-simulation/docs/html/index.html` in a web browser.

---
## License
The license that applies to the whole package content is GPLv3. Please look at the [license.md](./doc/license.md) file at the root of this repository for more details.

---
## Authors
**robotics-dual-arm-simulation** has been developed by the following authors: 
+ Nowak Jordan (JNo)

Please contact him for more information or questions.