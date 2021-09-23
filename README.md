# cs393r | Autobots

## Setup

### Prerequisites

Please refer to the [UT AUTOmata reference manual][manual] for
instructions on setting up the dependencies:

* [ROS](http://wiki.ros.org/ROS/Installation)
* [amrl_msgs](https://github.com/ut-amrl/amrl_msgs)
* [amrl_maps](https://github.com/ut-amrl/amrl_maps)
* [ut_automata](https://github.com/ut-amrl/ut_automata)

[manual]: https://drive.google.com/file/d/1OUp6FGUPEClpTbXKK8XssCad9MBMnHP-/view?usp=sharing

### Code Formatting

We use [clang-format](https://clang.llvm.org/docs/ClangFormat.html) in a
[pre-commit hook](.githooks/pre-commit) to format staged changes in C++
files. The file must first be symlinked into the local .git folder.

```shell
ln -s ../../.githooks/pre-commit ./git/hooks/
```

## Code Overview

There are three main executables: `navigation`, `particle_filter`, and `slam`. Each executable has a corresponding `.h` and `.cc` file that defines the class for the implementation. An associated `*_main.cc` file abstracts away ROS-specific details. For example, the `particle_filter` executable consists of three files:

```text
src
└── particle_filter
    ├── particle_filter.cc
    ├── particle_filter.h
    └── particle_filter_main.cc
```

Every header file includes documentation in comments for the variables and subroutines.
The project compiles with [Eigen](https://eigen.tuxfamily.org/) for linear algebra and coordinate geometry, the [amrl_shared_lib](https://github.com/ut-amrl/amrl_shared_lib) for commonly used robotics subroutines, and a custom simple priority queue implementation. Some useful references on how to use the libraries:

* See the included [`eigen_tutorial.cc`](src/eigen_tutorial.cc) file for example most common Eigen usage, and the [official Eigen tutorials](https://eigen.tuxfamily.org/dox/GettingStarted.html) for more extensive documentation.
* See the included [`simple_queue_test.cc`](src/navigation/simple_queue_test.cc) for usage of the simple priority queue.

### Building

The [./make](./make) script automatically updates the
`$ROS_PACKAGE_PATH` to include this project directory before building
executables. However, make sure that the [prerequisite](#Prerequisites)
packages are in `$ROS_PACKAGE_PATH`.

### Running the Code

Make sure you recompile your code between changes.

* To run navigation:

    ```shell
    ./bin/navigation
    ```

* To run the particle filter:

    ```shell
    ./bin/particle_filter
    ```

* To run SLAM:

    ```shell
    ./bin/slam
    ```
