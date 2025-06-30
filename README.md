# Geometry Common

[![build status](https://git.locomotec.com:444/kelo/common/geometry_common/badges/master/build.svg)](https://git.locomotec.com:444/kelo/common/geometry_common/commits/master)

Common data structures, utilities and modules for geometric tasks.

## Documentation

We use [Doxygen](https://www.doxygen.nl/index.html) for code documentation.

**Note**: Building the documentation is disabled by default.

- To build the code documentation, Doxygen needs to be installed. On debian based
  systems, this can be achieved with
  ```bash
  sudo apt install doxygen
  ```

- Documentation can be built using the flag `-DBUILD_DOC=ON`
  ```bash
  colcon build --packages-up-to geometry_common --cmake-args -DBUILD_DOC=ON
  ```

- The documentation will be generated at
  `<YOUR_CATKIN_WS>/build/geometry_common/docs/html/index.html`

## Test

Run unit tests with

```bash
colcon test --packages-up-to geometry_common --event-handlers console_direct+
```

**Note**: Requires `GTest` package (`sudo apt install libgtest-dev`)
