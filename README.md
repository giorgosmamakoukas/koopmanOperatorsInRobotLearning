# Instructions
The structure of this project is as follows:
```
    .
    ├── headers
    │   ├── dataGeneration.hpp
    │   ├── dynamics.hpp
    │   ├── koopmanModel.hpp
    │   ├── plotting.hpp
    │   ├── riccatiSolver.hpp
    │   └── simulation.hpp
    ├── src
    │   ├── dataGeneration.cpp
    │   ├── dynamics.cpp
    │   ├── koopmanModel.cpp
    │   ├── main.cpp
    │   ├── plotting.cpp
    │   ├── riccatiSolver.cpp
    │   └── simulation.cpp
    ├── matplotlibcpp.h
    └── trainAndControlKoopman
```

- The backbone of the program is in `main.cpp`, which calls on the appropriate functions to generate simulation data, train a Koopman model, and control the dynamics.
- `dataGeneration.cpp/hpp` contain code to simulate a dynamical system forward in time in order to generate training data
- `dynamics.cpp/hpp` contain the dynamics of the inverted pendulum. Expand these files with more complicated dynamics.
- `koopmanModel.cpp/hpp` contain functions that lift system states to Koopman basis functions and train a Koopman model. Currently, only a least-squares computation is supported.
- `plotting.cpp/hpp` is used to plot the state of the controlled system using Koopman-based control
- `riccatiSolver.cpp/hpp` is used to compute solutions to the discrete-time algebraic Riccati equation
- `simulation.hpp` simulated the controlled system forward in time with Koopman-based control

## Compilation
To compile the code, run : `g++ [source files] [compiler language] -I [directories to include]. -o [name of executable]`.

For example, run the following from the root of the repository:
```
g++ src/*.cpp -std=c++17 -DWITHOUT_NUMPY -I. -I/usr/local/Cellar/python@3.11/3.11.9/Frameworks/Python.framework/Versions/3.11/include/python3.11/ -L /usr/local/Cellar/python@3.11/3.11.9/Frameworks/Python.framework/Versions/3.11/lib -lpython3.11 -lpthread -lutil -ldl -o trainAndControlKoopman
```

Use `-std=c++2a` is used for C++20, `-std=c++17` is used for C++17. If successful, find the executable created where you ran the compilation and execute.

NOTE: The Riccati solver may fail to generate LQR gains. If that is the case, try changing the weight matrices and especially increasing the penalty on the control effort (increasing weights in `R` matrix). 

## Execution
After compiling the files, an executable file is created (listed in the repo for convenience under the name `trainAndControlKoopman`). Running the executable prints the state and control efforts in the terminal and plots the states of the inverted pendulum dynamics. Unfortunately, it has been observed that running the same executable file may generate different results in successive runs, which is the source for future investigation. 

## Future Work
- [ ] Ensure the executable gives deterministic results when run multiple times
- [ ] Expand dynamics 
- [ ] Expand training methods of Koopman models
- [ ] Declare all configuration parameters in a single file (e.g. as a class object) for convenience
