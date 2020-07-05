# MATLAB Wrapper Example Project

This project serves as a lightweight example for demonstrating how to wrap C++ code in MATLAB using GTSAM.

## Compiling

We follow the regular build procedure inside the `example_project` directory:

```sh
mkdir build && cd build
cmake ..
make -j8
sudo make install

sudo ldconfig  # ensures the shared object file generated is correctly loaded
```

## Usage

Now you can open MATLAB and add the `gtsam_toolbox` to the MATLAB path

```matlab
addpath('/usr/local/gtsam_toolbox')
```

At this point you are ready to run the example project. Starting from the `example_project` directory inside MATLAB, simply run code like regular MATLAB, e.g.

```matlab
pe = example.PrintExamples();
pe.sayHello();
pe.sayGoodbye();
```