# 2D Pressurized Softbody Physics

A project to simulate 2D pressurized soft body physics in Java (using MVC architecture).

## Descritption

The softbodies consist of springs and mass points. Each spring has two mass points on each end (a spring is a Line).

## Configure

- All physical properties can be configured in `ModelConfig.java` (except gravity, which can be edited in `SoftBodyModel.java`). 
- Graphical settings are located in `SoftBodyModel.java` (booleans for drawing points, springs, filling the body etc), and `ViewConfig.java`. 

## How to run

1. Run the Driver.java file.

### Controls

- all controls can be changed in the `ControllerConfig.java` file

- `Left Click`: Select a softbody to control
- `WASD`: Move the selected softbody

  #### Physical Property Change
- `7`: Indicate Pressure Change
- `8`: Indicate Mass Change
- `9`: Indicate Spring Constant Change
- `0`: Indicate Spring Resting Length Change
- `-`: Decrement selected property
- `+`: Increment selected property
  
