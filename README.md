# WanderOnSteroids
Planning in Player/Stage

## Functionality

- In this controller (**shared library**) the robot registers itself as an observer of its own updates.

- In each cycle of the simulation, both the **LIDAR** laser and the position values ​​are updated.

- The **LaserUpdate()** function is called in each iteration, controlling the robot's movement through callbacks. It uses the Stage model to receive the updates and the robot object.

- To avoid obstacles, the robot evaluates the proximity sensors on both sides (left and right), turning towards the side where it detects more space, i.e. towards the freest area, to **avoid collision**. 

- Simple algorithms can solve complex autonomous navigation problems in simulated environments.

## Video

You can view the proof of concept videos here:

- **[Cave](https://drive.google.com/file/d/1ewsV_P0ZdYmvAkKmHDIo_R-MycgKufWb/preview?usp=sharing)**
- **[Autolab](https://drive.google.com/file/d/1ugxIJvs1lYWfH86m5tng3e-Fo9fj0mWl/preview?usp=drive_link)**
