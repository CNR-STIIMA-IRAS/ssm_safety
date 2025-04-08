# ssm_safety

The package has three modules that provide an optimized speed and separation monitoring according to ISO TS 15066 to slow down the robot in case of human-robot proximity.

An example of usage with ROS2 is available [here](https://github.com/JRL-CARI-CNR-UNIBS/ssm_safety_ros).

The three modules are:

## Continuous speed adaptation 
The robot slows down according to the relative human robot position and velocity.
The speed modulation is continuous, the speed scaling is computed automatically according to the guidelines provided in ISO TS 15066.

## [fixed_areas_ssm](fixed_areas_ssm/README.md)
The module needs the position of the human in the cell. The robot slows down if the human enters in predefined zones.
Safety zones and speed scaling can be configured from parameters.

## [fixed_distance_ssm](fixed_distance_ssm/README.md)
The module needs the position of the human in the cell. The robot slows down if the relative distance between the human and the robot end-effector is below a given threshold.
Distances and speed scaling can be configured from parameters.

## Requirements

* rdyn: https://github.com/CNR-STIIMA-IRAS/rosdyn.git (version: modern_cmake)

## References

The concept behind the implementation of package velocity_scaling_iso15066 was described in Sec. III-F of the following [paper](https://arxiv.org/pdf/2210.11655.pdf ):
```
@article{faroni2022safety,
  title={Safety-aware time-optimal motion planning with uncertain human state estimation},
  author={Faroni, Marco and Beschi, Manuel and Pedrocchi, Nicola},
  journal={IEEE Robotics and Automation Letters},
  volume={7},
  number={4},
  pages={12219--12226},
  year={2022},
  publisher={IEEE}
}
```

