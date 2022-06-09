![alt text](https://github.com/KumarRobotics/kr_autonomous_flight/blob/master/docs/falcon4-compressed.jpg)

This is the autonomous flight code stack used at KumarRobotics, providing a complete solution for GPS-denied quadcopter autonomy. It has been tested extensively in challenging urban and rural (under forest canopy) environments.

![Docker Build Base](https://github.com/kumarrobotics/kr_autonomous_flight/actions/workflows/docker-build-base.yaml/badge.svg)
![Docker Build Client](https://github.com/kumarrobotics/kr_autonomous_flight/actions/workflows/docker-build-client.yaml/badge.svg)
![Docker Build Control](https://github.com/kumarrobotics/kr_autonomous_flight/actions/workflows/docker-build-control.yaml/badge.svg)
![Docker Build Estimation](https://github.com/kumarrobotics/kr_autonomous_flight/actions/workflows/docker-build-estimation.yaml/badge.svg)
![Docker Build Map_plan](https://github.com/kumarrobotics/kr_autonomous_flight/actions/workflows/docker-build-map-plan.yaml/badge.svg)
![Docker Build State_machine](https://github.com/kumarrobotics/kr_autonomous_flight/actions/workflows/docker-build-state-machine.yaml/badge.svg)
![Docker Build Sim](https://github.com/kumarrobotics/kr_autonomous_flight/actions/workflows/docker-build-sim.yaml/badge.svg)

## High-level Code Structure
![alt text](https://github.com/KumarRobotics/kr_autonomous_flight/blob/master/docs/autonomy_stack_pipeline.png)

## Documentation and Instructions
Please refer to [our Wiki page](https://github.com/KumarRobotics/kr_autonomous_flight/wiki) for detailed instructions about how to use this code.

## Videos
[Real-world experiments in large-scale autonomous flight with real-time semantic SLAM in forests](https://www.youtube.com/watch?v=Ad3ANMX8gd4)

[Real-world experiments in fast, autonomous, GPS-Denied quadrotor flight](https://m.youtube.com/watch?v=6eeetSVHXPk)

[Simulation experiments in fast, autonomous flight in urban and rural environments](https://www.youtube.com/watch?v=l1esgtJ4C6s)

## Contributing
Report issues: Open an [issue](https://github.com/KumarRobotics/kr_autonomous_flight/issues) on Github.

Merge code changes: Open a [pull request](https://github.com/KumarRobotics/kr_autonomous_flight/pulls) on Github.

## Citation
If you use our code in your work, please cite:
```

@article{mohta2018fast,
  title={Fast, autonomous flight in GPS-denied and cluttered environments},
  author={Mohta, Kartik and Watterson, Michael and Mulgaonkar, Yash and Liu, Sikang and Qu, Chao and Makineni, Anurag and Saulnier, Kelsey and Sun, Ke and Zhu, Alex and Delmerico, Jeffrey and Thakur, Dinesh and Karydis, Konstantinos and Atanasov, Nikolay and Loianno, Giuseppe and Scaramuzza, Davide and Daniilidis, Kostas and Taylor, Camillo Jose and Kumar, Vijay},
  journal={Journal of Field Robotics},
  volume={35},
  number={1},
  pages={101--120},
  year={2018}
}
```

```
@article{mohta2018experiments,
  title={Experiments in fast, autonomous, gps-denied quadrotor flight},
  author={Mohta, Kartik and Sun, Ke and Liu, Sikang and Watterson, Michael and Pfrommer, Bernd and Svacha, James and Mulgaonkar, Yash and Taylor, Camillo Jose and Kumar, Vijay},
  booktitle={2018 IEEE International Conference on Robotics and Automation (ICRA)},
  pages={7832--7839},
  year={2018},
  organization={IEEE}
}
```

```
@article{liu2022large,
  title={Large-Scale Autonomous Flight With Real-Time Semantic SLAM Under Dense Forest Canopy},
  author={Liu, Xu and Nardari, Guilherme V. and Ojeda, Fernando Cladera and Tao, Yuezhan and Zhou, Alex and Donnelly, Thomas and Qu, Chao and Chen, Steven W. and Romero, Roseli A. F. and Taylor, Camillo J. and Kumar, Vijay},
  journal={IEEE Robotics and Automation Letters},
  year={2022},
  volume={7},
  number={2},
  pages={5512-5519},
}
```

## Acknowledgement
This is a multi-year project. We gratefully acknowledge contributions from our current members and lab alumni. The contributors can be found in the papers listed above. In addition, [Laura Jarin-Lipschitz](https://github.com/ljarin), [Justin Thomas](https://github.com/justinthomas) also contributed to this work. We also thank our [funding agencies](https://www.kumarrobotics.org/research/).

## License

This code is released using the Penn Software Licence.
Please refer to `LICENSE.txt` for details.
