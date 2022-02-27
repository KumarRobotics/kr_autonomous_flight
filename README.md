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
Real-world experiments on large-scale autonomous flight and semantic SLAM in forests: [3-min video-only version](https://www.youtube.com/watch?v=Ad3ANMX8gd4) and [5-min voice-over version](https://www.youtube.com/watch?v=kbyNrRoT9zo)

[Experiments in fast, autonomous, GPS-Denied quadrotor flight](https://m.youtube.com/watch?v=6eeetSVHXPk)

[Simulation experiments on fast autonomous flight in urban and rural environments](https://www.youtube.com/watch?v=l1esgtJ4C6s)

## Contributing
Report issues: Open an [issue](https://github.com/KumarRobotics/kr_autonomous_flight/issues) on Github.

Merge code changes: Open a [pull request](https://github.com/KumarRobotics/kr_autonomous_flight/pulls) on Github.

## Citation
If you use this stack in your work, please cite:

```
@inproceedings{liu2022large,
title={Large-scale Autonomous Flight with Real-time Semantic SLAM under Dense Forest Canopy},
author={Liu, Xu and Nardari, Guilherme V and Ojeda, Fernando Cladera and Tao, Yuezhan and Zhou, Alex and Donnelly, Thomas and Qu, Chao and Chen, Steven W and Romero, Roseli AF and Taylor, Camillo J and Kumar, Vijay},
journal={IEEE Robotics and Automation Letters (RA-L)},
year={2022}
}
```

```
@article{mohta2018fast,
  title={Fast, autonomous flight in GPS-denied and cluttered environments},
  author={Mohta, Kartik and Watterson, Michael and Mulgaonkar, Yash and Liu, Sikang and Qu, Chao and Makineni, Anurag and Saulnier, Kelsey and Sun, Ke and Zhu, Alex and Delmerico, Jeffrey and others},
  journal={Journal of Field Robotics},
  volume={35},
  number={1},
  pages={101--120},
  year={2018},
  publisher={Wiley Online Library}
}
```
## Acknowledgement 
This is a multi-year project. We gratefully acknowledge contributions from our current members and lab alumni, which include:

[Kartik Mohta](https://github.com/kartikmohta), [Xu Liu](https://scholar.google.com/citations?user=dSIEUlEAAAAJ), [Sikang Liu](https://github.com/sikang), [Fernando Cladera Ojeda](https://github.com/fcladera), [Chao Qu](https://github.com/versatran01), [Michael Watterson](https://github.com/mwatterson), [Dinesh Thakur](https://github.com/tdinesh), [Ke Sun](https://github.com/ke-sun), [Steven W. Chen](https://github.com/chenste), [Bernd Pfrommer](https://github.com/berndpfrommer), [Yuezhan Tao](https://github.com/tyuezhan), [Laura Jarin-Lipschitz](https://github.com/ljarin), [Anurag Makineni](https://github.com/anuragmakineni), [Justin Thomas](https://github.com/justinthomas), [Guilherme V. Nardari](https://github.com/gnardari).

We also thank our [funding agencies](https://www.kumarrobotics.org/research/).

## License

This code is released using the Penn Software Licence.
Please refer to `LICENSE.txt` for details.
