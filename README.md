![alt text](https://github.com/KumarRobotics/kr_autonomous_flight/blob/master/docs/falcon4-compressed.jpg)

This is the autonomous flight code stack used at KumarRobotics, providing a complete solution for GPS-denied quadcopter autonomy. It has been tested extensively in challenging urban and rural (under forest canopy) environments.

![Docker Build Base](https://github.com/kumarrobotics/kr_autonomous_flight/actions/workflows/docker-build-base.yaml/badge.svg)
![Docker Build Client](https://github.com/kumarrobotics/kr_autonomous_flight/actions/workflows/docker-build-client.yaml/badge.svg)
![Docker Build Control](https://github.com/kumarrobotics/kr_autonomous_flight/actions/workflows/docker-build-control.yaml/badge.svg)
![Docker Build Estimation](https://github.com/kumarrobotics/kr_autonomous_flight/actions/workflows/docker-build-estimation.yaml/badge.svg)
![Docker Build Map_plan](https://github.com/kumarrobotics/kr_autonomous_flight/actions/workflows/docker-build-map-plan.yaml/badge.svg)
![Docker Build State_machine](https://github.com/kumarrobotics/kr_autonomous_flight/actions/workflows/docker-build-state-machine.yaml/badge.svg)

## High-level Code Structure 
![alt text](https://github.com/KumarRobotics/kr_autonomous_flight/blob/master/docs/autonomy_stack_pipeline.png)

## Documentation and Instructions
Please refer to [the Wiki](https://github.com/KumarRobotics/kr_autonomous_flight/wiki) for detailed instructions about how to use this code.

## Videos
[Real-world experiments on large-scale autonomous flight and semantic SLAM in forests (3-min video-only)](https://www.youtube.com/watch?v=Ad3ANMX8gd4)

[Real-world experiments on large-scale autonomous flight and semantic SLAM in forests (5-min with voice-over)](https://www.youtube.com/watch?v=kbyNrRoT9zo)

[Simulation experiments on fast autonomous flight in urban and rural environments](https://www.youtube.com/watch?v=l1esgtJ4C6s)

## Contributing
Please do not hesitate to open a PR on Github.

## Citation
If you use this stack in your work, please cite:

```
@misc{liu2022largescale,
      title={Large-scale Autonomous Flight with Real-time Semantic SLAM under Dense Forest Canopy}, 
      author={Xu Liu, Guilherme V. Nardari, Fernando Cladera Ojeda, Yuezhan Tao, Alex Zhou, Thomas Donnelly, Chao Qu, Steven W. Chen, Roseli A. F. Romero, Camillo J. Taylor, Vijay Kumar},
      year={2022},
      eprint={2109.06479},
      archivePrefix={arXiv},
      primaryClass={cs.RO}
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
## License

This code is released using the Penn Software KR\_AUTONOMOUS\_FLIGHT licence.
Please refer to `LICENSE.txt` for details.
