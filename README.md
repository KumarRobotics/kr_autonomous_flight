![alt text](https://github.com/KumarRobotics/kr_autonomous_flight/blob/master/docs/Falcon4.jpg?raw=true)

This is the autonomous flight code stack used at KumarRobotics, providing a complete solution for GPS-denied quadcopter autonomy. We have tested our stack in multiple scenarios, including urban and rural settings (orchards, forests).

![Docker Build Base](https://github.com/kumarrobotics/kr_autonomous_flight/actions/workflows/docker-build-base.yaml/badge.svg)
![Docker Build Calibration](https://github.com/kumarrobotics/kr_autonomous_flight/actions/workflows/docker-build-calibration.yaml/badge.svg)
![Docker Build Client](https://github.com/kumarrobotics/kr_autonomous_flight/actions/workflows/docker-build-client.yaml/badge.svg)
![Docker Build Control](https://github.com/kumarrobotics/kr_autonomous_flight/actions/workflows/docker-build-control.yaml/badge.svg)
![Docker Build Estimation](https://github.com/kumarrobotics/kr_autonomous_flight/actions/workflows/docker-build-estimation.yaml/badge.svg)
![Docker Build Map_plan](https://github.com/kumarrobotics/kr_autonomous_flight/actions/workflows/docker-build-map-plan.yaml/badge.svg)
![Docker Build State_machine](https://github.com/kumarrobotics/kr_autonomous_flight/actions/workflows/docker-build-state-machine.yaml/badge.svg)

## Videos
[Large scale flight in forests (3-min video-only)](https://www.youtube.com/watch?v=Ad3ANMX8gd4)

[Large scale flight in forests (5-min with voice-over)](https://www.youtube.com/watch?v=kbyNrRoT9zo)

## Documentation and Instructions
Please refer to [the Wiki](https://github.com/KumarRobotics/kr_autonomous_flight/wiki) for detailed instructions about how to use this code.

## Contributing
Please do not hesitate to open a PR on Github.

## Citation
If you use this stack in your work, please cite:

```
@misc{liu2022largescale,
      title={Large-scale Autonomous Flight with Real-time Semantic SLAM under Dense Forest Canopy}, 
      author={Xu Liu and Guilherme V. Nardari and Fernando Cladera Ojeda and Yuezhan Tao and Alex Zhou and Thomas Donnelly and Chao Qu and Steven W. Chen and Roseli A. F. Romero and Camillo J. Taylor and Vijay Kumar},
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

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS, CONTRIBUTORS, AND THE TRUSTEES OF THE UNIVERSITY OF PENNSYLVANIA "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER, CONTRIBUTORS OR THE TRUSTEES OF THE UNIVERSITY OF PENNSYLVANIA BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
