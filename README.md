# Autonomy Stack

This is the autonomus flight code stack used at KumarRobotics. 
 - For detailed instructions on how to run in the **Gazebo** simulator, please visit: https://github.com/KumarRobotics/autonomy_stack/blob/master/docs/GazeboSimulatorSetup.md.
 - For detailed instructions on how to run in the **Unity** simulator, please visit: https://github.com/KumarRobotics/autonomy_stack/blob/master/docs/UnitySimulatorSetup.md.
 - For detailed instructions on how to set up a **real robot**, please visit: https://github.com/KumarRobotics/autonomy_stack/wiki.

## How to build Docker images

If you need to build a Docker image locally, you may do it as follows. Replace
`img_name` with the name of the image you would like to build (`base`,
`calibration`, `client`,...).

```
IMG=img_name; docker build -t "kumarrobotics/autonomy:$IMG" -f "$IMG/Dockerfile" .
```

## Build Status
![Docker Build Base](https://github.com/kumarrobotics/autonomy_stack/actions/workflows/docker-build-base.yaml/badge.svg)
![Docker Build Calibration](https://github.com/kumarrobotics/autonomy_stack/actions/workflows/docker-build-calibration.yaml/badge.svg)
![Docker Build Client](https://github.com/kumarrobotics/autonomy_stack/actions/workflows/docker-build-client.yaml/badge.svg)
![Docker Build Control](https://github.com/kumarrobotics/autonomy_stack/actions/workflows/docker-build-control.yaml/badge.svg)
![Docker Build Estimation](https://github.com/kumarrobotics/autonomy_stack/actions/workflows/docker-build-estimation.yaml/badge.svg)
![Docker Build Map_plan](https://github.com/kumarrobotics/autonomy_stack/actions/workflows/docker-build-map-plan.yaml/badge.svg)
![Docker Build State_machine](https://github.com/kumarrobotics/autonomy_stack/actions/workflows/docker-build-state-machine.yaml/badge.svg)
