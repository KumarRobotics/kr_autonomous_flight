#!/usr/bin/env python3

import yaml
import argparse
import numpy as np
"""
Convert the kalibr output yaml into the necessary format for S-MSCKF
"""

parser = argparse.ArgumentParser()
parser.add_argument("-k", "--kalibr") # input kalibr file path (from pwd)
parser.add_argument("-o", "--output") # output file path (from pwd)
args = parser.parse_args()

if args.kalibr:
    kablir_file = args.kalibr
else:
    kalibr_file = "calib_files/kalibr/camchain-imucam.yaml"

if args.output:
    output_file = args.output
else:
    output_file = "calib_files/kalibr/camchain-imucam-format.yaml"

with open(kalibr_file, "r") as stream:
    try:
        k_data = yaml.full_load(stream)
    except yaml.YAMLError as exc:
        print(exc)

for cam in k_data:
    for key in k_data[cam]:
        entry  = k_data[cam][key]
        if isinstance(entry, list):
            if len(np.shape(entry)) > 1:
                k_data[cam][key] = [item for sublist in entry for item in sublist]

k_data['T_imu_body'] = [item for sublist in np.eye(4).tolist() for item in sublist]
with open(output_file, "w") as file:
    yaml.dump(k_data, file, default_flow_style=None)
