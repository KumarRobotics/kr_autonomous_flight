#!/usr/bin/env python3

import yaml
import argparse
import numpy as np
import pdb
"""
Update the kalibr output yaml with multicam output yaml
1) T_cn_cnm1
2) distortion_coeffs (for both cameras)
3) intrinsics (for both cameras)
"""

parser = argparse.ArgumentParser()
parser.add_argument("-k", "--kalibr") # input kalibr file path (from pwd)
parser.add_argument("-m", "--multicam") # input multicam file path (from pwd)
parser.add_argument("-o", "--output") # output file path (from pwd)
args = parser.parse_args()

if args.kalibr:
    kalibr_file = args.kalibr
else:
    kalibr_file = "calib_files/kalibr/camchain-imucam-format.yaml"

if args.multicam:
    multicam_file = args.multicam
else:
    multicam_file = "calib_files/multicam/emu1-latest.yaml"

if args.output:
    output_file = args.output
else:
    output_file = "calib_files/kalibr/camchain-imucam-update.yaml"

# Read kalibr yaml
with open(kalibr_file, "r") as stream:
    try:
        k_data = yaml.full_load(stream)
    except yaml.YAMLError as exc:
        print(exc)

# Read multicam yaml
with open(multicam_file, "r") as stream:
    try:
        m_data = yaml.full_load(stream)
    except yaml.YAMLError as exc:
        print(exc)

# Update k_data
for cam in m_data:
    for key in m_data[cam]:
        entry  = m_data[cam][key]
        if isinstance(entry, list):
            if len(np.shape(entry)) > 1:
                k_data[cam][key] = [item for sublist in entry for item in sublist]
            else:
                k_data[cam][key] = entry
        else:
            k_data[cam][key] = entry

k_data['T_imu_body'] = [item for sublist in np.eye(4).tolist() for item in sublist]
with open(output_file, "w") as file:
    yaml.dump(k_data, file, default_flow_style=None)
