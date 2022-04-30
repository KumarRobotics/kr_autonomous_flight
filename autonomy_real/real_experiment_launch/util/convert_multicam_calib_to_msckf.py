import ruamel.yaml
import copy

# TODO: maybe make those as input args
in_folder_name = "/home/dcist/multicam_calibration/calib/example/"
out_folder_name = "../config/"
input_fname = 'ovc-latest.yaml'
output_fname = 'msckf_calib.yaml'
output_calib = None

with open(in_folder_name + input_fname, 'r') as file:

    loaded_calib = ruamel.yaml.safe_load(file)
    output_calib = copy.deepcopy(loaded_calib)

    # add T_imu_body as identity transform
    output_calib['T_imu_body'] = [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
  0.0, 1.0]

    # remove unused data
    del output_calib['cam0']['rectification_matrix']
    del output_calib['cam0']['projection_matrix']
    del output_calib['cam1']['rectification_matrix']
    del output_calib['cam1']['projection_matrix']
    del output_calib['cam1']['T_cam_imu']

    # modify the T_cam_imu and T_cn_cnm1 format
    output_calib['cam0']['T_cam_imu'] = []
    output_calib['cam1']['T_cn_cnm1'] = []

    for i in range(4):
        for j in range(4):
            output_calib['cam0']['T_cam_imu'].append(loaded_calib['cam0']['T_cam_imu'][i][j])
            output_calib['cam1']['T_cn_cnm1'].append(loaded_calib['cam1']['T_cn_cnm1'][i][j])
        if j != 3:
            output_calib['cam0']['T_cam_imu'][i].append(',')
            output_calib['cam1']['T_cn_cnm1'][i].append(',')

with open(out_folder_name + output_fname, 'w') as file:
        ruamel.yaml.safe_dump(output_calib, file)
