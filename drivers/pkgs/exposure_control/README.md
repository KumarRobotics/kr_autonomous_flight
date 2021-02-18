# Exposure Control

This package provides an auto exposure control node that subscribes an
image topic, evaluates the image, and publishes the desired exposure
parameters to the "exposure" topic.

## how to run

Usually this node is started along with the `cam_imu_sync`
package. This is how to launch it for testing with one camera. If you
launch it with a bad serial number for the camera it'll tell you in
the error message what cameras are available:

	roslaunch fla_drivers_launch vi_forward_autoexp.launch num_cameras:=1 cam0:=16306381

Or if you just want to launch it stand-alone (frame rate must match
the sync rate of the camera/imu system):

	roslaunch exposure_control exposure_control.launch frame_rate:=40

## dynamically configurable parameters

Currently a simple controller is implemented that tries to match a
target brightness inside the region of interest (ROI). The control
loop is just:

    shutter_new/shutter_current = (brightness_desired/brightness_current)^alpha

So `shutter_new` grows/shrinks until the brightness target is reached.

This controller was found in "Camera Auto Exposure Control for VSLAM
Applications" by Michael Muehlebach, equation (2.6).

The  controller assumes a linear dependency between image brightness and shutter
speed. The constant `alpha` is the equivalent of the "P" of a PID
controller, but in log space. Must be less than 1.0.

   - `enabled` set this to true to enable the node (default)
   - `min_shutter` minimum allowed shutter speed in milliseconds
   - `max_shutter` max shutter (in msec) allowed. Use this to put an
      upper limit on the shutter. Note that the frame rate also
      implicitly caps the maximum shutter time!
   - `brightness` the target image brightness in the ROI. Value from 0..255.
   - `min_gain` minimum gain
   - `max_gain` maximum gain
   - `auto_shutter_alpha`. Parameter controlling convergence,
      corresponds to "P" in log space, see above, must be <= 1.0.
   - `down_sampling_rate`. Window size for downsampling. Only use
      powers of 2, preferably 16, 32, 64 for efficiency. If you set it to
      16, then only one in 16^2 = 256 pixels will be used to determine
      brightness. Usually that's plenty enough.
   - `top_margin`. What percentage of the top of the image to ignore.
   - `bottom_margin`. What percentage of the bottom to ignore (leave
      at 0).
   - `wait_frames`. This is the number of frames it takes the camera
      to apply a change in shutter or gain. Must wait at least this
	  number of frames before evaluating the exposure again. According to
	  PointGrey docs, this should be 1 or 2 when running in trigger mode,
	  but in practice it was found to be 4! Too small values here will
	  lead to flicker, unnecessarily large values will slow down the
	  speed of the auto-exposure. [Update: after firmware upgrade the
      Flir cameras now actually update exposure within one frame, so
	  theoretically wait_frames could be set to 2 (it takes one extra
      frame for the round-trip between exposure control node and cam
      sync driver)
   


