Only used in real-robot experiments. 
You will need to update the following propeller parameters in SO3_command_to_mavros.launch according to the following equation:
Thrust = a * RPM^2 + b * RPM + c (Thrust is in Newton not grams-force)
<arg name="thrust_vs_rpm_coeff_a" default="8.401e-7"/>
<arg name="thrust_vs_rpm_coeff_b" default="-1.400e-3"/>
<arg name="thrust_vs_rpm_coeff_c" default="1.12"/>
Contact the maintainers if you are not sure how to calculate those parameters.