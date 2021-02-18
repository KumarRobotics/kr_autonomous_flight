#include <primitive_to_traj_opt/convert.h>

// too lazy to include boost math
uint factorial(uint i){
    if(i==0)
        return 1;
    else
        return i*factorial(i-1);
}

/**
 * @brief convert traj msg for visualization?
 */
traj_opt_msgs::Trajectory PrimitiveToTrajOpt::convert(const planning_ros_msgs::Trajectory &msg) {
    traj_opt_msgs::Trajectory traj;
    traj.header = msg.header;
    
    double T=0.0;
    for(uint s=0;s<msg.primitives.size();s++) {
        T+= msg.primitives.at(s).t;
    }


    for(uint d=0;d<3;d++) {
        traj_opt_msgs::Spline spline;
        for(uint s=0;s<msg.primitives.size();s++) {
            const std::vector<double> * co;
            // get correct field
            if(d==0)
                co = &(msg.primitives.at(s).cx);
            if(d==1)
                co = &(msg.primitives.at(s).cy);
            if(d==2)
                co = &(msg.primitives.at(s).cz);
            traj_opt_msgs::Polynomial poly;
            for(uint c=0;c<co->size();c++){
	      uint cr = co->size()-1-c;
                poly.coeffs.push_back(co->at(cr)*std::pow(msg.primitives.at(s).t,c)/double(factorial(c)));
                //poly.coeffs.push_back(co->at(cr)/std::pow(msg.primitives.at(s).t,c)/double(factorial(c)));
            }
            poly.dt = msg.primitives.at(s).t;
            poly.degree = co->size()-1;
            spline.segs.push_back(poly);
        }
        spline.segments = msg.primitives.size();
        spline.t_total = T;
        traj.data.push_back(spline);
    }
    traj.dimensions = 3;
    return traj;
}
