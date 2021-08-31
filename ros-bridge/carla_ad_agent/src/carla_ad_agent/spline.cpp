// #include "spline.h"
// #include "carla_ad_agent/GetPath.h"

// bool getPath(carla_ad_agent::GetPath::Request &req,
//                 carla_ad_agent::GetPath::Response &res)
// {
//     tk::spline sp;
//     std::vector<double> p(2); std::vector<double> s(2);
//     p[0] = req.p_start
//     p[1] = req.p_end
//     s[0] = req.s_start
//     s[1] = req.s_end
//     sp.set_boundary(tk::spline::first_deriv, tan(req.angle), tk::spline::first_deriv, 0, false);
//     sp.set_points(s,p);
    
// }

// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "spline_calculator");
//     ros::NodeHandle n;
//     ros::ServiceServer service = n.advertiseService("spline_calc", getPath)
//     ros::spin()

//     return 0;
// }