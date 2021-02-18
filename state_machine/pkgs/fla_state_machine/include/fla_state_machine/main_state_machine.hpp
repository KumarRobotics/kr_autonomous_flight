#ifndef MAIN_STATE_MACHINE_HPP
#define MAIN_STATE_MACHINE_HPP

#include <iostream>
// back-end
#include <boost/msm/back/state_machine.hpp>
// front-end
#include <boost/msm/front/state_machine_def.hpp>

namespace msm = boost::msm;
namespace mpl = boost::mpl;

namespace {
struct toggle_motors {};
struct take_off {};
struct land {};
struct hover {};

}  // namespace

#endif  // MAIN_STATE_MACHINE_HPP
