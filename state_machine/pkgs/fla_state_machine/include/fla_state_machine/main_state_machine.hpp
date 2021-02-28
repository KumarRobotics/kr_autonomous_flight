#ifndef STATE_MACHINE_PKGS_FLA_STATE_MACHINE_INCLUDE_FLA_STATE_MACHINE_MAIN_STATE_MACHINE_HPP_
#define STATE_MACHINE_PKGS_FLA_STATE_MACHINE_INCLUDE_FLA_STATE_MACHINE_MAIN_STATE_MACHINE_HPP_

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

#endif  // STATE_MACHINE_PKGS_FLA_STATE_MACHINE_INCLUDE_FLA_STATE_MACHINE_MAIN_STATE_MACHINE_HPP_
