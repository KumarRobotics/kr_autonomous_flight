#include "mpl_planner/map_planner.h"

namespace MPL {

template <int Dim>
MapPlanner<Dim>::MapPlanner(bool verbose) {
  this->planner_verbose_ = verbose;
  if (this->planner_verbose_) {
    printf(ANSI_COLOR_CYAN
           "[MapPlanner] PLANNER VERBOSE ON\n" ANSI_COLOR_RESET);
  }
}

template <int Dim>
void MapPlanner<Dim>::setMapUtil(
    const std::shared_ptr<MapUtil<Dim>> &map_util) {
  this->env_.reset(new MPL::EnvMap<Dim>(map_util));
  map_util_ = map_util;
}

template class MapPlanner<2>;
template class MapPlanner<3>;

}  // namespace MPL
