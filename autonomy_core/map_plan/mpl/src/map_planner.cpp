#include "mpl_planner/map_planner.h"

namespace MPL {

template <int Dim>
void MapPlanner<Dim>::setMapUtil(
    const std::shared_ptr<MapUtil<Dim>> &map_util) {
  this->env_.reset(new MPL::EnvMap<Dim>(map_util));
}

template class MapPlanner<3>;

}  // namespace MPL
