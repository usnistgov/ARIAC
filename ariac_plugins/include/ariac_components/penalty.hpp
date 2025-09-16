#pragma once

#include <string>
#include <gz/sim/components/Component.hh>
#include <gz/sim/components/Factory.hh>

namespace ariac_components
{
  enum class PenaltyType {
    GOOD_CELL_IN_INSPECTION_BIN,
    CELL_IN_CONVEYOR_BIN,
    OBJECT_ON_INVALID_SURFACE,
    AGV_COLLISION,
    ROBOT_COLLISION
  };
  
  struct Penalty{
    PenaltyType type;
    double time;
    std::string description;

    static bool equal(const ariac_components::Penalty &a, const ariac_components::Penalty &b){
      return (a.type == b.type &&
              a.description == b.description && 
              a.time == b.time);
    }

    friend std::ostream& operator<<(std::ostream& os, const Penalty& p){
      os << "Time: " << p.time
         << ", Description: " << p.description;
      return os;
    }
  };
}

namespace gz::sim::components
{
  struct PenaltyTag;
  using Penalty = Component<ariac_components::Penalty, PenaltyTag>;

  GZ_SIM_REGISTER_COMPONENT("Penalty", Penalty)
}