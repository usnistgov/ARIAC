#pragma once

#include <string>
#include <gz/sim/components/Component.hh>
#include <gz/sim/components/Factory.hh>

namespace ariac_components
{
  struct Cell{
    std::string cell_name;
    int cell_type;
    bool defective;
    double voltage;
    int defect_type;
    double rotation;
    double time_created;

    gz::sim::Entity cell_entity;

    static bool equal(const ariac_components::Cell &a, const ariac_components::Cell &b){
      return (a.cell_name == b.cell_name &&
              a.cell_type == b.cell_type &&
              a.defective == b.defective &&
              a.voltage == b.voltage && 
              a.defect_type == b.defect_type &&
              a.rotation == b.rotation &&
              a.time_created == b.time_created
            );
    }
    
    friend std::ostream& operator<<(std::ostream& os, const Cell& c){
      os << "Name: " << c.cell_name << ", Type: " << c.cell_type << ", Voltage: " << c.voltage;
      return os;
    }
  };
}

namespace gz::sim::components
{
  // struct CellTag;
  using Cell = Component<ariac_components::Cell, struct CellTag>;

  GZ_SIM_REGISTER_COMPONENT("Cell", Cell)
}