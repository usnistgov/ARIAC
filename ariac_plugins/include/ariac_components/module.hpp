#pragma once

#include <string>
#include <gz/sim/components/Component.hh>
#include <gz/sim/components/Factory.hh>

namespace ariac_components
{
  enum class CellOrientation {
    UP,
    DOWN,
    NOT_PRESENT
  };

  struct Module
  {
    std::map<int, CellOrientation> cell_orientation = {
      {1, CellOrientation::NOT_PRESENT}, 
      {2, CellOrientation::NOT_PRESENT}, 
      {3, CellOrientation::NOT_PRESENT}, 
      {4, CellOrientation::NOT_PRESENT}
    };
    std::map<int, gz::sim::Entity> cell_entities = {
      {1, gz::sim::kNullEntity}, 
      {2, gz::sim::kNullEntity}, 
      {3, gz::sim::kNullEntity}, 
      {4, gz::sim::kNullEntity}
    };
    std::map<int, bool> top_welds = {
      {1, false}, 
      {2, false}, 
      {3, false}, 
      {4, false}
    };
    std::map<int, bool> bottom_welds = {
      {1, false}, 
      {2, false}
    };
    gz::sim::Entity bottom_shell_entity = gz::sim::kNullEntity;
    gz::sim::Entity top_shell_entity = gz::sim::kNullEntity;

    static bool equal(const ariac_components::Module &a, const ariac_components::Module &b)
    {
      return a.cell_orientation == b.cell_orientation &&
            a.cell_entities == b.cell_entities &&
            a.top_welds == b.top_welds &&
            a.bottom_welds == b.bottom_welds &&
            a.bottom_shell_entity == b.bottom_shell_entity &&
            a.top_shell_entity == b.top_shell_entity;
    }

    friend std::ostream& operator<<(std::ostream& os, const Module& m){
      os << "Bottom shell entity: " << m.bottom_shell_entity
         << ", Top shell entity: " << m.top_shell_entity;
      return os;
    }
  };
}

namespace gz::sim::components
{
  struct ModuleTag;
  using Module = Component<ariac_components::Module, ModuleTag>;

  GZ_SIM_REGISTER_COMPONENT("Module", Module)
}