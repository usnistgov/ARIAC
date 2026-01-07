#ifndef ARIAC_COMPONENTS_MODULE_HH_
#define ARIAC_COMPONENTS_MODULE_HH_

#include <map>
#include <string>
#include <gz/sim/Entity.hh>
#include <gz/sim/components/Component.hh>
#include <gz/sim/components/Factory.hh>
#include <gz/sim/config.hh>

namespace ariac_components
{
  enum class CellOrientation
  {
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

    bool operator==(const Module &_other) const
    {
      return this->cell_orientation == _other.cell_orientation &&
             this->cell_entities == _other.cell_entities &&
             this->top_welds == _other.top_welds &&
             this->bottom_welds == _other.bottom_welds &&
             this->bottom_shell_entity == _other.bottom_shell_entity &&
             this->top_shell_entity == _other.top_shell_entity;
    }

    static bool equal(const Module &_a, const Module &_b)
    {
      return _a == _b;
    }
  };

  namespace serializers
  {
    class ModuleSerializer
    {
      public: static std::ostream &Serialize(std::ostream &_out,
                                             const Module &_module)
      {
        for (int i = 1; i <= 4; i++)
        {
          _out << static_cast<int>(_module.cell_orientation.at(i)) << " ";
        }

        for (int i = 1; i <= 4; i++)
        {
          _out << _module.cell_entities.at(i) << " ";
        }

        for (int i = 1; i <= 4; i++)
        {
          _out << _module.top_welds.at(i) << " ";
        }

        for (int i = 1; i <= 2; i++)
        {
          _out << _module.bottom_welds.at(i) << " ";
        }

        _out << _module.bottom_shell_entity << " "
             << _module.top_shell_entity;

        return _out;
      }

      public: static std::istream &Deserialize(std::istream &_in,
                                               Module &_module)
      {
        for (int i = 1; i <= 4; i++)
        {
          int orientation;
          _in >> orientation;
          _module.cell_orientation[i] = static_cast<CellOrientation>(orientation);
        }

        for (int i = 1; i <= 4; i++)
        {
          _in >> _module.cell_entities[i];
        }

        for (int i = 1; i <= 4; i++)
        {
          _in >> _module.top_welds[i];
        }

        for (int i = 1; i <= 2; i++)
        {
          _in >> _module.bottom_welds[i];
        }

        _in >> _module.bottom_shell_entity >> _module.top_shell_entity;

        return _in;
      }
    };
  }
}

namespace gz::sim::components
{
  using Module = Component<ariac_components::Module,
                           class ModuleTag,
                           ariac_components::serializers::ModuleSerializer>;
  GZ_SIM_REGISTER_COMPONENT("ariac_components.Module", Module)
}

#endif