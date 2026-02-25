#ifndef ARIAC_COMPONENTS_CELL_HH_
#define ARIAC_COMPONENTS_CELL_HH_

#include <string>
#include <gz/sim/Entity.hh>
#include <gz/sim/components/Component.hh>
#include <gz/sim/components/Factory.hh>
#include <gz/sim/config.hh>

namespace ariac_components
{
  struct Cell
  {
    std::string cell_name;
    int cell_type;
    bool defective;
    double voltage;
    int defect_type;
    double rotation;
    double time_created;
    gz::sim::Entity cell_entity;

    bool operator==(const Cell &_other) const
    {
      return (this->cell_name == _other.cell_name &&
              this->cell_type == _other.cell_type &&
              this->defective == _other.defective &&
              this->voltage == _other.voltage &&
              this->defect_type == _other.defect_type &&
              this->rotation == _other.rotation &&
              this->time_created == _other.time_created);
    }
  };

  namespace serializers
  {
    class CellSerializer
    {
      public: static std::ostream &Serialize(std::ostream &_out,
                                             const Cell &_cell)
      {
        _out << _cell.cell_name << " "
             << _cell.cell_type << " "
             << _cell.defective << " "
             << _cell.voltage << " "
             << _cell.defect_type << " "
             << _cell.rotation << " "
             << _cell.time_created << " "
             << _cell.cell_entity;
        return _out;
      }

      public: static std::istream &Deserialize(std::istream &_in,
                                               Cell &_cell)
      {
        _in >> _cell.cell_name
            >> _cell.cell_type
            >> _cell.defective
            >> _cell.voltage
            >> _cell.defect_type
            >> _cell.rotation
            >> _cell.time_created
            >> _cell.cell_entity;
        return _in;
      }
    };
  }
}

namespace gz::sim::components
{
  using Cell = Component<ariac_components::Cell,
                         class CellTag>;
  GZ_SIM_REGISTER_COMPONENT("ariac_components.Cell", Cell);
}

#endif