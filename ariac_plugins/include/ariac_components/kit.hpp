#ifndef ARIAC_COMPONENTS_KIT_HH_
#define ARIAC_COMPONENTS_KIT_HH_

#include <map>
#include <optional>
#include <string>
#include <gz/sim/components/Component.hh>
#include <gz/sim/components/Factory.hh>
#include <gz/sim/config.hh>

namespace ariac_components
{
  struct SlotCellInfo
  {
    int cell_type;
    bool defective;
    double voltage;

    bool operator==(const SlotCellInfo &_other) const
    {
      return this->cell_type == _other.cell_type &&
             this->defective == _other.defective &&
             this->voltage == _other.voltage;
    }
  };

  struct Kit
  {
    std::map<int, std::optional<SlotCellInfo>> slots = {
      {1, std::nullopt},
      {2, std::nullopt},
      {3, std::nullopt},
      {4, std::nullopt}
    };

    bool operator==(const Kit &_other) const
    {
      for (int i = 1; i <= 4; i++)
      {
        if (this->slots.at(i).has_value() != _other.slots.at(i).has_value())
        {
          return false;
        }

        if (this->slots.at(i).has_value())
        {
          if (!(this->slots.at(i) == _other.slots.at(i)))
          {
            return false;
          }
        }
      }
      return true;
    }

    static bool equal(const Kit &_a, const Kit &_b)
    {
      return _a == _b;
    }
  };

  namespace serializers
  {
    class KitSerializer
    {
      public: static std::ostream &Serialize(std::ostream &_out,
                                             const Kit &_kit)
      {
        for (int i = 1; i <= 4; i++)
        {
          const auto& slot = _kit.slots.at(i);
          _out << slot.has_value() << " ";
          if (slot.has_value())
          {
            _out << slot->cell_type << " "
                 << slot->defective << " "
                 << slot->voltage << " ";
          }
        }
        return _out;
      }

      public: static std::istream &Deserialize(std::istream &_in,
                                               Kit &_kit)
      {
        for (int i = 1; i <= 4; i++)
        {
          bool has_value;
          _in >> has_value;
          if (has_value)
          {
            SlotCellInfo info;
            _in >> info.cell_type >> info.defective >> info.voltage;
            _kit.slots[i] = info;
          }
          else
          {
            _kit.slots[i] = std::nullopt;
          }
        }
        return _in;
      }
    };
  }
}

namespace gz::sim::components
{
  using Kit = Component<ariac_components::Kit,
                        class KitTag>;
  GZ_SIM_REGISTER_COMPONENT("ariac_components.Kit", Kit);
}

#endif