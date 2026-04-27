#ifndef ARIAC_COMPONENTS_PENALTY_HH_
#define ARIAC_COMPONENTS_PENALTY_HH_

#include <string>
#include <gz/sim/components/Component.hh>
#include <gz/sim/components/Factory.hh>
#include <gz/sim/config.hh>

namespace ariac_components
{
  enum class PenaltyType {
    GOOD_CELL_IN_INSPECTION_BIN,
    CELL_IN_CONVEYOR_BIN,
    OBJECT_ON_INVALID_SURFACE,
    AGV_COLLISION,
    ROBOT_COLLISION
  };

  struct Penalty
  {
    PenaltyType type;
    double time;
    std::string description;

    bool operator==(const Penalty &_other) const
    {
      return (this->type == _other.type &&
              this->time == _other.time &&
              this->description == _other.description);
    }

    static bool equal(const ariac_components::Penalty &a, const ariac_components::Penalty &b)
    {
      return a == b;
    }
  };

  namespace serializers
  {
    class PenaltySerializer
    {
      public: static std::ostream &Serialize(std::ostream &_out,
                                             const Penalty &_penalty)
      {
        _out << static_cast<int>(_penalty.type) << " "
             << _penalty.time << " "
             << _penalty.description;
        return _out;
      }

      public: static std::istream &Deserialize(std::istream &_in,
                                               Penalty &_penalty)
      {
        int type_value;
        _in >> type_value
            >> _penalty.time
            >> _penalty.description;
        _penalty.type = static_cast<PenaltyType>(type_value);
        return _in;
      }
    };
  }
}

inline std::ostream& operator<<(std::ostream& _out, const ariac_components::Penalty &_penalty)
{
  return ariac_components::serializers::PenaltySerializer::Serialize(_out, _penalty);
}

inline std::istream& operator>>(std::istream& _in, ariac_components::Penalty &_penalty)
{
  return ariac_components::serializers::PenaltySerializer::Deserialize(_in, _penalty);
}

namespace gz::sim::components
{
  using Penalty = Component<ariac_components::Penalty,
                            class PenaltyTag>;
  GZ_SIM_REGISTER_COMPONENT("ariac_components.Penalty", Penalty);
}

#endif
