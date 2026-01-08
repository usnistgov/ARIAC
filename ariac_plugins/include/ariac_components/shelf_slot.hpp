#ifndef ARIAC_COMPONENTS_SHELF_SLOT_HH_
#define ARIAC_COMPONENTS_SHELF_SLOT_HH_

#include <string>
#include <vector>
#include <gz/math/Pose3.hh>
#include <gz/sim/components/Component.hh>
#include <gz/sim/components/Factory.hh>
#include <gz/sim/config.hh>

namespace ariac_components
{
  struct ShelfSlot
  {
    inline static const std::vector<gz::math::Pose3d> KIT_TRAY_SHELF_SLOTS = {
      gz::math::Pose3d(-0.45, 0.0, 0.125, 0.0, 0.0, 0.0),
      gz::math::Pose3d(-0.15, 0.0, 0.125, 0.0, 0.0, 0.0),
      gz::math::Pose3d(0.15, 0.0, 0.125, 0.0, 0.0, 0.0),
      gz::math::Pose3d(0.45, 0.0, 0.125, 0.0, 0.0, 0.0),
      gz::math::Pose3d(-0.45, 0.0, 0.450, 0.0, 0.0, 0.0),
      gz::math::Pose3d(-0.15, 0.0, 0.450, 0.0, 0.0, 0.0),
      gz::math::Pose3d(0.15, 0.0, 0.450, 0.0, 0.0, 0.0),
      gz::math::Pose3d(0.45, 0.0, 0.450, 0.0, 0.0, 0.0),
      gz::math::Pose3d(-0.45, 0.0, 0.775, 0.0, 0.0, 0.0),
      gz::math::Pose3d(-0.15, 0.0, 0.775, 0.0, 0.0, 0.0),
      gz::math::Pose3d(0.15, 0.0, 0.775, 0.0, 0.0, 0.0),
      gz::math::Pose3d(0.45, 0.0, 0.775, 0.0, 0.0, 0.0),
    };

    inline static const std::vector<gz::math::Pose3d> HIGH_PRIO_SHELF_SLOTS = {
      gz::math::Pose3d(-0.15, 0.0, 0.125, 0.0, 0.0, 0.0),
      gz::math::Pose3d(0.15, 0.0, 0.125, 0.0, 0.0, 0.0),
      gz::math::Pose3d(-0.15, 0.0, 0.450, 0.0, 0.0, 0.0),
      gz::math::Pose3d(0.15, 0.0, 0.450, 0.0, 0.0, 0.0),
      gz::math::Pose3d(-0.15, 0.0, 0.775, 0.0, 0.0, 0.0),
      gz::math::Pose3d(0.15, 0.0, 0.775, 0.0, 0.0, 0.0)
    };

    inline static const std::vector<gz::math::Pose3d> MODULE_SHELF_SLOTS = {
      gz::math::Pose3d(-0.525, 0.0, 0.125, 0.0, 0.0, 0.0),
      gz::math::Pose3d(-0.315, 0.0, 0.125, 0.0, 0.0, 0.0),
      gz::math::Pose3d(-0.105, 0.0, 0.125, 0.0, 0.0, 0.0),
      gz::math::Pose3d(0.105, 0.0, 0.125, 0.0, 0.0, 0.0),
      gz::math::Pose3d(0.315, 0.0, 0.125, 0.0, 0.0, 0.0),
      gz::math::Pose3d(0.525, 0.0, 0.125, 0.0, 0.0, 0.0),
      gz::math::Pose3d(-0.525, 0.0, 0.350, 0.0, 0.0, 0.0),
      gz::math::Pose3d(-0.315, 0.0, 0.350, 0.0, 0.0, 0.0),
      gz::math::Pose3d(-0.105, 0.0, 0.350, 0.0, 0.0, 0.0),
      gz::math::Pose3d(0.105, 0.0, 0.350, 0.0, 0.0, 0.0),
      gz::math::Pose3d(0.315, 0.0, 0.350, 0.0, 0.0, 0.0),
      gz::math::Pose3d(0.525, 0.0, 0.350, 0.0, 0.0, 0.0),
      gz::math::Pose3d(-0.525, 0.0, 0.575, 0.0, 0.0, 0.0),
      gz::math::Pose3d(-0.315, 0.0, 0.575, 0.0, 0.0, 0.0),
      gz::math::Pose3d(-0.105, 0.0, 0.575, 0.0, 0.0, 0.0),
      gz::math::Pose3d(0.105, 0.0, 0.575, 0.0, 0.0, 0.0),
      gz::math::Pose3d(0.315, 0.0, 0.575, 0.0, 0.0, 0.0),
      gz::math::Pose3d(0.525, 0.0, 0.575, 0.0, 0.0, 0.0),
    };

    int index = 0;

    bool operator==(const ShelfSlot &_other) const
    {
      return this->index == _other.index;
    }

    static bool equal(const ShelfSlot &_a, const ShelfSlot &_b)
    {
      return _a == _b;
    }
  };

  namespace serializers
  {
    class ShelfSlotSerializer
    {
      public: static std::ostream &Serialize(std::ostream &_out,
                                             const ShelfSlot &_slot)
      {
        _out << _slot.index;
        return _out;
      }

      public: static std::istream &Deserialize(std::istream &_in,
                                               ShelfSlot &_slot)
      {
        _in >> _slot.index;
        return _in;
      }
    };
  }
}

namespace gz::sim::components
{
  using ShelfSlot = Component<ariac_components::ShelfSlot,
                              class ShelfSlotTag,
                              ariac_components::serializers::ShelfSlotSerializer>;
  GZ_SIM_REGISTER_COMPONENT("ariac_components.ShelfSlot", ShelfSlot)
}

#endif