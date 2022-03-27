Wiki | [Home](../../README.md) | [Documentation](../documentation/documentation.md) | [Tutorials](../tutorials/tutorials.md) | [Qualifiers](../qualifiers/qualifier.md) | [Finals](../finals/finals.md)

-------------------------------------------------


# Wiki | Documentation | Frames

- This page outlines the specifications of the pose requirements specified in each Order.
- As outlined in the [competition specifications](competition_specifications.md#Order), an Order specifies a list of products to be put into each shipment. Each product has a specified type and required position and orientation in the shipping box (movable tray or briefcase).
- As specified on the [competition interface](competition_interface_documentation.md) page, Orders are communicated to teams with the `nist_gear/Order` ROS message on the topic `ariac/orders`.

## Type of Parts

- The type of the part is specified as its product name, such as `assembly_regulator_red` or `assembly_pump_blue`.
- The availability of these products in the workcell may be determined by querying the `material_locations` ROS service as specified [in this tutorial](../tutorials/gear_interface.md). Note that this service will not work during the qualifiers and the finals. It can only be used during development.

## Pose of Parts in Frames

- The pose of the product is composed of the position and the orientation of the product **specified in the reference frame of the movable tray or briefcase**.
  - The frame of each briefcase is broadcast on `/tf`
  - The frame of the movable tray is not broadcast on `/tf`, however, a movable tray is detected by logical cameras.


### Part Frames

- The pose of the product in the workcell environment will vary over time as the product is moved.
- The frame of each product is typically at the center of the product: it can be visualized by clicking on the product in the simulated workcell environment and pressing `t`: this will display the axes of the frame of the product.

### Flipped Products

- An order could contain a product that requires to be flipped, in which case the requested `roll` for the product will be specified as `pi`. 
- The pump part (`assembly_pump_red`, `assembly_pump_green`, and `assembly_pump_blue`)  is the only product in the environment designed to be flippable. It has a flat collision surface on its top and bottom ends making it ideal for grasping it with a vacuum gripper. 


### Determining the product pose in the tray programmatically

- The logical camera reports the pose of products and shipping boxes with respect to the pose of the camera.
- See the [sensor interface tutorial](../tutorials/sensor_interface.md) for details of how to use this sensor to programmatically determine the pose of products on the trays.
- Other sensors can also be used if combined with perception algorithms.

-------------------------------------------------
- Wiki | [Home](../../README.md) | [Documentation](../documentation/documentation.md) | [Tutorials](../tutorials/tutorials.md) | [Qualifiers](../qualifiers/qualifier.md) | [Finals](../finals/finals.md)

-------------------------------------------------
