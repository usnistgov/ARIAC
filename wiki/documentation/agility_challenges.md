-------------------------------------------------
- Wiki | [Home](../../README.md) | [Documentation](../documentation/documentation.md) | [Tutorials](../tutorials/tutorials.md) | [Qualifiers](../qualifiers/qualifier.md) | [Finals](../finals/finals.md)
-------------------------------------------------

# Wiki | Documentation | Agility Challenges

- This page explains the details of agility challenges that may be present in trials.
- Each challenge has at least one accompanying sample trial config file provided in the `config` directory that can be used to practice.
- The sample trial config files explain the expected behavior at the top of the file.
- See the [configuration tutorial](configuration_spec.md) for how to use these trial config files.


## Faulty products
  * Faulty products should not be used to fulfill the orders.
  * Sample trial:
    * [<b>sample_faulty_products.yaml</b>](https://github.com/usnistgov/ARIAC/blob/master/nist_gear/config/sample_faulty_products.yaml)
  * See the tutorial on [interacting with GEAR](../tutorials/gear_interface.md#Faulty-Products) for details on working with faulty products.

## Insufficiently many products
 * Not enough non-faulty products are in the environment to fulfill all requested orders.
 * Teams must send the AGV with an incomplete kit.
 * Sample trial:
   * [<b>sample_not_enough_products.yaml</b>](https://github.com/usnistgov/ARIAC/blob/master/nist_gear/config/sample_not_enough_products.yaml)


## Flipped products
* An order contains products that must be flipped.
* Only the `pulley_part` will ever be requested to be flipped.
* Sample trial:
  * [<b>sample_flipped_products.yaml</b>](https://github.com/usnistgov/ARIAC/blob/master/nist_gear/config/sample_flipped_products.yaml)
* See documentation on [frame specifications](frame_specifications.md#flipped-products) for details on flipping products.

## In-process order update

* An update to a previously assigned order is sent, identifiable with the order ID such as "order_0_update_0".
* Shipments will be evaluated against the updated order.
* Teams should respond by filling the updated order as usual (submitting shipments named "order_0_shipment_0" still), instead of the original order.
* Sample trial:
  * [<b>sample_order_update.yaml</b>](https://github.com/usnistgov/ARIAC/blob/master/nist_gear/config/sample_order_update.yaml)

## Sensor blackout

* Communication with the sensors will be lost temporarily, referred to as a "sensor blackout".
* Teams should continue to fill the order as usual during this time.
* At the start of the trial the sensors will be publishing data normally, and at a particular instance *all* sensors will *stop* publishing for a fixed period of time.
* This applies to team-specified sensors and sensors that are present by default in the environment such as the quality control sensors.
* The communication will be lost for a duration in the range of 10 to 100 simulation seconds.
* Note that re-connecting to some sensors during development will cause them to resume publishing data, but this functionality is blocked in the automated evaluation setup.
* Sample trial:
  * [<b>sample_sensor_blackout.yaml</b>](https://github.com/usnistgov/ARIAC/blob/master/nist_gear/config/sample_sensor_blackout.yaml)


## Dropped products

* The gripper becomes faulty at various instances, e.g. when a product is retrieved from the storage bins, or when a product is being placed into a kit tray.
* Recovery could include retrieving the dropped product or fetching a new product.
* Sample trial:
  * [<b>sample_dropped_product.yaml</b>](https://github.com/usnistgov/ARIAC/blob/master/nist_gear/config/sample_dropped_product.yaml)

## In-process order interruption

* A second order is announced part-way into the completion of the first order.
* Kits from both orders can be submitted after this time, but the second order is higher priority and for maximum points it should be completed as fast as possible.
* Sample trial:
  * [<b>sample_interruption.yaml</b>](https://github.com/usnistgov/ARIAC/blob/master/nist_gear/config/sample_interruption.yaml)

-------------------------------------------------
- Wiki | [Home](../../README.md) | [Documentation](../documentation/documentation.md) | [Tutorials](../tutorials/tutorials.md) | [Qualifiers](../qualifiers/qualifier.md) | [Finals](../finals/finals.md)

-------------------------------------------------
