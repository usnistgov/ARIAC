-------------------------------------------------
- Wiki | [Home](../../README.md) | [Documentation](../documentation/documentation.md) | [Tutorials](../tutorials/tutorials.md) | [Qualifiers](../qualifiers/qualifier.md) | [Finals](../finals/finals.md)
-------------------------------------------------



## Released 2020, March 21

- Added qual A yaml files (see [here](../qualifiers/qualifier_scenarios.md)).
- Added challenge yaml files (see [here](../documentation/agility_challenges.md)).
  - Major changes to the wiki, which is now  up to date (except for [this page](../tutorials/gear_interface.md)) .
  - Please contact me (zeid.kootbally@nist.gov) if anything is out of place.

## Released 2020, March 20

- Changed `product_` to `part_` in yaml trial config files.

- For instance, the following code:

  ```yaml
  products:
        product_0:
          type: piston_rod_part_blue
          pose:
            xyz: [0.1, -0.1, 0]
            rpy: [0, 0, 0]
  ```

- is replaced with:

```yaml
products:
      part_0:
        type: piston_rod_part_blue
        pose:
          xyz: [0.1, -0.1, 0]
          rpy: [0, 0, 0]
```



## Released 2020, March 13

- Cost of the RGBD camera is set to be the same as the Logical camera ($500).