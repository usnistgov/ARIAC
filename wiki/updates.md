# ARIAC competition software updates

_These updates are posted to [this forum thread](https://discourse.ros.org/t/ariac-code-release-updates/4009), which participants can subscribe to for notifications._

## **Changes to ARIAC software**
_To determine the version of the ARIAC software that you're running, check the `~/.ariac/log/performance.log` file, which will contain e.g. "ARIAC VERSION: 2.0.2"._
_To update your ARIAC software, run `sudo apt-get update && sudo apt-get install ariac3`._

_With each ARIAC software release, a new version of the [automated evaluation setup](https://bitbucket.org/osrf/ariac/wiki/2019/automated_evaluation) will be released that uses the latest ARIAC version. See [these instructions](https://github.com/osrf/ariac-docker/blob/master/README.md#keeping-the-competition-setup-software-up-to-date) for how to update._


## 3.0.5 Released 2019, April 25

This release includes performance improvements and the Qualifier part B trial configs.

 * Parts taken off trays now have AutoDisable re-enabled
 * Gasket part collision simplified to reduce number of contacts
 * Pulley part now uses 2 cylinders instead of many boxes for collisions
 * Vacuum gripper update uses Sim time instead of Wall time
 * Qualifier Part B trial configs released

Thanks Steven Gray for contributing performance improvements!

## 3.0.4 Released 2019, April 22

This fixes a crash at shutdown that corrupted the `state.log` file.
It also makes the score text in `performance.log` easier to read.

## 3.0.3 Released 2019, April 5

This fixes a crash when using laser profilers.
The automated evaluation docker images were updated to fix an issue where `models_to_spawn` only worked in `--development-mode`.

## 3.0.2-1 Released 2019, April 3

This fixes a bug in 3.0.2-0 where the software incorrectly logged its version as 3.0.1.

## 3.0.2-0 Released 2019, April 1

The Part A trial config files for the ARIAC 2019 qualifier have been released in version 3.0.2.

Please see the [qualifier page](./qualifier) to learn how qualification works and the [qualifier scenarios page](./qualifer_scenarios) for details on running the Part A trial configs.

### Documentation update 2019, March 30

[Fixed mistake in Competition Interface Documentation](https://bitbucket.org/osrf/ariac/wiki/2019/competition_interface_documentation) found in #164.

The cheat service `/ariac/submit_tray` is actually named `/ariac/submit_shipment` and has type osrf_gear/SubmitShipment.



### 3.0.1 Released 2019, March 28

[Discourse Post](https://discourse.ros.org/t/ariac-code-release-updates/4009/18)

Feature complete release of ARIAC 2019 Software