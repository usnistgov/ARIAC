# Introduction
- This version is a stand-alone implementation (without integration into the task_manager plugin & with integration into the ARIAC world)
- This is able to emulate the 2 proposed scenarios (flickering & dim lights) for the lighting challenge & temporally trigger the same without using any ROS services.

# Changeable Parameters:
- Challenge scenario:
    - 2 scenarios can be mentioned [here](https://github.com/sparsh-b/ARIAC/blob/777d442f134d388869115e584b4efebdec11b128/ariac_plugins/src/light_malfunction_plugin.cpp#L118): `flicker` and `dim`.
- Challenge Trigger time:
    - The sim time (in seconds) at which the challenge has to be triggered can be modified [here](https://github.com/sparsh-b/ARIAC/blob/777d442f134d388869115e584b4efebdec11b128/ariac_plugins/src/light_malfunction_plugin.cpp#L120)
- Challenge Duration:
    - The duration of the challenge in sim time clock (in seconds) can be mentioned [here](https://github.com/sparsh-b/ARIAC/blob/777d442f134d388869115e584b4efebdec11b128/ariac_plugins/src/light_malfunction_plugin.cpp#L119)
- Brightness for the `dim` challenge scenario
    - A value of [0,1] can be given ([here](https://github.com/sparsh-b/ARIAC/blob/777d442f134d388869115e584b4efebdec11b128/ariac_plugins/src/light_malfunction_plugin.cpp#L127))
- Nominal Brightness:
    - It can take values from [0,100]. Change it in the ariac.world file [here](https://github.com/sparsh-b/ARIAC/blob/777d442f134d388869115e584b4efebdec11b128/ariac_gazebo/worlds/ariac.world#L124) file
## Note:
- These parameteres should be read from config file eventually.

# How to run:
- Build & source the workspace.
- Export the env variable for Gazebo to find the plugin: `export GAZEBO_PLUGIN_PATH=<path-to-your-workspace-root>/build/ariac_plugins`
- Export the env variable for Gazebo to find the models in ARIAC world: `export GAZEBO_MODEL_PATH=<path-to-your-workspace-root>/src/ARIAC/ariac_gazebo/models`
- Launch the ariac.world file with gazebo: `gazebo <path-to-your-workspace-root>/src/ARIAC/ariac_gazebo/worlds/ariac.world --verbose`
- Gazebo should start & the lighting challenge should get triggered & stopped automatically - according to the parameters you've set.