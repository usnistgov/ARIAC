-------------------------------------------------
- Wiki | [Home](../../README.md) | [Documentation](../documentation/documentation.md) | [Tutorials](../tutorials/tutorials.md) | [Qualifiers](../qualifiers/qualifier.md) | [Finals](../finals/finals.md)
-------------------------------------------------
# Wiki | Documentation | Log Files

- When enabled, log files are written to the `~/.ariac/log` directory.
- The following files are generated.
  - Trial performance log file
    - The trial performance log file is written to `~/.ariac/log/performance.log`.
    - This file will be reviewed to evaluate your performance in the trial.
  - Simulation state log file
    - The simulation state log file is written to `~/.ariac/log/gazebo/state.log`.
    - This log stores all of the simulation data during the trial. It is useful for re-playing a simulation for inspection/debugging purposes and may be used to confirm your trial performance log file.

# Enabling log file generation

- The trial performance log file is always generated.
- The simulation state log file is only generated if the `--state-logging=true` option is passed to `gear.py`.

**_If you see the following output when logging is enabled, it is safe to ignore it:_**

```
Error [parser_urdf.cc:3474] Unable to call parseURDF on robot model
Error [parser.cc:342] parse as old deprecated model file failed.
```

# Copying log files

- **Log files are over-written with every run or playback of the ARIAC software, so if you want to keep them, be sure to make a backup before running more trials.**
- As the performance log is a symbolic link to another file, be sure to use the `cp --dereference` option when copying the `~/.ariac/log` directory, e.g:

```
cp --recursive --dereference ~/.ariac/log ~/ariac_log_files
```

# Playback of state log files

- The most recent state log file can be played back using the following invocation:


```
roslaunch nist_gear gear_playback.launch
```

- Or, you can replay a specific log file with the following (be sure to use an absolute path):

```
roslaunch nist_gear gear_playback.launch state_log_path:=`pwd`/specific_log_file.log
```

- Log files are not necessarily playable when transferred to another computer; they must have ARIAC installed in the same location.
- Don't worry if you see the following warning, it's safe to ignore it:

```
[Wrn] [msgs.cc:1793] Conversion of sensor type[logical_camera] not suppported.
```

- If you see the error below then you may find `pkill -f gzserver` useful for killing gazebo servers that have not closed properly.

```
[Err] [Master.cc:96] EXCEPTION: Unable to start server[bind: Address already in use]. There is probably another Gazebo process running.
```

-------------------------------------------------
- Wiki | [Home](../../README.md) | [Documentation](../documentation/documentation.md) | [Tutorials](../tutorials/tutorials.md) | [Qualifiers](../qualifiers/qualifier.md) | [Finals](../finals/finals.md)

-------------------------------------------------
