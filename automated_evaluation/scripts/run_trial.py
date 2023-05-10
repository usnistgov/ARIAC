#!/usr/bin/env python3

import os
import sys
import subprocess
import yaml


def main():
    # Get yaml file name
    if len(sys.argv) <= 1:
        print("Please include an argument for the yaml file to run")
        exit()

    yaml_file = sys.argv[1] + '.yaml'
    print(f'Running {yaml_file}')

    if not os.path.isfile(yaml_file):
        print(f'{yaml_file} not found')
        exit()

    # Parse yaml file
    with open(yaml_file, "r") as stream:
        try:
            data = yaml.safe_load(stream)
        except yaml.YAMLError:
            print("Unable to parse yaml file")
            exit()

    # Store data from yaml filyaml_path
    try:
        package_name = data["competition"]["package_name"]
    except KeyError:
        print("Unable to find package_name")
        exit()

    try:
        launch_file = data["competition"]["launch_file"]
    except KeyError:
        print("Unable to find launch_file")
        exit()

    # Run 
    launch_cmd = f"ros2 launch {package_name} {launch_file} trial_name:={sys.argv[2]}"
    subprocess.run(launch_cmd, shell=True, env=dict(os.environ, DISPLAY=":1.0"))


if __name__=="__main__":
    main()
