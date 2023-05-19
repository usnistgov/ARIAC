#!/usr/bin/env python3

import os
import sys
from subprocess import Popen, call
from signal import SIGINT
import yaml


def main():
    # Get team file name
    yaml_file = sys.argv[1] + '.yaml'

    if not os.path.isfile(yaml_file):
        print(f'{yaml_file} not found')
        sys.exit()

    # Parse yaml file
    with open(yaml_file, "r") as stream:
        try:
            data = yaml.safe_load(stream)
        except yaml.YAMLError:
            print("Unable to parse yaml file")
            sys.exit()

    # Store data from yaml filyaml_path
    try:
        package_name = data["competition"]["package_name"]
    except KeyError:
        print("Unable to find package_name")
        sys.exit()

    try:
        launch_file = data["competition"]["launch_file"]
    except KeyError:
        print("Unable to find launch_file")
        sys.exit()

    trial_name = sys.argv[2]

    my_env = os.environ.copy()
    my_env["DISPLAY"] = ":1.0"
    process = Popen(["ros2", "launch", package_name, launch_file, f"competitor_pkg:={package_name}",
                    f"trial_name:={trial_name}", '--noninteractive'], env=my_env)

    # Continue execution of trial until log file is generated
    while True:
        if os.path.exists(f'/home/ubuntu/logs/{trial_name}.txt'):
            break

    print(f"==== Trial {trial_name} completed")

    process.send_signal(SIGINT)
    
    # Might raise a TimeoutExpired if it takes too long
    return_code = process.wait(timeout=10)
    print(f"return_code: {return_code}")
    
    


if __name__ == "__main__":
    main()
