#!/usr/bin/env python3

import sys
import yaml

def main():
    # Get yaml file name
    if len(sys.argv) <= 1:
        exit()

    yaml_file = sys.argv[1] + '.yaml'

    # Parse yaml file
    with open(yaml_file, "r") as stream:
        try:
            data = yaml.safe_load(stream)
        except yaml.YAMLError:
            exit()

    try:
        team_name = data["team_name"]
    except KeyError:
        exit()

    print(team_name)


if __name__=="__main__":
    main()
    