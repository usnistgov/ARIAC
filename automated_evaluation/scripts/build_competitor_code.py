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
        repository = data["github"]["repository"]
    except KeyError:
        print("Unable to find repository link")
        exit()
    
    try:
        token = data["github"]["personal_access_token"]
    except KeyError:
        print("Unable to find personal_access_token")
        exit()

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
    
    # Clone the repository
    clone_cmd = f"git clone https://{token}@{repository} ~/ariac_ws/src/{package_name}"
    subprocess.run(clone_cmd, shell=True)

    # Install extra packages
    for package in data["build"]["debian_packages"]:
        install_cmd = f"apt-get install {package} -y"
        subprocess.run(install_cmd, shell=True)
        
    if data["build"]["pip_packages"]:
        subprocess.run('apt install python3-pip -y' ,shell=True)
    
    for package in data["build"]["pip_packages"]:
        pip_command=f"yes | pip3 install {package}"
        subprocess.run(pip_command,shell=True)

    # Run custom build scripts
    os.chdir('/home/ubuntu/competitor_build_scripts') 
    for script in data["build"]["extra_build_scripts"]:
        subprocess.run(f"chmod +x {script}", shell=True)
        subprocess.run(f"./{script}", shell=True)

    # Install rosdep packages
    os.chdir('/home/ubuntu/ariac_ws') 
    rosdep_cmd = "rosdep install --from-paths src -y --ignore-src"
    subprocess.run(rosdep_cmd, shell=True)

    # Build the workspace
    subprocess.run("colcon build", shell=True)


if __name__=="__main__":
    main()
