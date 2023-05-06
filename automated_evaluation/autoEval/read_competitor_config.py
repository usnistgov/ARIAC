#!/usr/bin/env python3
import yaml
import subprocess
import os
if __name__=="__main__":
    subprocess.run("echo $PWD", shell=True)
    with open("../nist_competitor.yaml", "r") as stream:
        try:
            data = yaml.safe_load(stream)
        except yaml.YAMLError:
            print("Unable to parse yaml file")
    PAT=data["github"]["public_access_token"]
    repo_link=data["github"]["repo_link"]
    team_name=data["team_name"]
    launch_file_name=data["competition"]["launch_file"]
    all_files_current=os.listdir()
    allYamlFiles=[]
    for file in all_files_current:
        if".yaml" in file:
            if file!=team_name+".yaml" and file!="nist_competitor.yaml":
                os.system(f"cp {file} ~/ariac_ws/src/ariac/ariac_gazebo/config/trials/")
                allYamlFiles.append(file.replace(".yaml",""))
    clone_command=f"git clone https://{PAT}@{repo_link} ~/ariac_ws/src/{team_name}"
    subprocess.run(clone_command, shell=True)
    move_launch=f"cp ~/ariac_ws/src/{team_name}/launch/{launch_file_name} /home/ubuntu/ariac_ws/install/ariac_gazebo/share/ariac_gazebo/{launch_file_name}"
    subprocess.run(move_launch, shell=True)
    os.chdir('/home/ubuntu/ariac_ws') #go into the workspace
    for package in data["competition"]["debian_packages"]:
        dowload_deb_command=f"apt-get dowload {package}"
        subprocess.run(download_deb_command,shell=True)
        install_deb_command=f"apt install {package}"
        subprocess.run(install_deb_command,shell=True)
    for package in data["competition"]["pip_packages"]:
        pip_command=f"pip install {package}"
        subprocess.run(pip_command,shell=True)
    if "prebuild_script" in data["competition"]:
        script_path = data["competition"]["prebuild_script"]
        if script_path != "":
            os.chdir(f'/home/ubuntu/ariac_ws/src/{team_name}')
            subprocess.run(f"chmod 777 {script_path}", shell=True)
            script_launch=f"./{script_path}"
            subprocess.run(script_launch, shell=True)
            os.chdir('/home/ubuntu/ariac_ws') #go back into the workspace
    rosdep_command=f"rosdep install --from-paths src -y --ignore-src"
    subprocess.run(rosdep_command, shell=True)
    colcon_build_command=f"colcon build"
    subprocess.run(colcon_build_command, shell=True)
    os.chdir('/home/ubuntu/autoEval/autoEval')
    subprocess.run("ls", shell=True)
    run_launch=f"./runLaunch.sh {launch_file_name}"+((" "+" ".join(allYamlFiles)) if len(allYamlFiles)!=0 else "")
    subprocess.run(run_launch, shell=True)
