# Docker Setup

Installing Docker:
Follow these instructions: https://docs.docker.com/engine/install/

To pull the docker image, run `docker image pull jfernandez37/ariac`

To run the evaluation,
1. Navigate to the automated_evaluation directory in the ARIAC repository
2. Put a nist_competitor.yaml file in the autoEval directory. An example is below
3. Copy the yaml configuration file that you would like to test into the automated_evaluation directory
4. When in the automatic_evaluation directory, run "./runDockerTest.sh" in the command line

`team_name: "nist_competitor"

github:
    repository: "github.com/jfernandez37/nist_competitor" # remove https
    personal_access_token: "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"

competition:
    package_name: "nist_competitor"
    sensor_file: "sensors" # without .yaml
    launch_file: "competitor.launch.py"
    debian_packages: []
    pip_packages: []`
