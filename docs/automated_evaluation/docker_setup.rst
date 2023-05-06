.. _EVALUATION:


Submission Configuration File
==============================
To be able to participate in the qualifiers and in the finals, competitors need to provide one single YAML file. This file will be used to create a Docker container for each competitor. The Docker container will be used to run the evaluation of the submission. The structure of the YAML file is shown below using a NIST repository as an example.

.. code-block:: yaml
        :class: highlight

        team_name: "nist_competitor"

        github:
            repository: "github.com/jfernandez37/nist_competitor" # remove https
            personal_access_token: "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"

        competition:
            package_name: "nist_competitor"
            sensor_file: "sensors" # without .yaml
            launch_file: "competitor.launch.py"
            debian_packages: []
            pip_packages: []

The YAML structure is described below:

* :yamlname:`team_name`: The name of the team. This name will be used to identify the team in the leaderboard.
* :yamlname:`github`: The GitHub repository where the code of the submission is located. In this example, the repository is private and the personal access token will allow the evaluation to clone the repository. For more information on how to create a personal access token, please refer to `<https://docs.github.com/en/authentication/keeping-your-account-and-data-secure/creating-a-personal-access-token>`_.
* :yamlname:`competition`: 

    - :yamlname:`package_name` is the name of your package. 
    - :yamlname:`sensor_file` is the name of the sensor configuration file (see :ref:`Sensor Configuration File <sensor_configuration_file>`) which contains information about your sensors and cameras. A reminder that the sensor file must be placed in the :file:`config` folder in your package.
    - :yamlname:`launch_file` contains commands to start MoveIt (if you are using MoveIt) and your Nodes. 
    - :yamlname:`debian_packages` is a list of Debian packages which need to be installed for your package. Debian packages will be installed in the Docker container before running the evaluation. The Debian packages are optional.
    - :yamlname:`pip_packages` is a list of packages to be installed with the ``pip`` command. The packages will be installed in the Docker container before running the evaluation. The pip packages are optional.

Submission Procedure
==============================

Competitors must upload their submission configuration file on Google Drive. Please contact Anthony Downs (anthony.downs@nist.gov) to create a folder for your team. The submission configuration file must be named :file:`qualifiers.yaml`. 

Automated Evaluation Procedure
==============================

* Installing Docker: Follow these `instructions <https://docs.docker.com/engine/install/>`_
* To pull the docker image, run ``docker image pull jfernandez37/ariac:ariac_latest``

* To run the evaluation:

    * Navigate to the :file:`automated_evaluation`
    * Place a :file:`nist_competitor.yaml` file in the :file:`autoEval` directory
    * Copy the yaml configuration file that you would like to test into the :file:`automated_evaluation` directory
    * In the :file:`automated_evaluation` directory, run ``chmod 777 *``
    * When in the :file:`automated_evaluation` directory, run ``./runDockerTest.sh``
