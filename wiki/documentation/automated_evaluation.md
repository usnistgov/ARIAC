-------------------------------------------------
- Wiki | [Home](../../README.md) | [Documentation](../documentation/documentation.md) | [Tutorials](../tutorials/tutorials.md) | [Qualifiers](../qualifiers/qualifier.md) | [Finals](../finals/finals.md)
-------------------------------------------------

# Wiki | Documentation | Automated Evaluation

During ARIAC Qualifiers/Finals, each team will submit their system so that it can be run in the automated evaluation setup. This is to ensure that teams are not using prohibited interfaces to interact with the simulation.

## Overview of submission requirements

To allow the ARIAC competition controllers to automatically run each team's system, teams are required to submit the following files:

- **team_config.yaml**: The team's sensor configuration file. One sensor configuration is used for all trials.
- **build_team_system.bash**: A bash script that, when invoked from the command-line on a clean Ubuntu system, will install the necessary dependencies and build the team's code.
- **run_team_system.bash**: A bash script that, when invoked from the command-line on a system that has had **build_team_system.bash** run, will start the team's system and begin interacting with the ARIAC competition trial.

The **build_team_system.bash** script will be run only once to setup the team's system, and the **run_team_system.bash** script will be run one time for each of the scenarios to be tested against.


## Developing/testing your submission

* Teams should develop/test their submission using the instructions provided in [automated evaluation](../tutorials/automated_evaluation.md)
    * This details how to create the required bash scripts and test them in the mock competition setup.

- It is imperative that teams use the outlined process for testing their system, as it exactly replicates the process that will be used during automated evaluation.
- If your system does not score correctly in the mock competition setup, it will not score correctly when run by competition controllers.

* **IMPORTANT**: During the qualifiers and finals teams must test their systems in *Competition Mode*. That is, the option  `development-mode` should not be used.

## Uploading your submission
- Submissions will be made through secure workspaces directly with competition controllers.

- All registered teams must contact ariac@nist.gov to have their workspace prepared in advance of when they intend to submit. **This must not be left to the last minute** or teams risk missing the submission deadline.

- If your team's code is not open source, you can include access keys in your setup script: they will not be made public.

-------------------------------------------------
- Wiki | [Home](../../README.md) | [Documentation](../documentation/documentation.md) | [Tutorials](../tutorials/tutorials.md) | [Qualifiers](../qualifiers/qualifier.md) | [Finals](../finals/finals.md)

-------------------------------------------------
