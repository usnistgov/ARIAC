Wiki | [Home](../../README.md) | [Documentation](../documentation/documentation.md) | [Tutorials](../tutorials/tutorials.md) | [Qualifiers](../qualifiers/qualifier.md) | [Finals](../finals/finals.md)

-------------------------------------------------


- [Wiki | Qualifiers](#wiki--qualifiers)
- [Overview](#overview)
  - [Part A](#part-a)
    - [qual1](#qual1)
    - [qual2](#qual2)
    - [qual3](#qual3)
  - [Part B](#part-b)
- [Qualifying](#qualifying)
- [Developing your system](#developing-your-system)
- [Submission procedure](#submission-procedure)
- [Re-run policy](#re-run-policy)
- [Important notes](#important-notes)
- [Where to go for help](#where-to-go-for-help)

# Wiki | Qualifiers

- There is a single qualification round for ARIAC 2021, with top teams being chosen to participate in the Finals.
- Qualifier submissions will be evaluated following the [automated evaluation procedure](../documentation/automated_evaluation.md).

# Overview

<!-- This page will be updated around the beginning of April. -->

- The ARIAC qualification task has been designed to evaluate your performance in a subset of the challenges that will be present in the final competition.
- Up to **10** teams will qualify for the Finals based on their performance.
- Some of the trials have a time limit of 500 simulation seconds.
  - Any unfulfilled shipments when the time limit is reached will not be scored.
  - Subscribe to `/clock` to know the current simulation time.
- During the qualifiers and the finals the GEAR interface will work in competition mode. Therefore the argument `--development-mode` will be removed. In competition mode the use of any "cheat" interfaces is disabled and will be blocked in the [automated evaluation setup](../documentation/automated_evaluation.md).
- **Note:** By default simulation state logging is enabled in the trial config files. To disable it during development you can add `--state-logging=no` in the launch file.
- The first qualification task is composed of two parts.

## Part A

Part A consists of three trial files where each trial focuses on one of the [three scenarios](../documentation/competition_specifications.md#competition-scenarios) in ARIAC 2021. Competitors will have to practice with these three trial files to make sure their systems can fulfill the orders while addressing the challenges. Once competitors are satisfied, they should refer to the [submission procedure](#submission-procedure) to upload the required files.

- Three trial config files released to participants: [qual1.yaml](../../nist_gear/config/trial_config/qualifiers_practice/qual1.yaml), [qual2.yaml](../../nist_gear/config/trial_config/qualifiers_practice/qual2.yaml), and [qual3.yaml](../../nist_gear/config/trial_config/qualifiers_practice/qual3.yaml).

### qual1

- This trial focuses on baseline kit building.
    - One order with 2 shipments. Both shipments need to be built on the same agv (`agv3`).
    - Agility challenges:
      - Faulty sensors will be triggered after a second product is placed on any agv.
      - Faulty gripper: A blue battery will drop from the gripper's grasp when placing this part on the agv.
        - A set of faulty products is present in the environment.

### qual2

- This trial focuses on baseline assembly.
    - One assembly order must be built at `as1`.
    - Two agvs are already located at `as1` with products needed for assembly. Although assembly requires only two products, we have provided extra products just in case the robot drops inadvertently drops some products.
    - Agility challenges:
      - None.

### qual3

- This trial focuses on in-process order change.
    - The first order (`order_0`) is announced for assembly. Two agvs are at the station where assembly needs to be performed.
    - During assembly, a new order will be announced (`order_1`) for kitting. This new order has a priority higher than `order_0` and must be completed as soon as possible. `order_1` consists of two shipments, to be performed on different agvs, and to be sent to two different stations.
    - Once `order_1` is done, competitors need to resume `order_0`.
    - Agility challenges:
      - A set of faulty products is present in the environment. These faulty products are products used for kitting and not for assembly.

## Part B

Part B consists of three trial files very very similar to trial files used in Part A. If competitors do well in Part A, they will likely do well in Part B. Trial files used in Part B are not provided to competitors beforehand. We will use these trial files to assess competitors' systems using the scripts sent to us, as described in [submission procedure](#submission-procedure). The idea is to see if competitors made some efforts in fulfilling the orders while addressing the challenges. We are not looking for perfect scores during the qualifiers.

- The qualification task will be evaluated once through the [automated evaluation procedure](../documentation/automated_evaluation.md).
- Competitors will submit a single system that will be automatically evaluated against Part A and Part B, with the sum of all trial scores in part A and part B being a team's score for the qualifier.
- Competitors will have one attempt: if their system fails, there are no opportunities for a repeat attempt.
- Competitors will be able to test their system against Part A "at home", but Part B will be previously-unseen and will test system autonomy.


# Qualifying

- Up to **10** teams will qualify for the Finals through their performance in the qualifier.
- The scoring metrics that will be used for evaluating the first qualifier [are available here](../documentation/scoring.md).
- Only automated evaluation metrics will be used for the qualifier. Judges will be involved during the finals only.
- Any participants found violating submission guidelines (e.g. by moving arms to the products before starting the competition) will not be permitted to qualify for the finals.

# Developing your system

- Develop your system so that it can solve:
  - At least one of the qualification tasks in Part A.
  - All potential agility challenges that may be present in Part B (see [agility challenges](../documentation/agility_challenges.md)).


To ensure that your system can adapt to previously-unseen scenarios, we recommend that you test your system against all released sample trials, in addition to a variety of custom trials that you create following [this tutorial](../documentation/configuration_files.md).

# Submission procedure

* You will find details on how to prepare your submission, which will allow your system to undergo automated evaluation, on the [automated evaluation](../documentation/automated_evaluation.md) page.
* Keep in mind that the submission process is not trivial and should be prepared in advance of the submission deadline.

* Submissions will be made through secure workspaces directly with competition controllers.
* All registered teams must contact **ariac@nist.gov** to have their workspace prepared in advance of when they intend to submit.
* **This must not be left to the last minute** or teams risk missing the submission deadline.
If you are planning a submission and do not yet have a secure workspace, you must contact **ariac@nist.gov** ASAP; a minimum of 24 hours is required by the deadline.

# Re-run policy

**Competitors will have one and only one chance to have their system automatically evaluated.**
* If your system does not install correctly, you will score 0 points.
* If your system does not run correctly, crashes/freezes during trials, or does not score as expected for any other reason, **you will not have a second chance**.

The score that your system obtains will be the score that will be used in the rankings.
For that reason it is imperative that teams thoroughly test their submission in the mock automated evaluation setup.

* The only circumstance in which re-runs will be warranted are if a bug in the simulation impacts the competitor's performance in a trial.
* This includes reported issues at the discretion of the competition controllers.

# Important notes
Teams are advised to review the [update policy](../documentation/update_policy.md) and pay attention to the [updates](../misc/updates.md) page for upcoming and released changes to the software, rules and scoring metrics.


# Where to go for help
We strive to provide responsive and high-quality support for our software.
Please use the [Github Issues](https://github.com/usnistgov/ARIAC/issues) for submitting and following bugs, feature requests and enhancements.

If you have any questions about competition specifics that you would like to ask in private, please contact **ariac@nist.gov**.

-------------------------------------------------

Wiki | [Home](../../README.md) | [Documentation](../documentation/documentation.md) | [Tutorials](../tutorials/tutorials.md) | [Qualifiers](../qualifiers/qualifier.md) | [Finals](../finals/finals.md)
