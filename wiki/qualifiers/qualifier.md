-------------------------------------------------
- Wiki | [Home](../../README.md) | [Documentation](../documentation/documentation.md) | [Tutorials](../tutorials/tutorials.md) | [Qualifiers](../qualifiers/qualifier.md) | [Finals](../finals/finals.md)
-------------------------------------------------

# Wiki | Qualifiers
- There is a single qualification round for ARIAC 2020, with top teams being chosen to participate in the Finals.
- Qualifier submissions will be evaluated following the [automated evaluation procedure](../documentation/automated_evaluation.md).

# Overview

* The ARIAC qualification task has been designed to evaluate your performance in a subset of the challenges that will be present in the final competition.
* Up to **10** teams will qualify for the Finals based on their performance.

* The first qualification task is composed of two parts.
[Details of how to run Part A and what can be expected in Part B are available here](qualifier_scenarios.md).
In short:

* Part A: three trial config files released to participants:
    * **Config 1**:
        * faulty products
        * update to previously requested order
    * **Config 2**, some challenges from **Config 1**, plus:
        * restricted destination AGV
        * faulty gripper resulting in dropped products
        * moving obstacle
    * **Config 3**, some chalennges from **Config 1** and **Config 2**, plus:
        * high-priority order interruption
        * flipped products
* Part B: trial config file not released to participants.
    * Three config files similar to part A.

* The qualification task will be evaluated once through the [automated evaluation procedure](../documentation/automated_evaluation.md).
* Teams will submit a single system that will be automatically evaluated against Part A and Part B, with the sum of all trial scores in part A and part B being a team's score for the qualifier.
* Teams will have one attempt: if their system fails, there are no opportunities for a repeat attempt.
* Teams will be able to test their system against Part A "at home", but Part B will be previously-unseen and will test system autonomy.

# Important Dates
- **Registration Open** : Now Through April 2020
- **Competition Qualifier**: April 6-10, 2020
- **Competition Finals**: May 11-15, 2020

# Qualifying

* Up to **10** teams will qualify for the Finals through their performance in the qualifier.
* The scoring metrics that will be used for evaluating the first qualifier [are available here](../documentation/scoring.md).
Only automated evaluation metrics will be used for the qualifier.

* Any participants found violating submission guidelines (e.g. by moving arms to the products before starting the competition) will not be permitted to qualify for the Finals.

# Developing your system

* Develop your system so that it can solve:
   * At least one of the qualification tasks in Part A.
   * All potential agility challenges that may be present in Part B:
      * faulty products
      * flipped products
      * high-priority order interruption
      * faulty gripper resulting in dropped products
      * updates to previously requested orders
      * transferring products between arms

[Details of how to run Part A and what can be expected in Part B are available here.](qualifier_scenarios.md)

To ensure that your system can adapt to previously-unseen scenarios, we recommend that you test your system against all released sample trials, in addition to a variety of custom trials that you create following [this tutorial](../documentation/configuration_spec.md).

# Submission procedure

* You will find details on how to prepare your submission, which will allow your system to undergo automated evaluation, on the [automated evaluation](../documentation/automated_evaluation.md) page.
* Keep in mind that the submission process is not trivial and should be prepared in advance of the submission deadline.

* Submissions will be made through secure workspaces directly with competition controllers.
* All registered teams must contact **ariac@nist.gov** to have their workspace prepared in advance of when they intend to submit.
* **This must not be left to the last minute** or teams risk missing the submission deadline.
If you are planning a submission and do not yet have a secure workspace, you must contact **ariac@nist.gov** ASAP; a minimum of 24 hours is required.

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

<!---You can use the [GEAR/ARIAC support forum](https://discourse.ros.org/c/ariac-users) for public discussions about the competition in which other competition participants may participate.-->

-------------------------------------------------
- Wiki | [Home](../../README.md) | [Documentation](../documentation/documentation.md) | [Tutorials](../tutorials/tutorials.md) | [Qualifiers](../qualifiers/qualifier.md) | [Finals](../finals/finals.md)

-------------------------------------------------
