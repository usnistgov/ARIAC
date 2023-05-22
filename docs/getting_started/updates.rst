Updates
========

Release 2023_v1.5, May 19
-------------------------

- Added logging to the task manager when running in docker
- Reduced publish rate for certain topics
- Added ability to run multiple trials sequentially in docker
- Updated automated evaluation documentation
- Fixed humans not moving in docker
- Fixes issue with pump collision
- Fixes issue with perform quality check service 


Release 2023_v1.4, May 11
-------------------------

- Reworked automated evaluation scripts and updated documentation


Release 2023_v1.3, May 6
------------------------

- Added information on the :ref:`Automated Evaluation <EVALUATION>` explaining how to evaluate your own package. If your package requires complex installations, modify the appropriate files and place them in your google drive folder.
- Fixed issue with not getting the bonus points for kitting.
- Fixed an issue with the Human Challenge where the human teleports when getting to close to a non-moving AGV.
- Fixed issue with Gazebo crashing when a part is dropped on the floor.
- Added logic to the test competitor to submit a kitting order only when the AGV has reached the warehouse.


Release 2023_v1.2, April 14
---------------------------

- Added information on the :ref:`Automated Evaluation <EVALUATION>`.
- Added more information on the :ref:`Human Challenge <HUMANS>`.
.. - Fixed issue with not getting the bonus points for kitting.
.. - Fixed an issue with the Human Challenge where the human teleports when getting to close to a non-moving AGV.
.. - Fixed issue with Gazebo crashing when a part is dropped on the floor.
.. - Added logic to the test competitor to submit a kitting order only when the AGV has reached the warehouse.

Release 2023_v1.1, April 3
--------------------------------------------

- Modified :tuto:`Tutorials` :ref:`1 <TUTORIAL1>` and :ref:`2 <TUTORIAL2>`.
- Added :tuto:`Tutorials` :ref:`3 <TUTORIAL3>`, :ref:`4 <TUTORIAL4>`, :ref:`5 <TUTORIAL5>`, :ref:`6 <TUTORIAL6>`, and :ref:`7 <TUTORIAL7>`. 
- Documentation improvements.
- Added :ref:`human <HUMANS>` challenge.
- Fixed assembly bug with battery.

Release 2023_v1.0, March 1 -- Competition Version
--------------------------------------------

- The competition version of the software is now available. This release contains most of the features that were highlighted during the launch telecon. A list of the features that have been implemented since the previous release is provided below:
  
  - Assembly tasks.
  - Combined tasks.
  - Faulty gripper challenge.
  - Faulty part challenge.
- The test competitor package has been updated to include:
 
  - Assembly Task (assembly.yaml)
  - Combined Task (combined.yaml)
- The documentation was updated to include:
  
  - Tutorials
  - More information on the competition interface.
  - More information on the competition overview.
  - More information on the trial configuration files.
- A graphical user interface to generate trial configuration files.
- :red:`The human challenge will be added at a later date.`

Release 2023, Jan 23 -- Beta Version
------------------------------------

- This is the beta release of the software. This release contains the basic structure of the software and is missing some features that were highlighted during the launch telecon. These features will be added in the final release.
- A list of the features that are currently in progress is provided below:
 
  - The graphical user interface to generate trial configuration files.
  - Assembly tasks.
  - Combined tasks.
  - Faulty gripper challenge.
  - Human Operator challenge.
  - A functionality to address faulty parts is in progress.
- Documentation for features that come with this version has been provided.
- Documentation for features that are not currently supported will be provided in the final release.
- During the beta release, competitors are expected to:
 
  - Build a ROS2 package.
  - Understand the ARIAC interfaces. At a minimum, competitors must be capable of doing the following:
   
    - Move AGVs using the service or the velocity controllers.
    - Start and end the competition.
    - Retrieve information on part locations.
    - Receive and submit orders.
    - Move the robots to perform pick-and-place.
    - Understand trial configuration files and write custom ones.
    - Place sensors:
      
      - Sensor placement requires a good understanding of part locations and the tasks in ARIAC.

Improvements
------------

One of the goals of the beta release is to identify improvements that can be made to the interface so they can be addressed in the final release. Some bugs may still present in this release, so make sure to report the issues you find on the `ARIAC GitHub <https://github.com/usnistgov/ARIAC>`_ page.
