^^^^^^^^^
Changelog
^^^^^^^^^

2025.3.2 (2026-01-29)
----------------------
* Fixed defective cell counting error


2025.3.1 (2026-01-29)
----------------------
* Added cheat for logging cell information and defects
* Updated the AGV tray spawning to handle extra cells on the tray
* Updated the vacuum tool plugin to fix the suction cups are in contact with different shells error
* Robot collision penalty fix to avoid too many penalties in a short amount of time

2025.3.0 (2026-01-09)
----------------------
* Refactored AGV system with improved tray handling and collision detection
* Added simulation recording capability for playback
* Updated physical inspection system logic
* Fixed crash in check_kit_quality service with invalid cells

2025.2.1 (2025-12-05)
----------------------


2025.2.0 (2025-12-05)
----------------------
* Added new service to AGVs to check the quality of the kit on the AGV

2025.1.2 (2025-09-26)
----------------------
* Now allows controlling cell feed if all orders are complete

2025.1.1 (2025-09-19)
----------------------
* Fixed attached vacuum tool causing robot collision penalties
* Top shell insertion is now random instead of at the same pose

2025.1.0 (2025-09-17)
----------------------
* Initial release