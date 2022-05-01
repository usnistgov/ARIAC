FROM zeidk/ariac6-competitor-base-melodic:latest


COPY ./build_team_system.bash /
RUN chmod 755 /build_team_system.bash
RUN /build_team_system.bash


COPY ./run_team_system.bash /
RUN chmod 755 /run_team_system.bash
COPY ./run_team_system_with_delay.bash /
RUN chmod 755 /run_team_system_with_delay.bash

COPY ./competitor_entrypoint.sh /
RUN chmod 755 /competitor_entrypoint.sh
ENTRYPOINT ["/competitor_entrypoint.sh"]

# ENTRYPOINT ["/bin/bash"]
