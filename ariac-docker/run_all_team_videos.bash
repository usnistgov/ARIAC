#!/usr/bin/env bash

set -e

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

trial_config_DIR=${DIR}/trial_config/
team_config_DIR=${DIR}/team_config/

LIST_OF_TEAMS="$(ls ${team_config_DIR})"

for TEAM_NAME in ${LIST_OF_TEAMS}; do
  ./run_all_trial_videos.bash "${TEAM_NAME}"
done
