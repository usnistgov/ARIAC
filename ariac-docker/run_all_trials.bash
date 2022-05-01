#!/usr/bin/env bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Constants.
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
NOCOLOR='\033[0m'

TEAM_NAME=$1

trial_config_DIR=${DIR}/trial_config/

# Get the available trials from the config directory
get_list_of_trials()
{
  yaml_files=$(ls ${trial_config_DIR}/*.yaml)

  for f in $(ls ${trial_config_DIR}/*.yaml); do
    f=${f##*/}
    f=${f//.yaml}
    all_names="${all_names} ${f}"
  done

  echo $all_names
}

echo -e "
${GREEN}Running all trials for team: ${TEAM_NAME}${NOCOLOR}"

LIST_OF_TRIALS="$(get_list_of_trials)"

for TRIAL_NAME in ${LIST_OF_TRIALS}; do
  echo "Running trial: ${TRIAL_NAME}..."
  CONSOLE_OUTPUT_DIR=logs/${TEAM_NAME}/${TRIAL_NAME}
  mkdir -p ${CONSOLE_OUTPUT_DIR}
  ./run_trial.bash "${TEAM_NAME}" "${TRIAL_NAME}" > ${CONSOLE_OUTPUT_DIR}/output.txt 2>&1
  exit_status=$?
  if [ $exit_status -eq 0 ]; then
    echo -e "${GREEN}OK.${NOCOLOR}"
  else
    echo -e "${RED}TRIAL FAILED: ${TEAM_NAME}/${TRIAL_NAME}${NOCOLOR}" >&2
  fi
done
