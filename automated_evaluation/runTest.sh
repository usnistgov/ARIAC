#!/bin/bash
export ARIAC_REPO_HOME=/home/ubuntu/ariac_ws/src/ariac/
cd $ARIAC_REPO_HOME
echo "BEGIN; Temp as of 2023/05/05. The local repo '${ARIAC_REPO_HOME}' in the provided Docker image is outdated, so synching with the remote. Also switching to the temp dev branch '${TEMP_DEV_BRANCH}'."
export TEMP_DEV_BRANCH=documentation_staging
git fetch origin && git checkout documentation_staging && git rebase origin/${TEMP_DEV_BRANCH}
echo "END: Temp as of 2023/05/05"
cd $ARIAC_REPO_HOME/automated_evaluation/autoEval
chmod +x runAll.sh
chmod +x runLaunch.sh
chmod +x read_competitor_config.py
./runAll.sh
