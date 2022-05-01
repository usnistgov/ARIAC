#!/bin/bash -x
set -e

# Constants.
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
NOCOLOR='\033[0m'

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

echo -e "${YELLOW}Preparing the ARIAC competition setup${NOCOLOR}"
${DIR}/ariac-server/build-images.sh
${DIR}/ariac-competitor/build_competitor_base_image.bash
