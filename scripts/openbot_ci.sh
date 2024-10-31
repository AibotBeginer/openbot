#! /usr/bin/env bash

###############################################################################
# Copyright 2024 The DuYongQuan Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
###############################################################################

set -e

TOP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
source "${TOP_DIR}/scripts/openbot.bashrc"
source "${TOP_DIR}/scripts/openbot_base.sh"
ARCH="$(uname -m)"

: ${USE_ESD_CAN:=false}

OPENBOT_BUILD_SH="${OPENBOT_ROOT_DIR}/scripts/openbot_build.sh"
OPENBOT_TEST_SH="${OPENBOT_ROOT_DIR}/scripts/openbot_test.sh"
OPENBOT_LINT_SH="${OPENBOT_ROOT_DIR}/scripts/openbot_lint.sh"

function run_ci_build() {
  env USE_ESD_CAN=${USE_ESD_CAN} bash "${OPENBOT_BUILD_SH}"
}

function run_ci_test() {
  env USE_ESD_CAN=${USE_ESD_CAN} bash "${OPENBOT_TEST_SH}" --config=unit_test
}

function run_ci_lint() {
  env USE_ESD_CAN=${USE_ESD_CAN} bash "${OPENBOT_LINT_SH}" cpp
}

function main() {
  local cmd="$1"
  if [ -z "${cmd}" ]; then
    cmd="ALL"
    info "Running ALL ..."
    run_ci_lint
    run_ci_build
    run_ci_test
  elif [ "${cmd}" == "test" ]; then
    info "Running CI Test ..."
    run_ci_test
  elif [ "${cmd}" == "build" ]; then
    info "Running CI Build ..."
    run_ci_build
  elif [ "${cmd}" == "lint" ]; then
    info "Running CI Lint ..."
    run_ci_lint
  fi
  success "ci ${cmd} finished."
}

main "$@"
