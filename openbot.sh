#! /usr/bin/env bash
set -e

TOP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
source "${TOP_DIR}/scripts/openbot.bashrc"

ARCH="$(uname -m)"
SUPPORTED_ARCHS=" x86_64 aarch64 "
OPENBOT_VERSION="@non-git"
OPENBOT_ENV=""

: ${STAGE:=dev}

function check_architecture_support() 
{
    if [[ "${SUPPORTED_ARCHS}" != *" ${ARCH} "* ]]; then
        error "Unsupported CPU arch: ${ARCH}. Currently, Openbot only" \
            "supports running on the following CPU archs:"
        error "${TAB}${SUPPORTED_ARCHS}"
        exit 1
    fi
}

function check_platform_support() {
    local platform="$(uname -s)"
    if [ "$platform" != "Linux" ]; then
        error "Unsupported platform: ${platform}."
        error "${TAB}Openbot is expected to run on Linux systems (E.g., Debian/Ubuntu)."
        exit 1
    fi
}

function check_minimal_memory_requirement() {
    local minimal_mem_gb="2.0"
    local actual_mem_gb="$(free -m | awk '/Mem:/ {printf("%0.2f", $2 / 1024.0)}')"
    if (($(echo "$actual_mem_gb < $minimal_mem_gb" | bc -l))); then
        warning "System memory [${actual_mem_gb}G] is lower than the minimum required" \
            "[${minimal_mem_gb}G]. Openbot build could fail."
    fi
}


function check_openbot_version() 
{
    local branch="$(git_branch)"
    if [ "${branch}" == "${OPENBOT_VERSION}" ]; then
        return
    fi
    local sha1="$(git_sha1)"
    local stamp="$(git_date)"
    OPENBOT_VERSION="${branch}-${stamp}-${sha1}"
}

function openbot_env_setup() 
{
    check_openbot_version

    check_architecture_support
    check_platform_support
    check_minimal_memory_requirement
    determine_gpu_use_target

    OPENBOT_ENV="${OPENBOT_ENV} STAGE=${STAGE}"
    # Add more here ...

    touch "${OPENBOT_ROOT_DIR}/.openbot.bazelrc"

    info "Openbot Environment Settings:"
    info "${TAB}OPENBOT_ROOT_DIR: ${OPENBOT_ROOT_DIR}"
    info "${TAB}OPENBOT_CACHE_DIR: ${OPENBOT_CACHE_DIR}"
    info "${TAB}OPENBOT_IN_DOCKER: ${OPENBOT_IN_DOCKER}"
    info "${TAB}OPENBOT_VERSION: ${OPENBOT_VERSION}"
    if "${OPENBOT_IN_DOCKER}"; then
        info "${TAB}DOCKER_IMG: ${DOCKER_IMG##*:}"
    fi
    info "${TAB}OPENBOT_ENV: ${OPENBOT_ENV}"
    info "${TAB}USE_GPU: USE_GPU_HOST=${USE_GPU_HOST} USE_GPU_TARGET=${USE_GPU_TARGET}"

    if [ ! -f "${OPENBOT_ROOT_DIR}/.openbot.bazelrc" ]; then
        env ${OPENBOT_ENV} bash "${OPENBOT_ROOT_DIR}/scripts/openbot_config.sh" --noninteractive
    fi
}

#TODO(all): Update node modules
function build_dreamview_frontend() {
    pushd "${OPENBOT_ROOT_DIR}/modules/dreamview/frontend" >/dev/null
    yarn build
    popd >/dev/null
}

function build_test_and_lint() {
    env ${OPENBOT_ENV} bash "${build_sh}"
    env ${OPENBOT_ENV} bash "${test_sh}" --config=unit_test
    env ${OPENBOT_ENV} bash "${OPENBOT_ROOT_DIR}/scripts/openbot_lint.sh" cpp
    success "Build and Test and Lint finished."
}

function openbot_usage() {
    echo -e "\n${RED}Usage${NO_COLOR}:
    .${BOLD}/openbot.sh${NO_COLOR} [OPTION]"
    echo -e "\n${RED}Options${NO_COLOR}:
    ${BLUE}config [options]${NO_COLOR}: config bazel build environment either non-interactively (default) or interactively.
    ${BLUE}build [module]${NO_COLOR}: run build for cyber (<module> = cyber) or modules/<module>.  If <module> unspecified, build all.
    ${BLUE}build_dbg [module]${NO_COLOR}: run debug build.
    ${BLUE}build_opt [module]${NO_COLOR}: run optimized build.
    ${BLUE}build_cpu [module]${NO_COLOR}: build in CPU mode. Equivalent to 'bazel build --config=cpu'
    ${BLUE}build_gpu [module]${NO_COLOR}: run build in GPU mode. Equivalent to 'bazel build --config=gpu'
    ${BLUE}build_opt_gpu [module]${NO_COLOR}: optimized build in GPU mode. Equivalent to 'bazel build --config=opt --config=gpu'
    ${BLUE}test [module]${NO_COLOR}: run unittest for cyber (module='cyber') or modules/<module>. If unspecified, test all.
    ${BLUE}coverage [module]${NO_COLOR}: run coverage test for cyber (module='cyber') or modules/<module>. If unspecified, coverage all.
    ${BLUE}lint${NO_COLOR}: run code style check
    ${BLUE}buildify${NO_COLOR}: run 'buildifier' to fix style of bazel files.
    ${BLUE}check${NO_COLOR}: run build, test and lint on all modules. Recommmened before checking in new code.
    ${BLUE}build_fe${NO_COLOR}: compile frontend JS code for Dreamview. Requires all node_modules pre-installed.
    ${BLUE}build_teleop${NO_COLOR}: run build with teleop enabled.
    ${BLUE}build_prof [module]${NO_COLOR}: build with perf profiling support. Not implemented yet.
    ${BLUE}doc${NO_COLOR}: generate doxygen document
    ${BLUE}clean${NO_COLOR}: cleanup bazel output and log/coredump files
    ${BLUE}format${NO_COLOR}: format C++/Python/Bazel/Shell files
    ${BLUE}usage${NO_COLOR}: show this message and exit
    "
}

function main() {
    if [ "$#" -eq 0 ]; then
        openbot_usage
        exit 0
    fi

    openbot_env_setup

    local build_sh="${OPENBOT_ROOT_DIR}/scripts/openbot_build.sh"
    local test_sh="${OPENBOT_ROOT_DIR}/scripts/openbot_test.sh" 
    local coverage_sh="${OPENBOT_ROOT_DIR}/scripts/openbot_coverage.sh"
    local ci_sh="${OPENBOT_ROOT_DIR}/scripts/openbot_ci.sh"

    local cmd="$1"; 
    shift
    case "${cmd}" in
        config)
            env ${OPENBOT_ENV} bash "${OPENBOT_ROOT_DIR}/scripts/openbot_config.sh" "$@"
            ;;
        build)
            env ${OPENBOT_ENV} bash "${build_sh}" --config=cpu "$@"
            ;;
        build_opt)
            env ${OPENBOT_ENV} bash "${build_sh}" --config=opt "$@"
            ;;
        build_dbg)
            env ${OPENBOT_ENV} bash "${build_sh}" --config=dbg "$@"
            ;;
        build_cpu)
            env ${OPENBOT_ENV} bash "${build_sh}" --config=cpu "$@"
            ;;
        build_gpu)
            env ${OPENBOT_ENV} bash "${build_sh}" --config=gpu "$@"
            ;;
        build_opt_gpu)
            env ${OPENBOT_ENV} bash "${build_sh}" --config=opt --config=gpu "$@"
            ;;
        build_prof)
            env ${OPENBOT_ENV} bash "${build_sh}" --config=prof "$@"
            ;;
        build_teleop)
            env ${OPENBOT_ENV} bash "${build_sh}" --config=teleop "$@"
            ;;
        build_fe)
            build_dreamview_frontend
            ;;
        test)
            env ${OPENBOT_ENV} bash "${test_sh}" --config=unit_test "$@"
            ;;
        coverage)
            env ${OPENBOT_ENV} bash "${coverage_sh}" "$@"
            ;;
        cibuild)
            env ${OPENBOT_ENV} bash "${ci_sh}" "build"
            ;;
        citest)
            env ${OPENBOT_ENV} bash "${ci_sh}" "test"
            ;;
        cilint)
            env ${OPENBOT_ENV} bash "${ci_sh}" "lint"
            ;;
        check)
            build_test_and_lint
            ;;
        buildify)
            env ${OPENBOT_ENV} bash "${OPENBOT_ROOT_DIR}/scripts/openbot_buildify.sh"
            ;;
        lint)
            env ${OPENBOT_ENV} bash "${OPENBOT_ROOT_DIR}/scripts/openbot_lint.sh" "$@"
            ;;
        clean)
            env ${OPENBOT_ENV} bash "${OPENBOT_ROOT_DIR}/scripts/openbot_clean.sh" "$@"
            ;;
        doc)
            env ${OPENBOT_ENV} bash "${OPENBOT_ROOT_DIR}/scripts/openbot_docs.sh" "$@"
            ;;
        format)
            env ${OPENBOT_ENV} bash "${OPENBOT_ROOT_DIR}/scripts/openbot_format.sh" "$@"
            ;;
        usage)
            openbot_usage
            ;;
        -h|--help)
            openbot_usage
            ;;
        *)
            openbot_usage
            ;;
    esac
}

main "$@"
