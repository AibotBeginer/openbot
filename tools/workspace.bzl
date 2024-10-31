load("@bazel_tools//tools/build_defs/repo:http.bzl", _http_archive = "http_archive")
load("@bazel_tools//tools/build_defs/repo:utils.bzl", "maybe")

# Openbot external dependencies that can be loaded in WORKSPACE files.
load("//3rdparty/absl:workspace.bzl", absl = "repo")
load("//3rdparty/adolc:workspace.bzl", adolc = "repo")
load("//3rdparty/benchmark:workspace.bzl", benchmark = "repo")
load("//3rdparty/boost:workspace.bzl", boost = "repo")
load("//3rdparty/cpplint:workspace.bzl", cpplint = "repo")
load("//3rdparty/catch2:workspace.bzl", catch2 = "repo")
load("//3rdparty/eigen3:workspace.bzl", eigen = "repo")
load("//3rdparty/ffmpeg:workspace.bzl", ffmpeg = "repo")
load("//3rdparty/fftw3:workspace.bzl", fftw3 = "repo")
load("//3rdparty/fastrtps:workspace.bzl", fastrtps = "repo")
load("//3rdparty/glog:workspace.bzl", glog = "repo")
load("//3rdparty/gtest:workspace.bzl", gtest = "repo")
load("//3rdparty/gflags:workspace.bzl", gflags = "repo")
load("//3rdparty/ipopt:workspace.bzl", ipopt = "repo")
load("//3rdparty/nlohmann_json:workspace.bzl", nlohmann_json = "repo")
load("//3rdparty/opencv:workspace.bzl", opencv = "repo")
load("//3rdparty/osqp:workspace.bzl", osqp = "repo")
load("//3rdparty/poco:workspace.bzl", poco = "repo")
load("//3rdparty/proj:workspace.bzl", proj = "repo")
load("//3rdparty/protobuf:workspace.bzl", protobuf = "repo")
load("//3rdparty/tinyxml2:workspace.bzl", tinyxml2 = "repo")
load("//3rdparty/uuid:workspace.bzl", uuid = "repo")
load("//3rdparty/yaml_cpp:workspace.bzl", yaml_cpp = "repo")
# load("//3rdparty/glew:workspace.bzl", glew = "repo")
# load("//3rdparty/pcl:pcl_configure.bzl", "pcl_configure")

def initialize_3rdparty():
    """ Load third party repositories.  See above load() statements. """

    absl()
    adolc()
    benchmark()
    boost()
    cpplint()
    catch2()
    eigen()
    fastrtps()
    ffmpeg()
    fftw3()
    gflags()
    glog()
    gtest()
    ipopt()
    nlohmann_json()
    opencv()
    osqp()
    poco()
    proj()
    protobuf()
    tinyxml2()
    uuid()
    yaml_cpp()

# Define all external repositories required by
def openbot_repositories():
    initialize_3rdparty()

# Wrap http_archive with maybe so we don't try to declare a dependency twice
def http_archive(**kwargs):
    maybe(_http_archive, **kwargs)
