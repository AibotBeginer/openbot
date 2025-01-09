load("@rules_proto//proto:defs.bzl", "proto_library")
load("@protobuf//bazel:cc_proto_library.bzl", "cc_proto_library")

cc_proto_library(
    name = "common_cc_pb",
    deps = [":common_pb"],
    visibility = ["//visibility:public"],
)

proto_library(
    name = "common_pb",
    srcs = glob(["*.proto"]),
)
