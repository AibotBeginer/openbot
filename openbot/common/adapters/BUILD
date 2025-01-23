load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("@rules_proto//proto:defs.bzl", "proto_library")
load("@protobuf//bazel:cc_proto_library.bzl", "cc_proto_library")

cc_library(
    name = "adapter_gflags",
    srcs = glob([
        "*.cc",
    ], allow_empty = True),
    hdrs = glob([
                "*.h*",
    ], allow_empty = True),
    visibility = ["//visibility:public"],
    deps = [
            ":adapter_config_cc_pb",
            "@gflags",
    ],
)

cc_proto_library(
    name = "adapter_config_cc_pb",
    deps = [":adapter_config_pb"],
)

proto_library(
    name = "adapter_config_pb",
    srcs = glob(["proto/*.proto"]),
)
