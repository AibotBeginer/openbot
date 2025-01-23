load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")

cc_library(
    name = "covariance_insertion",
    srcs = glob([
        "*.cc",
    ], allow_empty = True),
    hdrs = glob([
                "*.h*",
    ], allow_empty = True),
    visibility = ["//visibility:public"],
    deps = [
            "@gflags",
            "//openbot/common/proto:common_cc_pb",
            "//openbot/common/helper_functions:helper_functions",
    ],
)
