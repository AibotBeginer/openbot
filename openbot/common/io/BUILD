load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")

cc_library(
    name = "common_io",
    srcs = glob([
        "*.cc",
    ], allow_empty = True),
    hdrs = glob([
                "*.h*",
    ], allow_empty = True),
    visibility = ["//visibility:public"],
    deps = [
            "//openbot/common/proto:common_cc_pb",
            "//openbot/common/utils:common_json_utils",
    ],
)
