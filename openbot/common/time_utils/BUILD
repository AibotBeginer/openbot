load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")

cc_library(
    name = "time_utils",
    srcs = glob(
    [
        "*.cc",
    ],
    exclude = [
        "*_test.cc",
    ],
    allow_empty = True),
    hdrs = glob([
                "*.h*",
    ], allow_empty = True),
    visibility = ["//visibility:public"],
)
