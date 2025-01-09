load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")

cc_library(
    name = "cv_bridge",
    srcs = glob([
        "*.cc",
    ], allow_empty = True),
    hdrs = glob([
                "*.h*",
    ], allow_empty = True),
    visibility = ["//visibility:public"],
    deps = [
            "@openbot_bridge//openbot_bridge:openbot_bridge_cc_pb",
    ],
    copts = ["-I/usr/include/opencv4"],
    linkopts = ["-L/usr/lib/x86_64-linux-gnu",
                "-L/usr/local/lib",
    ],
)
