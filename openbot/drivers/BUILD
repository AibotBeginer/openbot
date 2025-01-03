load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_proto_library")
load("@rules_proto//proto:defs.bzl", "proto_library")
# load("//tools:cc_so_proto_rules.bzl", "cc_so_proto_library")



cc_library(
    name = "drivers_component",
    srcs = glob([
        #"components/*.cc",
        "sensor/*.cc",
        # "sensor/**/*.cc",
    ], allow_empty = True),
    hdrs = glob([
                #"components/*.hpp",
                "sensor/*.h*",
                "sensor/**/*.h*",
    ], allow_empty = True),
    visibility = ["//visibility:public"],
    deps = [
            "@cyber",
            ":drivers_config_cc_pb",
            "@openbot_bridge//openbot_bridge:openbot_bridge_cc_pb",
            "@eigen",
            "@lua",
    ],
    # copts = ["-I/usr/include/opencv4"],
    # linkopts = ["-L/usr/lib/x86_64-linux-gnu",
    #             "-L/usr/local/lib",
    # ],
)

cc_proto_library(
    name = "drivers_config_cc_pb",
    deps = [":drivers_config_pb"],
)

proto_library(
    name = "drivers_config_pb",
    srcs = glob(["proto/*.proto"]),
)
