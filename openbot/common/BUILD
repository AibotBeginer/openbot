load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")

cc_library(
    name = "common",
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
    deps = [
            # "@nlohmann_json//:json",
            # "@protobuf",
            # "@abseil-cpp",
            # "//openbot/common/proto:common_cc_pb",
            "@eigen",
            # "@abseil-cpp//absl/strings",
            # "//openbot/common/utils:common_utils",
            # "@cyber",
            # "@openbot_bridge//openbot_bridge:openbot_bridge_cc_pb",
            # "//openbot/common/helper_functions:helper_functions",
            # "//openbot/common/state_vector:state_vector",
            "@boost.iostreams",
            # "@glog",
            
    ],
)

# cc_test(
#     name = "ins_ut",
#     srcs = [
#         "ut/ins_test.cc",
#     ],
#     includes = [
#         ".",
#     ],
#     deps = [
#         ":ins_device",
#         "//base",
#         "@com_google_googletest//:gtest_main",
#         "@local//:protobuf",
#     ],
# )