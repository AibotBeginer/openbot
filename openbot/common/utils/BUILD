load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")

cc_library(
    name = "common_blocking_queue",
    hdrs = ["blocking_queue.hpp"],
    visibility = ["//visibility:public"],
    deps = [
            "//openbot/common:common",
            ":common_time",
    ],
)

cc_library(
    name = "configuration_file_resolver",
    srcs = ["configuration_file_resolver.cc"],
    hdrs = ["configuration_file_resolver.hpp"],
    visibility = ["//visibility:public"],
    deps = [
            "//openbot/common:common",
            ":common_lua_parameter_dictionary",
    ],
)

cc_library(
    name = "common_utils_eigen_defs",
    hdrs = ["eigen_defs.hpp"],
    visibility = ["//visibility:public"],
    deps = [
            "@eigen",
    ],
)

cc_library(
    name = "common_utils_endian",
    srcs = ["endian.cc"],
    hdrs = ["endian.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        ":common_logging",
        "@boost.predef",
    ],
)

cc_test(
    name = "common_utils_endian_ut",
    srcs = [
        "endian_test.cc",
    ],
    deps = [
        ":common_utils_endian",
        "@googletest//:gtest_main",
    ],
)

cc_library(
    name = "common_utils_factory",
    hdrs = ["factory.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        "@glog",
    ],
)

cc_test(
    name = "common_utils_factory_ut",
    srcs = [
        "factory_test.cc",
    ],
    deps = [
        ":common_utils_factory",
        "@googletest//:gtest_main",
    ],
)

cc_library(
    name = "common_utils_file",
    srcs = ["file.cc"],
    hdrs = ["file.hpp"],
    visibility = ["//visibility:public"],
    deps = [
            ":common_utils_endian",
            ":common_logging",
    ],
    copts = ["-Wno-sign-compare", "-std=c++17"],
)

cc_test(
    name = "common_utils_file_ut",
    srcs = [
        "file_test.cc",
    ],
    deps = [
        ":common_utils_file",
        "@googletest//:gtest_main",
    ],
    copts = ["-std=c++17"],
)

cc_library(
    name = "common_json_utils",
    srcs = ["json_util.cc"],
    hdrs = ["json_util.hpp"],
    visibility = ["//visibility:public"],
    deps = [
            "@nlohmann_json//:json",
            "@protobuf",
            "@abseil-cpp//absl/strings",
    ],
)

cc_library(
    name = "common_logging",
    srcs = ["logging.cc"],
    hdrs = ["logging.hpp"],
    visibility = ["//visibility:public"],
    deps = [
            ":common_string_util",
            "@glog",
    ],
)

cc_test(
    name = "common_logging_ut",
    srcs = [
        "logging_test.cc",
    ],
    deps = [
        ":common_logging",
        "@googletest//:gtest_main",
    ],
)

cc_library(
    name = "common_util_lru_cache",
    hdrs = ["lru_cache.hpp"],
    visibility = ["//visibility:public"],
)

cc_test(
    name = "common_util_lru_cache_ut",
    srcs = [
        "lru_cache_test.cc",
    ],
    deps = [
        ":common_util_lru_cache",
        "@googletest//:gtest_main",
    ],
)

cc_library(
    name = "common_lua_parameter_dictionary",
    srcs = ["lua_parameter_dictionary.cc"],
    hdrs = ["lua_parameter_dictionary.hpp",
            "lua.hpp",
    ],
    visibility = ["//visibility:public"],
    copts = ["-Wno-sign-compare"],
    deps = [
            "@lua",
            "@glog",
            "//openbot/common:common",
            "//openbot/common/math:common_math",
    ],
)

cc_library(
    name = "common_message_util",
    hdrs = ["message_util.hpp"],
    visibility = ["//visibility:public"],
    deps = [
            "@protobuf",
            "@cyber",
            "@abseil-cpp//absl/strings:strings",
    ],
)

# cc_test(
#     name = "common_message_util_ut",
#     srcs = [
#         "message_util_test.cc",
#     ],
#     deps = [
#         ":common_message_util",
#         "@googletest//:gtest_main",
#     ],
# )

cc_library(
    name = "common_util_ply",
    srcs = ["ply.cc"],
    hdrs = ["ply.hpp"],
    visibility = ["//visibility:public"],
    deps = [
            "//openbot/common/math:common_math",
            ":common_utils_endian",
            ":common_utils_file",
            ":common_logging",
            "@eigen",
    ],
    copts = ["-std=c++17"],
)

cc_library(
    name = "common_string_util",
    srcs = ["string_util.cc"],
    hdrs = ["string_util.hpp",
            "future.hpp",
    ],
    visibility = ["//visibility:public"],
    deps = [
            "@abseil-cpp//absl/strings:strings",
            "@abseil-cpp//absl/strings:str_format",
    ],
)

cc_test(
    name = "common_string_util_ut",
    srcs = [
        "string_util_test.cc",
    ],
    deps = [
        ":common_string_util",
        "@googletest//:gtest_main",
    ],
)

cc_library(
    name = "common_util_task",
    srcs = [
        "task.cc",
        "thread_pool.cc",
        ],
    hdrs = ["task.hpp",
            "thread_pool.hpp",
    ],
    visibility = ["//visibility:public"],
    deps = [
            "@abseil-cpp//absl/strings:strings",
            "@abseil-cpp//absl/synchronization:synchronization",
            "@abseil-cpp//absl/container:flat_hash_map",
            "@glog",
    ],
)

cc_test(
    name = "common_util_task_ut",
    srcs = [
        "task_test.cc",
    ],
    deps = [
        ":common_util_task",
        "@googletest//:gtest_main",
    ],
)

cc_test(
    name = "common_util_thread_pool_ut",
    srcs = [
        "thread_pool_test.cc",
    ],
    deps = [
        ":common_util_task",
        "@googletest//:gtest_main",
    ],
)

cc_library(
    name = "common_time",
    srcs = ["time.cc",
            "time_util.cc"],
    hdrs = ["time.hpp",
            "time_util.hpp",
    ],
    visibility = ["//visibility:public"],
    deps = [
            "//openbot/common:common",
            "//openbot/common/math:common_math",
    ],
)

cc_test(
    name = "common_time_ut",
    srcs = [
        "time_util_test.cc",
    ],
    deps = [
        ":common_time",
        "@googletest//:gtest_main",
    ],
)

cc_library(
    name = "common_utils",
    srcs = ["util.cc"],
    hdrs = ["util.hpp"],
    visibility = ["//visibility:public"],
    deps = [
            "@cyber",
            "@openbot_bridge//openbot_bridge:openbot_bridge_cc_pb",
            "//openbot/common/math:common_math",
            "//openbot/common/configs:config_gflags",
    ],
)

cc_library(
    name = "common_utils_version",
    srcs = ["version.cc"],
    hdrs = ["version.hpp"],
    visibility = ["//visibility:public"],
    deps = [
            ":common_string_util",
    ],
)

