# load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")

# cc_library(
#     name = "optimization",
#     srcs = glob(
#     [
#         "*.cc",
#         "**/*.cc",
#     ],
#     exclude = [
#         "*_test.cc",
#         "**/*_test.cc",
#     ],
#     allow_empty = True),
#     hdrs = glob([
#                 "*.h*",
#                 "**/*.h*",
#     ], allow_empty = True),
#     visibility = ["//visibility:public"],
#     deps = [
#             # "@nlohmann_json//:json",
#             # "@protobuf",
#             # "@abseil-cpp",
#             # "//openbot/common/proto:common_cc_pb",
#             # "@eigen",
#             # "@abseil-cpp//absl/strings",
#             # "//openbot/common/utils:common_utils",
#             # "@cyber",
#             # "@openbot_bridge//openbot_bridge:openbot_bridge_cc_pb",
#             "//openbot/common/helper_functions:helper_functions",
#             # "//openbot/common/state_vector:state_vector",
#             "@osqp",
#             # "@qdldl",
#             # "@suitesparse//:config",
#             # "@suitesparse//:amd_hdrs",
#             # "@osqp-cpp//:osqp++",
#             "@glog",
            
#     ],
# )

# cc_test(
#     name = "more_thuente_line_search_ut",
#     srcs = [
#         "line_search/more_thuente_line_search_test.cc",
#     ],
#     # linkopts = [],
#     linkstatic = 1,
#     deps = [
#         ":optimization",
#         "@googletest//:gtest_main",
#         # "@osqp",
#         # "@qdldl",
#     ],
# )

# cc_test(
#     name = "cache_states_ut",
#     srcs = [
#         "cache_states_test.cc",
#     ],
#     includes = ["cache_states_test.hpp",],
#     linkstatic = 1,
#     deps = [
#         ":optimization",
#         "@googletest//:gtest_main",
#     ],
# )

# cc_test(
#     name = "csc_matrix_conv_ut",
#     srcs = [
#         "csc_matrix_conv_test.cc",
#     ],
#     linkstatic = 1,
#     deps = [
#         ":optimization",
#         "@googletest//:gtest_main",
#     ],
# )

# cc_test(
#     name = "newton_optimization_ut",
#     srcs = [
#         "newton_optimization_test.cc",
#     ],
#     includes = ["newton_optimization_test.hpp",],
#     linkstatic = 1,
#     deps = [
#         ":optimization",
#         "@googletest//:gtest_main",
#     ],
# )

# cc_test(
#     name = "osqp_interface_ut",
#     srcs = [
#         "osqp_interface_test.cc",
#     ],
#     linkstatic = 1,
#     deps = [
#         ":optimization",
#         "@googletest//:gtest_main",
#     ],
# )