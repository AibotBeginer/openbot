cmake_minimum_required(VERSION 3.10)
project(openbot_benchmarks)

find_package(openbot REQUIRED)
find_package(benchmark REQUIRED)

add_executable(benchmark_cost_functions cost_functions.cc)
target_link_libraries(benchmark_cost_functions PRIVATE openbot::openbot benchmark::benchmark)
