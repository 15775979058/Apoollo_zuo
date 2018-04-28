load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "kvasercan_checker_lib",
    srcs = [
        "kvasercan_checker.cc",
        "kvasercan_test.cc",
    ],
    hdrs = glob(["*.h"]),
    deps = [
        "//modules/common:log",
        "//modules/common:macro",
        "//modules/common/util:string_util",
        "//modules/monitor/hardware:hardware_checker",
        "//third_party/can_card_library/kvaser_can",
    ],
)

cpplint()
