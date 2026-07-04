# ESP-IDF component for the STM32 motor-controller UART link.
#
# trainsonic_link.h (the shared comm table) lives in the STM32 repo under
# protocol/. Make it reachable one of two ways:
#   1) add its path to INCLUDE_DIRS below (adjust the relative path to wherever
#      the protocol/ dir sits relative to this component), or
#   2) drop a copy of trainsonic_link.h next to ts_link.c (simplest, but keep it
#      in sync -- prefer a submodule/shared path).

idf_component_register(
    SRCS        "ts_link.c"
    INCLUDE_DIRS "."
    # INCLUDE_DIRS "." "${CMAKE_CURRENT_LIST_DIR}/../../../<path-to>/protocol"
    REQUIRES    driver esp_timer
)
