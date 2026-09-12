px4_add_library(ModeOverlay ModeOverlay.cpp)
target_link_libraries(ModeOverlay PUBLIC motion_planning geo)
px4_add_unit_gtest(SRC ModeOverlayPolicyTest.cpp LINKLIBS ModeOverlay)
