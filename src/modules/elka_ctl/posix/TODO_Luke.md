# Fuzzy PD gains
## Files to edit
spektrum_serial_test.cpp:
  spektrum_serial_test_main():
    Start another thread (line 50)
    Manage thread (`start`,`stop`,`status`)

CMakeLists.txt:
  px4_add_module():
    Add files that you create to SRCS variable

## Files to create
fuzzy_controller.cpp/h:
  void fuzzy_controller(gains_s k):
    Implement fuzzy controller
    Read current non-fuzzy PD gains
