#include <adptsysc/dsplib.hh>

#include <gtest/gtest.h>
#include <stdexcept>

#include <string>

using namespace adptsysc;

int adptsysc_test_main(const std::vector<std::string>& arguments) {
  return RUN_ALL_TESTS();
}

int main(int argc, char** argv) {
  try {
    std::vector<std::string> arguments;
    for (int i = 0; i < argc; ++i) {
      arguments.emplace_back(argv[i]);
    }
    testing::InitGoogleTest(&argc, argv);
    return adptsysc_test_main(arguments);
  } catch (const std::exception& e) {
    std::cerr << "Uncaught exception:" << std::string{e.what()} << std::endl;
    return 1;
  }
}
