#include <gtest/gtest.h>
#include <systemc>
#include <sysc/datatypes/fx/sc_fixed.h>
#include <sysc/datatypes/fx/sc_ufixed.h>

#include <adptsysc/syscfx-utils.hh>

namespace adptsysc {

struct TestArch {
  using Fxpt_T = sc_dt::sc_fixed<16, 12>;
  static constexpr int fx_word_bits = 16;
  static constexpr int fx_integer_bits = 12;
  static constexpr int fx_frac_bits = 4;
  static constexpr bool fx_signed = true;
};

TEST(SyscFxUtils, FixedToBinaryWord) {
  sc_dt::sc_fixed<16, 12> x = -1.5;
  EXPECT_EQ(syscfx_to_binword(x), "1111111111101000");
}

TEST(SyscFxUtils, FixedToHexWord) {
  sc_dt::sc_fixed<16, 12> x = -1.5;
  EXPECT_EQ(syscfx_to_hexword(x), "ffe8");
}

TEST(SyscFxUtils, ArchValToHexWord) {
  EXPECT_EQ(archval_to_syscfx_hexword<TestArch>(1.25f), "0014");
  EXPECT_EQ(archval_to_syscfx_hexword<TestArch>(-1.25f), "ffec");
}

TEST(SyscFxUtils, HexWordToArchFx) {
  auto x = archsyscfx_from_hexword<TestArch>("0014");
  EXPECT_NEAR(static_cast<double>(x), 1.25, 1e-9);

  auto y = archsyscfx_from_hexword<TestArch>("ffec");
  EXPECT_NEAR(static_cast<double>(y), -1.25, 1e-9);
}

TEST(SyscFxUtils, BinWordToArchFx) {
  auto x = archsyscfx_from_binword<TestArch>("0000000000010100");
  EXPECT_NEAR(static_cast<double>(x), 1.25, 1e-9);

  auto y = archsyscfx_from_binword<TestArch>("1111111111101100");
  EXPECT_NEAR(static_cast<double>(y), -1.25, 1e-9);
}

}  // namespace adptsysc