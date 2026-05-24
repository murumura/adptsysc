#include <gtest/gtest.h>
#include <systemc>
#include <sysc/datatypes/fx/sc_fixed.h>
#include <sysc/datatypes/fx/sc_ufixed.h>

#include <adptsysc/syscfx-utils.hh>

// Dummy SystemC entry point required because the test binary links libsystemc.
// GoogleTest still uses gtest_main as the real process main().
int sc_main(int argc, char* argv[]) {
  (void)argc;
  (void)argv;
  return 0;
}

namespace adptsysc {

struct TestArch {
  using Fxpt_T = sc_dt::sc_fixed<16, 12>;

  static constexpr int fx_word_bits = 16;
  static constexpr int fx_integer_bits = 12;
  static constexpr int fx_frac_bits = 4;
  static constexpr bool fx_signed = true;
};

struct TestUnsignedArch {
  using Fxpt_T = sc_dt::sc_ufixed<8, 4>;

  static constexpr int fx_word_bits = 8;
  static constexpr int fx_integer_bits = 4;
  static constexpr int fx_frac_bits = 4;
  static constexpr bool fx_signed = false;
};

struct TestSmallArch {
  using Fxpt_T = sc_dt::sc_fixed<8, 4>;

  static constexpr int fx_word_bits = 8;
  static constexpr int fx_integer_bits = 4;
  static constexpr int fx_frac_bits = 4;
  static constexpr bool fx_signed = true;
};

// -----------------------------------------------------------------------------
// Existing basic tests
// -----------------------------------------------------------------------------

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

// -----------------------------------------------------------------------------
// Prefix / point cleanup
// -----------------------------------------------------------------------------

TEST(SyscFxUtils, StripSyscRadixPrefix) {
  EXPECT_EQ(strip_sysc_radix_prefix("0b1010"), "1010");
  EXPECT_EQ(strip_sysc_radix_prefix("0B1010"), "1010");
  EXPECT_EQ(strip_sysc_radix_prefix("0x00ff"), "00ff");
  EXPECT_EQ(strip_sysc_radix_prefix("0X00ff"), "00ff");
  EXPECT_EQ(strip_sysc_radix_prefix("1234"), "1234");
}

TEST(SyscFxUtils, StripSyscPoint) {
  EXPECT_EQ(strip_sysc_point("1111.0000"), "11110000");
  EXPECT_EQ(strip_sysc_point("0014"), "0014");
  EXPECT_EQ(strip_sysc_point(""), "");
}

TEST(SyscFxUtils, KeepsPrefixWhenRequested) {
  sc_dt::sc_fixed<8, 4> x = 1.5;

  const std::string bin = syscfx_to_binword(x, true);
  const std::string hex = syscfx_to_hexword(x, true);

  EXPECT_NE(bin.find("0b"), std::string::npos);
  EXPECT_NE(hex.find("0x"), std::string::npos);
}

// -----------------------------------------------------------------------------
// More signed fixed-point word conversion
// Q12.4: fixed integer = value * 16
// -----------------------------------------------------------------------------

TEST(SyscFxUtils, SignedFixedHexWordMoreValues) {
  EXPECT_EQ(archval_to_syscfx_hexword<TestArch>(0.0f), "0000");
  EXPECT_EQ(archval_to_syscfx_hexword<TestArch>(1.0f), "0010");
  EXPECT_EQ(archval_to_syscfx_hexword<TestArch>(2.0f), "0020");
  EXPECT_EQ(archval_to_syscfx_hexword<TestArch>(7.5f), "0078");

  EXPECT_EQ(archval_to_syscfx_hexword<TestArch>(-1.0f), "fff0");
  EXPECT_EQ(archval_to_syscfx_hexword<TestArch>(-2.0f), "ffe0");
  EXPECT_EQ(archval_to_syscfx_hexword<TestArch>(-7.5f), "ff88");
}

TEST(SyscFxUtils, SignedFixedBinWordMoreValues) {
  EXPECT_EQ(archval_to_syscfx_binword<TestArch>(0.0f),
            "0000000000000000");

  EXPECT_EQ(archval_to_syscfx_binword<TestArch>(1.0f),
            "0000000000010000");

  EXPECT_EQ(archval_to_syscfx_binword<TestArch>(-1.0f),
            "1111111111110000");

  EXPECT_EQ(archval_to_syscfx_binword<TestArch>(-1.5f),
            "1111111111101000");
}

// -----------------------------------------------------------------------------
// Round-trip conversion
// -----------------------------------------------------------------------------

TEST(SyscFxUtils, HexRoundTripSigned) {
  const std::vector<double> values = {
      0.0,
      0.25,
      1.0,
      1.25,
      7.5,
      -0.25,
      -1.0,
      -1.25,
      -7.5,
  };

  for (double v : values) {
    const std::string word = archval_to_syscfx_hexword<TestArch>(v);
    auto q = archsyscfx_from_hexword<TestArch>(word);

    EXPECT_NEAR(static_cast<double>(q), v, 1e-9)
        << "word=" << word << " value=" << v;
  }
}

TEST(SyscFxUtils, BinRoundTripSigned) {
  const std::vector<double> values = {
      0.0,
      0.25,
      1.0,
      1.25,
      7.5,
      -0.25,
      -1.0,
      -1.25,
      -7.5,
  };

  for (double v : values) {
    const std::string word = archval_to_syscfx_binword<TestArch>(v);
    auto q = archsyscfx_from_binword<TestArch>(word);

    EXPECT_NEAR(static_cast<double>(q), v, 1e-9)
        << "word=" << word << " value=" << v;
  }
}

TEST(SyscFxUtils, HexInputAcceptsPrefix) {
  auto x = archsyscfx_from_hexword<TestArch>("0x0014");
  EXPECT_NEAR(static_cast<double>(x), 1.25, 1e-9);

  auto y = archsyscfx_from_hexword<TestArch>("0xffec");
  EXPECT_NEAR(static_cast<double>(y), -1.25, 1e-9);
}

TEST(SyscFxUtils, BinInputAcceptsPrefix) {
  auto x = archsyscfx_from_binword<TestArch>("0b0000000000010100");
  EXPECT_NEAR(static_cast<double>(x), 1.25, 1e-9);

  auto y = archsyscfx_from_binword<TestArch>("0b1111111111101100");
  EXPECT_NEAR(static_cast<double>(y), -1.25, 1e-9);
}

// -----------------------------------------------------------------------------
// Unsigned fixed-point behavior
// TestUnsignedArch is Q4.4 unsigned.
// -----------------------------------------------------------------------------

TEST(SyscFxUtils, UnsignedFixedHexWord) {
  EXPECT_EQ(archval_to_syscfx_hexword<TestUnsignedArch>(0.0f), "00");
  EXPECT_EQ(archval_to_syscfx_hexword<TestUnsignedArch>(1.0f), "10");
  EXPECT_EQ(archval_to_syscfx_hexword<TestUnsignedArch>(1.25f), "14");
  EXPECT_EQ(archval_to_syscfx_hexword<TestUnsignedArch>(15.5f), "f8");
}

TEST(SyscFxUtils, UnsignedHexWordToArchFx) {
  auto x = archsyscfx_from_hexword<TestUnsignedArch>("14");
  EXPECT_NEAR(static_cast<double>(x), 1.25, 1e-9);

  auto y = archsyscfx_from_hexword<TestUnsignedArch>("f8");
  EXPECT_NEAR(static_cast<double>(y), 15.5, 1e-9);
}

TEST(SyscFxUtils, UnsignedBinWordToArchFx) {
  auto x = archsyscfx_from_binword<TestUnsignedArch>("00010100");
  EXPECT_NEAR(static_cast<double>(x), 1.25, 1e-9);

  auto y = archsyscfx_from_binword<TestUnsignedArch>("11111000");
  EXPECT_NEAR(static_cast<double>(y), 15.5, 1e-9);
}

// -----------------------------------------------------------------------------
// Smaller signed format
// TestSmallArch is Q4.4 signed.
// -----------------------------------------------------------------------------

TEST(SyscFxUtils, SmallSignedHexWord) {
  EXPECT_EQ(archval_to_syscfx_hexword<TestSmallArch>(1.25f), "14");
  EXPECT_EQ(archval_to_syscfx_hexword<TestSmallArch>(-1.25f), "ec");
  EXPECT_EQ(archval_to_syscfx_hexword<TestSmallArch>(-1.5f), "e8");
}

TEST(SyscFxUtils, SmallSignedRoundTrip) {
  auto x = archsyscfx_from_hexword<TestSmallArch>("14");
  EXPECT_NEAR(static_cast<double>(x), 1.25, 1e-9);

  auto y = archsyscfx_from_hexword<TestSmallArch>("ec");
  EXPECT_NEAR(static_cast<double>(y), -1.25, 1e-9);

  auto z = archsyscfx_from_binword<TestSmallArch>("11101000");
  EXPECT_NEAR(static_cast<double>(z), -1.5, 1e-9);
}

// -----------------------------------------------------------------------------
// Generic arch fixed-point quantization helpers
// -----------------------------------------------------------------------------

TEST(SyscFxUtils, ScalarFromArchSyscFx) {
  EXPECT_NEAR(scalar_from_archsyscfx<TestArch>(1.25f), 1.25f, 1e-6f);
  EXPECT_NEAR(scalar_from_archsyscfx<TestArch>(-1.25f), -1.25f, 1e-6f);

  // Q12.4 resolution is 1/16 = 0.0625.
  // Default sc_fixed quantization truncates toward zero unless your Fxpt_T
  // specifies a different quantization mode.
  EXPECT_NEAR(scalar_from_archsyscfx<TestArch>(1.20f), 1.1875f, 1e-6f);
}

TEST(SyscFxUtils, ComplexFromArchSyscFx) {
  std::complex<float> z(1.25f, -1.25f);
  auto q = cmplx_from_archsyscfx<TestArch>(z);

  EXPECT_NEAR(q.real(), 1.25f, 1e-6f);
  EXPECT_NEAR(q.imag(), -1.25f, 1e-6f);
}

TEST(SyscFxUtils, VectorFromArchSyscFx) {
  std::vector<float> in = {
      0.0f,
      1.25f,
      -1.25f,
      1.20f,
  };

  auto q = vec_from_archsyscfx<TestArch>(in);

  ASSERT_EQ(q.size(), in.size());
  EXPECT_NEAR(q[0], 0.0f, 1e-6f);
  EXPECT_NEAR(q[1], 1.25f, 1e-6f);
  EXPECT_NEAR(q[2], -1.25f, 1e-6f);
  EXPECT_NEAR(q[3], 1.1875f, 1e-6f);
}

TEST(SyscFxUtils, ComplexVectorFromArchSyscFx) {
  std::vector<std::complex<float>> in = {
      {1.25f, -1.25f},
      {0.50f, -0.50f},
      {1.20f, -1.20f},
  };

  auto q = cmplxvec_from_archsyscfx<TestArch>(in);

  ASSERT_EQ(q.size(), in.size());

  EXPECT_NEAR(q[0].real(), 1.25f, 1e-6f);
  EXPECT_NEAR(q[0].imag(), -1.25f, 1e-6f);

  EXPECT_NEAR(q[1].real(), 0.50f, 1e-6f);
  EXPECT_NEAR(q[1].imag(), -0.50f, 1e-6f);

  EXPECT_NEAR(q[2].real(), 1.1875f, 1e-6f);
  EXPECT_NEAR(q[2].imag(), -1.1875f, 1e-6f);
}

// -----------------------------------------------------------------------------
// Width sanity
// -----------------------------------------------------------------------------

TEST(SyscFxUtils, WordLengthsMatchArchWidth) {
  EXPECT_EQ(archval_to_syscfx_binword<TestArch>(1.25f).size(),
            static_cast<std::size_t>(TestArch::fx_word_bits));

  EXPECT_EQ(archval_to_syscfx_binword<TestUnsignedArch>(1.25f).size(),
            static_cast<std::size_t>(TestUnsignedArch::fx_word_bits));

  EXPECT_EQ(archval_to_syscfx_binword<TestSmallArch>(1.25f).size(),
            static_cast<std::size_t>(TestSmallArch::fx_word_bits));
}

TEST(SyscFxUtils, HexWordLengthsMatchArchWidthRoundedUp) {
  EXPECT_EQ(archval_to_syscfx_hexword<TestArch>(1.25f).size(), 4u);
  EXPECT_EQ(archval_to_syscfx_hexword<TestUnsignedArch>(1.25f).size(), 2u);
  EXPECT_EQ(archval_to_syscfx_hexword<TestSmallArch>(1.25f).size(), 2u);
}

}  // namespace adptsysc