#include <adptsysc/dsplib.hh>
#ifdef DBUG_MODE
  #include <iostream>
  #include <iomanip>
#endif
namespace adptsysc {

template cfloat 
prod(const std::vector<cfloat>& vec, const cfloat v);

std::ostream& operator<<(std::ostream& os, const Zpk& zpk) {
  using c = std::complex<float>;
  
  std::ios_base::fmtflags orig = os.flags();
  os << std::fixed << std::setprecision(6);
  
  os << "Gain: " << zpk.k << "\n";
  
  os << "Zeros (" << zpk.zeros.size() << "):\n";
  for (std::size_t i = 0; i < zpk.zeros.size(); ++i) {
      os << "  z[" << i << "] = " << std::setw(10) << zpk.zeros[i].real() 
          << " + " << std::setw(10) << zpk.zeros[i].imag() << "j";
      if (std::isinf(zpk.zeros[i].real())) os << " (INFINITE)";
      os << "\n";
  }
  
  os << "Poles (" << zpk.poles.size() << "):\n";
  for (std::size_t i = 0; i < zpk.poles.size(); ++i) {
      os << "  p[" << i << "] = " << std::setw(10) << zpk.poles[i].real() 
          << " + " << std::setw(10) << zpk.poles[i].imag() << "j";
      if (std::isinf(zpk.poles[i].real())) os << " (INFINITE)";
      os << "\n";
  }
  
  os.flags(orig);
  return os;
}

#ifdef DBUG_MODE
  #define DBUG_LOG(msg) std::cout << "[DEBUG] " << msg << std::endl
  #define DBUG_VAR(var) std::cout << "[DEBUG] " << #var << " = " << (var) << std::endl
  #define DBUG_COMPLEX(var) std::cout << "[DEBUG] " << #var << " = " << (var).real() << " + " << (var).imag() << "j" << std::endl
  #define DBUG_SECTION(title) std::cout << "\n[DEBUG] === " << title << " ===\n"
  #define DBUG_FILTER_STATE(zpk) std::cout << "[DEBUG] Filter State:\n" << (zpk) << std::endl
#else
  #define DBUG_LOG(msg)
  #define DBUG_VAR(var)
  #define DBUG_COMPLEX(var)
  #define DBUG_SECTION(title)
  #define DBUG_FILTER_STATE(zpk)
#endif

template float 
apply_biquad_sample<float>(IirState<float>& filt, float x);

template cfloat 
apply_biquad_sample<cfloat>(IirState<cfloat>& filt, cfloat x);

template void 
apply_biquad_block<float>(IirState<float>&, std::span<float> data);

template void 
apply_biquad_block<float>(IirState<float>&, std::vector<float>& data);

template void 
apply_biquad_block<cfloat>(IirState<cfloat>&, std::span<cfloat> data);

template void 
apply_biquad_block<cfloat>(IirState<cfloat>&, std::vector<cfloat>& data);


template BiquadSection<float> 
zpk_to_biquad (const BiquadPair& pairs, const float k);

template BiquadSection<cfloat> 
zpk_to_biquad (const BiquadPair& pairs, const float k);


std::vector<cfloat> 
pair_conjugates(const std::vector<cfloat>& list) {
  if (list.empty()) {
    return {};
  }

  auto x = list;
  // Sort by real part, then by absolute imaginary part for consistency
  std::sort(x.begin(), x.end(),[](const cfloat& a, const cfloat& b) {
    if (a.real() != b.real()) {
      return a.real() < b.real();
    }
    // Secondary sort by absolute imaginary part to group conjugates
    return std::abs(a.imag()) < std::abs(b.imag());
  });

  constexpr float tol = std::numeric_limits<float>::epsilon() * 100.f;

  auto almost_conj = [&](const cfloat& a, const cfloat& b) {
    // Check if they are complex conjugates
    return (std::abs(a.real() - b.real()) < tol &&
            std::abs(a.imag() + b.imag()) < tol);
  };

  auto almost_real = [&](const cfloat& a) {
    return std::abs(a.imag()) < tol;
  };

  auto almost_complex = [&](const cfloat& a) {
    return std::abs(a.imag()) >= tol;
  };

  std::vector<cfloat> output;

  for (std::size_t i = 0; i < x.size();) {
    if (almost_real(x[i])) {
      // Real root - keep as is
      output.push_back(x[i]);
      i += 1;
    } else if (i + 1 < x.size() && almost_complex(x[i]) && almost_conj(x[i], x[i + 1])) {
      // Complex conjugate pair found
      // Keep the one with positive imaginary part, 
      // ensure consistent representation
      cfloat root = (x[i].imag() >= 0) ? x[i] : x[i + 1];
      output.push_back(root);
      i += 2; // Skip both
    } else {
      // Unpaired complex root 
      // shouldn't happen with proper input
      // but handle gracefully :)
      output.push_back(x[i]);
      i += 1;
    }
  }

  return output;
}

std::size_t 
get_nearest_root(const std::vector<cfloat>& list, 
                const cfloat& val, bool must_real) {

  constexpr float tol = std::numeric_limits<float>::epsilon() * 100.f;
  std::size_t best_idx = std::numeric_limits<std::size_t>::max();
  float best_dist = std::numeric_limits<float>::max();

  auto almost_real = [&](const cfloat& a) {
    return std::abs(a.imag()) < tol;
  };

  for (std::size_t i = 0; i < list.size(); ++i) {
    if (almost_real(list[i]) == must_real) {
      float dist = std::abs(val - list[i]);
      if (dist < best_dist) {
        best_dist = dist;
        best_idx = i;
      }
    }
  }

  if (best_idx == std::numeric_limits<std::size_t>::max()) {
    std::ostringstream oss;
    oss << "get_nearest_root: no matching root found for value ("
        << val.real() << " + " << val.imag() << "j)"
        << " with must_real=" << must_real << "\n";

    oss << "Candidate roots (" << list.size() << "): [";
    for (std::size_t i = 0; i < list.size(); ++i) {
      oss << "(" << list[i].real() << " + " << list[i].imag() << "j)";
      if (i + 1 < list.size()) oss << ", ";
    }
    oss << "]";
    throw std::runtime_error(oss.str());
  }

  return best_idx;
}

template <Number T>
IirParams<T> 
zpk_to_sos(Zpk& filter) {
  constexpr float tol = std::numeric_limits<float>::epsilon() * 100.f;

  auto count_real = [&](const std::vector<cfloat>& list) {
    return std::count_if(list.begin(), list.end(),
        [&](const cfloat& e) { return std::abs(e.imag()) < tol; });
  };

  // trivial case: no poles, no zeros
  if (filter.poles.empty() && filter.zeros.empty()) {
    return {BiquadSection<T>(filter.k, T(0), T(0), T(1), T(0), T(0))};
  }

  // Balance lengths by ensures both zeros and poles have 
  // the same length by padding with zeros 
  // (extra roots at the origin).
  std::size_t length = std::max(filter.poles.size(), filter.zeros.size());
  filter.poles.resize(length, {0, 0});
  filter.zeros.resize(length, {0, 0});

  // If odd, pad with zero root for both
  if (length & 1) {
    filter.poles.push_back({0, 0});
    filter.zeros.push_back({0, 0});
    ++length;
  }

  // Enforce conjugate pairing by keeping the one with
  // postive imaginary part left and real or non-pair complex
  // are kept as it is.
  // Note that once we go throgh belows code 
  // if found one root being complex such as a+bj (the one that left)
  // we could always making sure its conj a-bj exist
  filter.zeros = pair_conjugates(filter.zeros);
  filter.poles = pair_conjugates(filter.poles);
  
  std::size_t n_sections = length / 2;
  std::vector<BiquadPair> pairs(n_sections);

  // pick pole closest to unit circle and erase it from original list
  auto get_nearest_circle_pole = [](std::vector<cfloat>& poles) -> cfloat {
    if (poles.empty()) {
      throw std::runtime_error("No poles available");
    }
    auto best_it = poles.begin();
    float best_val = std::abs(std::abs(*best_it) - 1.0f);
    for (auto it = poles.begin() + 1; it != poles.end(); ++it) {
      float val = std::abs(std::abs(*it) - 1.0f);
      if (val < best_val) {
        best_val = val;
        best_it = it;
      }
    }
    cfloat p = *best_it;
    poles.erase(best_it);
    return p;
  };

  // Find nearest zero regardless of type
  auto get_nearest_zero_any = [](const std::vector<cfloat>& zeros, const cfloat& val) -> size_t {
    if (zeros.empty()) {
      throw std::runtime_error("No zeros available");
    }
    std::size_t best_idx = 0;
    float best_dist = std::abs(val - zeros[0]);
    for (std::size_t i = 1; i < zeros.size(); ++i) {
      float dist = std::abs(val - zeros[i]);
      if (dist < best_dist) {
        best_dist = dist;
        best_idx = i;
      }
    }
    return best_idx;
  };


  auto almost_real = [&](const cfloat& a) {
    return std::abs(a.imag()) < tol;
  };

  // Build pole/zero pairs for each SOS
  // For stability reasons, choose poles with magnitude closest to 1 
  // ("worst" pole) first. Then find a matching zero 
  // (either real or complex, nearest in distance).
  // If pole is complex, pair it with its conjugate; same for zero.
  // If both are real, just pair two reals.
  // Each section ends up with (p1,p2,z1,z2).
  for (std::size_t si = 0; si < n_sections; ++si) {
    cfloat p1 = get_nearest_circle_pole(filter.poles);
    cfloat p2 {0, 0}, z1 {0, 0}, z2 {0, 0};

    if (almost_real(p1) && count_real(filter.poles) == 0) {
      // Lone real pole, match with nearest real zero
      std::size_t z1_idx = get_nearest_root(filter.zeros, p1, true);
      z1 = filter.zeros[z1_idx];
      filter.zeros.erase(filter.zeros.begin() + z1_idx);
      p2 = z2 = {0, 0};  // First-order section
    } else {
      std::size_t z1_idx;
      if (!almost_real(p1) && count_real(filter.zeros) == 1) {
        // Complex pole and exactly one real zero remaining - force complex zero
        z1_idx = get_nearest_root(filter.zeros, p1, false);  // MUST be complex
      } else {
        // Pick nearest zero regardless of type
        z1_idx = get_nearest_zero_any(filter.zeros, p1);     // Any type allowed
      }
      z1 = filter.zeros[z1_idx];
      filter.zeros.erase(filter.zeros.begin() + z1_idx);

      if (!almost_real(p1)) {
        // Complex pole - automatically get conjugate
        p2 = cfloat(p1.real(), -p1.imag());
        if (!almost_real(z1)) {
          // Complex zero - automatically get conjugate
          z2 = cfloat(z1.real(), -z1.imag());
        } else {
          // Real zero - find another real zero
          std::size_t z2_idx = get_nearest_root(filter.zeros, p1, true);
          z2 = filter.zeros[z2_idx];
          filter.zeros.erase(filter.zeros.begin() + z2_idx);
        }
      } else {
        // Real pole
        if (!almost_real(z1)) {
          // Complex zero - automatically get conjugate
          z2 = cfloat(z1.real(), -z1.imag());
          // Find another real pole
          std::size_t p2_idx = get_nearest_root(filter.poles, p1, true);
          p2 = filter.poles[p2_idx];
          filter.poles.erase(filter.poles.begin() + p2_idx);
        } else {
          // Real zero - find another real pole and real zero
          std::size_t p2_idx = get_nearest_root(filter.poles, p1, true);
          p2 = filter.poles[p2_idx];
          filter.poles.erase(filter.poles.begin() + p2_idx);
          
          std::size_t z2_idx = get_nearest_root(filter.zeros, p2, true);
          z2 = filter.zeros[z2_idx];
          filter.zeros.erase(filter.zeros.begin() + z2_idx);
        }
      }
    }

    pairs[si] = {p1, p2, z1, z2};
  }
  
  // Finally convert each pair into SOS biquad
  // Each (z1,z2,p1,p2) pair is converted to polynomial 
  // coefficients (numerator and denominator).
  // First section carries the gain k; all others get gain = 1.
  // They reverse the order (n_sections - 1 - si) 
  // so that numerically worst sections appear last (common trick for stability).
  IirParams<T> result(n_sections);
  for (std::size_t si = 0; si < n_sections; ++si) {
    float gain = (si == 0 ? filter.k : 1.0f);
    result.sections[si] = zpk_to_biquad<T>(pairs[n_sections - 1 - si], gain);
  }

  return result;
}

template IirParams<float>  adptsysc::zpk_to_sos<float>(adptsysc::Zpk&);
template IirParams<cfloat> adptsysc::zpk_to_sos<cfloat>(adptsysc::Zpk&);

float f_prewarp(float freq, float fs) {
  freq = 2 * freq / fs;
  fs = 2.0f;
  float f_warp = 2 * fs * std::tan(kPi * freq / fs);
  return f_warp;
}

Zpk bilinear(const Zpk& filter, const float fs) {
  using T = cfloat;
  const float fs2 = 2.0f * fs;
  Zpk res;

  res.zeros.reserve(filter.zeros.size());
  for (const auto& zero : filter.zeros) {
    res.zeros.emplace_back((fs2 + zero) / (fs2 - zero));
  }

  res.poles.reserve(filter.poles.size());
  for (const auto& pole : filter.poles) {
    res.poles.emplace_back((fs2 + pole) / (fs2 - pole));
  }

  // Pad zeros with -1 if needed (to match pole count)
  if (res.zeros.size() < res.poles.size()) {
    res.zeros.resize(res.poles.size(), T(-1.0f));
  }

  auto calc_prod = [&](const std::vector<T>& vec) {
    return std::accumulate(vec.begin(), vec.end(), T(1.0f),
      [fs2](const auto& a, const auto& b) { return a * (fs2 - b); });
  };

  T np = calc_prod(filter.zeros);
  T dp = calc_prod(filter.poles);

  res.k = filter.k * std::real(np / dp);

  return res;
}

template <typename F>
static std::vector<cfloat> 
flatten_transform(const std::vector<cfloat>& vec, F f) {
  // A temporary vector to hold the transformed ranges.
  auto xform_ranges = vec | std::views::transform(f);

  std::vector<cfloat> res;

  for (const auto& inner_range : xform_ranges) {
    for (const auto& elem : inner_range) {
      res.push_back(elem);
    }
  }
  return res;
}

Zpk iirlp2hp_s(const Zpk& lpf, const float wc) {
  Zpk hpf;
  auto dived_by_wc = [wc](const cfloat& s) {
    return wc / s;
  };
  hpf.zeros = flatten_transform(lpf.zeros, [dived_by_wc](const cfloat& val) {
    return std::vector<cfloat>{dived_by_wc(val)};
  });
  hpf.zeros = flatten_transform(lpf.poles, [dived_by_wc](const cfloat& val) {
    return std::vector<cfloat>{dived_by_wc(val)};
  });
  hpf.zeros.resize(hpf.poles.size(), cfloat(0));
  hpf.k = lpf.k * std::real(prod(lpf.zeros, cfloat(-1)) / prod(lpf.poles, cfloat(-1)));
  return hpf;
}


Zpk iirlp2bp_s(const Zpk& lpf, const float wc, const float bw) {
  Zpk lpf_scaled, bpf;
  // Scale poles and zeros to desired bandwidth
  auto scale_plzro = [bw](const cfloat& val) -> cfloat {
    return 0.5f * bw * val;
  };
  lpf_scaled.zeros = flatten_transform(lpf.zeros, [scale_plzro](const cfloat& val) {
    return std::vector<cfloat>{scale_plzro(val)};
  });
  lpf_scaled.poles = flatten_transform(lpf.poles, [scale_plzro](const cfloat& val) {
    return std::vector<cfloat>{scale_plzro(val)};
  });

  auto get_quad_eqsol = [wc](const cfloat& s) -> std::vector<cfloat> {
    const cfloat s_sqr = s * s;
    const cfloat wc_sqr = cfloat(wc * wc, 0.0f);
    const cfloat term = std::sqrt(s_sqr - wc_sqr);
    return std::vector<cfloat>{s + term, s - term};
  };

  bpf.zeros = flatten_transform(lpf_scaled.zeros, get_quad_eqsol);
  bpf.poles = flatten_transform(lpf_scaled.poles, get_quad_eqsol);
  bpf.zeros.resize(bpf.zeros.size() + lpf.poles.size() - lpf.zeros.size(), cfloat(0));
  bpf.k = lpf.k * std::pow(bw, lpf.poles.size() - lpf.zeros.size());

  return bpf;
}

Zpk iirlp2bs_s(const Zpk& lpf, const float wc, const float bw) {
  Zpk lpf_scaled, bsf;
  // Scale poles and zeros to desired bandwidth
  auto scale_plzro = [bw](const cfloat& val) -> cfloat {
    return (0.5f * bw) / val;
  };

  lpf_scaled.zeros = flatten_transform(lpf.zeros, [scale_plzro](const cfloat& val) {
    return std::vector<cfloat>{scale_plzro(val)};
  });

  lpf_scaled.poles = flatten_transform(lpf.poles, [scale_plzro](const cfloat& val) {
    return std::vector<cfloat>{scale_plzro(val)};
  });

  auto get_quad_eqsol = [wc](const cfloat& s) -> std::vector<cfloat> {
    const cfloat s_sqr = s * s;
    const cfloat wc_sqr = cfloat(wc * wc, 0.0f);
    const cfloat term = std::sqrt(s_sqr - wc_sqr);
    return std::vector<cfloat>{s + term, s - term};
  };

  bsf.zeros = flatten_transform(lpf_scaled.zeros, get_quad_eqsol);
  bsf.poles = flatten_transform(lpf_scaled.poles, get_quad_eqsol);
  bsf.zeros.resize(bsf.zeros.size() + lpf.poles.size() - lpf.zeros.size(), cfloat(0, wc));
  bsf.zeros.resize(bsf.zeros.size() + lpf.poles.size() - lpf.zeros.size(), cfloat(0, -wc));
  bsf.k = lpf.k * std::real(prod(lpf.zeros, cfloat(-1)) / prod(lpf.poles, cfloat(-1)));

  return bsf;
}

Zpk iirlp2lp_z(const Zpk& proto, const float fc, 
  const float fs, const float fc_new, const float fs_new
) {
  Zpk res;
  
  // Normalized frequencies
  float wc_orig = fc / (fs / 2.0f) * kPi ;
  float wc_new = fc_new / (fs_new / 2.0f) * kPi;
  float alpha = std::sin((wc_orig - wc_new) / 2.0f) / 
                std::sin((wc_orig + wc_new) / 2.0f);

  for (auto z : proto.zeros) {
    // Check if zero is finite before transformation
    if (std::isfinite(z.real()) && std::isfinite(z.imag())) {
      cfloat z_new = (z + alpha) / (1.0f + alpha * z);
      res.zeros.emplace_back(z_new);
    } else {
      // Handle infinite zeros by mapping to appropriate finite location
      res.zeros.emplace_back(-1.0f / alpha); 
    }
  }

  // Transform poles
  for (auto p : proto.poles) {
    cfloat p_new = (p + alpha) / (1.0f + alpha * p);
    res.poles.push_back(p_new);
  }

  // Gain calculation
  auto dc_gain = [](const Zpk& zpk) -> cfloat {
    cfloat H = 1.0;
    for (const auto& z : zpk.zeros) {
      H *= (1.0f - z);
    }
    for (const auto& p : zpk.poles) {
      H /= (1.0f - p);
    }
    return H;
  };

  cfloat H_proto_dc = dc_gain(proto);
  cfloat H_new_dc = dc_gain(res);
  
  if (std::abs(H_new_dc) > 1e-12f) {
      res.k = proto.k * (std::abs(H_proto_dc) / std::abs(H_new_dc));
  } else {
      res.k = proto.k;
  }

  return res;
}

// helper: evaluate ZPK at complex point z
static cfloat 
eval_zpk_at(const Zpk& zpk, const cfloat z_eval) {
  cfloat H(1.0f, 0.0f);
  for (auto z : zpk.zeros) {
    // if there's an explicit "infinite" marker,
    // skip here (we handle infinite zeros by explicit finite placements)
    if (!std::isinf(z.real()) && !std::isinf(z.imag()))
      H *= (z_eval - z);
  }
  for (auto pp : zpk.poles) {
    H /= (z_eval - pp);
  }
  return H;
}

Zpk iirlp2hp_z(const Zpk& proto, const float fc, 
  const float fs, const float fc_new, const float fs_new) {
  Zpk res;

  DBUG_SECTION("Highpass Transformation Started");
  DBUG_VAR(fc); DBUG_VAR(fs); DBUG_VAR(fc_new); DBUG_VAR(fs_new);
      
  float wc_orig = 2.0f * kPi * (fc / fs);
  float wc_new = 2.0f * kPi * (fc_new / fs_new);
  DBUG_VAR(wc_orig); DBUG_VAR(wc_new);
  float alpha = - std::cos((wc_orig + wc_new) / 2.0f)
                / std::cos((wc_orig - wc_new) / 2.0f);
  DBUG_VAR(alpha);

  // Show prototype filter
  DBUG_SECTION("Prototype Filter");
  DBUG_FILTER_STATE(proto);

  res.zeros.reserve(proto.zeros.size());
  for (auto zhat : proto.zeros) {
    if (std::isinf(zhat.real()) || std::isinf(zhat.imag())) {
      cfloat znew = (-1.0f / alpha);
      res.zeros.emplace_back(znew);
    } else {
      cfloat znew = -(zhat + alpha) / (1.0f + alpha * zhat);
      res.zeros.emplace_back(znew);
      DBUG_COMPLEX(zhat); DBUG_COMPLEX(znew);
    }
  }

  res.poles.reserve(proto.poles.size());
  for (auto phat : proto.poles) {
    cfloat pnew = -(phat + alpha) / (1.0f + alpha * phat);
    res.poles.emplace_back(pnew);
    DBUG_COMPLEX(phat); DBUG_COMPLEX(pnew);
  }

  cfloat z_proto_eval = std::exp(cfloat(0.0f, wc_orig));
  cfloat z_new_eval = std::exp(cfloat(0.0f, wc_new));
  
  cfloat H_proto_gain = eval_zpk_at(proto, z_proto_eval);
  cfloat H_new_gain = eval_zpk_at(res, z_new_eval);

  DBUG_COMPLEX(H_proto_nyq); DBUG_COMPLEX(H_new_gain);

  // Use complex division to preserve phase information
  if (std::abs(H_new_gain) > 1e-12f) {
    res.k = proto.k * std::abs(H_proto_gain / H_new_gain);
  } else {
    res.k = proto.k;
    DBUG_LOG("Warning: Division by zero in gain calculation, using prototype gain");
  }

  DBUG_VAR(res.k);
  DBUG_SECTION("Final Transformed Filter");
  DBUG_FILTER_STATE(res);
  return res;
}

using RootPairs = std::pair<cfloat, cfloat>;

// Solve quadratic A z^2 + B z + C = 0 (returns two roots)
static RootPairs 
solve_quad(cfloat A, cfloat B, cfloat C) {
  cfloat disc = std::sqrt(B * B - cfloat(4.0f) * A * C);
  cfloat twoA = cfloat(2.0f) * A;
  return {(-B + disc) / twoA, (-B - disc) / twoA};
}

// Holton bandpass transform G_bp(z) from Table 8.2:
// G(z) = - (z^2 - alpha(1+beta) z + beta) / (beta z^2 - alpha(1+beta) z + 1)
static cfloat 
iirlp2bp_gz(const cfloat z, const float alpha, const float beta) {
  cfloat z2 = z * z;
  cfloat num = z2 - (alpha * (1.0f + beta)) * z + beta;
  cfloat den = beta * z2 - (alpha * (1.0f + beta)) * z + cfloat(1.0f, 0.0f);
  // stability check
  if (std::abs(den) < 1e-12f) {
    return cfloat(std::numeric_limits<float>::infinity(), 0.0f);
  }
  return -num / den;
}

Zpk iirlp2bp_z(const Zpk& proto, const float fc, 
               const float fs, const float f1,
               const float f2, const float fs_new) {
  Zpk res;
  float wc_hat = 2.0f * kPi * (fc / fs);
  float w1 = 2.0f * kPi * (f1 / fs_new);
  float w2 = 2.0f * kPi * (f2 / fs_new);
  float dw = w2 - w1;
  float alpha = std::cos((w2 + w1) / 2.0f) / std::cos((w2 - w1) / 2.0f);
  float beta = -std::sin((dw - wc_hat) / 2.0f) / std::sin((dw + wc_hat) / 2.0f);

  // Transform each prototype pole ->
  // two new poles using quadratic coefficients derived:
  // A = beta * r + 1
  // B = -alpha(1+beta)(r + 1)
  // C = r + beta
  res.poles.reserve(proto.poles.size() * 2);
  for (auto r : proto.poles) {
    cfloat A = beta * r + cfloat(1.0f, 0.0f);
    cfloat B = -(alpha * (1.0f + beta)) * (r + cfloat(1.0f, 0.0f));
    cfloat C = r + cfloat(beta, 0.0f);
    auto roots = solve_quad(A, B, C);
    res.poles.emplace_back(roots.first);
    res.poles.emplace_back(roots.second);
  }

  // Zeros: infinite prototype zeros map to
  // band-edge zeros at e^{j w1}, e^{j w2}
  res.zeros.reserve(proto.zeros.size() * 2);
  
  for (auto r : proto.zeros) {
    if (std::isinf(r.real()) || std::isinf(r.imag())) {
      res.zeros.emplace_back(1.0f);
      res.zeros.emplace_back(-1.0f);
    } else {
      cfloat A = beta * r + cfloat(1.0f, 0.0f);
      cfloat B = -(alpha * (1.0f + beta)) * (r + cfloat(1.0f, 0.0f));
      cfloat C = r + cfloat(beta, 0.0f);
      auto roots = solve_quad(A, B, C);
      res.zeros.emplace_back(roots.first);
      res.zeros.emplace_back(roots.second);
    }
  }

  // --- Gain normalization ---
  // Choose band-center omega0
  float bw_ratio = (w2 - w1) / 0.5f * ((w2 + w1));
  float w_center = 0.5f * ((w2 + w1));
  if (bw_ratio > 0.5f) 
    w_center = std::sqrt(w2 * w1);

  cfloat z_center = std::exp(cfloat(0.0f, w_center));
  cfloat gz_center = iirlp2bp_gz(z_center, alpha, beta);

  cfloat H_proto = eval_zpk_at(proto, gz_center);

  Zpk tmp = res;
  tmp.k = proto.k;
  cfloat H_trans = eval_zpk_at(tmp, z_center);

  if (std::abs(H_trans) > 1e-12f) {
    res.k = proto.k * (std::abs(H_proto) / std::abs(H_trans));
  } else {
    res.k = proto.k;  // fallback
  }

  return res;
}

// Holton bandstop transform G_bs(z) from Table 8.2:
// G(z) = (z^2 - alpha(1-beta) z - beta) / (-beta z^2 - alpha(1-beta) z + 1)
static cfloat 
iirlp2bs_gz(const cfloat z, const float alpha, const float beta) {
  cfloat z2 = z * z;
  cfloat num = z2 - (alpha * (1.0f - beta)) * z - beta;
  cfloat den = -beta * z2 - (alpha * (1.0f - beta)) * z + cfloat(1.0f, 0.0f);
  return num / den;
}

Zpk iirlp2bs_z(const Zpk& proto, const float fc, 
               const float fs, const float f1,
               const float f2, const float fs_new) {
  Zpk res;

  // Convert to radians/sample
  float wc_hat = 2.0f * kPi * (fc / fs);
  float w1 = 2.0f * kPi * (f1 / fs_new);
  float w2 = 2.0f * kPi * (f2 / fs_new);
  float dw = w2 - w1;
  float w_center  = (w1 + w2) * 0.5f;
  cfloat z_center = std::exp(cfloat(0.0f, w_center)); 
  // Bandstop transformation parameters (from Holton Table 8.2)
  float alpha = std::cos((w2 + w1) / 2.0f) / std::cos(dw / 2.0f);
  float beta = -std::cos((dw + wc_hat) / 2.0f) / std::cos((dw - wc_hat) / 2.0f);

  // Transform poles using CORRECTED coefficients from your Python implementation:
  // A = beta * r + 1
  // B = alpha(1-beta)(r - 1)   NOTE: Positive sign, not negative!
  // C = -(r + beta)
  res.poles.reserve(proto.poles.size() * 2);
  for (auto r : proto.poles) {
    cfloat A = beta * r + cfloat(1.0f, 0.0f);
    cfloat B = (alpha * (1.0f - beta)) * (r - cfloat(1.0f, 0.0f));
    cfloat C = -(r + cfloat(beta, 0.0f));
    
    auto roots = solve_quad(A, B, C);
    res.poles.emplace_back(roots.first);
    res.poles.emplace_back(roots.second);
    
  }

  res.zeros.reserve(proto.zeros.size() * 2);
  for (auto r : proto.zeros) {
    if (std::isinf(r.real()) || std::isinf(r.imag())) {
      // map infinite zeros to center of stop bands
      res.zeros.emplace_back(z_center);
      res.zeros.emplace_back(std::conj(z_center));
    } else {
      cfloat A = beta * r + cfloat(1.0f, 0.0f);
      cfloat B = (alpha * (1.0f - beta)) * (r - cfloat(1.0f, 0.0f));
      cfloat C = -(r + cfloat(beta, 0.0f));
      
      auto roots = solve_quad(A, B, C);
      res.zeros.emplace_back(roots.first);
      res.zeros.emplace_back(roots.second);
    }
  }

  cfloat z_w0 = std::exp(cfloat(0.0f, wc_hat));  // z = e^{jωc_hat}
  cfloat g_z = iirlp2bs_gz(z_w0, alpha, beta);
  
  // Evaluate prototype at the mapped frequency G(z_w0)
  cfloat H_proto = eval_zpk_at(proto, g_z);
  
  Zpk tmp = res;
  tmp.k = proto.k;
  cfloat H_trans = eval_zpk_at(tmp, z_w0);
  
  if (std::abs(H_trans) > 1e-12f) {
    res.k = proto.k * std::abs(H_proto) / std::abs(H_trans);
  } else {
    res.k = proto.k;
  }

  return res;
}

}  // namespace adptsysc