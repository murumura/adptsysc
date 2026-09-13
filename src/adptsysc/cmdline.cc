#include <adptsysc/adptsysc.hh>
#include <adptsysc/arch.hh>
#include <adptsysc/config.hh>

#include <cctype>
#include <cstdlib>
#include <filesystem>
#include <span>
#include <string>
#include <string_view>
#include <unordered_set>
#include <vector>
#include <unistd.h>

namespace adptsysc {

static const char helpmsg[] = R"(
Options:
  --help
  -C DIR, --directory DIR
  -o FILE, --output FILE
  --text-output FILE
  --chroot DIR
  --verbose
  --quick-exit / --no-quick-exit
  --run-testbench
  --trace
  --fixedpoint-eval
  --fixedpoint-tol VALUE
  --fft / --ifft
  --signal-trace [--signal-trace-file FILE]
  --waveform [--waveform-file FILE]
  --sv-trace [--sv-trace-dir DIR]
  --load-file FILE
  --textload-file FILE
  --load-offset OFFSET
  --oformat=[binary,hex]
  --mem-read-delay-cycles N
  --mem-write-delay-cycles N
  -e TARGET, --emulation TARGET

Targets:
  syscmem
  r2sdf_fft_tlm
  syscmem_cycle
  r2sdf_fft_cycle
  overlap_save_fdaf_cycle
)";

template <typename E>
static std::vector<std::string_view>
read_response_file(Context<E>& ctx, std::string_view path, i64 depth) {
  if (depth > 10) {
    Fatal(ctx) << path << ": response file nesting too deep\n";
  }

  std::vector<std::string_view> vec;
  MappedFile* mf = must_open_file(ctx, std::string(path));
  std::string_view data(reinterpret_cast<char*>(mf->data), mf->size);
  mf->is_dependency = false;

  while (!data.empty()) {
    if (std::isspace(static_cast<unsigned char>(data[0]))) {
      data.remove_prefix(1);
      continue;
    }

    auto read_quoted = [&]() {
      const char quote = data[0];
      data.remove_prefix(1);
      std::string buf;
      while (!data.empty() && data[0] != quote) {
        if (data[0] == '\\' && data.size() >= 2) {
          buf.push_back(data[1]);
          data.remove_prefix(2);
        } else {
          buf.push_back(data[0]);
          data.remove_prefix(1);
        }
      }
      if (data.empty()) {
        Fatal(ctx) << path << ": premature end of input\n";
      }
      data.remove_prefix(1);
      return save_string(ctx, buf);
    };

    auto read_unquoted = [&]() {
      std::string buf;
      while (!data.empty() &&
             !std::isspace(static_cast<unsigned char>(data[0]))) {
        if (data[0] == '\\' && data.size() >= 2) {
          buf.push_back(data[1]);
          data.remove_prefix(2);
        } else {
          buf.push_back(data[0]);
          data.remove_prefix(1);
        }
      }
      return save_string(ctx, buf);
    };

    std::string_view tok =
        (data[0] == '\'' || data[0] == '"') ? read_quoted() : read_unquoted();

    if (tok.starts_with('@')) {
      append(vec, read_response_file(ctx, tok.substr(1), depth + 1));
    } else {
      vec.push_back(tok);
    }
  }

  return vec;
}

template <typename E>
std::vector<std::string_view>
expand_response_files(Context<E>& ctx, char** argv) {
  std::vector<std::string_view> vec;
  for (i64 i = 0; argv[i] != nullptr; ++i) {
    if (argv[i][0] == '@') {
      append(vec, read_response_file(ctx, argv[i] + 1, 1));
    } else {
      vec.push_back(argv[i]);
    }
  }
  return vec;
}

template <typename E>
static i64 parse_number(Context<E>& ctx,
                        const std::string& opt,
                        std::string_view value) {
  std::size_t nread = 0;
  const std::string text(value);
  try {
    const long long v = std::stoll(text, &nread, 0);
    if (nread != text.size()) {
      Fatal(ctx) << "option --" << opt << ": not a number: " << value << "\n";
    }
    return static_cast<i64>(v);
  } catch (...) {
    Fatal(ctx) << "option --" << opt << ": not a number: " << value << "\n";
  }
  return 0;
}

static std::vector<std::string> add_dashes(const std::string& name) {
  if (name.size() == 1) {
    return {"-" + name};
  }
  if (!name.empty() && name[0] == 'o') {
    return {"--" + name};
  }
  return {"-" + name, "--" + name};
}

template <typename E>
std::vector<std::string> parse_nonpositional_args(Context<E>& ctx) {
  std::span<std::string_view> args = ctx.cmdline_args;
  args = args.subspan(1);

  std::vector<std::string> remaining;
  std::string_view arg;
  ctx.arg.color_diagnostics = isatty(STDERR_FILENO);

  auto read_arg = [&](const std::string& name) {
    for (const auto& opt : add_dashes(name)) {
      if (!args.empty() && args[0] == opt) {
        if (args.size() == 1) {
          Fatal(ctx) << "option " << opt << ": argument missing\n";
        }
        arg = args[1];
        args = args.subspan(2);
        return true;
      }

      const std::string prefix = (name.size() == 1) ? opt : opt + "=";
      if (!args.empty() && args[0].starts_with(prefix)) {
        arg = args[0].substr(prefix.size());
        args = args.subspan(1);
        return true;
      }
    }
    return false;
  };

  auto read_flag = [&](const std::string& name) {
    for (const auto& opt : add_dashes(name)) {
      if (!args.empty() && args[0] == opt) {
        args = args.subspan(1);
        return true;
      }
    }
    return false;
  };

  while (!args.empty()) {
    if (read_flag("help")) {
      Out(ctx) << "Usage: " << ctx.cmdline_args[0]
               << " [options] file...\n" << helpmsg;
      std::exit(0);
    }

    if (read_arg("o") || read_arg("output")) {
      ctx.arg.output = arg;
    } else if (read_arg("text-output")) {
      ctx.arg.text_output = arg;
    } else if (read_flag("trace")) {
      ctx.arg.trace_enabled = true;
    } else if (read_flag("run-testbench")) {
      ctx.arg.run_testbench = true;
    } else if (read_flag("out-shared")) {
      ctx.arg.out_shared = true;
    } else if (read_flag("quick-exit")) {
      ctx.arg.quick_exit = true;
    } else if (read_flag("no-quick-exit")) {
      ctx.arg.quick_exit = false;
    } else if (read_flag("fixedpoint-eval")) {
      ctx.arg.fixedpoint_eval = true;
    } else if (read_arg("fixedpoint-tol")) {
      ctx.arg.fixedpoint_tol = std::stod(std::string(arg));
    } else if (read_flag("ifft")) {
      ctx.arg.compute_ifft = true;
    } else if (read_flag("fft")) {
      ctx.arg.compute_ifft = false;
    } else if (read_flag("signal-trace")) {
      ctx.arg.signal_trace = true;
    } else if (read_arg("signal-trace-file")) {
      ctx.arg.signal_trace = true;
      ctx.arg.signal_trace_file = arg;
    } else if (read_flag("waveform")) {
      ctx.arg.waveform = true;
    } else if (read_arg("waveform-file")) {
      ctx.arg.waveform = true;
      ctx.arg.waveform_file = arg;
    } else if (read_flag("sv-trace")) {
      ctx.arg.sv_trace = true;
    } else if (read_arg("sv-trace-dir")) {
      ctx.arg.sv_trace = true;
      ctx.arg.sv_trace_dir = arg;
    } else if (read_arg("C") || read_arg("directory")) {
      ctx.arg.directory = arg;
    } else if (read_arg("chroot")) {
      ctx.arg.chroot = arg;
    } else if (read_flag("color-diagnostics") ||
               read_flag("color-diagnostics=auto")) {
      ctx.arg.color_diagnostics = isatty(STDERR_FILENO);
    } else if (read_flag("color-diagnostics=always")) {
      ctx.arg.color_diagnostics = true;
    } else if (read_flag("color-diagnostics=never")) {
      ctx.arg.color_diagnostics = false;
    } else if (read_flag("verbose")) {
      ctx.arg.verbose = true;
    } else if (read_arg("thread-count")) {
      ctx.arg.thread_count = parse_number(ctx, "thread-count", arg);
    } else if (read_flag("threads")) {
      ctx.arg.thread_count = 0;
    } else if (read_flag("no-threads")) {
      ctx.arg.thread_count = 1;
    } else if (read_arg("mem-read-delay-cycles")) {
      ctx.arg.mem_rddly_cycls = static_cast<int>(
          parse_number(ctx, "mem-read-delay-cycles", arg));
    } else if (read_arg("mem-write-delay-cycles")) {
      ctx.arg.mem_wrdly_cycls = static_cast<int>(
          parse_number(ctx, "mem-write-delay-cycles", arg));
    } else if (read_arg("load-file")) {
      ctx.arg.load_file = arg;
    } else if (read_arg("textload-file") || read_arg("text-load-file")) {
      ctx.arg.text_loadfile = arg;
    } else if (read_arg("load-offset")) {
      ctx.arg.load_offset = static_cast<int>(parse_number(ctx, "load-offset", arg));
    } else if (read_arg("dependency-file")) {
      ctx.arg.dependency_file = arg;
    } else if (read_arg("oformat")) {
      if (arg == "binary") {
        ctx.arg.oformat_binary = true;
        ctx.arg.oformat_hex = false;
      } else if (arg == "hex") {
        ctx.arg.oformat_binary = false;
        ctx.arg.oformat_hex = true;
      } else {
        Fatal(ctx) << "--oformat: " << arg << " is not supported\n";
      }
    } else if (read_arg("e") || read_arg("emulation")) {
      if (arg == SyscMemArch::name) {
        ctx.arg.emulation = SyscMemArch::name;
      } else if (arg == R2SdfFFTTLMArch::name) {
        ctx.arg.emulation = R2SdfFFTTLMArch::name;
      } else if (arg == SyscMemoryCycleArch::name) {
        ctx.arg.emulation = SyscMemoryCycleArch::name;
      } else if (arg == R2SdfFFTCycleArch::name) {
        ctx.arg.emulation = R2SdfFFTCycleArch::name;
      } else if (arg == OverlapSaveFdafCycleArch::name) {
        ctx.arg.emulation = OverlapSaveFdafCycleArch::name;
      } else {
        Fatal(ctx) << "unknown -e argument: " << arg << "\n";
      }
    } else {
      if (args[0].starts_with('-')) {
        Fatal(ctx) << "unknown command line option: " << args[0] << "\n";
      }
      remaining.emplace_back(args[0]);
      args = args.subspan(1);
    }
  }

  if (!ctx.arg.chroot.empty() && !ctx.arg.dependency_file.empty()) {
    ctx.arg.dependency_file = ctx.arg.chroot + "/" + ctx.arg.dependency_file;
  }
  ctx.overwrite_output_file = !ctx.arg.out_shared;
  return remaining;
}

using E = ADPT_TARGET;

template std::vector<std::string_view>
expand_response_files(Context<E>&, char**);

template std::vector<std::string>
parse_nonpositional_args(Context<E>&);

}  // namespace adptsysc