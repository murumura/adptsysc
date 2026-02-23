#include <adptsysc/adptsysc.hh>
#include <adptsysc/integers.hh>
#include <adptsysc/arch.hh>
#include <adptsysc/config.hh>
#include <regex>
#include <unordered_set>

namespace adptsysc {
namespace fs = std::filesystem;

static const char helpmsg[] = R"(
Options:
  --help                      Report usage information
  -C DIR, --directory DIR     Change to DIR before doing anything
  -o FILE, --output FILE      Set output filename
  --text-output FILE          Set text output filename
  --chroot DIR                Set a given path to the root directory
  --color-diagnostics=[auto,always,never]
                              Use colors in diagnostics
  --color-diagnostics         Alias for --color-diagnostics=always
  --dependency-file=FILE      Write Makefile-style dependency rules to FILE
  --fatal-warnings            Treat warnings as errors
  --no-fatal-warnings         Do not treat warnings as errors (default)
  --thread-count COUNT, --threads=COUNT
                              Use COUNT number of threads
  --threads                   Use multiple threads (default)
  --no-threads                Use single thread
  --verbose                   Enable verbose output
  --quick-exit                Use quick exit (default)
  --no-quick-exit             Don't use quick exit
  
  File I/O Options:
  --load-file FILE            Load memory from binary file
  --text-loadfile FILE        Load memory from text file
  --load-offset OFFSET        Offset for loading files
  --oformat=[binary,hex]      Set output format (binary or hex)
  
  Filter Options:
  -e TARGET, --emulation TARGET
                              Set emulation target (syscmem, lms)
  --filter-type TYPE          Set filter type (default: LMSArch)
  --behavior-filter           Enable behavior filter (default)
  --no-behavior-filter        Disable behavior filter
  --fixedpoint-eval           Enable fixed-point evaluation
  --no-fixedpoint-eval        Disable fixed-point evaluation (default)
  --polyphase                 Enable polyphase filter
  --no-polyphase              Disable polyphase filter (default)
  
  Simulation Options:
  --run-testbench             Run SystemC testbench simulation
  --trace                     Enable VCD trace file generation
  --out-shared                Allow output file sharing (disable overwrite)

adpt: supported targets: syscmem, lms)";

template <typename E>
static std::vector<std::string_view> 
read_response_file(Context<E>& ctx, 
                   std::string_view path, i64 depth) {
  if (depth > 10)
    Fatal(ctx) << path << ": response file nesting too deep" << "\n";

  std::vector<std::string_view> vec;
  MappedFile* mf = must_open_file(ctx, std::string(path));
  std::string_view data((char*)mf->data, mf->size);

  mf->is_dependency = false;

  while (!data.empty()) {
    if (isspace(data[0])) {
      data = data.substr(1);
      continue;
    }

    auto read_quoted = [&]() {
      char quote = data[0];
      data = data.substr(1);

      std::string buf;
      while (!data.empty() && data[0] != quote) {
        if (data[0] == '\\' && data.size() >= 1) {
          buf.append(1, data[1]);
          data = data.substr(2);
        } else {
          buf.append(1, data[0]);
          data = data.substr(1);
        }
      }
      if (data.empty())
        Fatal(ctx) << path << ": premature end of input";
      data = data.substr(1);
      return save_string(ctx, buf);
    };

    auto read_unquoted = [&] {
      std::string buf;
      while (!data.empty()) {
        if (data[0] == '\\' && data.size() >= 1) {
          buf.append(1, data[1]);
          data = data.substr(2);
          continue;
        }

        if (!isspace(data[0])) {
          buf.append(1, data[0]);
          data = data.substr(1);
          continue;
        }
        break;
      }
      return save_string(ctx, buf);
    };

    std::string_view tok;
    if (data[0] == '\'' || data[0] == '\"')
      tok = read_quoted();
    else
      tok = read_unquoted();

    if (tok.starts_with('@'))
      append(vec, read_response_file(ctx, tok.substr(1), depth + 1));
    else
      vec.push_back(tok);
  }
  return vec;
}

// Replace "@path/to/some/text/file" with its file contents.
template <typename E>
std::vector<std::string_view> 
expand_response_files(Context<E>& ctx, char** argv) {
  std::vector<std::string_view> vec;
  for (i64 i = 0; argv[i]; i++) {
    if (argv[i][0] == '@')
      append(vec, read_response_file(ctx, argv[i] + 1, 1));
    else
      vec.push_back(argv[i]);
  }
  return vec;
}

static std::string_view string_trim(std::string_view str) {
  std::size_t pos = str.find_first_not_of(" \t");
  if (pos == str.npos)
    return "";
  str = str.substr(pos);

  pos = str.find_last_not_of(" \t");
  if (pos == str.npos)
    return str;
  return str.substr(0, pos + 1);
}

template <typename E> 
static i64 parse_hex(Context<E>& ctx, std::string opt, 
                    std::string_view value) {
  auto flags = std::regex_constants::optimize | std::regex_constants::ECMAScript;
  static std::regex re(R"((?:0x|0X)?([0-9a-fA-F]+))", flags);

  std::cmatch m;
  if (!std::regex_match(value.data(), value.data() + value.size(), m, re))
    Fatal(ctx) << "option -" << opt << ": not a hexadecimal number";
  return std::stoul(m[1], nullptr, 16);
}

template <typename E>
static i64 parse_number(Context<E>& ctx, std::string opt, 
                        std::string_view value) {
  std::size_t nread;
  // Negative Check
  if (value.starts_with('-')) {
    // converts the string to an unsigned long with base 0 (auto-detects decimal)
    i64 ret = std::stoul(std::string(value.substr(1)), &nread, 0);
    if (value.size() - 1 != nread)
      Fatal(ctx) << "option -" << opt << ": not a number: " << value << "\n";
    return -ret;
  }

  i64 ret = std::stoul(std::string(value), &nread, 0);
  if (value.size() != nread)
    Fatal(ctx) << "option -" << opt << ": not a number: " << value << "\n";
  return ret;
}

static char from_hex(char c) {
  if ('0' <= c && c <= '9')
    return c - '0';
  if ('a' <= c && c <= 'f')
    return c - 'a' + 10;
  assert('A' <= c && c <= 'F');
  return c - 'A' + 10;
}

static std::vector<std::string_view> 
split_by_comma_or_colon(std::string_view str) {
  std::vector<std::string_view> vec;

  for (;;) {
    i64 pos = str.find_first_of(",:");
    if (pos == str.npos) {
      vec.push_back(str);
      break;
    }
    vec.push_back(str.substr(0, pos));
    str = str.substr(pos + 1);
  }
  return vec;
}

static std::vector<std::string> add_dashes(std::string name) {
  // Single-letter option
  if (name.size() == 1)
    return {"-" + name};

  // Multi-letter options can be preceded by either a single
  // dash or double dashes except ones starting with "o", which must
  // be preceded by double dashes. For example, "-omagic" is
  // interpreted as "-o magic". If you really want to specify the
  // "omagic" option, you have to pass "--omagic".
  if (name[0] == 'o')
    return {"--" + name};
  return {"-" + name, "--" + name};
}

template <typename E>
std::vector<std::string> 
parse_nonpositional_args(Context<E>& ctx) {
  std::span<std::string_view> args = ctx.cmdline_args;
  args = args.subspan(1);

  std::vector<std::string> remaining;
  std::string_view arg;
  std::unordered_set<std::string_view> rpaths;

  ctx.arg.color_diagnostics = isatty(STDERR_FILENO);

  auto add_rpath = [&](std::string_view arg) {
    if (rpaths.insert(arg).second) {
      if (!ctx.arg.rpaths.empty())
        ctx.arg.rpaths += ':';
      ctx.arg.rpaths += arg;
    }
  };

  auto read_arg = [&](std::string name) {
    for (const std::string& opt : add_dashes(name)) {
      if (args[0] == opt) {
        if (args.size() == 1)
          Fatal(ctx) << "option -" << name << ": argument missing\n";
        arg = args[1];
        args = args.subspan(2);
        return true;
      }

      std::string prefix = (name.size() == 1) ? opt : opt + "=";
      if (args[0].starts_with(prefix)) {
        arg = args[0].substr(prefix.size());
        args = args.subspan(1);
        return true;
      }
    }
    return false;
  };

  auto read_eq = [&](std::string name) {
    for (const std::string& opt : add_dashes(name)) {
      if (args[0].starts_with(opt + "=")) {
        arg = args[0].substr(opt.size() + 1);
        args = args.subspan(1);
        return true;
      }
    }
    return false;
  };

  auto read_flag = [&](std::string name) {
    for (const std::string& opt : add_dashes(name)) {
      if (args[0] == opt) {
        args = args.subspan(1);
        return true;
      }
    }
    return false;
  };

  auto read_z_flag = [&](std::string name) {
    if (args.size() >= 2 && args[0] == "-z" && args[1] == name) {
      args = args.subspan(2);
      return true;
    }

    if (!args.empty() && args[0] == "-z" + name) {
      args = args.subspan(1);
      return true;
    }
    return false;
  };

  auto read_z_arg = [&](std::string name) {
    if (args.size() >= 2 && args[0] == "-z" && args[1].starts_with(name + "=")) {
      arg = args[1].substr(name.size() + 1);
      args = args.subspan(2);
      return true;
    }

    if (!args.empty() && args[0].starts_with("-z" + name + "=")) {
      arg = args[0].substr(name.size() + 3);
      args = args.subspan(1);
      return true;
    }
    return false;
  };

  while (!args.empty()) {
    if (read_flag("help")) {
      Out(ctx) << "Usage: " << ctx.cmdline_args[0] << " [options] file...\n" << helpmsg;
      exit(0);
    }

    if (read_arg("o") || read_arg("output")) {
      ctx.arg.output = arg;
    } else if (read_arg("text-output")) {
      ctx.arg.text_output = arg;
    }  else if (read_flag("polyphase")) {
      ctx.arg.use_polyphase = true;
    } else if (read_flag("no-polyphase")) {
      ctx.arg.use_polyphase = false;
    } else if (read_flag("trace")) {
      ctx.arg.trace_enabled = true;
    } else if (read_flag("run-testbench")) {
      ctx.arg.run_testbench = true;
    } else if (read_flag("out-shared")) {
      ctx.arg.out_shared = true;
    } else if (read_flag("behavior-filter")) {
      ctx.arg.behavior_filter = true;
    } else if (read_flag("no-behavior-filter")) {
      ctx.arg.behavior_filter = false;
    } else if (read_flag("quick-exit")) {
      ctx.arg.quick_exit = true;
    } else if (read_flag("no-quick-exit")) {
      ctx.arg.quick_exit = false;
    } else if (read_flag("fixedpoint-eval")) {
      ctx.arg.fixedpoint_eval = true;
    } else if (read_flag("no-fixedpoint-eval")) {
      ctx.arg.fixedpoint_eval = false;
    } else if (read_arg("C") || read_arg("directory")) {
      ctx.arg.directory = arg;
    } else if (read_arg("chroot")) {
      ctx.arg.chroot = arg;
    } else if (read_flag("color-diagnostics") || read_flag("color-diagnostics=auto")) {
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
    } else if (read_eq("threads")) {
      ctx.arg.thread_count = parse_number(ctx, "threads", arg);
    } else if (read_arg("mem-read-lat")) {
      ctx.arg.mem_read_lat = (int)parse_number(ctx, "mem-read-lat", arg);
      if (ctx.arg.mem_read_lat < 0)
        Fatal(ctx) << "--mem-read-lat must be non-negative\n";
    } else if (read_arg("mem-write-lat")) {
      ctx.arg.mem_write_lat = (int)parse_number(ctx, "mem-write-lat", arg);
      if (ctx.arg.mem_write_lat < 0)
        Fatal(ctx) << "--mem-write-lat must be non-negative\n"; 
    } else if (read_arg("load-file")) {
      ctx.arg.load_file = arg;
    } else if (read_arg("textload-file")) {
      ctx.arg.text_loadfile = arg;
    } else if (read_arg("load-offset")) {
      ctx.arg.load_offset = parse_number(ctx, "load-offset", arg);
    } else if (read_arg("dependency-file")) {
      // e.g. usage --oformat=binary
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
    } else if (read_arg("e") || read_arg("emulation")) { // emulation
      auto check = [&](bool supported, std::string_view name) {
        if (!supported)
          Fatal(ctx) << "'-e=" << arg << "' is not supported; you may want to"
                     << " rebuild with " << name << " support\n";
      };
      // Check valid
      if (arg == "syscmem") {
        check(HAVE_SyscMemArch, SyscMemArch::name);
        ctx.arg.emulation = SyscMemArch::name;
      }  //else if (arg == "lms") {
        // check(HAVE_LMSArch, LMSArch::name);
        // ctx.arg.emulation = LMSArch::name;
      //} 
      else {
        Fatal(ctx) << "unknown -e argument: " << arg;
      }

    } else {
      if (args[0].starts_with('-'))
        Fatal(ctx) << "unknown command line option: " << args[0] << "\n";
      remaining.emplace_back(args[0]);
      args = args.subspan(1);
    }
  }

  if (!ctx.arg.chroot.empty()) {
    if (!ctx.arg.dependency_file.empty())
      ctx.arg.dependency_file = ctx.arg.chroot + "/" + ctx.arg.dependency_file;
  }

  ctx.overwrite_output_file = !ctx.arg.out_shared;

  return remaining;
}

static bool is_file(const fs::path& path) {
  std::error_code error;
  return !fs::is_directory(path, error) && !error;
}

using E = ADPT_TARGET;

template std::vector<std::string_view> 
expand_response_files(Context<E> &, char **);

template std::vector<std::string> 
parse_nonpositional_args(Context<E> &ctx);

}  // namespace adptsysc