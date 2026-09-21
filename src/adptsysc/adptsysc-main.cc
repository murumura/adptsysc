#include <adptsysc/adptsysc.hh>
#include <adptsysc/arch.hh>
#include <adptsysc/config.hh>
#include <adptsysc/sysc-mem-tlm.hh>
#include <adptsysc/sysc-r2sdffft-tlm.hh>
#include <adptsysc/sysc-mem-cycle.hh>
#include <adptsysc/sysc-r2sdffft-cycle.hh>
#include <adptsysc/ovsfdaft-cycle.hh>
namespace adptsysc {


template <typename E>
std::unique_ptr<OutputFile<E>>
OutputFile<E>::open(Context<E>& ctx, std::string path, i64 filesize, mode_t perm) {
  if (path.starts_with('/') && !ctx.arg.chroot.empty()) {
    path = ctx.arg.chroot + "/" + path_clean(path);
  }

  auto file = std::make_unique<OutputFile<E>>(std::move(path), filesize, perm);

  if (ctx.arg.filler != -1 && file->buf) {
    std::memset(file->buf, ctx.arg.filler, static_cast<std::size_t>(filesize));
  }

  return file;
}

template <typename E>
void TraceFile<E>::open(std::string path, i64 filesize, mode_t perm) {
  close();

  const std::string saved_path = path;

  outfile = OutputFile<E>::open(ctx, std::move(path), filesize, perm);
  is_enabled = (outfile != nullptr);

  // persistent label backed by ctx.string_pool
  trace_name = save_string(ctx, "trace");
  trace_path = save_string(ctx, saved_path);
}

// Since adptsysc_main is a template, we can't run it without a type parameter.
// We speculatively run adptsysc_main with SyscMemArch, 
// and if the speculation was wrong, re-run it with an actual target type.
template <typename E>
int redo_main(std::string_view target, int argc, char** argv) {
  if constexpr (HAVE_SyscMemArch)
    if (target == SyscMemArch::name)
      return adptsysc_main<SyscMemArch>(argc, argv);

  if constexpr (HAVE_R2SdfFFTTLMArch)
    if (target == R2SdfFFTTLMArch::name)
      return adptsysc_main<R2SdfFFTTLMArch>(argc, argv);

  if constexpr (HAVE_SyscMemoryCycleArch)
    if (target == SyscMemoryCycleArch::name)
      return adptsysc_main<SyscMemoryCycleArch>(argc, argv);

  if constexpr (HAVE_R2SdfFFTCycleArch)
    if (target == R2SdfFFTCycleArch::name)
      return adptsysc_main<R2SdfFFTCycleArch>(argc, argv);

  if constexpr (HAVE_OverlapSaveFdafCycleArch)
    if (target == OverlapSaveFdafCycleArch::name)
      return adptsysc_main<OverlapSaveFdafCycleArch>(argc, argv);

  if constexpr (HAVE_WbcicEngineArch)
    if (target == WbcicEngineArch::name)
      return adptsysc_main<WbcicEngineArch>(argc, argv);

  std::abort();
}

template <typename E>
int adptsysc_main(int argc, char **argv) {
  Context<E> ctx;
  // Parse non-positional command line options
  ctx.cmdline_args = expand_response_files(ctx, argv);
  std::vector<std::string> file_args = parse_nonpositional_args(ctx);

  // If no -e option is given, deduce it from input files.
  if (ctx.arg.emulation.empty())
    Fatal(ctx) << "Emulation option empty\n";

  // Redo if -e does not match with our speculation.
  if (ctx.arg.emulation != E::name)
    return redo_main<E>(ctx.arg.emulation, argc, argv);

  if (!ctx.arg.directory.empty())
    if (chdir(ctx.arg.directory.c_str()) == -1)
      Fatal(ctx) << "chdir failed: " << ctx.arg.directory
                 << ": " << errno_string() << "\n";
  
  // Run testbench if requested
  if (ctx.arg.run_testbench) {
    Out(ctx) << "run_testbench for " <<  E::name << "\n";
    bool test_success = E::Impl_T::run_testbench(ctx);
    if (!test_success) {
      return 1; // Return error code if testbench failed
    }
  }

  if (ctx.arg.quick_exit)
    _exit(0);
  ctx.checkpoint();
  return 0;
}

using E = ADPT_TARGET;
template class OutputFile<E>;
template class TraceFile<E>;
template int adptsysc_main<E>(int, char **);
template int redo_main<E>(std::string_view, int, char **);

}