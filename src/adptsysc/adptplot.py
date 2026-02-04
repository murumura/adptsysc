#!/usr/bin/env python3
import argparse
import numpy as np
import matplotlib.pyplot as plt

def apply_style():
  """Apply ZepolA-like gray theme styling."""
  plt.rcParams.update({
    "figure.facecolor": "#ada6a6",
    "savefig.facecolor": "#ada6a6",

    "axes.facecolor": "#ada6a6",
    "axes.edgecolor": "#000000",
    "axes.linewidth": 1.0,
    "axes.labelcolor": "#000000",
    "axes.titlecolor": "#000000",

    "axes.grid": True,
    "grid.color": "#888888",
    "grid.linewidth": 0.5,
    "grid.alpha": 0.6,
    "grid.linestyle": "--",

    "font.size": 10,
    "font.family": "sans-serif",
    "axes.labelsize": 11,
    "axes.titlesize": 12,
    "xtick.labelsize": 9,
    "ytick.labelsize": 9,

    "xtick.color": "#000000",
    "ytick.color": "#000000",

    "lines.linewidth": 1.5,
    "lines.antialiased": True,

    "legend.frameon": False,
  })


# PSD PLOT
def plot_psd(freqs, psd,
             fs,
             title="PSD",
             out=None,
             log_freq=False,
             linear=False):

  apply_style()

  freqs = np.asarray(freqs)
  psd = np.asarray(psd)

  if not linear:
    psd = 10.0 * np.log10(np.maximum(psd, 1e-20))

  fig = plt.figure(figsize=(9.6, 4.0), facecolor="#ada6a6")
  ax = fig.add_axes([0.08, 0.12, 0.90, 0.80])
  ax.set_facecolor("none")

  ax.plot(freqs, psd, color="#000000", linewidth=2.0, zorder=5)

  if log_freq:
    ax.set_xscale("log")
    ax.set_xlim([max(1.0, freqs[1]), fs / 2])
  else:
    ax.set_xlim([0, fs / 2])

  ax.set_ylim([-160, 0])
  ax.set_yticks(np.arange(-160, 20, 20))

  x_max = fs / 2
  if x_max >= 500:
    ax.set_xticks(np.arange(0, 501, 100))

  ax.set_xlabel("Frequency (Hz)")
  ax.set_ylabel("PSD (dB/Hz)")
  ax.set_title(title, pad=12)

  for s in ax.spines.values():
    s.set_color("#000000")
    s.set_linewidth(1.0)

  if out:
    plt.savefig(out, dpi=150,
                facecolor=fig.get_facecolor(),
                bbox_inches="tight")
  else:
    plt.show()

  plt.close(fig)


# ============================================================
# ZPK PLOT
# ============================================================
def plot_zpk(zeros, poles, k, title="ZPK", out=None):
  apply_style()

  fig = plt.figure(figsize=(5.2, 5.2), facecolor="#ada6a6")
  ax = fig.add_axes([0.10, 0.10, 0.85, 0.85])
  ax.set_facecolor("none")

  # Unit circle
  th = np.linspace(0, 2*np.pi, 512)
  ax.plot(np.cos(th), np.sin(th),
          color="black", linewidth=1.2)

  # Real / Imag axes
  ax.axhline(0, color="#666666", linestyle="--", linewidth=0.6)
  ax.axvline(0, color="#666666", linestyle="--", linewidth=0.6)

  if len(zeros):
    ax.scatter(zeros[:, 0], zeros[:, 1],
               facecolors="none",
               edgecolors="black",
               s=80,
               linewidths=1.6,
               label="Zeros")

  if len(poles):
    ax.scatter(poles[:, 0], poles[:, 1],
               marker="x",
               color="black",
               s=110,
               linewidths=2.2,
               label="Poles")

  ax.set_aspect("equal", adjustable="box")
  ax.set_xlim([-1.8, 1.8])
  ax.set_ylim([-1.8, 1.8])

  ax.set_xlabel("Real")
  ax.set_ylabel("Imag")
  ax.set_title(f"{title}  (k={k:.3f})", pad=10)

  for s in ax.spines.values():
    s.set_color("#000000")
    s.set_linewidth(1.0)

  ax.legend(loc="upper right")

  if out:
    plt.savefig(out, dpi=150,
                facecolor=fig.get_facecolor(),
                bbox_inches="tight")
  else:
    plt.show()

  plt.close(fig)


def main():
  parser = argparse.ArgumentParser(
    description="ZePolA-style DSP plotting tool",
    formatter_class=argparse.ArgumentDefaultsHelpFormatter
  )

  sub = parser.add_subparsers(dest="cmd", required=True)

  # PSD
  p_psd = sub.add_parser("psd")
  p_psd.add_argument("--freqs", required=True)
  p_psd.add_argument("--psd", required=True)
  p_psd.add_argument("--fs", type=float, required=True)
  p_psd.add_argument("--title", default="PSD")
  p_psd.add_argument("--log-freq", action="store_true")
  p_psd.add_argument("--linear", action="store_true")
  p_psd.add_argument("--out", required=True)

  # ZPK
  p_zpk = sub.add_parser("zpk")
  p_zpk.add_argument("--zeros", required=True)
  p_zpk.add_argument("--poles", required=True)
  p_zpk.add_argument("--k", type=float, required=True)
  p_zpk.add_argument("--title", default="ZPK")
  p_zpk.add_argument("--out", required=True)

  args = parser.parse_args()

  if args.cmd == "psd":
    freqs = np.loadtxt(args.freqs)
    psd = np.loadtxt(args.psd)
    plot_psd(freqs, psd,
             fs=args.fs,
             title=args.title,
             log_freq=args.log_freq,
             linear=args.linear,
             out=args.out)

  elif args.cmd == "zpk":
    zeros = np.loadtxt(args.zeros, delimiter=",")
    poles = np.loadtxt(args.poles, delimiter=",")
    zeros = zeros.reshape(-1, 2) if zeros.size else np.empty((0, 2))
    poles = poles.reshape(-1, 2) if poles.size else np.empty((0, 2))
    plot_zpk(zeros, poles, args.k,
             title=args.title,
             out=args.out)


if __name__ == "__main__":
  main()
