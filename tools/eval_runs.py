#!/usr/bin/env python3
"""Repeatable evaluation: the same replay over several start times (real images) or several seeds
(synthetic), with median and range of the results. Single runs can differ a lot (a start one frame
later changes which frames get the visual update), so results are only quoted this way.

Per run: ATE (navigation, % and m), ATE aligned, RPE, hover drift (with --hover), mean NEES of
position and attitude, share of position errors inside 1 and 3 sigma and of attitude errors inside
3 sigma (per axis), share of MSCKF features rejected by the chi2 test, td and its sigma at the end.

Examples (from the repository root):
  tools/eval_runs.py ~/datasets/euroc/MH_01_easy --starts 21.5 22 22.05 22.1 22.5 23 \\
      --configs config/vio.yaml config/replay_euroc.yaml --rpe-length 10 --out /tmp/eval/euroc
  tools/eval_runs.py ~/datasets/synthetic/fig8_tilt_img --starts 0 0.034 0.067 0.1 0.134 0.167 \\
      --configs config/vio.yaml config/replay_synthetic.yaml config/replay_render.yaml --out /tmp/eval/tilt
  tools/eval_runs.py --seeds 1 2 3 4 5 6 7 8 --gen "--scenario figure8" \\
      --configs config/vio.yaml config/replay_synthetic.yaml --out /tmp/eval/fig8
"""

import argparse
import json
import os
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from consistency import evaluate  # noqa: E402


def run_one(job, args):
    """Replay, metrics and consistency of one run. Returns a dict of numbers (or an error)."""
    name, session, start = job
    out = os.path.join(args.out, name + ".csv")
    cmd = [args.replay, session, "--mode", "vio", "--out", out]
    if start is not None:
        cmd += ["--start", str(start)]
    for c in args.configs:
        cmd += ["--config", c]
    log = subprocess.run(cmd, capture_output=True, text=True)
    if log.returncode != 0:
        return {"name": name, "error": log.stderr.strip().splitlines()[-1] if log.stderr else "failed"}
    m_json = os.path.join(args.out, name + ".json")
    m_cmd = [sys.executable, os.path.join(HERE, "metrics.py"), out, session, "--json", m_json]
    if args.rpe_length:
        m_cmd += ["--rpe-length", str(args.rpe_length)]
    if args.hover:
        m_cmd += ["--hover"] + [str(x) for x in args.hover]
    subprocess.run(m_cmd, capture_output=True, text=True, check=True)
    with open(m_json) as f:
        m = json.load(f)
    r = evaluate(out, session)
    err, sig = r["err"], r["sigma"]
    used, rej = np.sum(r["c"]["n_msckf"]), np.sum(r["c"]["n_rejected"])
    res = {
        "name": name,
        "ate_pct": m.get("ate_rmse_pct"), "ate_m": m.get("ate_rmse_m"),
        "ate_al_m": m.get("ate_aligned_rmse_m"), "rpe_pct": m.get("rpe_mean_pct"),
        "hover": m.get("hover_drift_m_per_min"),
        "nees_pos": float(np.mean(r["nees"]["pos"])), "nees_att": float(np.mean(r["nees"]["att"])),
        "in1_pos": float(np.mean(np.abs(err[:, :3]) < sig[:, :3]) * 100),
        "in3_pos": float(np.mean(np.abs(err[:, :3]) < 3 * sig[:, :3]) * 100),
        "in3_att": float(np.mean(np.abs(err[:, 3:]) < 3 * sig[:, 3:]) * 100),
        "rej_pct": float(rej / max(used + rej, 1) * 100),
        "td_ms": float(r["c"]["td_ms"][-1]),
    }
    if "td_sigma_ms" in r["c"]:
        res["td_sig_ms"] = float(r["c"]["td_sigma_ms"][-1])
    return res


COLUMNS = [("ate_pct", "ATE %", 3), ("ate_m", "ATE m", 3), ("ate_al_m", "ATE al m", 3),
           ("rpe_pct", "RPE %", 2), ("hover", "hover m/min", 3), ("nees_pos", "NEES pos", 2),
           ("nees_att", "NEES att", 2), ("in1_pos", "pos in 1sig %", 1), ("in3_pos", "pos in 3sig %", 1), ("in3_att", "att in 3sig %", 1),
           ("rej_pct", "chi2 rej %", 1), ("td_ms", "td ms", 2),
           ("td_sig_ms", "td sig ms", 2)]


def main():
    ap = argparse.ArgumentParser(description="Replay over several starts or seeds, median and range.")
    ap.add_argument("session", nargs="?", help="session folder (with --starts)")
    ap.add_argument("--starts", type=float, nargs="+", help="start times, s (vio_replay --start)")
    ap.add_argument("--seeds", type=int, nargs="+", help="generate synthetic sessions with these seeds")
    ap.add_argument("--gen", default="", help="extra arguments for gen_synthetic.py (with --seeds)")
    ap.add_argument("--reuse", action="store_true",
                    help="use existing seed folders as they are (sessions made outside this tool)")
    ap.add_argument("--configs", nargs="+", required=True, help="config files, in order")
    ap.add_argument("--out", required=True, help="folder for the runs (and generated sessions)")
    ap.add_argument("--rpe-length", type=float, help="RPE segment length, m")
    ap.add_argument("--hover", type=float, nargs=2, help="hover window START END, s")
    ap.add_argument("--jobs", type=int, default=6)
    ap.add_argument("--replay", default="build/vio_replay")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    jobs = []
    if args.seeds:
        for s in args.seeds:
            sess = os.path.join(args.out, f"seed{s}")
            # The generator arguments are kept in the session folder; an existing folder is only
            # reused when they match (or with --reuse, for sessions made outside this tool).
            marker = os.path.join(sess, "eval_runs_gen.txt")
            if os.path.isdir(sess) and not args.reuse:
                old = open(marker).read() if os.path.exists(marker) else None
                if old != args.gen:
                    sys.exit(f"error: {sess} exists but was made with generator arguments "
                             f"{old!r}, not {args.gen!r}. Use a new --out, or --reuse.")
            if not os.path.isdir(sess):
                subprocess.run([sys.executable, os.path.join(HERE, "gen_synthetic.py"), sess,
                                "--seed", str(s)] + args.gen.split(), check=True,
                               capture_output=True)
                with open(marker, "w") as f:
                    f.write(args.gen)
            jobs.append((f"seed{s}", sess, None))
    elif args.session and args.starts:
        jobs = [(f"start{s:g}", os.path.expanduser(args.session), s) for s in args.starts]
    else:
        sys.exit("error: give a session with --starts, or --seeds")

    with ThreadPoolExecutor(args.jobs) as ex:
        results = list(ex.map(lambda j: run_one(j, args), jobs))

    cols = [c for c in COLUMNS if any(r.get(c[0]) is not None for r in results)]
    print(f"{'run':10s}" + "".join(f"{h:>16s}" for _, h, _ in cols))
    for r in results:
        if "error" in r:
            print(f"{r['name']:10s}  error: {r['error']}")
            continue
        print(f"{r['name']:10s}" + "".join(
            f"{r[k]:16.{p}f}" if r.get(k) is not None else f"{'-':>16s}" for k, _, p in cols))
    ok = [r for r in results if "error" not in r]
    line = f"{'median':10s}"
    rng = f"{'range':10s}"
    for k, _, p in cols:
        v = np.array([r[k] for r in ok if r.get(k) is not None])
        line += f"{np.median(v):16.{p}f}" if len(v) else f"{'-':>16s}"
        rng += f"{f'{v.min():.{p}f}-{v.max():.{p}f}':>16s}" if len(v) else f"{'-':>16s}"
    print(line)
    print(rng)
    if len(ok) < len(results):
        print(f"{len(results) - len(ok)} of {len(results)} runs failed")


if __name__ == "__main__":
    main()
