"""
Live-read a global variable from the running STM32 over SWD, CubeMonitor-style.

Requires CLion's debug session to be STOPPED (pyocd needs exclusive probe access).

Setup (uv):
    uv sync
    uv run pyocd pack install stm32g431rbtx   # one-time

Usage:
    uv run python test.py --list                    # dump all globals + addrs
    uv run python test.py --symbol uwTick           # scalar print
    uv run python test.py --plot-mpu                # live plot of `raw` (accel+gyro)
    uv run python test.py --plot-pitch              # live plot of complementary filter
    uv run python test.py --plot-all                # MPU raw + pitch in one figure
    uv run python test.py --plot-control            # pitch+target+e (rad), u (N), duty (us)
"""

import argparse
import struct
import sys
import time
from collections import deque

from elftools.elf.elffile import ELFFile
from pyocd.core.helpers import ConnectHelper

ELF_DEFAULT = "../../build/Debug/ESET462Project.elf"
TARGET_DEFAULT = "stm32g431rbtx"

TYPE_FMT = {
    "u8": "<B", "i8": "<b",
    "u16": "<H", "i16": "<h",
    "u32": "<I", "i32": "<i",
    "u64": "<Q", "i64": "<q",
    "f32": "<f", "f64": "<d",
}

# MPU6050_RawDataTypeDef: accel.x, accel.y, accel.z, temp, gyro.x, gyro.y, gyro.z
MPU_FMT = "<7h"
MPU_NBYTES = struct.calcsize(MPU_FMT)
MPU_LABELS = ("accel.x", "accel.y", "accel.z", "temp", "gyro.x", "gyro.y", "gyro.z")


def load_symbols(elf_path):
    """Return dict: name -> (addr, size). Objects only (globals/statics)."""
    out = {}
    with open(elf_path, "rb") as f:
        elf = ELFFile(f)
        symtab = elf.get_section_by_name(".symtab")
        if symtab is None:
            raise RuntimeError("ELF has no .symtab (stripped?)")
        for sym in symtab.iter_symbols():
            if sym.entry["st_info"]["type"] != "STT_OBJECT":
                continue
            addr = sym.entry["st_value"]
            size = sym.entry["st_size"]
            if addr == 0 or size == 0:
                continue
            out[sym.name] = (addr, size)
    return out


def open_session(target_name):
    session = ConnectHelper.session_with_chosen_probe(
        target_override=target_name,
        connect_mode="attach",
        options={"resume_on_disconnect": True},
    )
    if session is None:
        print("no debug probe found", file=sys.stderr)
        sys.exit(1)
    return session


def run_scalar(target, addr, fmt, label, rate):
    nbytes = struct.calcsize(fmt)
    period = 1.0 / rate
    t0 = time.time()
    while True:
        data = bytes(target.read_memory_block8(addr, nbytes))
        (val,) = struct.unpack(fmt, data)
        print(f"{time.time() - t0:8.3f}  {label} = {val}")
        time.sleep(period)


def run_counter(target, addr, rate):
    """Poll mpu_int_count and print value + rate (Hz) for ISR verification."""
    period = 1.0 / rate
    t_prev = time.time()
    (v_prev,) = struct.unpack("<I", bytes(target.read_memory_block8(addr, 4)))
    t0 = t_prev
    while True:
        time.sleep(period)
        t_now = time.time()
        (v_now,) = struct.unpack("<I", bytes(target.read_memory_block8(addr, 4)))
        dv = (v_now - v_prev) & 0xFFFFFFFF
        hz = dv / (t_now - t_prev) if t_now > t_prev else 0.0
        print(f"{t_now - t0:8.3f}  count = {v_now:10d}  d={dv:6d}  rate = {hz:8.1f} Hz")
        t_prev, v_prev = t_now, v_now


def _autoscale(ax, xs, ys, window_s, pad_min=10):
    if not xs:
        return
    ax.set_xlim(max(0, xs[-1] - window_s), max(window_s, xs[-1]))
    if ys:
        lo, hi = min(ys), max(ys)
        pad = max(abs(hi - lo) * 0.1, pad_min)
        ax.set_ylim(lo - pad, hi + pad)


def run_mpu_plot(target, addr, rate, window_s):
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    maxlen = max(int(rate * window_s), 10)
    t_buf = deque(maxlen=maxlen)
    bufs = [deque(maxlen=maxlen) for _ in MPU_LABELS]

    fig, (ax_a, ax_g) = plt.subplots(2, 1, sharex=True, figsize=(10, 6))
    fig.suptitle("MPU6050 raw (SWD live)")
    ax_a.set_ylabel("accel (LSB)")
    ax_g.set_ylabel("gyro (LSB)")
    ax_g.set_xlabel("t (s)")

    accel_lines = [ax_a.plot([], [], label=MPU_LABELS[i])[0] for i in range(3)]
    gyro_lines = [ax_g.plot([], [], label=MPU_LABELS[i + 4])[0] for i in range(3)]
    ax_a.legend(loc="upper right")
    ax_g.legend(loc="upper right")
    ax_a.grid(True)
    ax_g.grid(True)

    t0 = time.time()

    def update(_frame):
        try:
            data = bytes(target.read_memory_block8(addr, MPU_NBYTES))
        except Exception as e:
            print(f"read error: {e}", file=sys.stderr)
            return []
        vals = struct.unpack(MPU_FMT, data)
        t_buf.append(time.time() - t0)
        for i, v in enumerate(vals):
            bufs[i].append(v)

        xs = list(t_buf)
        for i, line in enumerate(accel_lines):
            line.set_data(xs, list(bufs[i]))
        for i, line in enumerate(gyro_lines):
            line.set_data(xs, list(bufs[i + 4]))  # skip temp at idx 3

        _autoscale(ax_a, xs, [v for i in (0, 1, 2) for v in bufs[i]], window_s)
        _autoscale(ax_g, xs, [v for i in (4, 5, 6) for v in bufs[i]], window_s)
        return accel_lines + gyro_lines

    interval_ms = max(int(1000 / rate), 10)
    _anim = FuncAnimation(fig, update, interval=interval_ms, blit=False, cache_frame_data=False)
    plt.show()


def run_pitch_plot(target, addr_filt, addr_acc, rate, window_s):
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    maxlen = max(int(rate * window_s), 10)
    t_buf = deque(maxlen=maxlen)
    filt_buf = deque(maxlen=maxlen)
    acc_buf = deque(maxlen=maxlen)

    fig, ax = plt.subplots(figsize=(10, 4))
    fig.suptitle("Complementary filter (SWD live)")
    ax.set_ylabel("pitch (rad)")
    ax.set_xlabel("t (s)")
    (l_filt,) = ax.plot([], [], label="pitch_rad (fused)")
    (l_acc,) = ax.plot([], [], label="pitch_acc_rad (accel only)", alpha=0.5)
    ax.legend(loc="upper right")
    ax.grid(True)

    t0 = time.time()

    def update(_frame):
        try:
            filt = struct.unpack("<f", bytes(target.read_memory_block8(addr_filt, 4)))[0]
            acc = struct.unpack("<f", bytes(target.read_memory_block8(addr_acc, 4)))[0]
        except Exception as e:
            print(f"read error: {e}", file=sys.stderr)
            return []
        t_buf.append(time.time() - t0)
        filt_buf.append(filt)
        acc_buf.append(acc)
        xs = list(t_buf)
        l_filt.set_data(xs, list(filt_buf))
        l_acc.set_data(xs, list(acc_buf))
        _autoscale(ax, xs, list(filt_buf) + list(acc_buf), window_s, pad_min=1)
        return [l_filt, l_acc]

    interval_ms = max(int(1000 / rate), 10)
    _anim = FuncAnimation(fig, update, interval=interval_ms, blit=False, cache_frame_data=False)
    plt.show()


def run_control_plot(target, addrs, rate, window_s):
    """Tuning dashboard: angle tracking, pitch rate, u decomposition, actuator.

    addrs is a dict with: pitch, target, acc, rate_, e, u_ff, u_pd, u, duty,
                          kp, kd, ff_gain
    """
    import math
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    maxlen = max(int(rate * window_s), 10)
    bufs = {k: deque(maxlen=maxlen) for k in
            ("t", "pitch", "target", "acc", "rate_", "e", "u_ff", "u_pd", "u", "duty")}

    fig, (ax_ang, ax_rate, ax_u, ax_d) = plt.subplots(
        4, 1, sharex=True, figsize=(11, 10),
        gridspec_kw={"height_ratios": [1.3, 1.0, 1.3, 1.0]})
    fig.suptitle("PD + gravity feedforward (SWD live)")

    ax_ang.set_ylabel("angle (rad)")
    ax_rate.set_ylabel("rate (rad/s)")
    ax_u.set_ylabel("thrust (N)")
    ax_d.set_ylabel("duty (us)")
    ax_d.set_xlabel("t (s)")

    (l_pitch,)  = ax_ang.plot([], [], label="pitch", color="tab:blue", lw=1.6)
    (l_target,) = ax_ang.plot([], [], label="target", color="black", ls="--", lw=1.2)
    (l_acc,)    = ax_ang.plot([], [], label="accel-only", color="tab:gray", alpha=0.5, lw=1.0)
    (l_e,)      = ax_ang.plot([], [], label="error", color="tab:orange", alpha=0.8, lw=1.0)

    (l_rate,) = ax_rate.plot([], [], label="pitch_rate", color="tab:purple", lw=1.2)
    ax_rate.axhline(0, color="black", lw=0.5, alpha=0.5)

    (l_uff,) = ax_u.plot([], [], label="u_ff (gravity)", color="tab:green", lw=1.3)
    (l_upd,) = ax_u.plot([], [], label="u_pd (PD)", color="tab:orange", lw=1.3)
    (l_u,)   = ax_u.plot([], [], label="u_total", color="tab:red", lw=1.6)
    ax_u.axhline(0, color="black", lw=0.5, alpha=0.5)

    (l_duty,) = ax_d.plot([], [], label="duty_cmd", color="tab:green", lw=1.4)
    ax_d.axhline(1000, color="red", lw=0.8, ls=":", alpha=0.6, label="min/max")
    ax_d.axhline(2000, color="red", lw=0.8, ls=":", alpha=0.6)

    for a in (ax_ang, ax_rate, ax_u, ax_d):
        a.legend(loc="upper right", fontsize=9)
        a.grid(True, alpha=0.3)

    gain_txt = fig.text(0.01, 0.98, "", fontsize=9, family="monospace",
                        verticalalignment="top")

    t0 = time.time()

    def f32(a):  # read float32
        return struct.unpack("<f", bytes(target.read_memory_block8(a, 4)))[0]

    def u32(a):
        return struct.unpack("<I", bytes(target.read_memory_block8(a, 4)))[0]

    def update(_frame):
        try:
            pitch  = f32(addrs["pitch"])
            tgt    = f32(addrs["target"])
            acc    = f32(addrs["acc"])
            pr     = f32(addrs["rate_"])
            e      = f32(addrs["e"])
            u_ff   = f32(addrs["u_ff"])
            u_pd   = f32(addrs["u_pd"])
            u      = f32(addrs["u"])
            duty   = u32(addrs["duty"])
            kp     = f32(addrs["kp"])
            kd     = f32(addrs["kd"])
            ff_gn  = f32(addrs["ff_gain"])
        except Exception as ex:
            print(f"read error: {ex}", file=sys.stderr)
            return []

        bufs["t"].append(time.time() - t0)
        bufs["pitch"].append(pitch)
        bufs["target"].append(tgt)
        bufs["acc"].append(acc)
        bufs["rate_"].append(pr)
        bufs["e"].append(e)
        bufs["u_ff"].append(u_ff)
        bufs["u_pd"].append(u_pd)
        bufs["u"].append(u)
        bufs["duty"].append(duty)

        xs = list(bufs["t"])
        l_pitch.set_data(xs,  list(bufs["pitch"]))
        l_target.set_data(xs, list(bufs["target"]))
        l_acc.set_data(xs,    list(bufs["acc"]))
        l_e.set_data(xs,      list(bufs["e"]))
        l_rate.set_data(xs,   list(bufs["rate_"]))
        l_uff.set_data(xs,    list(bufs["u_ff"]))
        l_upd.set_data(xs,    list(bufs["u_pd"]))
        l_u.set_data(xs,      list(bufs["u"]))
        l_duty.set_data(xs,   list(bufs["duty"]))

        _autoscale(ax_ang,  xs, list(bufs["pitch"]) + list(bufs["target"])
                                 + list(bufs["acc"]) + list(bufs["e"]),
                   window_s, pad_min=0.1)
        _autoscale(ax_rate, xs, list(bufs["rate_"]), window_s, pad_min=0.2)
        _autoscale(ax_u,    xs, list(bufs["u_ff"]) + list(bufs["u_pd"])
                                 + list(bufs["u"]),
                   window_s, pad_min=0.2)
        _autoscale(ax_d,    xs, list(bufs["duty"]) + [1000, 2000],
                   window_s, pad_min=50)

        # Live gain + state readout. Handy while you twiddle from the debugger.
        gain_txt.set_text(
            f"Kp={kp:6.3f} N/rad   Kd={kd:6.3f} N·s/rad   ff_gain={ff_gn:5.3f}\n"
            f"target={math.degrees(tgt):+6.1f}°   pitch={math.degrees(pitch):+6.1f}°   "
            f"e={math.degrees(e):+6.1f}°   ω={math.degrees(pr):+7.1f}°/s\n"
            f"u_ff={u_ff:+5.3f}N  u_pd={u_pd:+5.3f}N  u={u:+5.3f}N   duty={duty} us"
        )

        return [l_pitch, l_target, l_acc, l_e, l_rate, l_uff, l_upd, l_u, l_duty, gain_txt]

    interval_ms = max(int(1000 / rate), 10)
    _anim = FuncAnimation(fig, update, interval=interval_ms, blit=False, cache_frame_data=False)
    plt.tight_layout(rect=(0, 0, 1, 0.94))
    plt.show()


def run_all_plot(target, addr_mpu, addr_filt, addr_acc, rate, window_s):
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    maxlen = max(int(rate * window_s), 10)
    t_buf = deque(maxlen=maxlen)
    bufs = [deque(maxlen=maxlen) for _ in MPU_LABELS]
    filt_buf = deque(maxlen=maxlen)
    acc_buf = deque(maxlen=maxlen)

    fig, (ax_a, ax_g, ax_p) = plt.subplots(3, 1, sharex=True, figsize=(10, 9))
    fig.suptitle("MPU6050 raw + pitch (SWD live)")
    ax_a.set_ylabel("accel (LSB)")
    ax_g.set_ylabel("gyro (LSB)")
    ax_p.set_ylabel("pitch (rad)")
    ax_p.set_xlabel("t (s)")

    accel_lines = [ax_a.plot([], [], label=MPU_LABELS[i])[0] for i in range(3)]
    gyro_lines = [ax_g.plot([], [], label=MPU_LABELS[i + 4])[0] for i in range(3)]
    (l_filt,) = ax_p.plot([], [], label="pitch_rad (fused)")
    (l_acc,) = ax_p.plot([], [], label="pitch_acc_rad (accel)", alpha=0.5)
    for a in (ax_a, ax_g, ax_p):
        a.legend(loc="upper right")
        a.grid(True)

    t0 = time.time()

    def update(_frame):
        try:
            mpu_data = bytes(target.read_memory_block8(addr_mpu, MPU_NBYTES))
            filt = struct.unpack("<f", bytes(target.read_memory_block8(addr_filt, 4)))[0]
            acc = struct.unpack("<f", bytes(target.read_memory_block8(addr_acc, 4)))[0]
        except Exception as e:
            print(f"read error: {e}", file=sys.stderr)
            return []
        vals = struct.unpack(MPU_FMT, mpu_data)
        t_buf.append(time.time() - t0)
        for i, v in enumerate(vals):
            bufs[i].append(v)
        filt_buf.append(filt)
        acc_buf.append(acc)

        xs = list(t_buf)
        for i, line in enumerate(accel_lines):
            line.set_data(xs, list(bufs[i]))
        for i, line in enumerate(gyro_lines):
            line.set_data(xs, list(bufs[i + 4]))
        l_filt.set_data(xs, list(filt_buf))
        l_acc.set_data(xs, list(acc_buf))

        _autoscale(ax_a, xs, [v for i in (0, 1, 2) for v in bufs[i]], window_s)
        _autoscale(ax_g, xs, [v for i in (4, 5, 6) for v in bufs[i]], window_s)
        _autoscale(ax_p, xs, list(filt_buf) + list(acc_buf), window_s, pad_min=1)
        return accel_lines + gyro_lines + [l_filt, l_acc]

    interval_ms = max(int(1000 / rate), 10)
    _anim = FuncAnimation(fig, update, interval=interval_ms, blit=False, cache_frame_data=False)
    plt.show()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--elf", default=ELF_DEFAULT)
    ap.add_argument("--target", default=TARGET_DEFAULT)
    ap.add_argument("--symbol", help="global to poll (scalar mode)")
    ap.add_argument("--type", default="u32", choices=list(TYPE_FMT))
    ap.add_argument("--rate", type=float, default=50.0, help="Hz")
    ap.add_argument("--list", action="store_true", help="list globals and exit")
    ap.add_argument("--plot-mpu", action="store_true",
                    help="live-plot MPU6050_RawDataTypeDef global (default: 'raw')")
    ap.add_argument("--plot-pitch", action="store_true",
                    help="live-plot pitch_rad vs pitch_acc_rad")
    ap.add_argument("--plot-all", action="store_true",
                    help="live-plot MPU raw + pitch in one figure")
    ap.add_argument("--plot-control", action="store_true",
                    help="live-plot pitch/target/e (rad), ctrl_u (N), duty_cmd (us)")
    ap.add_argument("--window", type=float, default=5.0, help="plot window seconds")
    ap.add_argument("--count", action="store_true",
                    help="poll mpu_int_count and print rate (Hz)")
    args = ap.parse_args()

    syms = load_symbols(args.elf)

    if args.list:
        for name, (addr, size) in sorted(syms.items(), key=lambda kv: kv[1][0]):
            print(f"0x{addr:08x}  {size:5d}  {name}")
        return

    if args.count:
        name = args.symbol or "mpu_int_count"
        if name not in syms:
            print(f"symbol '{name}' not found; try --list", file=sys.stderr)
            sys.exit(1)
        addr, size = syms[name]
        print(f"polling {name} @ 0x{addr:08x} ({size}B) @ {args.rate} Hz")
        print("press Ctrl-C to stop\n")
        with open_session(args.target) as s:
            try:
                run_counter(s.target, addr, args.rate)
            except KeyboardInterrupt:
                print("\nstopped")
        return

    def require(name):
        if name not in syms:
            print(f"symbol '{name}' not found; try --list", file=sys.stderr)
            sys.exit(1)
        return syms[name]

    if args.plot_mpu:
        name = args.symbol or "raw"
        addr, size = require(name)
        if size < MPU_NBYTES:
            print(f"warn: '{name}' is {size}B, expected >= {MPU_NBYTES}B", file=sys.stderr)
        print(f"plotting {name} @ 0x{addr:08x} ({size}B) @ {args.rate} Hz")
        with open_session(args.target) as s:
            run_mpu_plot(s.target, addr, args.rate, args.window)
        return

    if args.plot_pitch:
        addr_filt, _ = require("pitch_rad")
        addr_acc, _ = require("pitch_acc_rad")
        print(f"plotting pitch_rad/pitch_acc_rad @ {args.rate} Hz")
        with open_session(args.target) as s:
            run_pitch_plot(s.target, addr_filt, addr_acc, args.rate, args.window)
        return

    if args.plot_control:
        addrs = {
            "pitch":   require("pitch_rad")[0],
            "target":  require("pitch_target_rad")[0],
            "acc":     require("pitch_acc_rad")[0],
            "rate_":   require("pitch_rate_rad_s")[0],
            "e":       require("ctrl_e")[0],
            "u_ff":    require("ctrl_u_ff")[0],
            "u_pd":    require("ctrl_u_pd")[0],
            "u":       require("ctrl_u")[0],
            "duty":    require("duty_cmd")[0],
            "kp":      require("ctrl_kp")[0],
            "kd":      require("ctrl_kd")[0],
            "ff_gain": require("ff_gain")[0],
        }
        print(f"plotting PD controller @ {args.rate} Hz")
        with open_session(args.target) as s:
            run_control_plot(s.target, addrs, args.rate, args.window)
        return

    if args.plot_all:
        mpu_name = args.symbol or "raw"
        addr_mpu, size = require(mpu_name)
        if size < MPU_NBYTES:
            print(f"warn: '{mpu_name}' is {size}B, expected >= {MPU_NBYTES}B", file=sys.stderr)
        addr_filt, _ = require("pitch_rad")
        addr_acc, _ = require("pitch_acc_rad")
        print(f"plotting {mpu_name} + pitch @ {args.rate} Hz")
        with open_session(args.target) as s:
            run_all_plot(s.target, addr_mpu, addr_filt, addr_acc, args.rate, args.window)
        return

    if not args.symbol:
        ap.error("--symbol is required (or use --list / --plot-mpu)")
    if args.symbol not in syms:
        print(f"symbol '{args.symbol}' not found; try --list", file=sys.stderr)
        sys.exit(1)

    addr, size = syms[args.symbol]
    fmt = TYPE_FMT[args.type]
    if struct.calcsize(fmt) > size:
        print(f"warn: --type is {struct.calcsize(fmt)}B but symbol is {size}B", file=sys.stderr)

    print(f"{args.symbol} @ 0x{addr:08x} ({size}B) as {args.type} @ {args.rate} Hz")
    print("press Ctrl-C to stop\n")
    with open_session(args.target) as s:
        try:
            run_scalar(s.target, addr, fmt, args.symbol, args.rate)
        except KeyboardInterrupt:
            print("\nstopped")


if __name__ == "__main__":
    main()