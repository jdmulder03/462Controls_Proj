"""
Live-read a global variable from the running STM32 over SWD, CubeMonitor-style.

Requires CLion's debug session to be STOPPED (pyocd needs exclusive probe access).

Setup (uv):
    uv sync
    uv run pyocd pack install stm32g431rbtx   # one-time

Usage:
    uv run python test.py --list                    # dump all globals + addrs
    uv run python test.py --symbol uwTick           # scalar print
    uv run python test.py --plot-mpu                # live plot of `raw`
    uv run python test.py --plot-mpu --symbol raw   # override struct symbol
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

        for ax, idxs in ((ax_a, (0, 1, 2)), (ax_g, (4, 5, 6))):
            if xs:
                ax.set_xlim(max(0, xs[-1] - window_s), max(window_s, xs[-1]))
                ys = [v for i in idxs for v in bufs[i]]
                if ys:
                    lo, hi = min(ys), max(ys)
                    pad = max(abs(hi - lo) * 0.1, 10)
                    ax.set_ylim(lo - pad, hi + pad)
        return accel_lines + gyro_lines

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
    ap.add_argument("--window", type=float, default=5.0, help="plot window seconds")
    args = ap.parse_args()

    syms = load_symbols(args.elf)

    if args.list:
        for name, (addr, size) in sorted(syms.items(), key=lambda kv: kv[1][0]):
            print(f"0x{addr:08x}  {size:5d}  {name}")
        return

    if args.plot_mpu:
        name = args.symbol or "raw"
        if name not in syms:
            print(f"symbol '{name}' not found; try --list", file=sys.stderr)
            sys.exit(1)
        addr, size = syms[name]
        if size < MPU_NBYTES:
            print(f"warn: '{name}' is {size}B, expected >= {MPU_NBYTES}B", file=sys.stderr)
        print(f"plotting {name} @ 0x{addr:08x} ({size}B) @ {args.rate} Hz")
        with open_session(args.target) as s:
            run_mpu_plot(s.target, addr, args.rate, args.window)
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