#!/usr/bin/env python3
"""
PC tool for the SD-SPI emulator: talks to the card (ESP32-S3) and the test
host (classic ESP32) over their serial consoles and runs fault scenarios.

  sdemu.py run  --card /dev/ttyACM0 --host /dev/ttyUSB0 tools/scenarios/*.yaml
  sdemu.py card --card /dev/ttyACM0 "status"
  sdemu.py host --host /dev/ttyUSB0 "fat 16"

Close any idf.py monitor on those ports first. Opening a port toggles DTR/RTS
and resets the board, so the tool waits for both boards to boot.

Scenario format (YAML):

  name: write_error
  description: the 2nd single-block write is rejected
  faults:                       # card "fault add" lines, applied before the steps
    - action=write_error cmd=24 nth=2
  steps:                        # host commands and what their RESULT must say
    - host: write 3000
      expect: {status: ok}
    - host: write 3001
      expect: {status: fail, err: ESP_FAIL, step: write}
    - host: write 3002
      expect: {status: ok, min_ms: 0, max_ms: 5000}
"""
import argparse
import re
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("needs pyserial: pip install pyserial")

BOOT_WAIT_S = 3.5
RESULT_RE = re.compile(r"^RESULT\s+(.*)$")


class Console:
    """Line-based serial console with a prompt/reply convention."""

    def __init__(self, port, name, verbose=False):
        self.name = name
        self.verbose = verbose
        self.ser = serial.Serial(port, 115200, timeout=0.1)

    def close(self):
        self.ser.close()

    def drain(self):
        time.sleep(0.05)
        self.ser.reset_input_buffer()

    def send(self, line):
        if self.verbose:
            print(f"  {self.name} <- {line}")
        self.ser.write((line + "\n").encode())
        self.ser.flush()

    def lines(self, timeout):
        """Yield decoded lines until timeout."""
        deadline = time.monotonic() + timeout
        buf = b""
        while time.monotonic() < deadline:
            chunk = self.ser.read(256)
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                text = raw.decode(errors="replace").strip()
                text = re.sub(r"\x1b\[[0-9;]*m", "", text)   # ESP_LOG colours
                if self.verbose and text:
                    print(f"  {self.name} -> {text}")
                yield text


class Card(Console):
    def command(self, line, timeout=5.0):
        """Send a card command; return (ok, output lines)."""
        self.send(line)
        out = []
        for text in self.lines(timeout):
            if text == "OK":
                return True, out
            if text.startswith("ERR"):
                return False, out + [text]
            if text and not text.startswith("T "):
                out.append(text)
        return False, out + ["(timeout)"]


class Host(Console):
    def run(self, line, timeout=60.0):
        """Send a host test; return the RESULT key/values (status=timeout if none)."""
        self.send(line)
        for text in self.lines(timeout):
            m = RESULT_RE.match(text)
            if m:
                return dict(kv.split("=", 1) for kv in m.group(1).split() if "=" in kv)
        return {"status": "timeout", "err": "-", "step": "-", "ms": "-1"}


def load_scenario(path):
    try:
        import yaml
    except ImportError:
        sys.exit("needs PyYAML: pip install pyyaml")
    with open(path) as f:
        sc = yaml.safe_load(f)
    if not isinstance(sc, dict) or "steps" not in sc:
        raise ValueError(f"{path}: needs at least 'steps'")
    sc.setdefault("name", path)
    sc.setdefault("faults", [])
    for i, step in enumerate(sc["steps"]):
        if "host" not in step:
            raise ValueError(f"{path}: step {i} has no 'host' command")
        step.setdefault("expect", {"status": "ok"})
    return sc


def check(expect, res):
    """Return a list of mismatches between an expectation and a RESULT."""
    problems = []
    for key in ("status", "err", "step"):
        if key in expect and str(expect[key]) != res.get(key):
            problems.append(f"{key}={res.get(key)} (want {expect[key]})")
    ms = int(res.get("ms", "-1"))
    if "min_ms" in expect and ms < int(expect["min_ms"]):
        problems.append(f"ms={ms} (want >= {expect['min_ms']})")
    if "max_ms" in expect and ms > int(expect["max_ms"]):
        problems.append(f"ms={ms} (want <= {expect['max_ms']})")
    return problems


def run_scenario(card, host, sc):
    print(f"\n=== {sc['name']}: {sc.get('description', '').strip()}")
    for cmd in ("trace off", "fault clear", "diag reset"):
        ok, out = card.command(cmd)
        if not ok:
            print(f"  card '{cmd}' failed: {out}")
            return False
    for rule in sc["faults"]:
        ok, out = card.command(f"fault add {rule}")
        if not ok:
            print(f"  card rejected fault '{rule}': {out}")
            return False
        print(f"  fault: {rule}")

    passed = True
    for step in sc["steps"]:
        res = host.run(step["host"], timeout=float(step.get("timeout", 60)))
        problems = check(step["expect"], res)
        mark = "ok  " if not problems else "FAIL"
        summary = f"status={res.get('status')} err={res.get('err')} step={res.get('step')} ms={res.get('ms')}"
        print(f"  [{mark}] {step['host']:<14} {summary}" + (f"  <- {', '.join(problems)}" if problems else ""))
        passed &= not problems

    ok, out = card.command("fault list")
    for line in out:
        print(f"  {line}")
    card.command("fault clear")
    print(f"  -> {'PASS' if passed else 'FAIL'}")
    return passed


def cmd_run(args):
    scenarios = [load_scenario(p) for p in args.scenarios]
    card = Card(args.card, "card", args.verbose)
    host = Host(args.host, "host", args.verbose)
    print(f"waiting {BOOT_WAIT_S} s for both boards to boot")
    time.sleep(BOOT_WAIT_S)
    card.drain()
    host.drain()
    try:
        results = [(sc["name"], run_scenario(card, host, sc)) for sc in scenarios]
    finally:
        card.close()
        host.close()
    print("\nsummary:")
    for name, ok in results:
        print(f"  {'PASS' if ok else 'FAIL'}  {name}")
    failed = sum(1 for _, ok in results if not ok)
    if args.summary:
        with open(args.summary, "w") as f:
            for name, ok in results:
                f.write(f"{'PASS' if ok else 'FAIL'} {name}\n")
    print(f"{len(results) - failed}/{len(results)} scenarios behaved as expected")
    return 1 if failed else 0


def cmd_card(args):
    card = Card(args.card, "card", args.verbose)
    time.sleep(BOOT_WAIT_S)
    card.drain()
    ok, out = card.command(" ".join(args.command), timeout=args.timeout)
    print("\n".join(out))
    card.close()
    return 0 if ok else 1


def cmd_host(args):
    host = Host(args.host, "host", args.verbose)
    time.sleep(BOOT_WAIT_S)
    host.drain()
    res = host.run(" ".join(args.command), timeout=args.timeout)
    print(" ".join(f"{k}={v}" for k, v in res.items()))
    host.close()
    return 0 if res.get("status") == "ok" else 1


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("-v", "--verbose", action="store_true", help="echo all serial traffic")
    sub = p.add_subparsers(dest="cmd", required=True)

    r = sub.add_parser("run", help="run scenario files")
    r.add_argument("--card", required=True, help="serial port of the S3 card")
    r.add_argument("--host", required=True, help="serial port of the ESP32 host")
    r.add_argument("--summary", help="also write 'PASS|FAIL name' lines to this file")
    r.add_argument("scenarios", nargs="+")
    r.set_defaults(fn=cmd_run)

    c = sub.add_parser("card", help="send one command to the card")
    c.add_argument("--card", required=True)
    c.add_argument("--timeout", type=float, default=5.0)
    c.add_argument("command", nargs="+")
    c.set_defaults(fn=cmd_card)

    h = sub.add_parser("host", help="run one host test")
    h.add_argument("--host", required=True)
    h.add_argument("--timeout", type=float, default=60.0)
    h.add_argument("command", nargs="+")
    h.set_defaults(fn=cmd_host)

    args = p.parse_args()
    sys.exit(args.fn(args))


if __name__ == "__main__":
    main()
