#!/usr/bin/env python3
import argparse, csv, heapq, importlib, os, time, yaml
from pathlib import Path
from plugins.csv_source import CsvSource
from plugins.csv_sink import CsvSink

def load_yaml(p: Path):
    with open(p, "r") as f: return yaml.safe_load(f) or {}

def resolve(p: str, base: Path) -> Path:
    s = os.path.expandvars(p); P = Path(s)
    return (base / P).resolve() if not P.is_absolute() else P

def build(dotted: str, params: dict):
    mod, cls = dotted.rsplit(".", 1)
    return getattr(importlib.import_module(mod), cls)(**(params or {}))

def merge_streams(streams):
    heap = []
    for i, it in enumerate(streams):
        try:
            t, row = next(it); heap.append((t, i, row, it))
        except StopIteration: 
            pass
    heapq.heapify(heap)
    while heap:
        t, i, row, it = heapq.heappop(heap); yield (t, row)
        try:
            nt, nrow = next(it); heapq.heappush(heap, (nt, i, nrow, it))
        except StopIteration: 
            pass

def main():
    ap = argparse.ArgumentParser("MASV local runner")
    ap.add_argument("--config", required=True)
    args = ap.parse_args()

    cfg_path = Path(args.config).resolve(); cfg_dir = cfg_path.parent
    cfg = load_yaml(cfg_path)

    # sources
    streams = []
    for s in cfg.get("sources", []):
        prm = s.get("params", {})
        streams.append(CsvSource(
            path=str(resolve(prm["path"], cfg_dir)),
            ts_col=prm.get("ts_col","timestamp"),
            topic=prm.get("topic","imu"),
            rename=prm.get("rename", {})).start()
        )
    if not streams: 
        raise RuntimeError("No sources.")

    # node
    node = build(cfg["node"]["type"], cfg["node"].get("params", {}))
    if hasattr(node, "reset"): node.reset()

    # sink
    sink = CsvSink(path=str(resolve(cfg["sink"]["params"]["path"], cfg_dir)))

    # replay
    BATCH_ORDER = {"imu": 0, "gnss": 1, "mag": 2}  # order of batch
    EPS = 1e-9

    batch = []
    current_t = None

    def flush_batch(tstamp):
        # base on BATCH_ORDER to deal with all same timestamp events
        for pkt in sorted(batch, key=lambda p: BATCH_ORDER.get(p["_topic"], 99)):
            out = node.on_event({"_ts": tstamp, **pkt})
        if out is not None:
            sink.write(out)

    for t, packet in merge_streams(streams):
        if current_t is None:
            current_t = t
        if abs(t - current_t) < EPS:
            batch.append(packet)
            continue

        flush_batch(current_t)      # deal with last timestamp batch
        batch = [packet]            # new batch
        current_t = t

    # last batch
    if batch:
        flush_batch(current_t)

    sink.close()
    print("[OK] wrote", cfg["sink"]["params"]["path"])

if __name__ == "__main__":
    main()
