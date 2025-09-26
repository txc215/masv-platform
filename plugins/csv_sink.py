# plugins/csv_sink.py

import csv
from pathlib import Path

class CsvSink:
    def __init__(self, path: str):
        self.path = Path(path); self.f=None; self.w=None; self.fields=None

    def open(self):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.f = open(self.path, "w", newline="")

    def write(self, ev):
        if self.f is None: self.open()
        # 動態欄位：常用優先
        if self.w is None:
            base = ["timestamp","x","y","yaw","vx","vy","omega"]
            keys = [k for k in base if k in ev] + [k for k in ev.keys() if k not in base and not k.startswith("_")]
            self.fields = keys
            self.w = csv.DictWriter(self.f, fieldnames=self.fields)
            self.w.writeheader()
        self.w.writerow({k: ev.get(k,"") for k in self.fields})

    def close(self):
        if self.f: self.f.close()
