# plugins/csv_source.py

import csv
from pathlib import Path
from typing import Iterator, Tuple, Dict, Any

class CsvSource:
    def __init__(self, path: str, ts_col: str, topic: str, rename: Dict[str,str]=None):
        self.path = Path(path); self.ts_col = ts_col; self.topic = topic
        self.rename = rename or {}

    def start(self) -> Iterator[Tuple[float, Dict[str, Any]]]:
        with open(self.path, "r", newline="") as f:
            r = csv.DictReader(f)
            for row in r:
                row = {self.rename.get(k, k): v for k, v in row.items()}
                if self.ts_col not in row:
                    continue
                try:
                    t = float(row[self.ts_col])
                except:
                    continue
                ev = {"_topic": self.topic, "timestamp": t}
                for k, v in row.items():
                    if k == self.ts_col: 
                        continue
                    try: ev[k] = float(v)
                    except: ev[k] = v
                yield (t, ev)
