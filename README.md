# Modular AI Simulation and Validation (MASV) — Sensor Fusion & ML (ROS2 + Local Runner)

**MASV** is a modular toolkit for testing and comparing sensor-fusion and ML algorithms with **ROS2** or a **lightweight local runner** (no ROS2).
Use ROS2 for live systems and integration; use the local runner for quick offline experiments on CSV logs. Switch between them without changing algorithms.

**Algorithms included**

* **EKF** (Extended Kalman Filter)
* **UKF** (Unscented Kalman Filter)
* **GRU** (ONNX inference on 6-D IMU sequences)

**What you get**

* **ROS2 workflow**: nodes, params (YAML), topics (`/imu`, `/gnss`, `/mag`), and a containerized setup.
* **Local runner**: single `main.py` + YAML, same algorithms, same configs; batch-safe same-timestamp handling; CSV in/out.
* **Viewers**: model viewer (wxPython) and result viewer for quick inspection.
* **CI hooks**: smoke tests on simulated data.

**What this README covers**

* Running the local simulation pipeline (EKF / UKF / GRU) with YAML + CSV.

**Looking for something eles?**

- ROS2 (containerized with docker): coming soon
- CI (GitHub Actions): coming soon
- Model Viewer (wxPython): coming soon
- Result Viewer: coming soon
- Algorithm details (EKF/UKF/GRU): coming soon

---

## Two ways to run (pick what you need)

**1) ROS2 (recommended for integration & real time)**

* Launch the EKF/UKF/GRU nodes with ROS2 parameter files.
* Reuse the same algorithm cores as the local runner.

**2) Local runner (fast offline on CSV)**

* Keep your CSV headers; map via YAML (`rename` / `map_*`).
* One predict per timestamp; other same-time measurements are update-only.
* Outputs to `outputs/state_*.csv`.

```bash
# create env from environment.yml
conda env create -f environment.yml
conda activate mod_ai_svp

# run
python main.py --config configs/run_ekf.yaml
python main.py --config configs/run_ukf.yaml
python main.py --config configs/run_gru.yaml

```

Outputs go to `data/simple_outputs/state_*.csv` with columns like `timestamp,x,y,yaw,...`.

---

## How `main.py` works

1. **Sources**: read CSV(s), apply optional `rename`, yield events `{"_topic": "...", "timestamp": t, ...}`.
2. **Merge**: k-way merge by timestamp across sources.
3. **Batching**: group **same-timestamp** events and process in fixed order (default **IMU -> GNSS -> MAG**).

   * One **predict** per timestamp; other same-time events are **update** only.
   * Writer policy: **write once per timestamp** (last state of that time).
4. **Node**: call `node.on_event(ev)`; nodes implement:

   ```python
   class BaseNode:
       def reset(self) -> None: ...
       def on_event(self, ev: dict) -> dict | None: ...
   ```
5. **Sink**: write returned dict to CSV.

---

## Data format (for local runner)

* IMU: `timestamp_sec, accel_x_mps2, accel_y_mps2, accel_z_mps2, gyro_x_radps, gyro_y_radps, gyro_z_radps`
* MAG: `timestamp_sec, mag_x_uT, mag_y_uT, mag_z_uT`
* GNSS: `timestamp_sec, gnss_lat_deg, gnss_lon_deg, gnss_alt_m`
  Units: accel **m/s²**, gyro **rad/s**, lat/lon in **deg** (converted to meters internally).

---

## Minimal UKF config (example)


```yaml
sources:
  - type: csv
    params:
      path: $your_imu_csv_data
      ts_col: timestamp_sec
      topic: imu
      rename: { gyro_z_radps: wz }     # optional header mapping
  - type: csv
    params:
      path: $your_gnss_csv_data
      ts_col: timestamp_sec
      topic: gnss
      rename: { gnss_lat_deg: lat, gnss_lon_deg: lon }
  - type: csv
    params:
      path: $your_mag_csv_data
      ts_col: timestamp_sec
      topic: mag
      rename: { mag_x_uT: mx, mag_y_uT: my }

node:
  type: plugins.algos.ukf_node.UKFNode   # dotted path to your node class
  params:
    Q: [[0.01,0,0],[0,0.01,0],[0,0,0.001]]
    R_gnss: [[1,0],[0,1]]
    R_mag: [[0.05]]

sink:
  type: plugins.csv_sink.CsvSink
  params:
    path: data/simple_outputs/state_ukf.csv

options:
  realtime: false   # replay with wall-clock pacing if true
```

---

## Flowchart

```mermaid
flowchart TD
  A[Start] --> B[Load config .yaml]
  B --> C[Init Sources .csv file]
  C --> D[Init Node <br/>EKF / UKF / GRU]
  D --> E[Merge by timestamp]
  E --> F{Same-timestamp batch}
  F --> G[Process batch events by topic order]
  G --> H[Keep last state of this timestamp]
  H --> I[Write once to CSV]
  I --> J{More events?}
  J --> |Yes| F
  J -->|No| K[Done]

```
---

## How things stay consistent

* The **same EKF/UKF/GRU cores** power both ROS2 nodes and the local runner.
* Local runner batches same-timestamp events in a fixed order (default **IMU -> GNSS -> MAG**) and **writes once per timestamp**.

---

## Switching algorithms

Change the node dotted path in YAML:

* EKF: `plugins.algos.ekf_node.EKFNode`
* UKF: `plugins.algos.ukf_node.UKFNode`
* GRU: `plugins.algos.gru_node.GRUNode` (expects `[ax,ay,az,gx,gy,gz]` window; set `onnx_path`, `seq_len`)

No changes to `main.py` required.

---

## Simulated data (optional)

Use a small script to generate:

* `data/sample_inputs/imu.csv` (e.g., 50 Hz), `data/sample_inputs/mag.csv` (10 Hz), `data/sample_inputs/gnss.csv` (1 Hz), 10 s arc motion.
  This is handy for smoke tests and CI.

---

## Extending

* Add a new node in `plugins/algos/new_node.py` implementing `reset` and `on_event`.
* Point YAML `new_node.type` to `plugins.algos.new_node.NewNode`.
* Add new sources/sinks by mirroring `csv_source.py` / `csv_sink.py`.


---

## Troubleshooting

- In progress

---

## Acknowledgment

The dataset used in this project (`utbm_robocar_dataset_20190110_noimage.bag`) was kindly provided by the **Université de Technologie de Belfort-Montbéliard (UTBM)** as part of the [UTBM Robocar Dataset](https://github.com/rwth-asic/utbm_robocar_dataset). 

We thank the UTBM team for their contribution to the open-source robotics and autonomous driving research community.
