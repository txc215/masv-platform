from flask import Flask, render_template, jsonify
import os
import json
import glob
import pandas as pd


CURR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_ANALYSIS = os.path.normpath(os.path.join(CURR, "..", "data", "analysis"))
# DATA_DIR = os.environ.get("ANALYSIS_DIR", DEFAULT_ANALYSIS)
DATA_DIR = "/root/data/sample_outputs/" # show output data

app = Flask(__name__)

def latest_data_file(pattern=("*.csv", "*.tsv")):
    files = []
    for pat in (pattern if isinstance(pattern, (list, tuple)) else [pattern]):
        files.extend(glob.glob(os.path.join(DATA_DIR, pat)))
    if not files:
        return None
    # find the lateset data
    return max(files, key=os.path.getmtime)

def read_data(path):

    df = pd.read_csv(path, sep=None, engine="python")
    # find time column
    time_col_candidates = ["timestamp_sec", "t", "time", "stamp"]
    time_col = next((c for c in time_col_candidates if c in df.columns), None)
    if time_col is None:
        df.insert(0, "idx", range(len(df)))
        time_col = "idx"

    time_t = df[time_col].tolist()
    display_data = []

    for col in df.columns:
        if col == time_col:
            continue
        trace = {
            'x': time_t,
            'y': df[col].tolist(),
            'name': col,
            'mode': 'lines',
            'type': 'scatter'
        }
        display_data.append(trace)
    return display_data


@app.route("/api/list")
def list_files():
    files = sorted([os.path.basename(p) for p in glob.glob(os.path.join(DATA_DIR, "*")) if os.path.isfile(p)])
    return jsonify({"dir": DATA_DIR, "files": files})

@app.route("/api/csv/<name>")
def get_csv_as_json(name):
    path = os.path.join(DATA_DIR, name)
    if not os.path.isfile(path):
        abort(404, f"File not found: {name}")
    df = pd.read_csv(path, sep=None, engine="python")
    return df.to_json(orient="records")

@app.route("/")
def index():
    path = latest_data_file()
    if not path:
        # if not data, show error msg
        return render_template("index.html", plot_data=[], message=f"No files in {DATA_DIR}")
    return render_template("index.html", plot_data=read_data(path), message=os.path.basename(path))

@app.route("/api/log")
def load_log():

    log_path = os.path.normpath(os.path.join(CURR, "..", "data", "custom_logs", "sample_log.json"))
    if not os.path.isfile(log_path):
        return jsonify({"error": f"Not found: {log_path}"}), 404
    with open(log_path) as f:
        return jsonify(json.load(f))

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, debug=True)



