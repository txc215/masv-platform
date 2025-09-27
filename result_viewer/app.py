from flask import Flask, render_template, jsonify
import os
import json
import pandas

curr_path = os.path.dirname(os.path.abspath(__file__))

__output_data__ = curr_path + "/../data/sample_outputs/dummy_gnss_imu_ekf.csv"

app = Flask(__name__)

def read_data():
    df = pandas.read_csv(__output_data__, delimiter="\t")
    time_t = df['timestamp_sec'].tolist()
    display_data = []

    for col in df.columns:
        if col == "timestamp_sec":
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

@app.route('/')
def index():
    plotDataSet = read_data()
    return render_template('index.html', plot_data=plotDataSet)

@app.route('/api/log')
def load_log():
    log_path = '../data/sample_outputs/custom_logs/sample_log.json'
    with open(log_path) as file_p:
        data_log = json.load(file_p)
    return jsonify(data_log)

if __name__ == '__main__':
    app.run(debug=True)
