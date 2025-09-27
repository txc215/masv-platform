"""
 config.py
"""

# data path
DATA_PATH = "../../../data/rosbag_logs/processed_dataset/utbm_robocar_20190110"
MODEL_PATH = "../models/"
GRU_MODEL_DEF =  "../train"

FILE_NAME = "gru_model.pt"


# file name
X_FILE = "X_train.npy"
Y_FILE = "Y_train.npy"
Y_PRED_FILE = "Y_pred.npy"

# parameters
INPUT_SIZE = 6
BATCH_SIZE = 64
SEQ_LEN = 20
EPOCHS = 50
LEARNING_RATE = 0.001
VAL_RATIO = 0.2
HIDDEN_SIZE = 64
GRU_OUTPUT_SIZE = 3
ONNX＿OPSET = 13
ONNX_SUBSET = 0

