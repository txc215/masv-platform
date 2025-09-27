from core_algorithms.EKF.simple_heading_ekf import SimpleHeadingEKF as EKF
from core_algorithms.ukf import UKF
# from core_algorithms.rnn import RNNModel
from core_algorithms.transformer_model.transformer import TransformerModel

def get_algorithm(name:str, config: dict = None):
    algm = name.lower()

    if alogm == 'ekf':
        return EKF(config)
    elif algm == 'ukf':
        return UKF(config)
    elif algm == 'transformer':
        return TransformerModel(config)
    else:
        raise ValueError(f"Unknown Algorithm Module: {name}")