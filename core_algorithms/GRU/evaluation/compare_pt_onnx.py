# compare_pt_onnx.py
import os, time, argparse, numpy as np, torch, onnxruntime as ort
import sys
sys.path.insert(0, '../')
import config
sys.path.insert(0, config.GRU_MODEL_DEF)
from train_gru import GRUNet


def project_path(p):
    # support $PROJECT or other important path
    base = ""
    p = os.path.expandvars(p)
    if p.startswith("$") or p.startswith("~"):
        p = os.path.expanduser(os.path.expandvars(p))
    if not os.path.isabs(p):
        p = os.path.normpath(os.path.join(base, p))
    return p

def default_paths():
    data_dir  = project_path(config.DATA_PATH)
    model_dir = project_path(config.MODEL_PATH)
    return {
        "x": os.path.join(data_dir, config.X_FILE),
        "pt": os.path.join(model_dir, config.FILE_NAME),
        "onnx": os.path.join(model_dir, os.path.splitext(config.FILE_NAME)[0] + ".onnx"),
    }

ap = argparse.ArgumentParser("Compare PyTorch .pt vs ONNX outputs")
paths = default_paths()
ap.add_argument("--x",     default=paths["x"])
ap.add_argument("--pt",    default=paths["pt"])
ap.add_argument("--onnx",  default=paths["onnx"])
ap.add_argument("--input_size",  type=int, default=config.INPUT_SIZE)
ap.add_argument("--hidden_size", type=int, default=config.HIDDEN_SIZE)
ap.add_argument("--output_size", type=int, default=config.GRU_OUTPUT_SIZE)
ap.add_argument("--seq_len",     type=int, default=config.SEQ_LEN)
ap.add_argument("--opset",       type=int, default=config.ONNX_OPSET)
ap.add_argument("--subset",      type=int, default=config.ONNX_SUBSET)
ap.add_argument("--save_preds",  action="store_true")
ap.add_argument("--force_export",action="store_true", help="if exist .onnx still export")
ap.add_argument("--skip_pt",     action="store_true", help="skip PyTorch inference")
ap.add_argument("--skip_onnx",   action="store_true", help="skip ONNX inference")
args = ap.parse_args()

DEVICE = "cuda" if torch.cuda.is_available() else "cpu"

# read data
X = np.load(args.x).astype(np.float32)
if args.subset > 0: X = X[:args.subset]
N, T, D = X.shape
assert D == args.input_size, f"X feature dim {D} != input_size {args.input_size}"

# PyTorch inference
if not args.skip_pt:
    model = GRUNet(input_size=args.input_size, hidden_size=args.hidden_size, output_size=args.output_size).to(DEVICE)
    state = torch.load(args.pt, map_location=DEVICE, weights_only=True)
    model.load_state_dict(state); model.eval()
    with torch.no_grad():
        t0 = time.perf_counter()
        y_pt = model(torch.from_numpy(X).to(DEVICE)).cpu().numpy()
        t1 = time.perf_counter()
    pt_ms = (t1 - t0) * 1000
    print(f"[PT]  out: {y_pt.shape}, latency: {pt_ms:.2f} ms")

# export ONNX（if not exist or --force_export）
need_export = (not os.path.isfile(args.onnx)) or args.force_export
if need_export:
    dummy = torch.randn(1, T, args.input_size, device=DEVICE)
    model_exp = GRUNet(args.input_size, args.hidden_size, args.output_size).to(DEVICE)
    state = torch.load(args.pt, map_location=DEVICE, weights_only=True)
    model_exp.load_state_dict(state); model_exp.eval()
    torch.onnx.export(
        model_exp, dummy, args.onnx,
        input_names=["input"], output_names=["output"],
        dynamic_axes={"input": {0:"batch",1:"seq"}, "output": {0:"batch"}},
        opset_version=args.opset, do_constant_folding=True
    )
    print(f"[ONNX] exported to {args.onnx}")
else:
    print(f"[ONNX] exists, skip export: {args.onnx}")

# ONNX inference
if not args.skip_onnx:
    sess = ort.InferenceSession(args.onnx, providers=["CPUExecutionProvider"])
    t0 = time.perf_counter()
    y_onnx = sess.run(None, {"input": X})[0]
    t1 = time.perf_counter()
    onnx_ms = (t1 - t0) * 1000
    print(f"[ONNX] out: {y_onnx.shape}, latency: {onnx_ms:.2f} ms")

# compare data
if not args.skip_pt and not args.skip_onnx:
    assert y_pt.shape == y_onnx.shape
    diff = np.abs(y_pt - y_onnx)
    print(f"Compare -> max diff: {diff.max():.3e}, mean diff: {diff.mean():.3e}")

# save data
if args.save_preds:
    base = os.path.dirname(args.onnx) or "."
    if not args.skip_pt:   np.save(os.path.join(base, "Y_pred_pt.npy"),   y_pt)
    if not args.skip_onnx: np.save(os.path.join(base, "Y_pred_onnx.npy"), y_onnx)
