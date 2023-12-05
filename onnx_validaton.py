import onnx
from onnx import onnx_pb

# バリデーション対象のONNXモデルファイルへのパス
model_path = "/home/moriokalab/catkin_ws/src/hello/src/downloaded_model.onnx"

# ONNXモデルを読み込む
onnx_model = onnx.load(model_path)

# ONNXモデルのバリデーションを実行
try:
    onnx.checker.check_model(onnx_model)
    print("ONNXモデルは正しい形式です。")
except onnx.onnx_cpp2py_export.checker.ValidationError as e:
    print("ONNXモデルのバリデーションエラー:")
    print(e)
