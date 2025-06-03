import numpy as np
import pandas as pd

# === CSV読み込み ===
df = pd.read_csv("mag_data.csv")
X = df[["x", "y", "z"]].to_numpy()
N = X.shape[0]

# === Step1: 正規方程式で中心(x0, y0, z0)を推定 ===
A = np.hstack([2 * X, np.ones((N, 1))])  # [2x 2y 2z 1]
b = np.sum(X**2, axis=1).reshape(-1, 1)  # x^2 + y^2 + z^2

params, *_ = np.linalg.lstsq(A, b, rcond=None)
x0, y0, z0, d = params.flatten()

# === Step2: 中心で補正し、各軸のスケール(a,b,c)を推定 ===
Xc = X - np.array([x0, y0, z0])
radii = np.sqrt(np.sum(Xc**2, axis=1))
a = np.std(Xc[:, 0])
b = np.std(Xc[:, 1])
c = np.std(Xc[:, 2])

# === Step3: スケール平均 ===
rwo = (a + b + c) / 3.0

# === 結果出力 ===
print("=== Ellipsoid Fit Parameters ===")
print(f"x0, y0, z0 = {x0:.3f}, {y0:.3f}, {z0:.3f}")
print(f"a, b, c     = {a:.3f}, {b:.3f}, {c:.3f}")
print(f"rwo (mean) = {rwo:.3f}")
