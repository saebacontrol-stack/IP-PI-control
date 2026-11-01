# Interactive I-P/PI control Simulator

PythonによるI-P/PI制御モード切替時挙動変化の動的可視化ツールです。

![Python](https://img.shields.io/badge/Python-3.x-blue.svg)
![License](https://img.shields.io/badge/License-MIT-green.svg)

## 概要 (Overview)
このシミュレータではI-P制御モード(α=0)で初回シミュレーションを実行します。
パラメータαを1に近づけるほどPI制御に近づき、指令追従が高速化する代わりにオーバーシュートが出てくる様子をインタラクティブに観察できます。

- スライダーまたは数値の直接入力により、パラメータαをリアルタイムに変更可能
- 速度指令は常に1[rad/s](ステップ指令)

---

## 実行方法 (Usage)

### 必要なライブラリ
```bash
pip install numpy matplotlib
```
### 実行手順
```bash
python sample_discretization.py
```
