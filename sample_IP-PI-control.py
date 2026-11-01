import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, TextBox, Button

# パラメータ
Ts = 1e-4       # サンプリング周期[s]
Jm = 1e-4       # モータイナーシャ[kgm^2]
Alpha = 0       # 制御モード切替パラメータ(0≦Alpha≦1)
wv = 1000       # 速度制御応答周波数[rad/s]

t_start = 0.000     # シミュレーション開始時間[s]
t_final = 0.035     # シミュレーション最大時間[s]
dt = 1e-5           # ソルバー時間刻み幅[s](=0.05[ms])

step = int(round(t_final/dt))       # ステップ数
Ns = int(round(Ts/dt))              # 離散時間系の1サンプルあたりのステップ数

# データの生成・初期化
t = np.arange(t_start, t_final, dt)     # 時間データの生成
v = np.zeros((step, 1))                 # 速度データ[ステップ数 x 1]


def sys_calc(x, u):
    """状態方程式 dx/dt = A@x + B@u 計算"""
    A = np.array([0])
    B = np.array([1/Jm])
    return A @ x + B @ u

def solve_rk4(sys_calc, x, u, dt):
    """4次ルンゲ・クッタ法による1ステップ数値積分
    引数:
    -----------
    sys_calc : 状態方程式 dx/dt = A@x + B@u 計算用関数
    x : 現在の状態変数
    u : 現在の入力
    dt : 積分時間刻み幅[s]

    戻り値:
    --------
    x_next : dt 秒後の状態変数
    """
    k1 = sys_calc(x, u)
    k2 = sys_calc(x + 0.5 * dt * k1, u)
    k3 = sys_calc(x + 0.5 * dt * k2, u)
    k4 = sys_calc(x + dt * k3, u)

    x_next = x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
    return x_next

def simulation():
  """シミュレーション実行関数"""
  x = np.array([0])     # 連続時間状態変数の初期状態
  u = np.array([0])     # 制御入力の初期状態
  vref = np.array([1])  # 速度指令(ステップ入力なので常に1)

  cnt = Ns      # カウント変数
  Intg = 0      # 制御器内積分用変数(初期値=0)

  Kp = Jm*wv    # 比例ゲイン
  Ki = 0.25*wv  # 積分ゲイン

  for k in range(step):
    # 連続時間系シミュレーション(制御対象部分)
    v[k] = x
 
    if cnt==Ns:
        # 離散時間系シミュレーション(制御器部分)
        Intg += Ts * (vref - v[k])                      # 速度誤差の積分計算
        u = Kp * (Alpha * vref - v[k] + Ki * Intg)      # 速度制御器のメイン計算
        cnt = 0                                         # カウントリセット(Ns回に1回のみ離散時間系シミュレーションを実行するため)
    
    x = solve_rk4(sys_calc, x, u, dt)   # 1ステップ更新
    cnt += 1                            # カウントアップ


# グラフ描画準備
fig, ax = plt.subplots(figsize=(10, 7))                                                                         # グラフのサイズ指定(横：10インチ、縦：7インチ)
plt.subplots_adjust(bottom=0.30)                                                                                # グラフ下部にGUIスペースを確保(最上部～下30%までが描画スペース)
fig.text(0.06, 0.20, 'Sampling Time:0.1[ms]    (Alpha = 0 : I-P Mode  ,  Alpha = 1 : PI Mode)', fontsize=11)    # 画面左端6%,下端15%の点からテキスト表示

# 初期グラフの描画
simulation()
line_result, = ax.plot(t * 1000, v)

# グラフ詳細設定
ax.set_xlim(t_start*1000, t_final*1000)                 # X軸表示範囲指定(単位をmsにするため1000倍)
ax.set_ylim(-0.2, 1.2)                                  # Y軸表示範囲指定
ax.set_title('Interactive I-P/PI Control Simulator')    # タイトル指定
ax.set_xlabel('Time[ms]')                               # X軸ラベル指定
ax.set_ylabel('Velocity[rad/s]')                        # Y軸ラベル指定
ax.grid(True)                                           # グリッド表示指定
ax.legend(loc='lower right')                            # 凡例表示指定

# スライダー
Alpha_min = 0
Alpha_max = 1
ax_slider_Alpha  = plt.axes([0.20, 0.12, 0.35, 0.04])                                                       # 画面左端20%,下端12%の点をスライダー描画開始点に指定(横幅：画面40%,縦幅：画面4%)
slider_Alpha  = Slider(ax_slider_Alpha, 'Alpha[-]', Alpha_min, Alpha_max, valinit=Alpha, valfmt='%.2f')     # パラメータα設定用スライダー

# テキストボックス（直接入力用）
ax_text_Alpha  = plt.axes([0.245, 0.06, 0.10, 0.04])                                 # 画面左端24.5%,下端6%の点をテキストボックス描画開始点に指定(横幅：画面10%,縦幅：画面4%)
text_Alpha  = TextBox(ax_text_Alpha, 'Alpha[-](Direct) ', initial=f"{Alpha:.2f}")    # パラメータα設定用テキストボックス

# リセットボタン
ax_button = plt.axes([0.75, 0.06, 0.10, 0.04])                                      # 画面左端75%,下端6%の点をスライダー描画開始点に指定(横幅：画面10%,縦幅：画面4%)
button_reset = Button(ax_button, 'Reset', color='lightgray', hovercolor='0.9')      # リセットボタン

def on_slider_change(val):
    """スライダー操作時イベント"""
    global Alpha
    Alpha = slider_Alpha.val                # スライダーでのAlpha設定値の読み込み
    text_Alpha.set_val(f"{Alpha:.2f}")      # Alpha設定用テキストボックスの表示更新(スライダーに連動して更新)
    simulation()                            # simulationの実行
    line_result.set_ydata(v)                # 再計算したデータをセット
    fig.canvas.draw_idle()

def on_text_submit(text):
    """テキストボックス入力時イベント(Enterキー押下時)"""
    try:
        Alpha = float(text_Alpha.text)

        if Alpha<Alpha_min:
            Alpha = Alpha_min   # Alpha最小値チェック
        elif Alpha>Alpha_max:
            Alpha = Alpha_max   # Alpha最大値チェック

        slider_Alpha.set_val(Alpha)   # Alpha設定用スライダーの表示更新(テキストボックスに連動して更新)
    except ValueError:
        pass # 文字データなどが入力された場合は無視

def reset(event):
    """リセットボタン押下時イベント"""
    # テキストボックス表示はスライダーリセットに連動する形で更新
    slider_Alpha.reset()

# イベントの紐付け
slider_Alpha.on_changed(on_slider_change)
text_Alpha.on_submit(on_text_submit)
button_reset.on_clicked(reset)

plt.show()
