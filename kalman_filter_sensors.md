# カルマンフィルタによるセンサー統合分析

## 概要
M5StampFlyプロジェクトでは、カルマンフィルタが主に高度推定（Altitude Estimation）に使用されています。カルマンフィルタは、`alt_kalman.hpp` と `alt_kalman.cpp` で実装されており、その主な目的は、複数のセンサーから得られたデータを統合して、より安定した高度と垂直速度の推定値を生成することです。

## 使用されているカルマンフィルタ

### 高度カルマンフィルタ (Alt_kalman)
- 実装ファイル: `src/alt_kalman.hpp` および `src/alt_kalman.cpp`
- インスタンス: `EstimatedAltitude` (`sensor.cpp` で定義)
- 更新周期: メインループと同じ400Hz（0.0025秒ごと）

## 統合されているセンサー

### 1. ToF (Time of Flight) センサー
- 関連ファイル: `src/tof.cpp`, `src/tof.hpp`
- 用途: 地面との距離（高度）を測定
- 更新周期: 約30Hz（割り込みベース）
  - 計算式: `400Hz / 30Hz + 1 = 14.33` のinterval値を使用
  - ToFセンサーからの割り込み（INT_BOTTOMピン）発生時にデータ取得
- 特徴:
  - VL53LXセンサーを使用
  - 下向きセンサー (ToF_bottom) は高度測定用
  - 前向きセンサー (ToF_front) は障害物検出用
  - 計測タイミング設定: 33000μs (33ms)

### 2. IMU (慣性計測ユニット)
- 関連ファイル: `src/imu.cpp`, `src/imu.hpp`
- 用途: 加速度とジャイロデータの提供
- 更新周期: 400Hz (メインループと同期)
  - データレート設定: BMI270センサーは`BMI2_ACC_ODR_400HZ`および`BMI2_GYR_ODR_400HZ`に設定
- 特徴:
  - BMI270センサーを使用
  - 垂直方向の加速度 (Accel_z) をカルマンフィルタに提供
  - 加速度によって高さの変化率を推定するために使用
  - **重要**: Z軸加速度は機体の姿勢を考慮した線形加速度として処理されています

## システム全体の更新周期

- メインループ: 400Hz (0.0025秒ごと)
  ```cpp
  float Control_period = 0.0025f;  // 400Hz
  ```
- 姿勢推定（Madgwickフィルタ）: 400Hz
  ```cpp
  Drone_ahrs.begin(400.0);
  ```
- カルマンフィルタ更新: 400Hz（メインループと同期）
  ```cpp
  EstimatedAltitude.update(Altitude, Az, Interval_time);
  ```
- ToFセンサー読み取り: 約30Hz（割り込みベース）
  ```cpp
  const uint8_t interval = 400 / 30 + 1;  // 約14.33回のメインループごとに1回
  ```

## カルマンフィルタの数学モデル

### 状態変数
高度カルマンフィルタでは、以下の3つの状態変数を推定しています：
- $v$ (velocity): 垂直方向の速度
- $h$ (altitude): 高度
- $b$ (bias): 加速度センサーのバイアス

### 状態方程式
システムの状態遷移は以下の方程式で表されます：

$$
\begin{bmatrix}
v_{k+1} \\
h_{k+1} \\
b_{k+1}
\end{bmatrix} = 
\begin{bmatrix}
1 & 0 & -\Delta t \\
\Delta t & 1 & 0 \\
0 & 0 & 1 + \beta\Delta t
\end{bmatrix}
\begin{bmatrix}
v_k \\
h_k \\
b_k
\end{bmatrix} + 
\begin{bmatrix}
\Delta t & 0 \\
0 & 0 \\
0 & \Delta t
\end{bmatrix}
\begin{bmatrix}
a_k \\
w_k
\end{bmatrix}
$$

ここで：
- $\Delta t$: サンプリング時間（1/400秒 = 0.0025秒）
- $a_k$: 測定された加速度
- $w_k$: バイアスの変化を表すプロセスノイズ
- $\beta$: バイアスの減衰係数（-0.01）

### 観測方程式
高度の観測は以下の方程式で表されます：

$$
z_k = 
\begin{bmatrix}
0 & 1 & 0
\end{bmatrix}
\begin{bmatrix}
v_k \\
h_k \\
b_k
\end{bmatrix} + v_k
$$

ここで：
- $z_k$: ToFセンサーによる高度測定値
- $v_k$: 観測ノイズ

### 実装パラメータ

コード内では以下のようにパラメータが設定されています：

```cpp
// 状態遷移行列 F
float f11 = 1.0, f12 = 0.0, f13 = -step;
float f21 = step, f22 = 1.0, f23 = 0.0;
float f31 = 0.0, f32 = 0.0, f33 = 1 + beta * step;

// 入力行列 B
float b11 = step, b12 = 0.0;
float b21 = 0.0, b22 = 0.0;
float b31 = 0.0, b32 = step;

// 観測行列 H
float h1 = 0.0, h2 = 1.0, h3 = 0.0;

// プロセスノイズ共分散 Q
float q1 = 0.1 * 0.1;  // 加速度のプロセスノイズ
float q2 = 1.0 * 1.0;  // バイアスのプロセスノイズ

// 観測ノイズ共分散 R
float R = 0.004 * 0.004;  // ToFセンサーの測定ノイズ

// P行列の初期値（状態の不確かさ）
float p11 = 100.0, p12 = 0.0, p13 = 0.0;
float p21 = 0.0, p22 = 100.0, p23 = 0.0;
float p31 = 0.0, p32 = 0.0, p33 = 100.0;
```

### カルマンフィルタの更新サイクル

カルマンフィルタの更新は次の2つのステップで行われます：

#### 1. 予測ステップ

状態の予測：
```cpp
// 速度の予測
velocity_ = velocity + (accel - bias) * step;
// 高度の予測
altitude_ = altitude + velocity * step;
// バイアスの予測
bias_ = bias * (1 + step * beta);
```

共分散行列Pの予測：
```cpp
p11_ = p11 - step * (p31 + p13) + step * step * p33 + step * step * q1;
p12_ = step * (p11 - p32) - step * step * p31 + p12;
p13_ = (1 + beta * step) * (p13 - step * p33);
// その他のP成分の計算...
```

#### 2. 修正ステップ

カルマンゲインの計算：
```cpp
float s = p22_ + R;  // イノベーション共分散
k1 = p12_ / s;       // 速度のカルマンゲイン
k2 = p22_ / s;       // 高度のカルマンゲイン
k3 = p32_ / s;       // バイアスのカルマンゲイン
```

イノベーション（測定と予測の差）：
```cpp
float e = z_sens - altitude_;  // 実測高度と予測高度の差
```

状態の更新：
```cpp
velocity = velocity_ + k1 * e;  // 速度の更新
altitude = altitude_ + k2 * e;  // 高度の更新
bias = bias_ + k3 * e;         // バイアスの更新
```

共分散行列Pの更新：
```cpp
p11 = p11_ - k1 * p21_;
p12 = p12_ - k1 * p22_;
// その他のP成分の更新...
```

## 異なる更新周期のセンサーデータ統合

M5StampFlyの高度推定では、更新周期の異なる2つのセンサーを統合する必要があります：
- IMU（加速度）: 400Hz
- ToFセンサー（高度）: 約30Hz

このような異なる更新周期のセンサーデータを統合するために、以下のアプローチが採用されています：

1. **ToFデータが利用可能な場合**:
   - ToFセンサーからの割り込みが発生すると、最新の高度データを取得
   - カルマンフィルタの完全な更新（予測＋修正）を実行

2. **ToFデータが利用できない場合**:
   - IMUからの加速度データのみを使用して状態を予測
   - 修正ステップをスキップし、予測値をそのまま使用

このアプローチにより、高い更新周期（400Hz）で状態を推定しつつ、より低い周波数で到着する高度測定値を適切に統合しています。ToFデータが到着するまでの間は、IMUの加速度情報を使って物理モデルに基づく予測を行うことで、推定の連続性を確保しています。

## Z軸加速度の処理フロー

Z軸加速度は以下の手順で処理され、姿勢を考慮した線形加速度として扱われます：

1. **IMUからの生データ取得**:
   ```cpp
   imu_update();
   acc_z = imu_get_acc_z();
   ```

2. **座標系の変換**:
   ```cpp
   // BMI270の座標系から航空工学の座標系に変換
   // Z軸：上下（BMI270では上が正） → 下上（航空工学では下が正）
   Accel_z_raw = -acc_z;
   ```

3. **オフセット補正とフィルタリング**:
   ```cpp
   // オフセット補正と低域通過フィルタの適用
   Accel_z = raw_az_filter.update(Accel_z_raw, Interval_time);
   Accel_z_d = raw_az_d_filter.update(Accel_z_raw - Accel_z_offset, Interval_time);
   ```

4. **姿勢推定（Madgwickフィルタ）との統合**:
   ```cpp
   // Madgwickフィルタに加速度とジャイロデータを入力して姿勢を推定
   Drone_ahrs.updateIMU((Pitch_rate) * RAD_TO_DEG, (Roll_rate) * RAD_TO_DEG,
                       -(Yaw_rate) * RAD_TO_DEG, Accel_y, Accel_x, -Accel_z);
   ```

5. **垂直方向の加速度の抽出**:
   ```cpp
   // 垂直方向の加速度を抽出（姿勢を考慮した線形加速度）
   Az = az_filter.update(-Accel_z_d, sens_interval);
   ```

6. **カルマンフィルタへの入力**:
   ```cpp
   // カルマンフィルタに高度データとZ軸加速度を入力
   EstimatedAltitude.update(Altitude, Az, Interval_time);
   ```

この処理により、Z軸加速度は単なる生のセンサー値ではなく、機体の姿勢（ロール、ピッチ）を考慮した真の垂直方向の加速度として扱われています。これによって、機体が傾いている状態でも正確な高度推定が可能となります。

## センサー統合の流れ

1. ToFセンサーから高度測定値 (`z_sens`) を取得（約30Hz）
2. IMUから姿勢を考慮した垂直加速度 (`accel` = `Az`) を取得（400Hz）
3. カルマンフィルタの `update()` メソッドを呼び出し:
   ```cpp
   void Alt_kalman::update(float z_sens, float accel, float h)
   ```
   - `z_sens`: ToFセンサーからの高度測定値
   - `accel`: 姿勢を考慮した垂直加速度
   - `h`: 時間ステップ (通常は 1/400秒 = 0.0025秒)

4. 予測ステップ:
   - 前回の推定値と加速度情報から現在の状態を予測
   - 速度、高度、バイアスの予測更新

5. 修正ステップ:
   - ToFセンサー測定値と予測値の差から最適な修正を計算
   - カルマンゲインを使用して状態推定値を更新

6. 更新された推定値を `Velocity`, `Altitude`, `Bias` 変数に格納

## フライトコントロールでの使用

カルマンフィルタの出力 (`EstimatedAltitude.Altitude`, `EstimatedAltitude.Velocity`) は、ドローンの高度制御システムで使用されます。`flight_control.cpp`で実装されている高度PID制御は、この推定値に基づいて垂直方向の動きを制御します。

## まとめ

M5StampFlyプロジェクトのカルマンフィルタは、ToFセンサー（距離測定、約30Hz）とIMU（姿勢を考慮した加速度、400Hz）のデータを統合して、400Hzでの安定した高度推定を実現しています。異なる更新周期のセンサーデータを効果的に組み合わせることで、より高い周波数（400Hz）での高精度な状態推定が可能になっています。特にZ軸加速度は、Madgwickフィルタによる姿勢推定と連携し、機体の傾きを考慮した真の垂直方向の加速度として処理されています。これにより、ドローンの垂直方向の挙動が安定し、より正確な高度維持や高度変更が可能になります。 