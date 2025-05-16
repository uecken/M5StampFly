# カルマンフィルタによるセンサー統合分析

## 概要
M5StampFlyプロジェクトでは、カルマンフィルタが主に高度推定（Altitude Estimation）に使用されています。カルマンフィルタは、`alt_kalman.hpp` と `alt_kalman.cpp` で実装されており、その主な目的は、複数のセンサーから得られたデータを統合して、より安定した高度と垂直速度の推定値を生成することです。

## 使用されているカルマンフィルタ

### 高度カルマンフィルタ (Alt_kalman)
- 実装ファイル: `src/alt_kalman.hpp` および `src/alt_kalman.cpp`
- インスタンス: `EstimatedAltitude` (`sensor.cpp` で定義)

## 統合されているセンサー

### 1. ToF (Time of Flight) センサー
- 関連ファイル: `src/tof.cpp`, `src/tof.hpp`
- 用途: 地面との距離（高度）を測定
- 特徴:
  - VL53LXセンサーを使用
  - 下向きセンサー (ToF_bottom) は高度測定用
  - 前向きセンサー (ToF_front) は障害物検出用
  - INT_BOTTOMピンの割り込みを使用してデータ取得のタイミングを制御

### 2. IMU (慣性計測ユニット)
- 関連ファイル: `src/imu.cpp`, `src/imu.hpp`
- 用途: 加速度とジャイロデータの提供
- 特徴:
  - BMI270センサーを使用
  - 垂直方向の加速度 (Accel_z) をカルマンフィルタに提供
  - 加速度によって高さの変化率を推定するために使用
  - **重要**: Z軸加速度は機体の姿勢を考慮した線形加速度として処理されています

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

## カルマンフィルタの状態変数

`Alt_kalman` クラスは以下の状態を推定しています:
- `velocity`: 垂直方向の速度
- `altitude`: 高度
- `bias`: 加速度センサーのバイアス

## センサー統合の流れ

1. ToFセンサーから高度測定値 (`z_sens`) を取得
2. IMUから姿勢を考慮した垂直加速度 (`accel` = `Az`) を取得
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

M5StampFlyプロジェクトのカルマンフィルタは、ToFセンサー（距離測定）とIMU（姿勢を考慮した加速度）のデータを統合して、安定した高度推定を実現しています。特にZ軸加速度は、Madgwickフィルタによる姿勢推定と連携し、機体の傾きを考慮した真の垂直方向の加速度として処理されています。これにより、ドローンの垂直方向の挙動が安定し、より正確な高度維持や高度変更が可能になります。 