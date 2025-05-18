# AP_NavEKF3 および EKFGSF_yaw 詳細調査

このドキュメントは、ArduPilotのAP_NavEKF3とEKFGSF_yaw実装に関する詳細な調査結果をまとめたものです。

## AP_NavEKF3 概要

AP_NavEKF3は、ArduPilotの最新の拡張カルマンフィルタ（EKF）の実装であり、航空機の姿勢、位置、速度を推定するための主要なセンサーフュージョンシステムです。

### 基本設定

- **更新レート**: 83.3Hz (12ms間隔)
  ```cpp
  #define EKF_TARGET_DT_MS 12
  #define EKF_TARGET_DT    0.012f
  ```
- **状態ベクトル次元**: 24次元（四元数、速度、位置、バイアス、磁場、風速）
- **主要処理**: 予測、測定更新、イノベーション一貫性チェック

### 状態変数 (24次元)

| インデックス | 状態変数 | 説明 | 初期共分散設定 |
|------------|---------|------|--------------|
| 0-3 | 四元数 | 姿勢表現 | 回転ベクトル分散による初期化 |
| 4-6 | 速度 | NED座標系での速度 | GPS速度ノイズに基づく |
| 7-9 | 位置 | NED座標系での位置 | GPS位置/気圧高度ノイズに基づく |
| 10-12 | ジャイロバイアス | デルタ角バイアス | `InitialGyroBiasUncertainty()` |
| 13-15 | 加速度バイアス | デルタ速度バイアス | `ACCEL_BIAS_LIM_SCALER * _accBiasLim` |
| 16-18 | 地球磁場 | 地球磁場ベクトル | `_magNoise` |
| 19-21 | 機体磁場 | 機体磁場ベクトル | `_magNoise` |
| 22-23 | 風速 | 風速ベクトル | 0.0初期化 |

### 主要なパラメータ設定

AP_NavEKF3は数多くの設定可能なパラメータを持ち、ユーザー/機体特性に合わせて調整可能です：

```cpp
// GPS測定パラメータ
#define VELNE_M_NSE_DEFAULT     0.5f    // GPS水平速度測定ノイズ (m/s)
#define VELD_M_NSE_DEFAULT      0.7f    // GPS垂直速度測定ノイズ (m/s)
#define POSNE_M_NSE_DEFAULT     0.5f    // GPS水平位置測定ノイズ (m)
#define ALT_M_NSE_DEFAULT       2.0f    // GPS高度測定ノイズ (m)

// IMUプロセスノイズ
#define GYRO_P_NSE_DEFAULT      1.5E-02f // ジャイロプロセスノイズ (rad/s)
#define ACC_P_NSE_DEFAULT       3.5E-01f // 加速度計プロセスノイズ (m/s^2)
#define GBIAS_P_NSE_DEFAULT     1.0E-03f // ジャイロバイアス状態プロセスノイズ (rad/s)
#define ABIAS_P_NSE_DEFAULT     2.0E-02f // 加速度バイアス状態プロセスノイズ (m/s^2)

// 磁力計関連
#define MAG_M_NSE_DEFAULT       0.05f   // 磁力計測定ノイズ (ガウス)
#define MAGB_P_NSE_DEFAULT      1.0E-04f // 機体磁場プロセスノイズ (ガウス/秒)
#define MAGE_P_NSE_DEFAULT      1.0E-03f // 地球磁場プロセスノイズ (ガウス/秒)

// イノベーションゲート閾値（一貫性チェック）
#define VEL_I_GATE_DEFAULT      500     // 速度イノベーション一貫性ゲート (%)
#define POS_I_GATE_DEFAULT      500     // 位置イノベーション一貫性ゲート (%)
#define HGT_I_GATE_DEFAULT      500     // 高度イノベーション一貫性ゲート (%)
#define MAG_I_GATE_DEFAULT      300     // 磁力計イノベーション一貫性ゲート (%)
```

### フィルタ実行フロー

AP_NavEKF3の主要な処理フローは以下の通りです：

1. **初期化（InitialiseFilterBootstrap）**:
   - 加速度計と磁力計のデータを使用して初期姿勢を推定
   - 共分散行列の初期化（CovarianceInit）

2. **フィルタ更新（UpdateFilter）**:
   - IMUデータを使用して状態予測（UpdateStrapdownEquationsNED）
   - 共分散予測（CovariancePrediction）
   - EKFGSF方位角推定器の予測ステップ実行（runYawEstimatorPrediction）
   - 各種センサーフュージョン:
     - 磁力計または外部方位角センサー（SelectMagFusion）
     - GPS/高度計データ（SelectVelPosFusion）
     - EKFGSF方位角推定器の補正ステップ（runYawEstimatorCorrection）
     - レンジビーコン（SelectRngBcnFusion）
     - 光学フロー（SelectFlowFusion）
     - ボディフレームオドメトリ（SelectBodyOdomFusion）
     - 対気速度（SelectTasFusion）

3. **測定更新（各種Fusion関数）**:
   - 測定値と予測値の差（イノベーション）の計算
   - カルマンゲイン計算
   - 状態と共分散の更新
   - イノベーション一貫性チェック

### 数値安定性と制約

- 共分散行列の対称性を維持
- 状態変数の制約（ConstrainVariances）:
  ```cpp
  // 例: 姿勢誤差共分散の制約
  for (uint8_t i=0; i<=3; i++) P[i][i] = constrain_ftype(P[i][i],0.0,1.0);
  
  // 速度状態共分散の制約
  for (uint8_t i=4; i<=5; i++) P[i][i] = constrain_ftype(P[i][i], VEL_STATE_MIN_VARIANCE, 1.0e3);
  ```

- 状態変数の制約表（2025年3月時点）:

```
+----------------------------------------------------------------------------------------------+
| State Index  |      State Name                 |  State Units  | Variance Constraint Range   |
+----------------------------------------------------------------------------------------------+
|  0 .. 3      | Attitude Quaternion             | unitless      | [0.0, 1.0]                  |
|  4 .. 5      | Velocity (North, East)          | m/s           | [1e-4, 1e3]                 |
|  6           | Velocity (Down)                 | m/s           | dynamic                     |
|  7 .. 9      | Position (North, East, Down)    | m             | [1e-4, 1e6]                 |
| 10 .. 12     | Gyro Bias (X, Y, Z)             | rad           | [0.0, (0.175 * dtEkfAvg)^2] |
| 13 .. 15     | Accel Bias (X, Y, Z)            | m/s^2         | dynamic                     |
| 16 .. 18     | Earth Magnetic Field (X, Y, Z)  | Gauss         | [0.0, 0.01]                 |
| 19 .. 21     | Body Magnetic Field (X, Y, Z)   | Gauss         | [0.0, 0.01]                 |
| 22 .. 23     | Wind Velocity (North, East)     | m/s           | [0.0, 400]                  |
+----------------------------------------------------------------------------------------------+
```

## EKFGSF_yaw 実装詳細

EKFGSF_yaw（Extended Kalman Filter Gaussian Sum Filter for Yaw estimation）は、特に磁力計が利用できない、または信頼性が低い状況で、方位角（ヨー）を推定するための特殊なアルゴリズムです。

### 基本構造

EKFGSF_yawは、次の主要コンポーネントで構成されています：

1. **複数のAHRSフィルタ**: 異なる初期ヨー角を持つ複数のAHRS（姿勢方位基準システム）の実装
2. **複数のEKFモデル**: 各AHRSフィルタの出力に基づく簡略化された3状態EKF
3. **ガウス和フィルタ（GSF）**: 各EKFモデルの出力を確率重み付けして結合

### AHRS構造体

各AHRSフィルタは次の状態を保持します：

```cpp
struct ahrs_struct {
    Matrix3F R;             // 機体から地球座標系への回転行列
    Vector3F gyro_bias;     // 四元数計算で学習・使用されるジャイロバイアス
    bool aligned;           // AHRSが整列されたときtrue
    ftype accel_FR[2];      // 水平面内の前方-右方加速度ベクトル (m/s/s)
    ftype vel_NE[2];        // 前回のGPS測定からの北東速度ベクトル (m/s)
    bool fuse_gps;          // そのフレームでGPSをフュージョンすべき場合true
    ftype accel_dt;         // _simple_accel_FRデータ生成時の時間ステップ (秒)
};
```

### EKF構造体

各EKFモデルは、速度と方位角を推定するシンプルな3状態フィルタです：

```cpp
struct EKF_struct {
    ftype X[3];     // 北方向速度(m/s), 東方向速度(m/s), ヨー角(rad)
    ftype P[3][3];  // 共分散行列
    ftype S[2][2];  // 北東速度イノベーション分散 (m/s)^2
    ftype innov[2]; // 速度北東イノベーション (m/s)
};
```

### GSF構造体

ガウス和フィルタは、各EKFモデルの推定結果を統合します：

```cpp
struct GSF_struct {
    ftype yaw;                      // ヨー角 (rad)
    ftype yaw_variance;             // ヨー状態の分散 (rad^2)
    ftype weights[N_MODELS_EKFGSF]; // 各EKFモデルに適用される重み（合計1）
};
```

### EKFGSF_yawの処理フロー

1. **初期化**:
   - 複数のAHRSを異なる初期ヨー角で初期化
   - 各EKFモデルに均等な初期重みを設定

2. **予測ステップ**:
   - IMUデータを使用して各AHRSフィルタを更新（predictAHRS）
   - 各EKFの状態と共分散を予測（predict）

3. **補正ステップ**:
   - GPS速度測定値を利用して各EKFを更新（correct）
   - イノベーション統計に基づいて各モデルの確率重みを再計算

4. **出力計算**:
   - 重み付けされたヨー角ベクトルの計算:
   ```cpp
   Vector2F yaw_vector;
   for (uint8_t mdl_idx = 0; mdl_idx < N_MODELS_EKFGSF; mdl_idx++) {
       yaw_vector[0] += GSF.weights[mdl_idx] * cosF(EKF[mdl_idx].X[2]);
       yaw_vector[1] += GSF.weights[mdl_idx] * sinF(EKF[mdl_idx].X[2]);
   }
   GSF.yaw = atan2F(yaw_vector[1],yaw_vector[0]);
   ```

### 主要パラメータ

```cpp
// パラメータ
const ftype EKFGSF_gyroNoise{1.0e-1};   // 共分散予測に使用されるヨーレートノイズ (rad/sec)
const ftype EKFGSF_accelNoise{2.0};     // 共分散予測に使用される水平加速度ノイズ (m/sec^2)
const ftype EKFGSF_tiltGain{0.2};       // 補完フィルタのチルトエラーからジャイロ補正へのゲイン (1/sec)
const ftype EKFGSF_gyroBiasGain{0.04};  // 補完フィルタのジャイロ補正積分に適用されるゲイン (1/sec)
const ftype EKFGSF_accelFiltRatio{10.0}; // AHRSのチルト補正の時定数と、AHRSで使用される加速度データに適用される一次LPFの時定数の比率
```

### AP_NavEKF3との統合

メインEKFシステム（AP_NavEKF3）とEKFGSF_yawは以下のように統合されています：

1. **予測ステップの実行**: AP_NavEKF3の`UpdateFilter`内で`runYawEstimatorPrediction`を呼び出し

2. **補正ステップの実行**: GPS速度の処理後に`runYawEstimatorCorrection`を呼び出し

3. **メインEKFへのヨー角適用**: EKFGSF_yawからのヨー推定値が十分な精度を持つ場合、メインEKFのヨー角をリセット:
   ```cpp
   if (yawResetRequest && EKFGSF_yaw_reset_request_ms != 0 && (imuSampleTime_ms - EKFGSF_yaw_reset_request_ms < YAW_RESET_TO_GSF_TIMEOUT_MS)) {
       // ...
       ResetQuaternion(Tbn_rotation);
       // ...
   }
   ```

## 結論

AP_NavEKF3とEKFGSF_yawは、ArduPilotのセンサーフュージョンシステムの中核をなす高度な実装です。AP_NavEKF3は、IMU、GPS、気圧計、磁力計などの複数のセンサーからのデータを融合し、ロバストな姿勢・位置・速度推定を提供します。EKFGSF_yawは、磁力計の信頼性が低い状況でも、GPS速度データを利用して正確な方位角推定を行うための革新的なアプローチです。

この両方のシステムは、ArduPilotのナビゲーション性能の向上に大きく貢献しており、特に磁気干渉の多い環境での飛行においてもロバストな方位角推定を実現しています。 