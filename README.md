# DBD_26

드론 비행 로그를 단일 비행, 10회 비행 cycle, RPT 단계의 세 수준으로 분석하는 코드입니다.

## 분석 실행

Python 3.10 이상에서 다음 패키지가 필요합니다.

```bash
python -m pip install -r requirements.txt
```

전체 데이터셋 분석:

```bash
./run_analysis.sh
```

이 명령은 가상환경과 필수 패키지를 확인한 뒤 전체 데이터셋을 한 번에 분석합니다. 팩별 분석
모드는 기본적으로 PB는 `mode == 3`(Vertical), PC는 `mode == 2`(Horizontal)로 자동 적용됩니다.
새 팩의 모드를 지정하거나 기본값을 바꾸려면 `--mode-map PACK=MODE`를 추가합니다.

단일 비행 분석:

```bash
python src/drone_flight_analysis.py \
  --flight-dir "Dataset/PB/방전로그/PB_0/2026-08-18 08-39-50_PB_0_V_1" \
  --result result
```

팩별 기본 분석 구간은 PB `mode == 3`, PC `mode == 2`이며, 진입 직후 3초는 안정화 구간으로
제외합니다. 모든 팩을 하나의 모드로 강제하려면 `--active-mode`, 팩별 모드는 `--mode-map`,
안정화 시간은 `--settle-seconds`로 변경할 수 있습니다. 시료별 추종오차 원시
시계열 CSV가 필요하면 `--save-error-series`를 추가합니다. 이 옵션은 큰 파일을 생성하므로
기본값은 꺼져 있습니다.

## 결과 구조

```text
result/
├── all_flight_summaries.csv       # 분석 가능한 모든 비행의 1행 요약
├── data_quality_report.csv        # 누락/빈 파일/분석 실패 목록
├── analysis_metadata.json         # 실행 조건과 처리 개수
├── flights/PB/RPT_n/V_n/
│   ├── flight_summary.csv
│   ├── performance_overview.png
│   ├── battery_diagnostics.png
│   ├── acceleration_diagnostics.png
│   └── tracking_errors.png
├── cycles/PB/RPT_n/
│   ├── flight_performance.csv     # V_1~V_10 성능 변화
│   ├── axis_control_performance.csv
│   ├── cycle_summary.csv          # 평균, 표준편차, 비행당 기울기
│   ├── cycle_performance.png
│   └── *_rmse_by_axis.png         # 위치/속도/가속도/자세/각속도 축별 변화
└── rpt/
    ├── rpt_summary.csv
    ├── all_packs_rpt_performance.png
    └── PB/
        ├── rpt_performance.csv
        ├── rpt_performance.png
        ├── flight_performance_by_rpt.csv  # V1~V10에서 RPT별 차이
        ├── flight_performance_by_rpt.png  # RPT 선을 겹쳐 비교
        ├── axis_control_performance_by_rpt.csv
        ├── *_rmse_by_axis_and_rpt.png     # V1~V10의 RPT 선 중첩
        └── *_rmse_trend_by_rpt.png        # RPT 평균±표준편차
```

단일 비행 요약에는 위치/자세/속도/각속도 추종 RMSE·MAE·최대오차, 비행시간·거리·속도,
PWM 부하와 모터 편차, 전압·전류·전력·방전에너지·온도·셀 불균형이 포함됩니다. 배터리
값은 `batteryTimestamp` 중복을 제거한 뒤 적분합니다. 향후 지표는
`src/drone_flight_analysis.py`의 `calculate_metrics()`와 `PLOT_METRICS`에 추가할 수 있습니다.

위치, 속도, 자세, 각속도는 기록된 목표값과 측정값의 X/Y/Z 축별 RMSE를 계산합니다.
가속도는 world-frame 속도와 목표 속도를 0.2초 이동평균한 뒤 각각 미분하여 축별 추종오차를
계산합니다. `accl[0:3]`은 중력을 포함할 수 있는 body-frame IMU 값이고 대응 목표값이 없으므로,
추종오차가 아닌 축별 평균·표준편차·RMS·최댓값으로 별도 기록합니다. 위치·속도·가속도
그래프는 X/Y를 Horizontal, Z를 Vertical로 구분하고, 자세·각속도는 Roll/Pitch/Yaw로
표시합니다.
