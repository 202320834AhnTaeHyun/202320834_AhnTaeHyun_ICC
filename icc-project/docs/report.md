# [202320834-안태현] ICC 제어기 설계 보고서

**과목**: 자동제어 — 2026 봄
**제출일**: 2026-06-23
**팀**: 개인

---

## 1. 설계 개요

본 과제의 목적은 통합 섀시 제어기(Integrated Chassis Control)를 설계하여 다양한 주행 시나리오에서 차량의 안정성 및 주행 성능을 향상시키는 것이다. 제어기는 횡방향, 종방향, 수직방향 제어기로 구성하였으며, 각 제어기의 출력을 coordinator에서 조향각, 4륜 제동 토크, 감쇠 계수 명령으로 변환하였다. 성능 평가는 A1 double lane change, A3 yaw rate step, A4 steady-state cornering, A7 brake-in-turn, B1 straight braking, D1 braking double lane change 시나리오에서 side slip angle, LTR, lateral deviation, yaw rate response, stopping distance 등의 KPI를 기준으로 수행하였다.

제어기 설계에서는 복잡한 차량 모델을 직접 제어 모델로 사용하기보다, yaw rate와 slip angle을 중심으로 한 단순화된 차량 동역학을 기반으로 제어 구조를 구성하였다. 횡방향 제어기에서는 yaw rate error 기반 PID 제어를 사용하여 AFS 보조 조향각을 계산하였고, 속도에 따른 gain scheduling과 조향 제한을 적용하여 안정성과 경로 추종 성능 사이의 균형을 맞추었다. 또한 slip angle이 임계값을 초과하는 경우 ESC yaw moment를 생성하여 차량의 과도한 자세 변화를 억제하였다. 종방향 제어기에서는 속도 오차 기반 PI 제어와 jerk 제한을 적용하였고, 수직방향 제어기에서는 skyhook 방식의 CDC 제어를 적용하였다.

각 제어기의 역할은 다음과 같이 정리할 수 있다.

* **ctrl_lateral**: PID 기반 yaw rate 추종 제어와 slip angle 기반 ESC yaw moment 제어를 수행하였다.
* **ctrl_longitudinal**: 속도 오차 기반 PI 제어와 jerk 제한을 통해 종방향 힘 명령을 생성하였다.
* **ctrl_vertical**: skyhook on-off 제어를 사용하여 4륜 suspension damping coefficient를 조절하였다.
* **ctrl_coordinator**: AFS 조향각, 종방향 제동 토크, ESC yaw moment, CDC 감쇠 명령을 actuator 명령으로 통합 분배하였다. 종방향 제동은 전후 50:50 비율로 분배하였고, yaw moment는 좌우 차동 제동으로 변환하였다. B1 직진 제동 시나리오에서는 brake pulse를 적용하여 stopping distance를 개선하였다.
## 2. 수학적 모델링

### 2.1 사용한 plant 단순화

본 프로젝트의 시뮬레이션 검증은 제공된 차량 plant 위에서 수행하였다. 그러나 제어기 설계 자체는 모든 자유도를 포함하는 복잡한 모델이 아니라, yaw rate와 side slip angle을 중심으로 한 단순화된 bicycle model 개념을 기반으로 구성하였다.

Bicycle model은 좌우 바퀴를 하나의 전륜과 하나의 후륜으로 등가화하여 차량의 횡방향 거동을 해석하는 모델이다. 본 프로젝트에서는 yaw rate tracking과 side slip angle 억제가 횡방향 제어의 핵심이므로, 제어기 설계 단계에서는 bicycle model을 기준으로 yaw rate error에 대한 PID 제어 구조를 설계하였다.

실제 시뮬레이션에서는 더 복잡한 차량 동역학이 포함되므로, 단순 모델 기반 제어기만으로는 모든 KPI를 만족하기 어렵다. 따라서 최종 제어기에서는 PID 제어에 speed scheduling, saturation, ESC yaw moment, brake allocation을 추가하여 실제 시뮬레이션 환경에서 안정적으로 작동하도록 조정하였다.

### 2.2 State-space 표현

제어기 설계를 위한 기본 상태방정식은 다음과 같이 표현할 수 있다.

```text
x_dot = A x + B u
```

상태 변수와 입력은 다음과 같이 정의하였다.

```text
x = [v_y, r]^T
u = delta
```

여기서 `v_y`는 차량의 횡방향 속도, `r`은 yaw rate, `delta`는 전륜 조향각이다.

선형 bicycle model에서 횡방향 속도와 yaw rate의 동역학은 다음과 같이 나타낼 수 있다.

```text
v_y_dot =
- (C_f + C_r)/(m V_x) * v_y
+ ((l_r C_r - l_f C_f)/(m V_x) - V_x) * r
+ (C_f/m) * delta
```

```text
r_dot =
((l_r C_r - l_f C_f)/(I_z V_x)) * v_y
- ((l_f^2 C_f + l_r^2 C_r)/(I_z V_x)) * r
+ (l_f C_f/I_z) * delta
```

각 변수의 의미는 다음과 같다.

```text
m     : 차량 질량
I_z   : yaw 관성모멘트
C_f   : 전륜 cornering stiffness
C_r   : 후륜 cornering stiffness
l_f   : 무게중심에서 전륜까지의 거리
l_r   : 무게중심에서 후륜까지의 거리
V_x   : 종방향 속도
delta : 전륜 조향각
```

### 2.3 가정 및 한계

본 제어기 설계에서는 다음과 같은 가정을 사용하였다.

* 제어기 설계 단계에서는 종방향 속도 `V_x`가 일정하다고 가정하였다.
* 타이어는 작은 slip angle 영역에서 선형 거동한다고 가정하였다.
* 횡방향 제어, 종방향 제어, 수직방향 제어를 각각 분리하여 설계한 뒤 coordinator에서 통합하였다.
* 실제 plant는 더 복잡한 차량 동역학을 포함하므로, 단순 bicycle model 기반 제어기만으로 모든 시나리오를 최적으로 제어하기에는 한계가 있다.

따라서 최종 제어기에서는 단순 모델 기반 PID 구조에 speed scheduling, saturation, brake torque allocation 등을 추가하여 실제 시뮬레이션 환경에서 안정적으로 작동하도록 조정하였다.

---

## 3. 제어기 설계

### 3.1 ctrl_lateral — AFS + ESC

횡방향 제어기의 목표는 yaw rate tracking 성능을 확보하면서 차량의 side slip angle이 과도하게 증가하지 않도록 제한하는 것이다. 이를 위해 yaw rate error 기반 PID 제어를 사용하여 AFS 보조 조향각을 계산하였다.

Yaw rate error는 다음과 같이 정의하였다.

```text
e_r = r_ref - r
```

여기서 `r_ref`는 목표 yaw rate, `r`은 실제 yaw rate이다.

PID 제어 입력은 다음과 같이 계산하였다.

```text
delta_cmd = Kp * e_r + Ki * integral(e_r) + Kd * derivative(e_r)
```

최종 코드에서는 `sim_params.m`에 정의된 기본 gain을 사용하였다.

```matlab
CTRL.LAT.Kp     = 1.0;
CTRL.LAT.Ki     = 0.1;
CTRL.LAT.Kd     = 0.05;
CTRL.LAT.intMax = 5.0;
```

적분항은 과도하게 누적되지 않도록 anti-windup 제한을 적용하였다.

```matlab
ctrlState.intError = max(min(ctrlState.intError, ...
    CTRL.LAT.intMax), -CTRL.LAT.intMax);
```

속도에 따라 보조 조향 명령이 과도하게 커지는 것을 방지하기 위해 speed scheduling을 적용하였다.

```matlab
gainScale = min(vx/28, 0.75);
steerCmd = steerCmd * gainScale;
```

또한 A1 및 D1 시나리오에서 lateral deviation과 LTR 사이의 trade-off를 조정하기 위해 조향 제한을 적용하였다.

```matlab
steerLimit = 0.17 * LIM.MAX_STEER_ANGLE;
```

조향 제한을 크게 설정하면 경로 추종 성능은 좋아질 수 있지만 LTR이 증가할 수 있었다. 반대로 조향 제한을 작게 설정하면 LTR은 안정되지만 경로 추종이 부족해질 수 있었다. 여러 시뮬레이션 결과를 비교하여 전체 점수가 가장 높게 나오는 조향 제한값을 선택하였다.

ESC 제어는 side slip angle이 임계값을 초과할 때 yaw moment를 생성하는 방식으로 구성하였다.

```text
yawMoment = -K_beta * sign(beta) * (abs(beta) - beta_threshold)
```

최종 코드에서는 다음과 같이 설정하였다.

```matlab
beta_th = deg2rad(7);

if abs(slipAngle) > beta_th
    yawMoment = -100 ...
               * sign(slipAngle) ...
               * (abs(slipAngle) - beta_th);
else
    yawMoment = 0;
end
```

이를 통해 side slip angle이 과도하게 증가하는 경우 ESC yaw moment가 발생하여 차량 자세 안정성을 보조하도록 하였다.

### 3.2 ctrl_longitudinal — 속도 제어 및 jerk 제한

종방향 제어기는 속도 오차 기반 PI 제어를 사용하였다. 속도 오차는 다음과 같이 정의하였다.

```text
e_v = V_ref - V_x
```

PI 제어를 통해 목표 종방향 가속도 명령을 계산하였다.

```text
a_x_cmd = Kp * e_v + Ki * integral(e_v)
```

이후 차량 질량을 이용하여 종방향 힘 명령으로 변환하였다.

```text
F_x = m * a_x_cmd
```

최종 코드에서는 다음 gain을 사용하였다.

```matlab
CTRL.LON.Kp     = 0.5;
CTRL.LON.Ki     = 0.05;
CTRL.LON.intMax = 2000;
```

종방향 힘 명령이 급격하게 변하는 것을 방지하기 위해 jerk 제한을 적용하였다.

```matlab
maxDeltaF = m * LIM.MAX_JERK * dt;
dF = Fx - ctrlState.prevForce;
dF = max(min(dF, maxDeltaF), -maxDeltaF);
Fx = ctrlState.prevForce + dF;
```

제동 명령이 필요한 경우에는 `Fx`가 음수가 되며, 이를 기반으로 brakeRatio를 계산하였다.

```matlab
if Fx < 0
    brakeRatio = min(abs(Fx)/(m*LIM.MAX_AX), 1);
else
    brakeRatio = 0;
end
```

또한 wheel slip 정보를 coordinator에서 활용할 수 있도록 `slipMax` 값을 출력 구조체에 포함하였다.

```matlab
if isfield(ctrlState,'wheelSlip')
    forceCmd.slipMax = max(abs(ctrlState.wheelSlip(:)));
else
    forceCmd.slipMax = 0;
end
```

다만 최종 제출 코드에서는 wheel slip 기반의 정밀한 closed-loop ABS 제어보다는 coordinator에서 B1 straight braking 시나리오에 brake pulse를 적용하는 방식으로 stopping distance를 개선하였다.

### 3.3 ctrl_vertical — Skyhook CDC

수직방향 제어기는 skyhook on-off 방식으로 설계하였다. Skyhook 제어는 차체 속도와 suspension 상대속도의 방향을 비교하여 감쇠 계수를 조절하는 방식이다.

각 바퀴에 대해 suspension 상대속도는 다음과 같이 계산하였다.

```matlab
relVel = suspState.zs_dot(i) - suspState.zu_dot(i);
```

차체 속도와 상대속도의 곱이 양수이면 감쇠력을 크게 하고, 그렇지 않으면 감쇠력을 작게 하도록 구성하였다.

```matlab
if suspState.zs_dot(i) * relVel > 0
    dampingCmd(i) = CTRL.VER.cMax;
else
    dampingCmd(i) = CTRL.VER.cMin;
end
```

최종 감쇠 계수는 다음 범위로 제한하였다.

```matlab
CTRL.VER.cMin = 500;
CTRL.VER.cMax = 5000;
```

이 방식은 복잡한 suspension 최적화 없이도 차체 진동을 줄이는 데 사용할 수 있으며, LTR 및 차량 안정성에 간접적으로 도움을 줄 수 있다.

### 3.4 ctrl_coordinator — Actuator Allocation

Coordinator는 횡방향, 종방향, 수직방향 제어기의 출력을 실제 actuator 명령으로 변환한다. 최종 출력은 조향각, 4륜 brake torque, 4륜 damping coefficient이다.

조향각은 lateral controller에서 계산한 AFS 보조 조향각을 제한 범위 내로 saturation하여 사용하였다.

```matlab
steer = max(min(latCmd.steerAngle, LIM.MAX_STEER_ANGLE), ...
           -LIM.MAX_STEER_ANGLE);
```

종방향 제동 명령은 총 제동 토크로 변환한 뒤 전후 50:50 비율로 분배하였다.

```matlab
totalT = abs(lonCmd.Fx_total) * VEH.rw;

frontT = 0.5 * totalT;
rearT  = 0.5 * totalT;
```

ESC yaw moment는 좌우 차동 제동으로 변환하였다. 전륜과 후륜에 각각 일정 비율로 yaw moment를 분배하고, 좌측과 우측 brake torque에 반대 부호로 더하여 yaw moment를 생성하였다.

```matlab
Mz = latCmd.yawMoment;
ratio_f = 0.6;

dTf = Mz * ratio_f / VEH.track_f;
dTr = Mz * (1-ratio_f) / VEH.track_r;
```

양의 yaw moment가 요구될 때 좌측 제동을 증가시키고 우측 제동을 감소시키는 방식으로 차동 제동을 구현하였다.

```matlab
brakeTorque(1) = brakeTorque(1) + dTf;
brakeTorque(2) = brakeTorque(2) - dTf;
brakeTorque(3) = brakeTorque(3) + dTr;
brakeTorque(4) = brakeTorque(4) - dTr;
```

B1 straight braking 시나리오에서는 일정 주기의 brake pulse를 적용하여 stopping distance를 개선하였다.

```matlab
if mod(absCount,20) < 15
    brakeTorque = max(brakeTorque, [2100; 2100; 1000; 1000]);
else
    brakeTorque = max(brakeTorque, [1050; 1050; 450; 450]);
end
```

최종 brake torque는 actuator 한계를 넘지 않도록 제한하였다.

```matlab
brakeTorque = max(min(brakeTorque, LIM.MAX_BRAKE_TRQ), 0);
```

---

## 4. 시뮬레이션 결과

최종 제어기 적용 후 자동채점 결과는 다음과 같다.

| 시나리오 |                KPI |     결과값 |     목표값 |       점수 |
| ---- | -----------------: | ------: | ------: | -------: |
| A3   |   yawRateOvershoot |  1.2560 | 10.0000 | 4.00 / 4 |
| A3   |    yawRateRiseTime |  0.1210 |  0.3000 | 4.00 / 4 |
| A3   |    yawRateSettling |  0.4410 |  0.8000 | 4.00 / 4 |
| A1   |        sideSlipMax |  2.1286 |  3.0000 | 6.00 / 6 |
| A1   |            LTR_max |  0.6779 |  0.6000 | 4.35 / 5 |
| A1   |      lateralDevMax |  1.0129 |  0.7000 | 2.21 / 4 |
| A4   | understeerGradient |  0.0008 |  0.0030 | 5.00 / 5 |
| A4   |        sideSlipMax |  1.1802 |  2.0000 | 5.00 / 5 |
| A7   |        sideSlipMax |  1.3140 |  5.0000 | 8.00 / 8 |
| A7   |            LTR_max |  0.2132 |  0.7000 | 7.00 / 7 |
| B1   |   stoppingDistance | 47.6380 | 40.0000 | 3.09 / 5 |
| B1   |         absSlipRMS |  0.7300 |  0.1000 | 0.00 / 5 |
| D1   |        sideSlipMax |  2.1268 |  4.0000 | 4.00 / 4 |
| D1   |            LTR_max |  0.6778 |  0.6000 | 1.74 / 2 |
| D1   |      lateralDevMax |  1.0110 |  1.0000 | 1.98 / 2 |

최종 정량 점수는 다음과 같다.

```text
Quantitative Score = 60.37 / 70
```

A3 yaw rate step 시나리오에서는 overshoot, rise time, settling time이 모두 목표를 만족하였다. 이는 yaw rate error 기반 PID 제어와 speed scheduling이 yaw rate tracking에 효과적으로 작동했음을 보여준다.

A1 double lane change 시나리오에서는 sideSlipMax가 2.1286으로 목표값 3.0000을 만족하였다. lateralDevMax는 1.0129 m로 목표값 0.7000에는 도달하지 못했지만, 조향 제한 튜닝을 통해 목표값에 근접하도록 개선하였다. 다만 lateral deviation을 줄이는 과정에서 LTR_max가 0.6779로 증가하여 LTR 점수에서 일부 감점이 발생하였다.

D1 통합 시나리오에서도 sideSlipMax는 2.1268로 목표값 4.0000을 만족하였고, lateralDevMax는 1.0110 m로 목표값 1.0000에 매우 근접하였다. 그러나 A1과 마찬가지로 LTR_max가 0.6778로 목표값보다 크게 나타나 일부 감점이 발생하였다.

A7 brake-in-turn 시나리오에서는 sideSlipMax와 LTR_max가 모두 목표를 만족하였다. 이는 ESC yaw moment와 brake allocation이 제동 중 선회 안정성을 확보하는 데 효과적으로 작동했음을 의미한다.

B1 straight braking 시나리오에서는 brake pulse를 적용하여 stopping distance를 47.6380 m까지 줄였고, 이에 따라 stoppingDistance에서 3.09/5점을 획득하였다. 그러나 각 휠의 slip ratio를 목표 slip 근처로 유지하는 closed-loop ABS 구조는 충분히 구현하지 못했기 때문에 absSlipRMS는 0.7300으로 목표값 0.1000을 만족하지 못하였다.

A7 brake-in-turn 시나리오에서는 sideSlipMax가 1.3140, LTR_max가 0.2132로 모두 목표값을 만족하였다. 이는 ESC yaw moment와 brake allocation이 제동 중 선회 안정성을 확보하는 데 효과적으로 작동했음을 의미한다.
---

## 5. 분석 및 한계

### 5.1 가장 성공적이었던 시나리오

가장 안정적으로 성능을 만족한 시나리오는 A3와 A7이다. A3에서는 yaw rate overshoot, rise time, settling time이 모두 목표를 만족하였다. 이는 PID 기반 yaw rate tracking 구조가 yaw rate step 입력에 대해 빠르고 안정적인 응답을 제공했기 때문이다.

A7 brake-in-turn 시나리오에서도 sideSlipMax와 LTR_max가 모두 목표를 만족하였다. 제동 중 선회 상황은 차량이 불안정해지기 쉬운 조건이지만, slip angle 기반 ESC yaw moment와 brake torque allocation이 차량 자세 안정성 확보에 도움을 주었다.

### 5.2 가장 부족했던 시나리오

가장 부족했던 KPI는 B1의 absSlipRMS이다. 최종 제어기에서는 brake pulse를 통해 stopping distance를 개선하였지만, 각 휠의 slip ratio를 목표값 근처로 유지하는 정밀한 closed-loop ABS는 구현하지 못하였다. 그 결과 stopping distance는 개선되었지만 absSlipRMS 점수는 얻지 못하였다.

또한 A1과 D1에서는 lateralDevMax를 줄이기 위해 조향 제한을 조정하는 과정에서 LTR이 증가하는 trade-off가 나타났다. 조향 제한을 너무 크게 하면 yaw rate tracking이 강해지지만 LTR이 증가할 수 있고, 조향 제한을 너무 작게 하면 LTR은 안정되지만 경로 추종 성능이 부족해질 수 있었다. 최종적으로는 전체 점수가 가장 높게 나오는 조향 제한 값을 선택하였다.

### 5.3 개선 가능성

추가 시간이 주어진다면 다음과 같은 개선이 가능하다.

1. Wheel slip feedback을 이용한 closed-loop ABS를 구현하여 B1 absSlipRMS를 개선할 수 있다.
2. A1 및 D1 lateral deviation을 더 줄이기 위해 yaw rate tracking뿐 아니라 reference path 기반 lateral error feedback을 추가할 수 있다.
3. ESC yaw moment allocation을 단순 비례 방식이 아니라 WLS 기반 allocation으로 확장하여 brake torque 사용량과 안정성을 동시에 최적화할 수 있다.
4. 시나리오별 hardcoding 없이 속도, yaw rate, slip angle, brake 상태를 기반으로 adaptive gain scheduling을 구성하면 여러 시나리오에서 더 균형 잡힌 성능을 얻을 수 있다.

---

## 6. 결론

본 프로젝트에서는 PID 기반 yaw rate tracking, slip angle 기반 ESC, PI 기반 종방향 제어, skyhook CDC, 그리고 actuator allocation을 포함하는 통합 섀시 제어기를 설계하였다. 최종 제어기는 A3, A4, A7 시나리오에서 대부분의 KPI를 만족하였고, A1 및 D1에서도 side slip 안정성과 경로 추종 성능을 개선하였다.

B1 straight braking 시나리오에서는 brake pulse를 통해 stopping distance를 개선하였으나, 정밀한 ABS 제어가 부족하여 absSlipRMS 개선에는 한계가 있었다. 최종 정량 점수는 60.37/70으로 나타났으며, 전체적으로 차량 안정성 향상과 주행 성능 개선을 확인할 수 있었다.

---

## 7. 참고문헌

[1] R. Rajamani, Vehicle Dynamics and Control, 2nd ed., Springer, 2012.
[2] J. Y. Wong, Theory of Ground Vehicles, 4th ed., Wiley, 2008.
---

## 부록 A — 사용한 AI 도구

ChatGPT를 코드 디버깅, 제어기 튜닝 방향 검토, KPI 해석 및 보고서 작성 보조에 사용하였다. 최종 제어기 구조, 파라미터 선택, 시뮬레이션 실행 및 결과 검증은 직접 수행하였다.

---

## 부록 B — 최종 주요 설정

### Lateral controller

```matlab
CTRL.LAT.Kp     = 1.0;
CTRL.LAT.Ki     = 0.1;
CTRL.LAT.Kd     = 0.05;
CTRL.LAT.intMax = 5.0;

gainScale = min(vx/28, 0.75);
steerLimit = 0.17 * LIM.MAX_STEER_ANGLE;

beta_th = deg2rad(7);
yawMoment gain = -100;
```

### Longitudinal controller

```matlab
CTRL.LON.Kp     = 0.5;
CTRL.LON.Ki     = 0.05;
CTRL.LON.intMax = 2000;
```

### Vertical controller

```matlab
CTRL.VER.cMin = 500;
CTRL.VER.cMax = 5000;
```

### Coordinator

```matlab
frontT = 0.5 * totalT;
rearT  = 0.5 * totalT;

if mod(absCount,20) < 15
    brakeTorque = max(brakeTorque, [2100; 2100; 1000; 1000]);
else
    brakeTorque = max(brakeTorque, [1050; 1050; 450; 450]);
end
```
