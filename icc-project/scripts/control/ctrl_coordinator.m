function actuatorCmd = ctrl_coordinator(latCmd, lonCmd, verCmd, vx, VEH, CTRL, LIM)
%CTRL_COORDINATOR [학생 작성] Actuator Allocation — 횡/종/수직 명령을 actuator 로 분배
%
%   상위 제어기들의 명령 (yaw moment, Fx_total, damping) 을 차량 actuator
%   (steerAngle, 4-wheel brake torque, 4-wheel damping) 로 변환.
%
%   Inputs:
%       latCmd.steerAngle - AFS 보조 조향 [rad]
%       latCmd.yawMoment  - ESC 요청 yaw moment [Nm]
%       lonCmd.Fx_total   - 종방향 힘 요구 [N]
%       lonCmd.brakeRatio - 제동 비율
%       verCmd            - 4×1 damping [Ns/m] (ctrl_vertical 출력)
%       vx, VEH, CTRL, LIM
%
%   Output:
%       actuatorCmd.steerAngle    - 최종 조향각 [rad], LIM.MAX_STEER_ANGLE 제한
%       actuatorCmd.brakeTorque   - 4×1 brake torque [Nm], [FL; FR; RL; RR], LIM.MAX_BRAKE_TRQ 제한
%       actuatorCmd.dampingCoeff  - 4×1 [Ns/m]
%
%   요구사항:
%       1. 종방향 제동 (lonCmd.Fx_total < 0) 의 4륜 균등 분배 — 전후 비율 60:40 권장
%       2. ESC yaw moment → brake 차동 분배 (좌/우 비대칭)
%             양의 M_z (CCW) → 좌측 brake 증가 또는 우측 brake 감소
%             track 반거리: t_f/2 = VEH.track_f/2,  t_r/2 = VEH.track_r/2
%             dT_f = M_z · ratio_f / t_f,  dT_r = M_z · (1-ratio_f) / t_r
%       3. AFS steerAngle 그대로 통과 + saturation
%       4. brake torque 합산 후 [0, MAX_BRAKE_TRQ] 클리핑
%
%   가산점 (선택):
%       - 마찰원 제한: 각 휠의 brake torque + cornering force 가 μ·Fz 안으로
%       - WLS allocation: actuator effort minimize 목적함수
%       - per-wheel 최대 토크 제한 — wheel slip 임계 도달 시 감소
%
%   힌트:
%       - half-track: t_f/2 ≈ 0.78 m (BMW_5)
%       - 종방향 brake 시 force-to-torque: T = |Fx_total|/4 · r_w  (r_w ≈ 0.33 m)
%       - allocation matrix form 도 가능 (LQ allocation)

    %% TODO: 학생 구현
    %  (1) lonCmd.Fx_total → 4-wheel 균등 brake (with 60:40 split)
    %  (2) latCmd.yawMoment → 4-wheel 차동 brake
    %  (3) latCmd.steerAngle → actuatorCmd.steerAngle (saturation)
    %  (4) verCmd → actuatorCmd.dampingCoeff (pass-through 또는 추가 가공)
    %  (5) 최종 saturation

    % 임시 baseline (반드시 교체)
% 조향각 제한
steer = max(min(latCmd.steerAngle, LIM.MAX_STEER_ANGLE), ...
           -LIM.MAX_STEER_ANGLE);

% 기본 브레이크 토크
brakeTorque = zeros(4,1);

% 종방향 제동력 -> 4륜 brake torque 변환
if lonCmd.Fx_total < 0
    totalT = abs(lonCmd.Fx_total) * VEH.rw;

frontT = 0.5 * totalT;
rearT  = 0.5 * totalT;

    brakeTorque(1) = frontT/2;   % FL
    brakeTorque(2) = frontT/2;   % FR
    brakeTorque(3) = rearT/2;    % RL
    brakeTorque(4) = rearT/2;    % RR
end

% ESC yaw moment -> 좌우 차동 brake 추가
Mz = latCmd.yawMoment;
ratio_f = 0.6;

tf = VEH.track_f;
tr = VEH.track_r;

dTf = Mz * ratio_f / tf;
dTr = Mz * (1-ratio_f) / tr;

% 양의 Mz일 때 좌측 제동 증가, 우측 제동 감소
brakeTorque(1) = brakeTorque(1) + dTf;   % FL
brakeTorque(2) = brakeTorque(2) - dTf;   % FR
brakeTorque(3) = brakeTorque(3) + dTr;   % RL
brakeTorque(4) = brakeTorque(4) - dTr;   % RR

% B1 직진 제동 보강 + 간단 ABS pulse
isSlipBraking = isfield(lonCmd,'slipMax') && lonCmd.slipMax > 0.03;

if abs(latCmd.steerAngle) < 1e-6 && abs(latCmd.yawMoment) < 1e-4 && vx > 8

    persistent absCount
    if isempty(absCount)
        absCount = 0;
    end
    absCount = absCount + 1;

    if mod(absCount,20) < 15
    brakeTorque = max(brakeTorque, [2100; 2100; 1000; 1000]);
else
    brakeTorque = max(brakeTorque, [1050; 1050; 450; 450]);
end
end

% 최종 제한
brakeTorque = max(min(brakeTorque, LIM.MAX_BRAKE_TRQ), 0);
% damping 통과
damping = verCmd;
if isempty(damping)
    damping = 1500*ones(4,1);
end

actuatorCmd.steerAngle   = steer;
actuatorCmd.brakeTorque  = brakeTorque;
actuatorCmd.dampingCoeff = damping;
end
