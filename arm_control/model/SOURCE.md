# 접근 구간 조회표

IntergrateSim `artifacts/deploy_2026-11-05/`(JaxIsaacLab `jax-one-arm-grasp-sim2real` 52e1154)의 v65 조회표를 그대로 복사했다. 사용법은 그 폴더 README 7.1–7.3절. 학습 모델이 아니라 큐브 위치 격자별로 미리 푼 IK 보정값이다(torch 텐서 파일, `arm_control/model_tables.py`가 torch 없이 읽는다). 해시는 `SHA256SUMS`.

# 쥐고 드는 정책과 로봇 모델 (2026-11-19 추가)

- `grasp_lift_25550.npz`: JaxIsaacLab `main` `artifacts/deploy_2026-11-05/grasp_lift_25550.onnx`(sha256 `59d5c351f278c96ebc57de77e94f09e5c54374122d2f12e768ee6dbbb1a382bd`)의 가중치를 numpy로 옮긴 것(mean, div = std + 0.01, w0/b0, w1/b1, w2/b2). onnxruntime 1.32 출력과 무작위 입력 200개에서 최대 4.3e-4 차이(float32). 실물 PC에 torch·onnxruntime이 없어도 돌게 하려고 바꿨다. 사용법은 그 폴더 README 7.4–8.2절, `arm_control/grasp.py`.
- `robot_v3.urdf`: 같은 폴더 `robot_v3/urdf/JAXV3_Assembly.urdf` 그대로(문서 부호 규약). `arm_control/robot_v3.py`가 순운동학만 쓴다(메시 없음). `handover_state_25550_example.json`의 손·손끝 위치를 0.1 mm 안에서 재현(`tests/check_grasp.py`).
