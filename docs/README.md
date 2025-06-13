stateDiagram-v2

    [*] --> NORMAL

    NORMAL --> WAIT_LIGHT: /detect/stop_line (True)
    WAIT_LIGHT --> NORMAL: /detect/traffic_light_color (GREEN)

    NORMAL --> STOPPING: /detect/sign (stop)
    STOPPING --> WAIT_LEVEL_CROSSING: 8초 경과
    WAIT_LEVEL_CROSSING --> NORMAL: /detect/level_crossing_state (go, after stop received)

    NORMAL --> LEFT_TURN: /detect/sign (left)
    LEFT_TURN --> NORMAL: Turn Completed

    NORMAL --> RIGHT_TURN: /detect/sign (right)
    RIGHT_TURN --> NORMAL: Turn Completed


### 회전
회전 표지판 감지 시 (/detect/sign)

    'left' → LEFT_TURN 상태 진입

    'right' → RIGHT_TURN 상태 진입

회전 진행 시간에 따라 진행률(progress) 계산

Bias 적용:

    진행률에 비례하여 차선 중심을 이동

    LEFT_TURN: desired_bias = -100 * progress

    RIGHT_TURN: desired_bias = 100 * progress

Bias가 적용된 중심값으로 PID 제어 수행 → 부드러운 선회

진행 완료 후 NORMAL 상태 복귀


### 발표
https://www.notion.so/3-1-21122b700e8380378729efcaf2fe4099?source=copy_link
