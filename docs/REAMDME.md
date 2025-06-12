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
