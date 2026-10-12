import math


def calculate_goal(x, y, theta, goal_x, goal_y):
    """現在位置と向きから、目標までの距離と旋回角度を計算する。"""

    dx = goal_x - x
    dy = goal_y - y

    distance = math.hypot(dx, dy)

    target_angle = math.atan2(dy, dx)
    angle_error = target_angle - theta

    # -π ～ +π に正規化
    angle_error = math.atan2(
        math.sin(angle_error),
        math.cos(angle_error)
    )

    return distance, angle_error