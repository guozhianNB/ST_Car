from maix import camera, display, image, app
from maix import uart, pinmap, err


# ============================================================
# 1. 摄像头与串口
# ============================================================

IMAGE_W = 640
IMAGE_H = 480
FPS = 30

ENABLE_UART = True
DEBUG_DRAW = True

cam = camera.Camera(
    IMAGE_W,
    IMAGE_H,
    fps=FPS,
    buff_num=2
)

disp = display.Display()

# 等待自动曝光稳定
cam.skip_frames(50)

# 最终光照固定后，可以尝试固定曝光和增益。
# 先保持关闭，确认识别代码正常后再开启。
FIX_EXPOSURE = False

if FIX_EXPOSURE:
    cam.exposure(1500)
    cam.gain(100)


serial = None

if ENABLE_UART:
    err.check_raise(
        pinmap.set_pin_function("A21", "UART4_TX"),
        "UART4 TX init failed"
    )

    err.check_raise(
        pinmap.set_pin_function("A22", "UART4_RX"),
        "UART4 RX init failed"
    )

    serial = uart.UART("/dev/ttyS4", 115200)


# ============================================================
# 2. 固定标定参数
# 根据你最新的 640×480 截图填写
# ============================================================

# 左侧标记对应 -10 cm
LEFT_REF_X = 27.0
LEFT_REF_Y = 287.0

# 右侧标记对应 +10 cm
RIGHT_REF_X = 585.0
RIGHT_REF_Y = 299.0

# 小球全局搜索区域
#
# 当前截图中管道中心大约位于 y=287~299。
# ROI 覆盖管道中间区域，但尽量避开上下边缘。
BALL_GLOBAL_ROI = [35, 270, 565, 58]


# ============================================================
# 3. 小球 LAB 阈值
# ============================================================

# 这是此前基本能够稳定检测的严格阈值。
BALL_THRESHOLD_STRICT = [
    50, 100,
    -12, 20,
    -8, 22
]

# 只有在上一帧附近的很小区域内才使用宽松阈值。
# 它不会参与全管道搜索，因此不容易跳到远处反光。
BALL_THRESHOLD_LOCAL = [
    38, 100,
    -18, 27,
    -12, 28
]


# ============================================================
# 4. 小球候选范围
# ============================================================

# 根据之前画面，小球有效色块大致在这个范围。
BALL_MIN_W = 7
BALL_MAX_W = 34

BALL_MIN_H = 7
BALL_MAX_H = 34

BALL_MIN_PIXELS = 12
BALL_MAX_PIXELS = 500

# 当前画面中的期望尺寸，仅用于评分，不作为硬限制
TARGET_BALL_W = 18.0
TARGET_BALL_H = 18.0

# 球心允许偏离标定轴线的距离
MAX_AXIS_DISTANCE = 18.0

# 正常跟踪时，每帧最大允许跳动
MAX_TRACK_JUMP = 60.0


# ============================================================
# 5. 跟踪和滤波参数
# ============================================================

# 局部搜索左右各覆盖多少像素
LOCAL_HALF_WIDTH = 48

# 当前位置的滤波权重
# 越高响应越快，越低越稳定
POSITION_ALPHA = 0.72

# 连续两帧检测失败时保持旧位置
MAX_HOLD_FRAMES = 2

# 连续丢失三帧后才允许全局重捕获
GLOBAL_REACQUIRE_FRAMES = 3

PRINT_INTERVAL = 30


# ============================================================
# 6. 工具函数
# ============================================================

def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def clamp_roi(roi):
    x, y, w, h = roi

    x = clamp(int(x), 0, IMAGE_W - 1)
    y = clamp(int(y), 0, IMAGE_H - 1)

    w = clamp(int(w), 1, IMAGE_W - x)
    h = clamp(int(h), 1, IMAGE_H - y)

    return [x, y, w, h]


def reference_line_y(x):
    """
    根据左右两个固定标定点计算管道轴线的 y。

    黄线方向完全由：
        (LEFT_REF_X, LEFT_REF_Y)
        (RIGHT_REF_X, RIGHT_REF_Y)
    决定。
    """

    denominator = RIGHT_REF_X - LEFT_REF_X

    if abs(denominator) < 0.001:
        return (LEFT_REF_Y + RIGHT_REF_Y) / 2.0

    ratio = (x - LEFT_REF_X) / denominator

    return (
        LEFT_REF_Y
        + ratio * (RIGHT_REF_Y - LEFT_REF_Y)
    )


def project_to_reference_axis(x, y):
    """
    把球心正交投影到左右标记定义的水管轴线。

    返回：
        ratio：左标记为 0，右标记为 1 的轴向比例
        projected_x / projected_y：投影点像素坐标
    """
    axis_x = RIGHT_REF_X - LEFT_REF_X
    axis_y = RIGHT_REF_Y - LEFT_REF_Y
    denominator = axis_x * axis_x + axis_y * axis_y
    if denominator < 0.001:
        return 0.5, LEFT_REF_X, LEFT_REF_Y
    ratio = (
        (x - LEFT_REF_X) * axis_x
        + (y - LEFT_REF_Y) * axis_y
    ) / denominator
    return (
        ratio,
        LEFT_REF_X + ratio * axis_x,
        LEFT_REF_Y + ratio * axis_y
    )


def axis_ratio_to_cm(ratio):
    """左标记 = -10 cm，右标记 = +10 cm。"""
    return -10.0 + ratio * 20.0


def make_local_roi(center_x):
    """
    只在上一帧小球附近进行宽松搜索。
    """

    x0 = center_x - LOCAL_HALF_WIDTH
    width = LOCAL_HALF_WIDTH * 2

    return clamp_roi([
        x0,
        BALL_GLOBAL_ROI[1],
        width,
        BALL_GLOBAL_ROI[3]
    ])


def median_value(values):
    ordered = sorted(values)
    count = len(ordered)

    if count == 0:
        return 0.0

    if count % 2 == 1:
        return ordered[count // 2]

    return (
        ordered[count // 2 - 1]
        + ordered[count // 2]
    ) / 2.0


# ============================================================
# 7. 选择最可信的小球候选
# ============================================================

def find_best_ball(
    img,
    threshold,
    roi,
    predicted_x=None,
    use_jump_gate=False
):
    blobs = img.find_blobs(
        [threshold],
        roi=roi,

        x_stride=1,
        y_stride=1,

        pixels_threshold=5,
        area_threshold=5,

        merge=True,
        margin=1
    )

    best_blob = None
    best_center = None
    best_score = None

    for blob in blobs:
        w = float(blob.w())
        h = float(blob.h())
        pixels = float(blob.pixels())

        # ------------------------------
        # 基础尺寸过滤
        # ------------------------------

        if w < BALL_MIN_W or w > BALL_MAX_W:
            continue

        if h < BALL_MIN_H or h > BALL_MAX_H:
            continue

        if pixels < BALL_MIN_PIXELS:
            continue

        if pixels > BALL_MAX_PIXELS:
            continue

        aspect = w / max(h, 1.0)

        # 排除细长的管道高光
        if aspect < 0.48 or aspect > 2.10:
            continue

        # 使用外框中心。
        # 相比 blob 质心，它较少受到球面高光位置变化的影响。
        cx = blob.x() + w / 2.0
        cy = blob.y() + h / 2.0

        expected_y = reference_line_y(cx)
        axis_distance = abs(cy - expected_y)

        if axis_distance > MAX_AXIS_DISTANCE:
            continue

        motion_distance = 0.0

        if predicted_x is not None:
            motion_distance = abs(cx - predicted_x)

            if use_jump_gate and motion_distance > MAX_TRACK_JUMP:
                continue

        # ------------------------------
        # 柔性评分：越低越可信
        # ------------------------------

        score = 0.0

        # 尺寸接近期望小球
        score += abs(w - TARGET_BALL_W) * 0.8
        score += abs(h - TARGET_BALL_H) * 0.8

        # 宽高越接近越好
        score += abs(w - h) * 1.4

        # 越靠近管道轴线越好
        score += axis_distance * 1.8

        # 越靠近上一帧位置越好
        score += motion_distance * 0.35

        # 略微鼓励有效像素较多的候选
        score -= min(pixels, 200.0) * 0.018

        if best_score is None or score < best_score:
            best_score = score
            best_blob = blob
            best_center = (cx, cy)

    return best_blob, best_center, best_score


# ============================================================
# 8. 绘制清晰的小球中心
# ============================================================

def draw_ball_center(img, x, y):
    x = int(round(x))
    y = int(round(y))

    # 白色外圈
    img.draw_circle(
        x,
        y,
        8,
        image.COLOR_WHITE,
        2
    )

    # 红色实心圆点
    img.draw_circle(
        x,
        y,
        5,
        image.COLOR_RED,
        -1
    )

    # 中央白色十字
    img.draw_line(
        x - 3,
        y,
        x + 3,
        y,
        image.COLOR_WHITE,
        1
    )

    img.draw_line(
        x,
        y - 3,
        x,
        y + 3,
        image.COLOR_WHITE,
        1
    )


# ============================================================
# 9. 状态变量
# ============================================================

frame_count = 0
lost_count = 0

filtered_x = None
raw_x_history = []
filtered_axis_ratio = None
raw_axis_history = []

last_ball_blob = None


# ============================================================
# 10. 主循环
# ============================================================

while not app.need_exit():
    img = cam.read()
    frame_count += 1

    ball_blob = None
    raw_center = None
    ball_score = None
    used_roi = BALL_GLOBAL_ROI

    # --------------------------------------------------------
    # A. 已经建立跟踪时，只在上一位置附近搜索
    # --------------------------------------------------------

    if filtered_x is not None:
        local_roi = make_local_roi(filtered_x)
        used_roi = local_roi

        (
            ball_blob,
            raw_center,
            ball_score
        ) = find_best_ball(
            img,
            BALL_THRESHOLD_LOCAL,
            local_roi,
            predicted_x=filtered_x,
            use_jump_gate=True
        )

    # --------------------------------------------------------
    # B. 初次启动，或连续丢失达到条件时，全局重捕获
    # --------------------------------------------------------

    allow_global_search = (
        filtered_x is None
        or lost_count >= GLOBAL_REACQUIRE_FRAMES
    )

    if ball_blob is None and allow_global_search:
        used_roi = BALL_GLOBAL_ROI

        (
            ball_blob,
            raw_center,
            ball_score
        ) = find_best_ball(
            img,
            BALL_THRESHOLD_STRICT,
            BALL_GLOBAL_ROI,
            predicted_x=None,
            use_jump_gate=False
        )

        # 全局搜索时要求候选质量较好，
        # 防止随便跳到一块反光。
        if (
            ball_blob is not None
            and ball_score is not None
            and ball_score > 55.0
        ):
            ball_blob = None
            raw_center = None

    status = 0
    x_cm = 0.0
    display_x = None
    display_y = None

    # --------------------------------------------------------
    # C. 检测成功
    # --------------------------------------------------------

    if ball_blob is not None:
        raw_x = raw_center[0]
        raw_y = raw_center[1]
        raw_axis_ratio, _, _ = project_to_reference_axis(raw_x, raw_y)

        # 长时间丢失后重新找到，清空旧滤波历史
        if filtered_x is None or lost_count >= GLOBAL_REACQUIRE_FRAMES:
            raw_x_history = []
            raw_axis_history = []
            filtered_x = raw_x
            filtered_axis_ratio = raw_axis_ratio

        raw_x_history.append(raw_x)
        raw_axis_history.append(raw_axis_ratio)

        if len(raw_x_history) > 3:
            raw_x_history.pop(0)
        if len(raw_axis_history) > 3:
            raw_axis_history.pop(0)

        median_x = median_value(raw_x_history)
        median_axis_ratio = median_value(raw_axis_history)

        filtered_x = (
            POSITION_ALPHA * median_x
            + (1.0 - POSITION_ALPHA) * filtered_x
        )
        filtered_axis_ratio = (
            POSITION_ALPHA * median_axis_ratio
            + (1.0 - POSITION_ALPHA) * filtered_axis_ratio
        )

        lost_count = 0
        status = 1

        x_cm = axis_ratio_to_cm(filtered_axis_ratio)
        display_x = (
            LEFT_REF_X
            + filtered_axis_ratio * (RIGHT_REF_X - LEFT_REF_X)
        )
        display_y = (
            LEFT_REF_Y
            + filtered_axis_ratio * (RIGHT_REF_Y - LEFT_REF_Y)
        )

        last_ball_blob = ball_blob

    # --------------------------------------------------------
    # D. 暂时丢失
    # --------------------------------------------------------

    else:
        lost_count += 1

        if (
            filtered_x is not None
            and lost_count <= MAX_HOLD_FRAMES
        ):
            # 丢失一两帧时保持最后位置，不进行乱猜
            status = 2
            x_cm = axis_ratio_to_cm(filtered_axis_ratio)
            display_x = (
                LEFT_REF_X
                + filtered_axis_ratio * (RIGHT_REF_X - LEFT_REF_X)
            )
            display_y = (
                LEFT_REF_Y
                + filtered_axis_ratio * (RIGHT_REF_Y - LEFT_REF_Y)
            )

        else:
            status = 0

            if lost_count >= 6:
                filtered_x = None
                raw_x_history = []
                filtered_axis_ratio = None
                raw_axis_history = []
                last_ball_blob = None

    # --------------------------------------------------------
    # E. UART
    # --------------------------------------------------------

    # UART 保持发送毫米，避免后续控制代码精度损失：
    #
    # $B,位置毫米,状态
    #
    # 例如：
    # $B,49.5,1
    if serial is not None:
        if status != 0:
            x_mm = x_cm * 10.0

            serial.write_str(
                "$B,%.1f,%d\n"
                % (x_mm, status)
            )
        else:
            serial.write_str(
                "$B,0.0,0\n"
            )

    # --------------------------------------------------------
    # F. 画面显示
    # --------------------------------------------------------

    # 正确的固定标定黄线
    img.draw_line(
        int(round(LEFT_REF_X)),
        int(round(LEFT_REF_Y)),
        int(round(RIGHT_REF_X)),
        int(round(RIGHT_REF_Y)),
        image.COLOR_YELLOW,
        2
    )

    # 左侧 -10 cm 参考点
    img.draw_circle(
        int(round(LEFT_REF_X)),
        int(round(LEFT_REF_Y)),
        5,
        image.COLOR_BLUE,
        -1
    )

    # 右侧 +10 cm 参考点
    img.draw_circle(
        int(round(RIGHT_REF_X)),
        int(round(RIGHT_REF_Y)),
        5,
        image.COLOR_GREEN,
        -1
    )

    # 真实检测到时才绘制小球色块框
    if ball_blob is not None:
        img.draw_rect(
            ball_blob.x(),
            ball_blob.y(),
            ball_blob.w(),
            ball_blob.h(),
            image.COLOR_RED,
            2
        )

    if display_x is not None:
        if status == 1:
            draw_ball_center(
                img,
                display_x,
                display_y
            )
        else:
            # 黄色空心圆表示只是短时保持值
            img.draw_circle(
                int(round(display_x)),
                int(round(display_y)),
                7,
                image.COLOR_YELLOW,
                2
            )

    if DEBUG_DRAW:
        img.draw_rect(
            used_roi[0],
            used_roi[1],
            used_roi[2],
            used_roi[3],
            image.COLOR_RED,
            1
        )

    # 屏幕统一显示厘米
    img.draw_string(
        4,
        4,
        "x=%+.2fcm s=%d lost=%d"
        % (x_cm, status, lost_count),
        image.COLOR_RED,
        0.9
    )

    # 不再每帧打印，避免终端拖慢程序
    if frame_count % PRINT_INTERVAL == 0:
        print(
            "x=%+.3fcm status=%d lost=%d score=%s"
            % (
                x_cm,
                status,
                lost_count,
                str(ball_score)
            )
        )

    disp.show(img)
