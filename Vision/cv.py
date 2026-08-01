#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MaixCAM2 - 电赛 H 题滚球位置检测（fp61：对侧机位、标记点在线标定与伪影抑制）

固定摄像头，三根白色塑料条与水管刚性连接。2026-07-31 相机移到
水管另一侧后，当前图像从左到右为：
    左侧：1 条蓝横线 = -10 cm
    中间：2 条蓝横线 =   0 cm
    右侧：3 条蓝横线 = +10 cm

当前结构在水管上方和下方都能看到同一组蓝横线，因此正常总数约为：
    +10 cm：6 个蓝色连通域
      0 cm：4 个蓝色连通域
    -10 cm：2 个蓝色连通域
代码同时兼容只拍到单侧时的 3 / 2 / 1 个蓝色连通域。

核心方法：
1. 只阈值蓝色，按横向位置聚类，不再识别橙管上的细油漆线；
2. 用 2/4/6（或 1/2/3）数量编码识别左/中/右三根塑料条；
3. 对每根塑料条的所有蓝横线中心做直线拟合；
4. 估计水管方向并旋转拉平，求塑料条中心线与钢球运动轴线的交点；
5. 三个交点分别作为 -10、0、+10 cm，做一维透视映射；
6. 在很窄的水管区域内寻找低饱和度、近圆形、具有灰度起伏的钢球；
7. UART2（B0 TX / B1 RX）输出：$B,位置毫米,有效标志\r\n

坐标约定：图像左侧 1 条蓝横线塑料条为 -100 mm，中间为 0 mm，
图像右侧 3 条蓝横线塑料条为 +100 mm。当前版本只发送 status=1（真实检测）
和 status=0（无效）；STM32 对 status=2 的兼容只用于旧版视觉程序。

建议：正式闭环时可以将 ENABLE_PREVIEW=False，UART 检测仍会全速运行。
"""

from maix import camera, display, image, app, time, uart, pinmap, gpio, err
import cv2
import numpy as np
import math
import os


# ============================================================
# 1. 用户配置
# ============================================================
# 直接使用 320×240。当前画面中 ±10 cm 两点相距约 210 像素，
# 即约 10 像素/cm，已经远高于题目 ±1 cm 的视觉分辨率要求。
FRAME_W = 320
FRAME_H = 240
CAMERA_FPS = 60
CAMERA_BUFFER_NUM = 2
CAMERA_MANUAL_EXPOSURE_US = 2500
CAMERA_MANUAL_GAIN = 8500

# MaixCAM2 背部板载照明 LED。官方硬件定义为 B25/GPIOB25，高电平点亮。
# 在创建相机和跳过起始帧之前打开，使固定曝光和视觉阈值始终面对同一光照。
FILL_LIGHT_ENABLED = True
FILL_LIGHT_PIN = "B25"
FILL_LIGHT_GPIO = "GPIOB25"

# 蓝色横线 HSV 阈值（OpenCV：H 范围 0~179）。
BLUE_H_MIN = 88
BLUE_H_MAX = 142
BLUE_S_MIN = 60
BLUE_V_MIN = 35

# 蓝横线连通域约束，按 320×240 实拍图设置。
BLUE_AREA_MIN = 18
BLUE_AREA_MAX = 420
BLUE_WIDTH_MIN = 5
BLUE_WIDTH_MAX = 45
BLUE_HEIGHT_MIN = 2
BLUE_HEIGHT_MAX = 22
BLUE_MAX_ANGLE_DEG = 27.0

# 图像边缘通常有 MaixVision 文字、线缆或其他蓝色物体，先屏蔽。
MARKER_X_MIN_RATIO = 0.09
MARKER_X_MAX_RATIO = 0.95
MARKER_Y_MIN_RATIO = 0.16
MARKER_Y_MAX_RATIO = 0.93

# 同一塑料条的蓝横线中心，沿水管方向的距离很小。
MARKER_CLUSTER_GAP_RATIO = 0.050

# 三个标定点的几何约束。
MARKER_SPAN_RATIO_MIN = 0.48
MARKER_SPAN_RATIO_MAX = 0.88
MARKER_ZERO_RATIO_MIN = 0.36
MARKER_ZERO_RATIO_MAX = 0.64
MARKER_LEFT_SINGLE_COUNT = 1
MARKER_ZERO_SINGLE_COUNT = 2
MARKER_RIGHT_SINGLE_COUNT = 3
# Projective mapping first treats the image-left marker as +10 cm.  Multiply
# by this sign so the current opposite-side camera reports image-left negative.
IMAGE_LEFT_POSITION_SIGN = -1.0
# VERIFIED 2026-07-31 with the camera rigidly mounted on the chassis and the
# ball at the left endpoint, leaving the centre/right strips unobscured.  Image
# left/centre/right are respectively -100/0/+100 mm in physical coordinates.
# Set to None only when deliberately performing a new camera calibration.
FIXED_MARKER_X = (42.3, 155.0, 269.8)
MARKER_HOLD_FRAMES = 60
MARKER_MAX_JUMP_PX = 8.0
MARKER_SPAN_CHANGE_MAX = 0.08
# The camera is rigidly fixed to the chassis.  Build the initial longitudinal
# calibration from several consecutive observations, then only let it follow
# the robust median very slowly.  The live tube-axis row remains unsmoothed.
MARKER_LOCK_SAMPLES = 7
MARKER_GEOMETRY_ALPHA = 0.05
# The camera and calibration strips are fixed.  Revalidate their geometry at a
# low rate and spend the intervening frames on ball tracking; the last verified
# three-point mapping remains the only source of the longitudinal coordinate.
MARKER_UPDATE_EVERY_N = 6

# 橙色水管行检测。旋转后只扫描画面中部。
ORANGE_H_MIN = 4
ORANGE_H_MAX = 33
ORANGE_S_MIN = 55
ORANGE_V_MIN = 55
TUBE_SEARCH_Y_MIN_RATIO = 0.27
TUBE_SEARCH_Y_MAX_RATIO = 0.68
TUBE_MIN_ROW_COVERAGE = 0.42

# 橙色响应峰通常在钢球运动轴线上方几像素。
# 当前 20 cm 标记跨度约 210 px，偏移约 4 px。
BALL_AXIS_OFFSET_RATIO = 0.019

# After rotation the tube-colour response line passes about 2 px above the
# visible steel-ball centre (roughly 1% of the calibrated 20 cm marker span).
# The tolerance accepts the ball silhouette as the tube tilts but rejects the
# fixed circular hardware about 15--20 px below the tube, without blacklisting
# any horizontal ball position.
BALL_CENTER_AXIS_OFFSET_RATIO = 0.010
# 背部补光后管子下沿的高对比反光会形成假圆点；固定顶视相机使真实球心
# 必须紧贴在线估计的运动轴，因此可用更窄的轴向门限排除这些固定反光。
BALL_CENTER_Y_TOLERANCE_DIAMETER = 0.52
# A tilted tube moves the apparent ball centroid vertically and can stretch
# its thresholded dark core.  Once temporal tracking is established, allow a
# wider vertical corridor while retaining the stricter cold-acquire gate.
BALL_TRACK_CENTER_Y_TOLERANCE_DIAMETER = 0.90
BALL_ROI_EXTENSION_RATIO = 0.200
BALL_VALID_POSITION_LIMIT_CM = 14.0
BALL_REPORTED_POSITION_LIMIT_CM = 14.0
# The free ball remains visible beyond the +/-100 mm strips.  Cold endpoint
# reacquisition is allowed in the full ROI, with a separate boundary-contact
# shape gate below to reject the wider fixed end hardware.
BALL_ACQUIRE_EDGE_GUARD_RATIO = -0.200

# 钢球检测。
BALL_LOW_SAT_MAX = 158
BALL_VALUE_MIN = 42
# Current fixed-camera calibration: the +100 mm to -100 mm marker span is
# about 310 px and the visible 10 mm steel ball is about 11.5 px.
BALL_DIAMETER_TO_MARKER_SPAN = 0.037
BALL_ACQUIRE_SCORE = 3.85
BALL_DARK_ACQUIRE_SCORE = 2.25
BALL_ENDPOINT_DARK_ACQUIRE_SCORE = 2.00
BALL_ACQUIRE_SCORE_MARGIN = 0.40
BALL_TRACK_SCORE = 2.20
BALL_PRIMARY_MAX_JUMP_PX = 35.0
BALL_LOCAL_SEARCH_HALF_WIDTH_PX = 80.0
BALL_OPTICAL_FLOW_MAX_JUMP_PX = 55.0
BALL_TRACK_JUMP_GROWTH_PX = 8.0
BALL_TRACK_MOTION_GAIN = 1.0
BALL_TRACK_MAX_EXTRA_JUMP_PX = 90.0
BALL_TRACK_DISTANCE_PENALTY = 1.20
BALL_FALLBACK_MAX_JUMP_PX = 55.0
# The gate is evaluated against elapsed wall-clock time, not an assumed frame
# rate.  2200 mm/s is above the speed needed to cross the entire 250 mm tube in
# 0.12 s, while a one-frame jump between opposite fixed highlights is still
# rejected.  STM32 uses the same ceiling for its independent UART sanity gate.
BALL_TRACK_MAX_SPEED_MM_S = 2200.0
BALL_TRACK_MAX_DT_S = 0.150
# A real ball cannot instantaneously become a stationary tube/marker feature.
# Use a deliberately generous physical acceleration envelope so reflections
# are rejected without constraining the controller's useful motion bandwidth.
BALL_TRACK_MAX_ACCEL_MM_S2 = 4500.0
BALL_TRACK_VELOCITY_SLACK_MM_S = 80.0
BALL_MARKER_MOTION_GUARD_SPEED_MM_S = 80.0
BALL_MARKER_MOTION_MIN_FRACTION = 0.20
BALL_HOLD_FRAMES = 30
BALL_MAX_CONSECUTIVE_FALLBACK_FRAMES = 2
BALL_MAX_CONSECUTIVE_LOCAL_FRAMES = 30
BALL_MAX_CONSECUTIVE_TEMPLATE_FRAMES = 10
BALL_TEMPLATE_MIN_CORRELATION = 0.42
BALL_HOUGH_FALLBACK_SCORE = 3.30
BALL_HOUGH_ACQUIRE_SCORE = 4.40
# HoughCircles costs 300--590 ms on MaixCAM2 when the ball is moving or briefly
# missed.  The lit fixed-camera path uses connected components + local tracking
# and must never fall back to this blocking global circle search.
BALL_ENABLE_HOUGH_FALLBACK = False
# The brute-force local specular scan evaluates hundreds of tiny patches and
# measured 180--300 ms whenever motion briefly defeats the primary detector.
# A single status=0 frame is safer for the controller than freezing capture,
# preview and UART together, so the real-time build keeps this path disabled.
BALL_ENABLE_LOCAL_SPECULAR_FALLBACK = False
BALL_ACQUIRE_CONFIRM_FRAMES = 4
BALL_ENDPOINT_ACQUIRE_CONFIRM_FRAMES = 2
# Reacquisition remains stricter than established tracking.  Its pixel gate is
# deliberately unchanged because the time-based high-speed path above keeps a
# genuine moving ball tracked without repeatedly falling back to acquisition.
BALL_ACQUIRE_CONFIRM_JUMP_PX = 16.0
# The three blue calibration strips intersect the ball ROI and can generate a
# compact dark/neutral component at exactly their calibrated x coordinate.
# A real 10 mm steel ball overlapping a strip has a substantially larger
# silhouette; reject only undersized components close to a marker.  Tracking
# fallbacks are also disabled there so a template cannot preserve the strip as
# a permanent false ball after the real ball has moved away.
BALL_MARKER_ARTIFACT_RADIUS_DIAMETERS = 1.60
BALL_MARKER_ARTIFACT_MIN_AREA_DIAMETER_SQ = 0.35
BALL_MARKER_BALL_SHAPE_SCORE_BONUS = 0.70
BALL_MARKER_BALL_MIN_DARK_FRACTION = 0.12
BALL_MARKER_BALL_MIN_ABS_CONTRAST = 12.0
# At a blocked physical end the steel silhouette can merge with a long dark
# tube/end-wall component.  The detector validates a ball-sized patch at the
# *outer edge* of that component, so these limits apply only to a component
# which touches the longitudinal ROI boundary; they are not global blob-size
# relaxations.
BALL_ENDPOINT_MERGED_MAX_WIDTH_DIAMETERS = 10.0
BALL_ENDPOINT_MERGED_MAX_AREA_DIAMETER_SQ = 12.0
# The old-side +0.85/+3.85 cm exclusions are invalid after moving the camera to
# the opposite side.  Keep global acquisition unmasked until a new endpoint-
# hidden/no-ball snapshot identifies fixed hardware in the current view.
BALL_ACQUIRE_FIXED_EXCLUSIONS_CM = ()
BALL_ACQUIRE_FIXED_EXCLUSION_RADIUS_CM = 0.70
# Both physical end openings are now blocked and the ball stays visible.  Do
# not constrain reacquisition to the last endpoint; every loss uses the same
# full visible corridor and the normal four-frame confirmation gate.
BALL_DARK_VALUE_MAX = 105
BALL_DARK_VALUE_MIN = 15
BALL_CORE_MASK_VALUE_MAX = 155
BALL_MIN_DARK_FRACTION = 0.03
BALL_MIN_LOCAL_CONTRAST = 3.0
BALL_TRACK_MIN_DARK_FRACTION = 0.05
# A fast polished ball can reflect the orange tube almost uniformly.  During
# established tracking, temporal prediction, axis proximity, and the compact
# component geometry already provide the discriminating gates; requiring a
# non-zero signed local contrast dropped the genuine 11 x 11 px ball for
# 0.1--0.8 s around the middle of the tube.  Keep the gray-variation check in
# _appearance_valid(), but allow either contrast polarity down to zero.
BALL_TRACK_MIN_LOCAL_CONTRAST = 0.0
BALL_LOW_CONTRAST_CUTOFF = 5.0
BALL_LOW_CONTRAST_MIN_DARK_FRACTION = 0.40
BALL_SPECULAR_MIN_DARK_FRACTION = 0.18
BALL_SPECULAR_MIN_GRAY_STD = 30.0

# Fixed-camera empty-tube reference assembled from the verified opposite-end
# snapshots: left half from the ball-at-right image and right half from the
# ball-at-left image.  Align it to the live orange axis before scoring.
BACKGROUND_IMAGE_PATH = os.path.join(
    os.path.dirname(__file__),
    "calibration",
    "fixed-camera-empty-tube.png",
)
BACKGROUND_AXIS_Y = 81.0
BACKGROUND_DIFF_THRESHOLD = 25
# A saved empty-tube frame is useful as a ranking hint, but its pixel values
# are installation-specific.  It must not veto an otherwise valid steel-ball
# candidate when lighting, camera exposure, or the competition background
# changes.  Keep the score bonus below and make all hard background fractions
# zero; geometry, axis proximity, appearance, temporal confirmation, and
# tracking continuity remain the acceptance gates.
BACKGROUND_ACQUIRE_MIN_FRACTION = 0.0
BACKGROUND_ENDPOINT_ACQUIRE_MIN_FRACTION = 0.0
BACKGROUND_TRACK_MIN_FRACTION = 0.0
BACKGROUND_TRACK_GLOBAL_MIN_FRACTION = 0.35
BACKGROUND_SCORE_GAIN = 0.0

VISION_BUILD = "2026-08-01-velocity-uart-v44-endpoint-false-reject"

# 最终坐标 Alpha-Beta 滤波。
POSITION_ALPHA = 0.50
POSITION_BETA = 0.08
POSITION_RESET_RESIDUAL_CM = 6.0

# UART2：B0 TX，B1 RX（MaixCAM2 背后引脚）。
ENABLE_UART = True
UART_DEVICE = "/dev/ttyS2"
UART_BAUD = 115200
UART_TX_PIN = "B0"
UART_RX_PIN = "B1"
UART_TX_FUNCTION = "UART2_TX"
UART_RX_FUNCTION = "UART2_RX"
UART_SEND_PERIOD_MS = 20

# 当前需要 MaixCAM2 本机屏幕持续显示实时画面；比赛/调试期间保持开启。
# 若未来为了最高视觉帧率关闭，必须明确告知操作者屏幕会停在最后一帧。
ENABLE_PREVIEW = True
PREVIEW_EVERY_N = 30
PREVIEW_JPEG_QUALITY = 35
DRAW_LEVEL = 2       # 0: 文字；1: 球；2: 再画标定点与轴线
PRINT_PERIOD_MS = 1000
DEBUG_SNAPSHOT_PREFIX = os.environ.get("BALL_DEBUG_SNAPSHOT", "")
DEBUG_LOSS_PREFIX = os.environ.get("BALL_DEBUG_LOSS", "")


# ============================================================
# 2. 通用工具
# ============================================================
def clamp(value, low, high):
    return low if value < low else high if value > high else value


def elapsed_ms(now, before):
    value = int(now) - int(before)
    return value if value >= 0 else 0


def affine_point(matrix, point):
    x = float(point[0])
    y = float(point[1])
    return np.array(
        (
            matrix[0, 0] * x + matrix[0, 1] * y + matrix[0, 2],
            matrix[1, 0] * x + matrix[1, 1] * y + matrix[1, 2],
        ),
        dtype=np.float32,
    )


def point_int(point):
    return int(round(float(point[0]))), int(round(float(point[1])))


def angle_difference_deg(a, b):
    value = float(a) - float(b)
    while value > 90.0:
        value -= 180.0
    while value < -90.0:
        value += 180.0
    return value


class AlphaBetaFilter:
    def __init__(self, alpha, beta):
        self.alpha = float(alpha)
        self.beta = float(beta)
        self.valid = False
        self.x = 0.0
        self.v = 0.0
        self.last_ms = 0

    def reset(self):
        self.valid = False
        self.x = 0.0
        self.v = 0.0
        self.last_ms = 0

    def update(self, measurement, now_ms):
        measurement = float(measurement)
        if not self.valid:
            self.valid = True
            self.x = measurement
            self.v = 0.0
            self.last_ms = int(now_ms)
            return self.x, self.v

        dt = clamp(elapsed_ms(now_ms, self.last_ms) / 1000.0, 0.005, 0.100)
        self.last_ms = int(now_ms)
        prediction = self.x + self.v * dt
        residual = measurement - prediction
        if abs(residual) > POSITION_RESET_RESIDUAL_CM:
            self.x = measurement
            self.v = 0.0
            return self.x, self.v
        self.x = prediction + self.alpha * residual
        self.v += self.beta * residual / dt
        self.v = clamp(self.v, -180.0, 180.0)
        return self.x, self.v


class RollingMedianFilter:
    """Reject short-lived alternate ball candidates before velocity estimation."""

    def __init__(self, window_size):
        self.window_size = int(window_size)
        self.samples = []

    def reset(self):
        self.samples = []

    def update(self, measurement):
        self.samples.append(float(measurement))
        if len(self.samples) > self.window_size:
            self.samples.pop(0)
        ordered = sorted(self.samples)
        middle = len(ordered) // 2
        if len(ordered) & 1:
            return ordered[middle]
        return 0.5 * (ordered[middle - 1] + ordered[middle])


# ============================================================
# 3. 塑料条标定点检测
# ============================================================
def component_axis_angle(xs, ys):
    """由连通域全部像素求主轴角度；返回弧度，方向规范到 x 正方向。"""
    x = xs.astype(np.float32)
    y = ys.astype(np.float32)
    x -= float(np.mean(x))
    y -= float(np.mean(y))

    cxx = float(np.mean(x * x))
    cyy = float(np.mean(y * y))
    cxy = float(np.mean(x * y))
    theta = 0.5 * math.atan2(2.0 * cxy, cxx - cyy)

    if math.cos(theta) < 0.0:
        theta += math.pi
    while theta > math.pi * 0.5:
        theta -= math.pi
    while theta < -math.pi * 0.5:
        theta += math.pi
    return theta


class PlasticMarkerTracker:
    def __init__(self):
        self.last_state = None
        self.marker_history = []
        self.lost_frames = 999
        self.just_lost = False
        self.just_reacquired = False
        self.debug_components = []
        self.debug_groups = []
        self.debug_selected = False
        self.fixed_axis_samples = []
        self.fixed_axis_y = None

    def reset(self):
        self.last_state = None
        self.marker_history = []
        self.lost_frames = 999
        self.just_lost = False
        self.just_reacquired = False
        self.debug_components = []
        self.debug_groups = []
        self.debug_selected = False
        self.fixed_axis_samples = []
        self.fixed_axis_y = None

    @staticmethod
    def _count_cost(count, single_count):
        """优先匹配上下两侧都可见的 2N 个横线，也兼容只见一侧的 N 个。"""
        double_count = single_count * 2
        if count == double_count:
            return 0.0
        if count == single_count:
            return 1.6
        return min(
            abs(count - double_count) * 1.30,
            1.8 + abs(count - single_count) * 1.40,
        )

    def _extract_groups(self, frame_bgr):
        height, width = frame_bgr.shape[:2]
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        blue = cv2.inRange(
            hsv,
            np.array((BLUE_H_MIN, BLUE_S_MIN, BLUE_V_MIN), dtype=np.uint8),
            np.array((BLUE_H_MAX, 255, 255), dtype=np.uint8),
        )

        blue[: int(height * MARKER_Y_MIN_RATIO), :] = 0
        blue[int(height * MARKER_Y_MAX_RATIO) :, :] = 0
        blue[:, : int(width * MARKER_X_MIN_RATIO)] = 0
        blue[:, int(width * MARKER_X_MAX_RATIO) :] = 0

        blue = cv2.morphologyEx(
            blue,
            cv2.MORPH_OPEN,
            cv2.getStructuringElement(cv2.MORPH_RECT, (2, 2)),
        )

        count, labels, stats, centers = cv2.connectedComponentsWithStats(blue, 8)
        components = []

        # 用双角平均角度，避免 180° 方向等价造成跳变。
        sum_cos2 = 0.0
        sum_sin2 = 0.0

        for index in range(1, count):
            x, y, w, h, area = [int(v) for v in stats[index]]
            cx = float(centers[index][0])
            cy = float(centers[index][1])

            if not (BLUE_AREA_MIN <= area <= BLUE_AREA_MAX):
                continue
            if not (BLUE_WIDTH_MIN <= w <= BLUE_WIDTH_MAX):
                continue
            if not (BLUE_HEIGHT_MIN <= h <= BLUE_HEIGHT_MAX):
                continue
            if max(w, h) / float(max(1, min(w, h))) < 1.18:
                continue

            local = labels[y : y + h, x : x + w]
            ys, xs = np.where(local == index)
            if xs.size < 8:
                continue
            xs = xs.astype(np.float32) + float(x)
            ys = ys.astype(np.float32) + float(y)

            theta = component_axis_angle(xs, ys)
            angle_deg = math.degrees(theta)
            if abs(angle_deg) > BLUE_MAX_ANGLE_DEG:
                continue

            component = {
                "center": np.array((cx, cy), dtype=np.float32),
                "area": float(area),
                "theta": float(theta),
                "bbox": (x, y, w, h),
            }
            components.append(component)
            sum_cos2 += float(area) * math.cos(2.0 * theta)
            sum_sin2 += float(area) * math.sin(2.0 * theta)

        self.debug_components = [
            (item["bbox"], round(math.degrees(item["theta"]), 1))
            for item in components
        ]
        if len(components) < 3:
            self.debug_groups = []
            return None

        rough_theta = 0.5 * math.atan2(sum_sin2, sum_cos2)
        rough_e = np.array(
            (math.cos(rough_theta), math.sin(rough_theta)), dtype=np.float32
        )
        rough_n = np.array((-rough_e[1], rough_e[0]), dtype=np.float32)

        for component in components:
            component["s"] = float(np.dot(component["center"], rough_e))
            component["t"] = float(np.dot(component["center"], rough_n))
        components.sort(key=lambda item: item["s"])

        cluster_gap = width * MARKER_CLUSTER_GAP_RATIO
        raw_groups = []
        for component in components:
            if not raw_groups:
                raw_groups.append([component])
                continue
            current_center = float(
                np.median([item["s"] for item in raw_groups[-1]])
            )
            if component["s"] - current_center > cluster_gap:
                raw_groups.append([component])
            else:
                raw_groups[-1].append(component)

        groups = []
        for items in raw_groups:
            s_values = np.array([item["s"] for item in items], dtype=np.float32)
            t_values = np.array([item["t"] for item in items], dtype=np.float32)
            groups.append(
                {
                    "components": items,
                    "count": len(items),
                    "s": float(np.median(s_values)),
                    "s_span": float(np.max(s_values) - np.min(s_values)),
                    "t_span": float(np.max(t_values) - np.min(t_values))
                    if len(items) > 1
                    else 0.0,
                }
            )

        self.debug_groups = [
            (item["count"], round(item["s"], 1), round(item["t_span"], 1))
            for item in groups
        ]
        return blue, groups

    def _select_three_groups(self, groups, frame_width, frame_height):
        best = None
        previous = self.last_state

        for i in range(len(groups)):
            for j in range(i + 1, len(groups)):
                for k in range(j + 1, len(groups)):
                    plus = groups[i]
                    zero = groups[j]
                    minus = groups[k]
                    s_plus = float(plus["s"])
                    s_zero = float(zero["s"])
                    s_minus = float(minus["s"])
                    span = s_minus - s_plus

                    if not (
                        frame_width * MARKER_SPAN_RATIO_MIN
                        <= span
                        <= frame_width * MARKER_SPAN_RATIO_MAX
                    ):
                        continue

                    zero_ratio = (s_zero - s_plus) / span
                    if not (
                        MARKER_ZERO_RATIO_MIN
                        <= zero_ratio
                        <= MARKER_ZERO_RATIO_MAX
                    ):
                        continue

                    score = 0.0
                    score += self._count_cost(
                        plus["count"], MARKER_LEFT_SINGLE_COUNT
                    )
                    score += self._count_cost(
                        zero["count"], MARKER_ZERO_SINGLE_COUNT
                    )
                    score += self._count_cost(
                        minus["count"], MARKER_RIGHT_SINGLE_COUNT
                    )
                    score += abs(zero_ratio - 0.5) * 5.0
                    score += abs(span / frame_width - 0.66) * 4.0

                    # 上下两侧都可见时，真塑料条应在法向有较大跨度。
                    for group, expected_double in (
                        (plus, MARKER_LEFT_SINGLE_COUNT * 2),
                        (zero, MARKER_ZERO_SINGLE_COUNT * 2),
                        (minus, MARKER_RIGHT_SINGLE_COUNT * 2),
                    ):
                        if (
                            group["count"] >= expected_double - 1
                            and group["t_span"] < frame_height * 0.12
                        ):
                            score += 1.5

                    if previous is not None:
                        old_span = previous["marker_x"][2] - previous["marker_x"][0]
                        if old_span > 1.0:
                            score += abs(span / old_span - 1.0) * 8.0

                    if best is None or score < best[0]:
                        best = (score, (plus, zero, minus))

        return None if best is None else best[1]

    @staticmethod
    def _orientation_from_selected(groups):
        sum_cos2 = 0.0
        sum_sin2 = 0.0
        total_weight = 0.0
        for group in groups:
            for component in group["components"]:
                weight = float(component["area"])
                theta = float(component["theta"])
                sum_cos2 += weight * math.cos(2.0 * theta)
                sum_sin2 += weight * math.sin(2.0 * theta)
                total_weight += weight
        if total_weight <= 0.0:
            return None
        return 0.5 * math.atan2(sum_sin2, sum_cos2)

    @staticmethod
    def _transform_component_centers(group, matrix):
        points = []
        weights = []
        for component in group["components"]:
            points.append(affine_point(matrix, component["center"]))
            weights.append(float(component["area"]))
        return np.asarray(points, dtype=np.float32), np.asarray(weights, dtype=np.float32)

    @staticmethod
    def _fit_strip_center_x(points, weights, axis_y):
        """拟合塑料条中心线 x=a*y+b，并在钢球轴线 y=axis_y 处取交点。"""
        if points.shape[0] <= 1:
            return float(points[0, 0])

        ys = points[:, 1].astype(np.float64)
        xs = points[:, 0].astype(np.float64)
        ws = np.maximum(weights.astype(np.float64), 1.0)
        sum_w = float(np.sum(ws))
        mean_y = float(np.sum(ws * ys) / sum_w)
        mean_x = float(np.sum(ws * xs) / sum_w)
        dy = ys - mean_y
        denominator = float(np.sum(ws * dy * dy))

        if denominator < 15.0:
            return float(np.median(xs))

        slope = float(np.sum(ws * dy * (xs - mean_x)) / denominator)
        slope = clamp(slope, -0.40, 0.40)
        return mean_x + slope * (float(axis_y) - mean_y)

    @staticmethod
    def _find_tube_axis(rotated, approximate_x, previous_axis=None):
        """Fit the orange tube centre as y = axis_y + slope*(x-width/2)."""
        height, width = rotated.shape[:2]
        x_left = max(0, int(round(approximate_x[0] - 12.0)))
        x_right = min(width, int(round(approximate_x[2] + 12.0)))
        if x_right - x_left < width * 0.45:
            return None

        if previous_axis is None:
            y_top = int(height * TUBE_SEARCH_Y_MIN_RATIO)
            y_bottom = int(height * TUBE_SEARCH_Y_MAX_RATIO)
        else:
            previous_y = (float(previous_axis[0]) if isinstance(previous_axis, tuple)
                          else float(previous_axis))
            y_top = max(0, int(round(previous_y - 35.0)))
            y_bottom = min(height, int(round(previous_y + 35.0)))

        crop = rotated[y_top:y_bottom, x_left:x_right]
        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        orange = cv2.inRange(
            hsv,
            np.array((ORANGE_H_MIN, ORANGE_S_MIN, ORANGE_V_MIN), dtype=np.uint8),
            np.array((ORANGE_H_MAX, 255, 255), dtype=np.uint8),
        )
        column_count = np.sum(orange > 0, axis=0)
        valid_columns = np.where(column_count >= 4)[0]
        if valid_columns.size < (x_right - x_left) * TUBE_MIN_ROW_COVERAGE:
            return None

        column_y = []
        column_x = []
        for local_x in valid_columns:
            ys = np.where(orange[:, int(local_x)] > 0)[0]
            if ys.size >= 4:
                split_at = np.where(np.diff(ys) > 2)[0] + 1
                runs = np.split(ys, split_at)
                longest = max(runs, key=lambda item: int(item.size))
                if longest.size < 4:
                    continue
                column_x.append(float(x_left + int(local_x)))
                column_y.append(
                    float(y_top) +
                    0.5 * (float(longest[0]) + float(longest[-1]))
                )
        if len(column_x) < int((x_right - x_left) * TUBE_MIN_ROW_COVERAGE):
            return None

        xs = np.asarray(column_x, dtype=np.float64)
        ys = np.asarray(column_y, dtype=np.float64)

        def fit_line(fit_x, fit_y):
            mean_x = float(np.mean(fit_x))
            mean_y = float(np.mean(fit_y))
            dx = fit_x - mean_x
            denominator = float(np.sum(dx * dx))
            if denominator < 20.0:
                return 0.0, mean_y
            fitted_slope = float(np.sum(dx * (fit_y - mean_y)) / denominator)
            fitted_slope = clamp(fitted_slope, -0.12, 0.12)
            fitted_axis = mean_y + fitted_slope * (width * 0.5 - mean_x)
            return fitted_slope, fitted_axis

        slope, axis_y = fit_line(xs, ys)
        residual = ys - (axis_y + slope * (xs - width * 0.5))
        keep = np.abs(residual - float(np.median(residual))) <= 5.0
        if int(np.sum(keep)) >= max(20, int(xs.size * 0.55)):
            slope, axis_y = fit_line(xs[keep], ys[keep])

        # The physical tilt moves the ball only a few pixels vertically across
        # the calibrated span.  A fixed camera-frame axis plus the detector's
        # one-diameter tolerance is more stable than the illumination-biased
        # orange slope estimate.
        return float(axis_y), 0.0

    @staticmethod
    def _validate_marker_x(marker_x, frame_width, previous):
        plus_x, zero_x, minus_x = [float(v) for v in marker_x]
        span = minus_x - plus_x
        if not (
            frame_width * MARKER_SPAN_RATIO_MIN
            <= span
            <= frame_width * MARKER_SPAN_RATIO_MAX
        ):
            return False
        ratio = (zero_x - plus_x) / span
        if not (MARKER_ZERO_RATIO_MIN <= ratio <= MARKER_ZERO_RATIO_MAX):
            return False

        if previous is not None:
            old = previous["marker_x"]
            old_span = float(old[2] - old[0])
            if old_span > 1.0 and abs(span / old_span - 1.0) > MARKER_SPAN_CHANGE_MAX:
                return False
            for current, old_value in zip(marker_x, old):
                if abs(float(current) - float(old_value)) > MARKER_MAX_JUMP_PX:
                    return False
        return True

    def update(self, frame_bgr):
        self.just_lost = False
        self.just_reacquired = False
        had_reference = self.last_state is not None

        if FIXED_MARKER_X is not None:
            previous_axis = (
                (self.last_state["axis_y"],
                 self.last_state.get("axis_slope", 0.0))
                if self.last_state is not None else None
            )
            tube_axis = self._find_tube_axis(
                frame_bgr,
                FIXED_MARKER_X,
                previous_axis,
            )
            if tube_axis is not None:
                axis_y, axis_slope = tube_axis
                # The camera is rigidly mounted directly above the tube pivot.
                # Tube tilt cannot move the centre projection by tens of pixels;
                # such jumps are illumination selecting another orange row.
                # Lock a robust startup median and keep that camera-frame axis.
                if self.fixed_axis_y is None:
                    self.fixed_axis_samples.append(float(axis_y))
                    if len(self.fixed_axis_samples) > 7:
                        self.fixed_axis_samples.pop(0)
                    axis_y = float(np.median(self.fixed_axis_samples))
                    if len(self.fixed_axis_samples) >= 7:
                        self.fixed_axis_y = axis_y
                else:
                    axis_y = self.fixed_axis_y
                axis_slope = 0.0
                matrix = cv2.getRotationMatrix2D(
                    (frame_bgr.shape[1] * 0.5, frame_bgr.shape[0] * 0.5),
                    0.0,
                    1.0,
                )
                state = {
                    "rotated": frame_bgr,
                    "matrix": matrix,
                    "inverse": cv2.invertAffineTransform(matrix),
                    "marker_x": tuple(float(v) for v in FIXED_MARKER_X),
                    "axis_y": float(axis_y),
                    "axis_slope": float(axis_slope),
                    "angle_deg": 0.0,
                    "mode": "FIXED",
                    "blue_mask": None,
                    "groups": [],
                }
                self.last_state = state
                self.lost_frames = 0
                self.debug_selected = True
                self.debug_components = []
                self.debug_groups = []
                if not had_reference:
                    self.just_reacquired = True
                return state

            self.lost_frames += 1
            if self.last_state is not None and self.lost_frames <= MARKER_HOLD_FRAMES:
                held = dict(self.last_state)
                held["rotated"] = frame_bgr
                held["mode"] = "FIXED-HOLD"
                return held
            if self.last_state is not None:
                self.last_state = None
                self.just_lost = True
            return None

        extracted = self._extract_groups(frame_bgr)
        if extracted is not None:
            blue_mask, groups = extracted
            selected = self._select_three_groups(
                groups,
                frame_bgr.shape[1],
                frame_bgr.shape[0],
            )
        else:
            blue_mask = None
            selected = None
        self.debug_selected = selected is not None

        if selected is not None:
            theta = self._orientation_from_selected(selected)
            if theta is not None:
                # Calibration strips and camera are fixed.  Their stripe angle
                # must not be mistaken for the live tube angle.
                angle_deg = 0.0
                if (
                    self.last_state is None
                    or abs(angle_difference_deg(angle_deg, self.last_state["angle_deg"]))
                    <= 8.0
                ):
                    height, width = frame_bgr.shape[:2]
                    matrix = cv2.getRotationMatrix2D(
                        (width * 0.5, height * 0.5),
                        angle_deg,
                        1.0,
                    )
                    # angle_deg is deliberately fixed at zero: calibration
                    # stripe orientation is not the live tube angle.  Avoid a
                    # full-frame identity warp on every geometry refresh.
                    rotated = frame_bgr

                    transformed = []
                    approximate_x = []
                    for group in selected:
                        points, weights = self._transform_component_centers(group, matrix)
                        transformed.append((points, weights))
                        approximate_x.append(float(np.median(points[:, 0])))

                    previous_axis = (
                        (self.last_state["axis_y"],
                         self.last_state.get("axis_slope", 0.0))
                        if self.last_state is not None else None
                    )
                    tube_axis = self._find_tube_axis(
                        rotated,
                        approximate_x,
                        previous_axis,
                    )
                    if tube_axis is not None:
                        axis_y, axis_slope = tube_axis
                        marker_x = []
                        for approximate, (points, weights) in zip(
                                approximate_x, transformed):
                            local_axis_y = (
                                axis_y + axis_slope * (approximate - width * 0.5)
                            )
                            marker_x.append(
                                self._fit_strip_center_x(
                                    points,
                                    weights,
                                    local_axis_y,
                                )
                            )

                        if self._validate_marker_x(
                            marker_x,
                            width,
                            self.last_state,
                        ):
                            self.marker_history.append(
                                tuple(float(v) for v in marker_x)
                            )
                            if len(self.marker_history) > MARKER_LOCK_SAMPLES:
                                self.marker_history.pop(0)
                            robust_marker_x = tuple(
                                float(np.median([
                                    sample[index]
                                    for sample in self.marker_history
                                ]))
                                for index in range(3)
                            )
                            if (self.last_state is None and
                                    len(self.marker_history) < MARKER_LOCK_SAMPLES):
                                self.lost_frames += 1
                                return None
                            if self.last_state is not None:
                                old_marker_x = self.last_state["marker_x"]
                                robust_marker_x = tuple(
                                    (1.0 - MARKER_GEOMETRY_ALPHA) * old_value +
                                    MARKER_GEOMETRY_ALPHA * new_value
                                    for old_value, new_value in zip(
                                        old_marker_x, robust_marker_x
                                    )
                                )
                            state = {
                                "rotated": rotated,
                                "matrix": matrix,
                                "inverse": cv2.invertAffineTransform(matrix),
                                "marker_x": robust_marker_x,
                                "axis_y": float(axis_y),
                                "axis_slope": float(axis_slope),
                                "angle_deg": float(angle_deg),
                                "mode": "PLASTIC",
                                "blue_mask": blue_mask,
                                "groups": selected,
                            }
                            self.last_state = state
                            self.lost_frames = 0
                            if not had_reference:
                                self.just_reacquired = True
                            return state

        self.lost_frames += 1
        if self.last_state is not None and self.lost_frames <= MARKER_HOLD_FRAMES:
            # 使用上一帧角度重新旋转当前原图，避免直接复用上一帧图像。
            height, width = frame_bgr.shape[:2]
            matrix = cv2.getRotationMatrix2D(
                (width * 0.5, height * 0.5),
                self.last_state["angle_deg"],
                1.0,
            )
            rotated = frame_bgr
            held = dict(self.last_state)
            held["rotated"] = rotated
            held["matrix"] = matrix
            held["inverse"] = cv2.invertAffineTransform(matrix)
            held["mode"] = "HOLD"
            return held

        # HOLD 用完后彻底丢弃旧参考。下一帧直接走与程序启动时完全相同的
        # 全局搜索，不增加二次确认，也不限制相邻两帧的角度/轴线/标记跳变。
        if self.last_state is not None:
            self.last_state = None
            self.lost_frames = 999
            self.just_lost = True

        return None

    def reuse_verified_geometry(self, frame_bgr):
        """Apply the last verified three-point calibration to the current frame."""
        if self.last_state is None:
            return None
        held = dict(self.last_state)
        held["rotated"] = frame_bgr
        self.last_state = held
        return held


# ============================================================
# 4. 钢球检测
# ============================================================
class BallDetector:
    def __init__(self):
        self.last_x = None
        self.last_y = None
        # Pixel velocity in px/s.  Earlier builds stored px/frame and therefore
        # rejected the same physical speed whenever preview reduced the FPS.
        self.motion_x = 0.0
        self.last_accept_ms = None
        self.current_ms = 0
        self.lost_frames = 999
        self.pending_x = None
        self.pending_y = None
        self.pending_count = 0
        self.fallback_streak = 0
        self.local_streak = 0
        self.last_gray = None
        self.ball_template = None
        self.template_streak = 0
        self.reacquire_side = 0
        self.debug_raw = []
        self.debug_appearance = []
        self.debug_candidates = []
        self.debug_hough = []
        self.debug_dark = []
        # 暗光下采集的空管参考不能与补光画面作差，否则整幅图都会被
        # 视为“变化”并给固定结构加分。亮灯版本依靠固定相机的窄轴线、
        # 球尺寸/形状和连续四帧确认；待取得亮灯空管拼接图后才能重启背景差分。
        self.background = (None if FILL_LIGHT_ENABLED else
                           cv2.imread(BACKGROUND_IMAGE_PATH))
        if self.background is None:
            print("BACKGROUND unavailable:", BACKGROUND_IMAGE_PATH)
        elif self.background.shape[:2] != (FRAME_H, FRAME_W):
            print("BACKGROUND size mismatch:", self.background.shape)
            self.background = None

    def reset(self):
        self.last_x = None
        self.last_y = None
        self.motion_x = 0.0
        self.last_accept_ms = None
        self.current_ms = 0
        self.lost_frames = 999
        self.fallback_streak = 0
        self.local_streak = 0
        self.last_gray = None
        self.ball_template = None
        self.template_streak = 0
        self.reacquire_side = 0
        self._clear_pending()
        self.debug_raw = []
        self.debug_appearance = []
        self.debug_candidates = []
        self.debug_hough = []
        self.debug_dark = []

    def _tracking_window(self, marker_span):
        """Return a wall-clock motion prediction and bounded residual gate."""
        if self.last_accept_ms is None:
            return float(self.last_x), BALL_PRIMARY_MAX_JUMP_PX
        dt_s = clamp(
            elapsed_ms(self.current_ms, self.last_accept_ms) / 1000.0,
            0.005,
            BALL_TRACK_MAX_DT_S,
        )
        px_per_mm = max(0.5, float(marker_span) / 200.0)
        max_speed_px_s = BALL_TRACK_MAX_SPEED_MM_S * px_per_mm
        velocity_px_s = clamp(
            self.motion_x,
            -max_speed_px_s,
            max_speed_px_s,
        )
        predicted_x = (
            float(self.last_x) +
            velocity_px_s * dt_s * BALL_TRACK_MOTION_GAIN
        )
        motion_extra = min(
            BALL_TRACK_MAX_EXTRA_JUMP_PX,
            abs(velocity_px_s) * dt_s * 0.35,
        )
        jump_limit = (
            BALL_PRIMARY_MAX_JUMP_PX
            + BALL_TRACK_JUMP_GROWTH_PX * float(self.lost_frames)
            + motion_extra
        )
        return predicted_x, jump_limit

    def _background_change(self, rotated, x, y, axis_y, expected_diameter):
        """Return (changed-pixel fraction, mean absolute delta) near a candidate."""
        if self.background is None:
            return 1.0, 0.0
        radius = max(5, int(round(expected_diameter * 0.75)))
        cx = int(round(float(x)))
        cy = int(round(float(y)))
        reference_cy = int(round(float(y) - (float(axis_y) - BACKGROUND_AXIS_Y)))
        x0 = max(0, cx - radius)
        x1 = min(rotated.shape[1], cx + radius + 1)
        y0 = max(0, cy - radius)
        y1 = min(rotated.shape[0], cy + radius + 1)
        by0 = reference_cy - (cy - y0)
        by1 = reference_cy + (y1 - cy)
        if (x1 <= x0 or y1 <= y0 or by0 < 0 or
                by1 > self.background.shape[0]):
            return 0.0, 0.0
        live_patch = rotated[y0:y1, x0:x1]
        background_patch = self.background[by0:by1, x0:x1]
        if live_patch.shape != background_patch.shape:
            return 0.0, 0.0
        delta = cv2.cvtColor(
            cv2.absdiff(live_patch, background_patch),
            cv2.COLOR_BGR2GRAY,
        )
        changed_fraction = float(np.mean(delta > BACKGROUND_DIFF_THRESHOLD))
        return changed_fraction, float(np.mean(delta))

    def _clear_pending(self):
        self.pending_x = None
        self.pending_y = None
        self.pending_count = 0

    @staticmethod
    def _candidate_gray_std(gray, cx, cy, radius):
        x1 = max(0, int(round(cx - radius)))
        x2 = min(gray.shape[1], int(round(cx + radius + 1)))
        y1 = max(0, int(round(cy - radius)))
        y2 = min(gray.shape[0], int(round(cy + radius + 1)))
        patch = gray[y1:y2, x1:x2]
        return float(np.std(patch)) if patch.size > 0 else 0.0

    @staticmethod
    def _candidate_appearance(rotated, cx, cy, diameter):
        """Measure the dark steel core against the surrounding orange tube."""
        radius = max(5, int(round(diameter * 0.90)))
        x1 = max(0, int(round(cx)) - radius)
        x2 = min(rotated.shape[1], int(round(cx)) + radius + 1)
        y1 = max(0, int(round(cy)) - radius)
        y2 = min(rotated.shape[0], int(round(cy)) + radius + 1)
        patch = rotated[y1:y2, x1:x2]
        if patch.shape[0] < 5 or patch.shape[1] < 5:
            return 0.0, 0.0

        gray = cv2.cvtColor(patch, cv2.COLOR_BGR2GRAY)
        value = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV)[:, :, 2]
        yy, xx = np.ogrid[y1:y2, x1:x2]
        distance_sq = (xx - float(cx)) ** 2 + (yy - float(cy)) ** 2
        inner_radius = max(2.0, diameter * 0.42)
        ring_inner = max(inner_radius + 1.0, diameter * 0.58)
        ring_outer = max(ring_inner + 1.0, diameter * 0.90)
        inner = distance_sq <= inner_radius * inner_radius
        ring = ((distance_sq >= ring_inner * ring_inner) &
                (distance_sq <= ring_outer * ring_outer))
        if not np.any(inner) or not np.any(ring):
            return 0.0, 0.0

        contrast = float(np.mean(gray[ring]) - np.mean(gray[inner]))
        dark_fraction = float(np.mean(value[inner] <= BALL_DARK_VALUE_MAX))
        return contrast, dark_fraction

    @staticmethod
    def _appearance_valid(contrast, dark_fraction, gray_std, tracking):
        minimum_contrast = (BALL_TRACK_MIN_LOCAL_CONTRAST if tracking
                            else BALL_MIN_LOCAL_CONTRAST)
        minimum_dark_fraction = (BALL_TRACK_MIN_DARK_FRACTION if tracking
                                 else BALL_MIN_DARK_FRACTION)
        if tracking:
            # Once tracking is established, the polished steel ball may flip
            # between a dark core and a bright specular core while moving.
            # Continuity and the tracking window already reject distant fixed
            # reflections, so accept either contrast polarity here.  Cold
            # acquisition below remains directional and strictly gated.
            conventional = (
                gray_std >= 8.0 and
                (abs(contrast) >= minimum_contrast or
                 dark_fraction >= minimum_dark_fraction)
            )
        else:
            conventional = (
                contrast >= minimum_contrast and
                dark_fraction >= minimum_dark_fraction and
                (contrast >= BALL_LOW_CONTRAST_CUTOFF or
                 dark_fraction >= BALL_LOW_CONTRAST_MIN_DARK_FRACTION)
            )
        specular = (
            dark_fraction >= BALL_SPECULAR_MIN_DARK_FRACTION and
            gray_std >= BALL_SPECULAR_MIN_GRAY_STD
        )
        return conventional or specular

    def _hough_fallback(
        self,
        rotated,
        x_left,
        x_right,
        axis_y,
        axis_slope,
        expected_diameter,
    ):
        """仅在普通连通域检测失败时运行，ROI 很小，不影响正常跟踪速度。"""
        tracking = self.last_x is not None and self.lost_frames <= BALL_HOLD_FRAMES
        height = rotated.shape[0]
        span = expected_diameter / BALL_DIAMETER_TO_MARKER_SPAN
        axis_origin_x = rotated.shape[1] * 0.5
        endpoint_axes = (
            axis_y + axis_slope * (x_left - axis_origin_x),
            axis_y + axis_slope * (x_right - axis_origin_x),
        )
        ball_axis_offset = span * BALL_CENTER_AXIS_OFFSET_RATIO
        y_half = max(10.0, expected_diameter * 1.40)
        y_top = max(0, int(round(min(endpoint_axes) + ball_axis_offset - y_half)))
        y_bottom = min(
            height,
            int(round(max(endpoint_axes) + ball_axis_offset + y_half + 1.0)),
        )
        crop = rotated[y_top:y_bottom, x_left:x_right]
        if crop.shape[0] < 15 or crop.shape[1] < 40:
            return None

        gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 1.0)
        circles = cv2.HoughCircles(
            blurred,
            cv2.HOUGH_GRADIENT,
            dp=1.0,
            minDist=max(12.0, expected_diameter * 1.4),
            param1=60,
            param2=9,
            minRadius=max(3, int(expected_diameter * 0.32)),
            maxRadius=max(5, int(expected_diameter * 0.85)),
        )
        if circles is None:
            return None

        best = None
        for local_x, local_y, radius in circles[0]:
            global_x = x_left + float(local_x)
            global_y = y_top + float(local_y)
            ball_axis_y = (
                axis_y + axis_slope * (global_x - axis_origin_x) +
                ball_axis_offset
            )
            if (abs(global_y - ball_axis_y) >
                    BALL_CENTER_Y_TOLERANCE_DIAMETER * expected_diameter):
                continue
            contrast, dark_fraction = self._candidate_appearance(
                rotated, global_x, global_y, expected_diameter
            )
            patch_radius = max(4, int(round(expected_diameter * 0.65)))
            xa = max(0, int(round(local_x - patch_radius)))
            xb = min(gray.shape[1], int(round(local_x + patch_radius + 1)))
            ya = max(0, int(round(local_y - patch_radius)))
            yb = min(gray.shape[0], int(round(local_y + patch_radius + 1)))
            gray_std = float(np.std(gray[ya:yb, xa:xb]))
            self.debug_hough.append(
                (round(global_x, 1), round(global_y, 1), round(float(radius), 1),
                 round(contrast, 1), round(dark_fraction, 2))
            )
            if not self._appearance_valid(
                    contrast, dark_fraction, gray_std, tracking):
                continue
            radius_score = math.exp(
                -abs(2.0 * float(radius) - expected_diameter)
                / (0.50 * expected_diameter)
            )
            y_score = math.exp(
                -abs(global_y - ball_axis_y) / (0.55 * expected_diameter)
            )

            score = (2.6 * radius_score + 2.0 * y_score +
                     0.020 * gray_std + 0.020 * abs(contrast) +
                     1.0 * dark_fraction)
            if tracking:
                score += 1.20 * math.exp(
                    -abs(global_x - self.last_x) / (2.0 * expected_diameter)
                )

            candidate = {
                "score": float(score),
                "x": float(global_x),
                "y": float(global_y),
                "bbox": (
                    int(round(global_x - radius)),
                    int(round(global_y - radius)),
                    int(round(radius * 2.0)),
                    int(round(radius * 2.0)),
                ),
                "area": 0,
                "gray_std": gray_std,
                "contrast": float(contrast),
                "dark_fraction": float(dark_fraction),
                "fill": 0.0,
                "diameter": float(expected_diameter),
                "fallback": True,
            }
            if best is None or candidate["score"] > best["score"]:
                best = candidate

        if best is None or best["score"] < BALL_HOUGH_FALLBACK_SCORE:
            return None
        return best

    def _local_specular_track(
        self,
        rotated,
        x_left,
        x_right,
        axis_y,
        axis_slope,
        span,
        expected_diameter,
    ):
        """Recover a previously locked shiny ball without global acquisition."""
        if self.last_x is None:
            return None
        edge_guard = span * 0.20
        if not (x_left + edge_guard <= self.last_x <= x_right - edge_guard):
            return None
        predicted_x, _ = self._tracking_window(span)
        search_left = max(
            x_left,
            int(round(predicted_x - BALL_LOCAL_SEARCH_HALF_WIDTH_PX)),
        )
        search_right = min(
            x_right,
            int(round(predicted_x + BALL_LOCAL_SEARCH_HALF_WIDTH_PX)),
        )
        if search_right <= search_left:
            return None

        gray = cv2.cvtColor(rotated, cv2.COLOR_BGR2GRAY)
        axis_origin_x = rotated.shape[1] * 0.5
        axis_offset = span * BALL_CENTER_AXIS_OFFSET_RATIO
        patch_radius = max(4.0, expected_diameter * 0.65)
        best = None
        for sample_x in range(search_left, search_right + 1, 2):
            local_axis_y = (
                axis_y + axis_slope * (sample_x - axis_origin_x) + axis_offset
            )
            for dy in (-6.0, -4.0, -2.0, 0.0, 2.0, 4.0, 6.0):
                sample_y = local_axis_y + dy
                if (abs(sample_y - local_axis_y) >
                        BALL_CENTER_Y_TOLERANCE_DIAMETER * expected_diameter):
                    continue
                contrast, dark_fraction = self._candidate_appearance(
                    rotated,
                    float(sample_x),
                    float(sample_y),
                    expected_diameter,
                )
                gray_std = self._candidate_gray_std(
                    gray,
                    float(sample_x),
                    float(sample_y),
                    patch_radius,
                )
                if not self._appearance_valid(
                        contrast, dark_fraction, gray_std, True):
                    continue
                score = (
                    3.05 +
                    0.020 * (gray_std - 8.0) +
                    0.45 * dark_fraction +
                    0.25 * math.exp(-abs(dy) / 3.0) +
                    0.35 * math.exp(-abs(sample_x - predicted_x) / 8.0)
                )
                candidate = {
                    "score": float(score),
                    "x": float(sample_x),
                    "y": float(sample_y),
                    "bbox": (
                        int(round(sample_x - expected_diameter * 0.5)),
                        int(round(sample_y - expected_diameter * 0.5)),
                        int(round(expected_diameter)),
                        int(round(expected_diameter)),
                    ),
                    "area": 0,
                    "gray_std": float(gray_std),
                    "contrast": float(contrast),
                    "dark_fraction": float(dark_fraction),
                    "fill": 0.0,
                    "diameter": float(expected_diameter),
                    "source": "local_specular",
                }
                if best is None or candidate["score"] > best["score"]:
                    best = candidate
        return best

    def _optical_flow_track(
        self,
        rotated,
        axis_y,
        axis_slope,
        span,
        expected_diameter,
    ):
        if self.last_gray is None or self.last_x is None or self.last_y is None:
            return None
        current_gray = cv2.cvtColor(rotated, cv2.COLOR_BGR2GRAY)
        previous_point = np.array(
            [[[float(self.last_x), float(self.last_y)]]],
            dtype=np.float32,
        )
        try:
            next_points, status, errors = cv2.calcOpticalFlowPyrLK(
                self.last_gray,
                current_gray,
                previous_point,
                None,
                winSize=(21, 21),
                maxLevel=2,
                criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 20, 0.03),
            )
        except Exception:
            return None
        if next_points is None or status is None or int(status.reshape(-1)[0]) == 0:
            return None
        error = float(errors.reshape(-1)[0]) if errors is not None else 0.0
        if error > 35.0:
            return None
        tracked_x = float(next_points.reshape(-1, 2)[0][0])
        tracked_y = float(next_points.reshape(-1, 2)[0][1])
        predicted_x, tracking_jump_limit = self._tracking_window(span)
        if (abs(tracked_x - predicted_x) >
                max(BALL_OPTICAL_FLOW_MAX_JUMP_PX, tracking_jump_limit)):
            return None
        axis_origin_x = rotated.shape[1] * 0.5
        local_axis_y = (
            axis_y + axis_slope * (tracked_x - axis_origin_x) +
            span * BALL_CENTER_AXIS_OFFSET_RATIO
        )
        if (abs(tracked_y - local_axis_y) >
                BALL_CENTER_Y_TOLERANCE_DIAMETER * expected_diameter):
            return None
        contrast, dark_fraction = self._candidate_appearance(
            rotated, tracked_x, tracked_y, expected_diameter
        )
        gray_std = self._candidate_gray_std(
            current_gray,
            tracked_x,
            tracked_y,
            max(4.0, expected_diameter * 0.65),
        )
        if not self._appearance_valid(
                contrast, dark_fraction, gray_std, True):
            return None
        return {
            "score": 4.0,
            "x": tracked_x,
            "y": tracked_y,
            "bbox": (
                int(round(tracked_x - expected_diameter * 0.5)),
                int(round(tracked_y - expected_diameter * 0.5)),
                int(round(expected_diameter)),
                int(round(expected_diameter)),
            ),
            "area": 0,
            "gray_std": float(gray_std),
            "contrast": float(contrast),
            "dark_fraction": float(dark_fraction),
            "fill": 0.0,
            "diameter": float(expected_diameter),
            "source": "optical_flow",
        }

    def _template_track(
        self,
        rotated,
        x_left,
        x_right,
        axis_y,
        axis_slope,
        span,
        expected_diameter,
    ):
        """Track only near a previously confirmed ball using its image patch."""
        if (self.ball_template is None or self.last_x is None or
                self.last_y is None or
                self.template_streak >= BALL_MAX_CONSECUTIVE_TEMPLATE_FRAMES):
            return None
        edge_guard = span * 0.20
        if not (x_left + edge_guard <= self.last_x <= x_right - edge_guard):
            return None

        gray = cv2.cvtColor(rotated, cv2.COLOR_BGR2GRAY)
        template_height, template_width = self.ball_template.shape[:2]
        half_width = template_width // 2
        half_height = template_height // 2
        predicted_x, _ = self._tracking_window(span)
        search_center_y = float(self.last_y)
        search_left = max(
            0,
            int(round(predicted_x - BALL_LOCAL_SEARCH_HALF_WIDTH_PX)) - half_width,
        )
        search_right = min(
            gray.shape[1],
            int(round(predicted_x + BALL_LOCAL_SEARCH_HALF_WIDTH_PX)) +
            half_width + 1,
        )
        search_top = max(0, int(round(search_center_y - 8.0)) - half_height)
        search_bottom = min(
            gray.shape[0],
            int(round(search_center_y + 8.0)) + half_height + 1,
        )
        search = gray[search_top:search_bottom, search_left:search_right]
        if (search.shape[0] < template_height or
                search.shape[1] < template_width):
            return None
        try:
            response = cv2.matchTemplate(
                search,
                self.ball_template,
                cv2.TM_CCOEFF_NORMED,
            )
            _, correlation, _, location = cv2.minMaxLoc(response)
        except Exception:
            return None
        if not math.isfinite(float(correlation)) or (
                float(correlation) < BALL_TEMPLATE_MIN_CORRELATION):
            return None

        tracked_x = float(search_left + location[0] + half_width)
        tracked_y = float(search_top + location[1] + half_height)
        axis_origin_x = rotated.shape[1] * 0.5
        local_axis_y = (
            axis_y + axis_slope * (tracked_x - axis_origin_x) +
            span * BALL_CENTER_AXIS_OFFSET_RATIO
        )
        if (tracked_x < x_left or tracked_x > x_right or
                abs(tracked_y - local_axis_y) >
                BALL_CENTER_Y_TOLERANCE_DIAMETER * expected_diameter):
            return None

        contrast, dark_fraction = self._candidate_appearance(
            rotated, tracked_x, tracked_y, expected_diameter
        )
        gray_std = self._candidate_gray_std(
            gray,
            tracked_x,
            tracked_y,
            max(4.0, expected_diameter * 0.65),
        )
        # Correlation alone can match the stationary tube texture after the
        # ball disappears.  Retain the measured steel-ball appearance gates.
        if dark_fraction < 0.18 or gray_std < 24.0:
            return None
        return {
            "score": float(BALL_TRACK_SCORE + correlation),
            "x": tracked_x,
            "y": tracked_y,
            "bbox": (
                int(round(tracked_x - expected_diameter * 0.5)),
                int(round(tracked_y - expected_diameter * 0.5)),
                int(round(expected_diameter)),
                int(round(expected_diameter)),
            ),
            "area": 0,
            "gray_std": float(gray_std),
            "contrast": float(contrast),
            "dark_fraction": float(dark_fraction),
            "fill": 0.0,
            "diameter": float(expected_diameter),
            "source": "template",
        }

    def detect(self, marker_state, now_ms):
        self.current_ms = int(now_ms)
        rotated = marker_state["rotated"]
        plus_x, zero_x, minus_x = marker_state["marker_x"]
        axis_y = float(marker_state["axis_y"])
        axis_slope = float(marker_state.get("axis_slope", 0.0))
        height, width = rotated.shape[:2]
        span = float(minus_x - plus_x)
        extension = span * BALL_ROI_EXTENSION_RATIO

        x_left = max(0, int(round(plus_x - extension)))
        x_right = min(width, int(round(minus_x + extension)))
        expected_diameter = clamp(
            span * BALL_DIAMETER_TO_MARKER_SPAN,
            7.0,
            18.0,
        )
        # The marker-derived axis follows the lower orange response band.  In
        # the rotation-normalized image the steel ball rests above that line;
        # centre the ROI on this measured relationship instead of including
        # round hardware below the tube.
        axis_origin_x = width * 0.5
        ball_axis_offset = span * BALL_CENTER_AXIS_OFFSET_RATIO
        endpoint_axes = (
            axis_y + axis_slope * (x_left - axis_origin_x),
            axis_y + axis_slope * (x_right - axis_origin_x),
        )
        y_half = max(8.0, expected_diameter * 1.05)
        y_top = max(0, int(round(min(endpoint_axes) + ball_axis_offset - y_half)))
        y_bottom = min(
            height,
            int(round(max(endpoint_axes) + ball_axis_offset + y_half + 1.0)),
        )
        if x_right - x_left < 50 or y_bottom - y_top < 8:
            return None

        crop = rotated[y_top:y_bottom, x_left:x_right]
        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
        saturation = hsv[:, :, 1]
        value = hsv[:, :, 2]

        neutral = np.zeros(saturation.shape, dtype=np.uint8)
        bright_neutral = ((saturation < BALL_LOW_SAT_MAX) &
                          (value > BALL_VALUE_MIN))
        neutral[bright_neutral] = 255
        neutral = cv2.morphologyEx(
            neutral,
            cv2.MORPH_OPEN,
            cv2.getStructuringElement(cv2.MORPH_RECT, (2, 2)),
        )
        neutral = cv2.morphologyEx(
            neutral,
            cv2.MORPH_CLOSE,
            cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)),
        )

        component_count, labels, stats, centers = cv2.connectedComponentsWithStats(
            neutral,
            8,
        )
        self.debug_raw = [
            (x_left + int(stats[index][0]), y_top + int(stats[index][1]),
             int(stats[index][2]), int(stats[index][3]), int(stats[index][4]))
            for index in range(1, component_count)
            if int(stats[index][4]) >= 3
        ]
        self.debug_appearance = []
        self.debug_candidates = []
        self.debug_hough = []
        self.debug_dark = []

        tracking = self.last_x is not None and self.lost_frames <= BALL_HOLD_FRAMES
        minimum_contrast = (BALL_TRACK_MIN_LOCAL_CONTRAST if tracking
                            else BALL_MIN_LOCAL_CONTRAST)
        minimum_dark_fraction = (BALL_TRACK_MIN_DARK_FRACTION if tracking
                                 else BALL_MIN_DARK_FRACTION)
        candidates = []

        for index in range(1, component_count):
            x, y, w, h, area = [int(v) for v in stats[index]]
            cx = float(centers[index][0])
            cy = float(centers[index][1])

            width_max = (2.55 if tracking else 1.85) * expected_diameter
            height_min = (0.24 if tracking else 0.30) * expected_diameter
            height_max = (2.20 if tracking else 1.55) * expected_diameter
            area_max = (2.80 if tracking else 1.90) * expected_diameter ** 2
            aspect_max = 3.20 if tracking else 2.20
            if not (0.35 * expected_diameter <= w <= width_max):
                continue
            if not (height_min <= h <= height_max):
                continue
            if not (
                0.12 * expected_diameter * expected_diameter
                <= area
                <= area_max
            ):
                continue

            aspect = w / float(max(1, h))
            if not (0.52 <= aspect <= aspect_max):
                continue

            global_x = x_left + cx
            global_y = y_top + cy
            ball_axis_y = (
                axis_y + axis_slope * (global_x - axis_origin_x) +
                ball_axis_offset
            )
            if (abs(global_y - ball_axis_y) >
                    BALL_CENTER_Y_TOLERANCE_DIAMETER * expected_diameter):
                continue
            fill = area / float(max(1, w * h))
            bbox_center_x = x_left + x + w * 0.5
            bbox_center_y = y_top + y + h * 0.5
            appearance_x = global_x * 0.45 + bbox_center_x * 0.55
            appearance_y = global_y * 0.45 + bbox_center_y * 0.55
            contrast, dark_fraction = self._candidate_appearance(
                rotated, appearance_x, appearance_y, expected_diameter
            )
            gray_std = self._candidate_gray_std(
                gray,
                cx,
                cy,
                max(4.0, expected_diameter * 0.65),
            )
            cold_marker_ball = (
                not tracking and
                any(
                    abs(float(global_x) - float(marker_x)) <=
                    BALL_MARKER_ARTIFACT_RADIUS_DIAMETERS * expected_diameter
                    for marker_x in (plus_x, zero_x, minus_x)
                ) and
                0.75 * expected_diameter <= float(w) <=
                1.65 * expected_diameter and
                0.55 * expected_diameter <= float(h) <=
                1.60 * expected_diameter and
                0.65 <= aspect <= 1.60 and
                gray_std >= 8.0 and
                (dark_fraction >= BALL_MARKER_BALL_MIN_DARK_FRACTION or
                 abs(contrast) >= BALL_MARKER_BALL_MIN_ABS_CONTRAST)
            )
            self.debug_appearance.append(
                (round(global_x, 1), round(global_y, 1), w, h, area,
                 round(contrast, 1), round(dark_fraction, 2),
                 round(gray_std, 1))
            )
            if (not cold_marker_ball and not self._appearance_valid(
                    contrast, dark_fraction, gray_std, tracking)):
                continue

            size_score = math.exp(
                -abs(w - expected_diameter) / (0.55 * expected_diameter)
            ) * math.exp(
                -abs(h - expected_diameter * 0.75)
                / (0.70 * expected_diameter)
            )
            y_score = math.exp(
                -abs(global_y - ball_axis_y) / (0.55 * expected_diameter)
            )

            score = (
                3.0 * size_score
                + 1.2 * fill
                + 0.020 * gray_std
                + 1.2 * y_score
                + 0.020 * abs(contrast)
                + 1.0 * dark_fraction
            )

            if self.last_x is not None and self.lost_frames <= BALL_HOLD_FRAMES:
                score += 1.25 * math.exp(
                    -abs(global_x - self.last_x) / (2.0 * expected_diameter)
                )

            candidates.append(
                {
                    "score": float(score),
                    "x": float(global_x),
                    "y": float(global_y),
                    "bbox": (x_left + x, y_top + y, w, h),
                    "area": int(area),
                    "gray_std": float(gray_std),
                    "contrast": float(contrast),
                    "dark_fraction": float(dark_fraction),
                    "fill": float(fill),
                    "diameter": float(expected_diameter),
                }
            )
            self.debug_candidates.append(
                ("N", round(global_x, 1), round(global_y, 1), round(score, 2))
            )

        # A moving steel ball does not always leave a separate bright-neutral
        # highlight: around the calibrated +50 mm region its highlight merges
        # into the long reflection on the orange tube.  Detect the compact dark
        # steel core as a second *primary* source.  Elongated tube edges and
        # saturated coloured calibration marks are rejected by geometry and
        # saturation before the normal appearance/confirmation gates run.
        dark = np.zeros(value.shape, dtype=np.uint8)
        # The polished ball reflects orange and can therefore be highly
        # saturated.  Use value for segmentation; the stricter local dark-core
        # fraction and surrounding-ring contrast checks still validate it.
        dark_core = ((value >= BALL_DARK_VALUE_MIN) &
                     (value <= BALL_CORE_MASK_VALUE_MAX))
        dark[dark_core] = 255
        dark = cv2.morphologyEx(
            dark,
            cv2.MORPH_OPEN,
            cv2.getStructuringElement(cv2.MORPH_RECT, (2, 2)),
        )
        dark = cv2.morphologyEx(
            dark,
            cv2.MORPH_CLOSE,
            cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)),
        )
        dark_count, _, dark_stats, dark_centers = cv2.connectedComponentsWithStats(
            dark,
            8,
        )
        self.debug_dark = [
            (x_left + int(dark_stats[index][0]),
             y_top + int(dark_stats[index][1]),
             int(dark_stats[index][2]), int(dark_stats[index][3]),
             int(dark_stats[index][4]))
            for index in range(1, dark_count)
            if int(dark_stats[index][4]) >= 3
        ]
        for index in range(1, dark_count):
            x, y, w, h, area = [int(v) for v in dark_stats[index]]
            cx = float(dark_centers[index][0])
            cy = float(dark_centers[index][1])
            global_x = x_left + cx
            global_y = y_top + cy
            endpoint_partial = global_x < plus_x or global_x > minus_x
            touches_outer_boundary = (
                x <= 1 or x + w >= dark.shape[1] - 1
            )
            endpoint_merged = (
                endpoint_partial and touches_outer_boundary and
                w > 1.65 * expected_diameter
            )
            dark_width_max = (((
                BALL_ENDPOINT_MERGED_MAX_WIDTH_DIAMETERS
                if endpoint_merged else 4.20
            ) if endpoint_partial else 2.80)
                              if tracking else
                              ((BALL_ENDPOINT_MERGED_MAX_WIDTH_DIAMETERS
                                if endpoint_merged else 4.20)
                               if endpoint_partial else 2.30)) * expected_diameter
            dark_height_min = (
                0.24 if tracking else (0.30 if endpoint_partial else 0.38)
            ) * expected_diameter
            dark_height_max = (2.30 if tracking else 2.40) * expected_diameter
            dark_aspect_max = ((5.00 if endpoint_merged else 4.00)
                               if endpoint_partial else
                               (4.00 if tracking else 2.00))
            dark_aspect_min = 0.20 if endpoint_partial else 0.58
            dark_fill_min = (0.15 if endpoint_partial else
                             (0.18 if tracking else 0.30))
            dark_area_max = (((
                BALL_ENDPOINT_MERGED_MAX_AREA_DIAMETER_SQ
                if endpoint_merged else 6.20
            ) if endpoint_partial else 2.80)
                             if tracking else
                             ((BALL_ENDPOINT_MERGED_MAX_AREA_DIAMETER_SQ
                               if endpoint_merged else 6.20)
                              if endpoint_partial else 3.00)) * expected_diameter ** 2
            dark_width_min = (
                0.12 if endpoint_partial else 0.38
            ) * expected_diameter
            if not (dark_width_min <= w <= dark_width_max):
                continue
            if not (dark_height_min <= h <= dark_height_max):
                continue
            if not (
                (0.06 if endpoint_partial else 0.10) *
                expected_diameter * expected_diameter
                <= area
                <= dark_area_max
            ):
                continue
            aspect = w / float(max(1, h))
            fill = area / float(max(1, w * h))
            if not (dark_aspect_min <= aspect <= dark_aspect_max and
                    dark_fill_min <= fill <=
                    (1.0 if endpoint_partial else 0.92)):
                continue

            # At a blocked end the ball silhouette can merge with the end
            # strip into one wide component.  The physical ball centre is at
            # the outer edge of that component, not at its combined centroid.
            endpoint_compact = (
                endpoint_partial and not endpoint_merged and
                0.65 * expected_diameter <= w <= 1.45 * expected_diameter and
                0.55 * expected_diameter <= h <= 1.30 * expected_diameter and
                0.65 <= aspect <= 1.70 and fill >= 0.32
            )
            if endpoint_merged:
                if global_x > minus_x:
                    global_x = x_left + x + w - expected_diameter * 0.5
                elif global_x < plus_x:
                    global_x = x_left + x + expected_diameter * 0.5

            ball_axis_y = (
                axis_y + axis_slope * (global_x - axis_origin_x) +
                ball_axis_offset
            )
            axis_tolerance = (
                0.95 * expected_diameter if endpoint_compact else
                (BALL_TRACK_CENTER_Y_TOLERANCE_DIAMETER if tracking else
                 BALL_CENTER_Y_TOLERANCE_DIAMETER) * expected_diameter
            )
            if abs(global_y - ball_axis_y) > axis_tolerance:
                continue
            contrast, dark_fraction = self._candidate_appearance(
                rotated, global_x, global_y, expected_diameter
            )
            gray_sample_x = global_x - x_left if endpoint_merged else cx
            gray_sample_y = global_y - y_top if endpoint_merged else cy
            gray_std = self._candidate_gray_std(
                gray,
                gray_sample_x,
                gray_sample_y,
                max(4.0, expected_diameter * 0.65),
            )
            self.debug_appearance.append(
                (round(global_x, 1), round(global_y, 1), w, h, area,
                 round(contrast, 1), round(dark_fraction, 2),
                 round(gray_std, 1), "D")
            )
            # At a blocked endpoint only a 2--3 px wide polished sliver can
            # remain visible.  Its dark fraction often rounds to zero even
            # though its contrast against the orange tube is strong.  Accept
            # that narrowly-scoped cold-acquire case; ordinary corridor
            # acquisition keeps the stricter dark-core test.
            endpoint_sliver = (
                not tracking and endpoint_partial and
                touches_outer_boundary and
                w <= 0.90 * expected_diameter and
                h >= 0.45 * expected_diameter and
                contrast >= 8.0 and gray_std >= 8.0
            )
            endpoint_contact_ball = (
                not tracking and endpoint_merged and
                0.50 * expected_diameter <= h <=
                2.40 * expected_diameter and
                gray_std >= 8.0 and
                (dark_fraction >= BALL_MARKER_BALL_MIN_DARK_FRACTION or
                 abs(contrast) >= BALL_MARKER_BALL_MIN_ABS_CONTRAST)
            )
            # Cold acquisition at a calibration strip needs the same explicit
            # ball-shape exception as the neutral-component path above.  The
            # strip can reduce the segmented dark fraction without removing
            # the ball-sized, high-variance silhouette.  This is tied to all
            # calibrated marker positions and generic appearance evidence, not
            # to the present room or a memorized background image.
            cold_marker_ball = (
                not tracking and
                any(
                    abs(float(global_x) - float(marker_x)) <=
                    BALL_MARKER_ARTIFACT_RADIUS_DIAMETERS * expected_diameter
                    for marker_x in (plus_x, zero_x, minus_x)
                ) and
                0.65 * expected_diameter <= float(w) <=
                1.65 * expected_diameter and
                0.55 * expected_diameter <= float(h) <=
                1.60 * expected_diameter and
                0.55 <= aspect <= 1.75 and
                gray_std >= 8.0 and
                (dark_fraction >= BALL_MARKER_BALL_MIN_DARK_FRACTION or
                 abs(contrast) >= BALL_MARKER_BALL_MIN_ABS_CONTRAST)
            )
            if (not endpoint_sliver and not endpoint_contact_ball and
                    not cold_marker_ball and
                    not self._appearance_valid(
                    contrast, dark_fraction, gray_std, tracking)):
                continue

            if endpoint_merged:
                size_score = math.exp(
                    -abs(h - expected_diameter) /
                    (0.80 * expected_diameter)
                )
            else:
                size_score = math.exp(
                    -abs(w - expected_diameter * 0.72) /
                    (0.55 * expected_diameter)
                ) * math.exp(
                    -abs(h - expected_diameter * 0.72) /
                    (0.55 * expected_diameter)
                )
            y_score = math.exp(
                -abs(global_y - ball_axis_y) / (0.60 * expected_diameter)
            )
            score = (3.2 * size_score + 1.0 * fill + 1.4 * y_score +
                     0.020 * abs(contrast) + 1.4 * dark_fraction)
            if tracking:
                score += 1.25 * math.exp(
                    -abs(global_x - self.last_x) / (2.0 * expected_diameter)
                )
            candidate = {
                "score": float(score),
                "x": float(global_x),
                "y": float(global_y),
                "bbox": (x_left + x, y_top + y, w, h),
                "area": int(area),
                "gray_std": float(gray_std),
                "contrast": float(contrast),
                "dark_fraction": float(dark_fraction),
                "fill": float(fill),
                "diameter": float(expected_diameter),
                "source": "dark",
                "endpoint_partial": endpoint_partial,
                "endpoint_merged": endpoint_merged,
            }
            candidates.append(candidate)
            self.debug_candidates.append(
                ("D", round(global_x, 1), round(global_y, 1), round(score, 2))
            )

        marker_artifact_radius = (
            BALL_MARKER_ARTIFACT_RADIUS_DIAMETERS * expected_diameter
        )
        marker_artifact_min_area = (
            BALL_MARKER_ARTIFACT_MIN_AREA_DIAMETER_SQ *
            expected_diameter * expected_diameter
        )

        def undersized_marker_artifact(item):
            close_to_marker = any(
                abs(float(item["x"]) - float(marker_x)) <=
                marker_artifact_radius
                for marker_x in (plus_x, zero_x, minus_x)
            )
            return (
                close_to_marker and
                float(item.get("area", 0.0)) < marker_artifact_min_area
            )

        def ball_shaped_marker_overlap(item):
            """Distinguish a round ball on a strip from the strip itself."""
            if not any(
                    abs(float(item["x"]) - float(marker_x)) <=
                    marker_artifact_radius
                    for marker_x in (plus_x, zero_x, minus_x)):
                return False
            _, _, item_w, item_h = item["bbox"]
            aspect = float(item_w) / float(max(1, item_h))
            return (
                0.75 * expected_diameter <= float(item_w) <=
                1.65 * expected_diameter and
                0.55 * expected_diameter <= float(item_h) <=
                1.60 * expected_diameter and
                0.65 <= aspect <= 1.60 and
                (float(item.get("dark_fraction", 0.0)) >=
                 BALL_MARKER_BALL_MIN_DARK_FRACTION or
                 abs(float(item.get("contrast", 0.0))) >=
                 BALL_MARKER_BALL_MIN_ABS_CONTRAST)
            )

        for item in candidates:
            if ball_shaped_marker_overlap(item):
                item["score"] += BALL_MARKER_BALL_SHAPE_SCORE_BONUS
            background_fraction, background_mean = self._background_change(
                rotated,
                item["x"],
                item["y"],
                axis_y,
                expected_diameter,
            )
            item["background_fraction"] = background_fraction
            item["background_mean"] = background_mean
            item["score"] += BACKGROUND_SCORE_GAIN * background_fraction

        if tracking:
            candidates = [
                item for item in candidates
                if ((not undersized_marker_artifact(item) or
                     ball_shaped_marker_overlap(item)) and
                    float(item.get("background_fraction", 0.0)) >=
                    BACKGROUND_TRACK_MIN_FRACTION)
            ]
        else:
            candidates = [
                item for item in candidates
                if ((not undersized_marker_artifact(item) or
                     ball_shaped_marker_overlap(item)) and
                    float(item.get("background_fraction", 1.0)) >= (
                        BACKGROUND_ENDPOINT_ACQUIRE_MIN_FRACTION
                        if item.get("endpoint_partial", False)
                        else BACKGROUND_ACQUIRE_MIN_FRACTION
                    ))
            ]
        # Only suppress fallbacks while a tracked ball is on the *inside* of an
        # endpoint strip (or on the centre strip).  Once its centre has crossed
        # beyond +/-100 mm it is still fully visible, so disabling template and
        # optical tracking there would create the observed endpoint dropouts.
        tracking_near_marker = (
            tracking and self.last_x is not None and (
                abs(float(self.last_x) - float(zero_x)) <= marker_artifact_radius or
                (plus_x <= float(self.last_x) <= plus_x + marker_artifact_radius) or
                (minus_x - marker_artifact_radius <= float(self.last_x) <= minus_x)
            )
        )
        # Template matching and optical flow are short-lived temporal cues,
        # not scene calibration.  They must remain available even when an
        # optional empty-tube image exists; otherwise a blurred moving ball
        # immediately falls back to invalid frames on the real field whenever
        # the stored bench background differs.
        allow_fallbacks = True

        # While tracking, reject physically impossible alternate reflections
        # before ranking.  Selecting the global maximum first and applying the
        # jump gate afterwards discarded the whole frame whenever a slightly
        # stronger static reflection beat the real, nearby ball candidate.
        # Acquisition remains unchanged and keeps its stricter score gates.
        ranked_candidates = candidates
        out_of_window_background = False
        if tracking:
            predicted_x, tracking_jump_limit = self._tracking_window(span)
            ranked_candidates = [
                item for item in candidates
                if abs(float(item["x"]) - predicted_x) <=
                tracking_jump_limit
            ]
            # A scene-specific background must never veto generic temporal
            # tracking.  Competing candidates outside the physical motion
            # window are ignored and the bounded fallback is allowed to run.
            out_of_window_background = False
        else:
            acquire_guard = span * BALL_ACQUIRE_EDGE_GUARD_RATIO
            ranked_candidates = [
                item for item in candidates
                if plus_x + acquire_guard <= float(item["x"]) <=
                minus_x - acquire_guard
            ]
            filtered_candidates = []
            for item in ranked_candidates:
                if item.get("endpoint_partial", False):
                    item_x, _, item_w, item_h = item["bbox"]
                    touches_outer_boundary = (
                        item_x <= x_left + 1 or
                        item_x + item_w >= x_right - 1
                    )
                    item_aspect = float(item_w) / float(max(1, item_h))
                    endpoint_is_partial_ball = (
                        (touches_outer_boundary and
                         float(item_w) <= 1.60 * expected_diameter) or
                        (item.get("endpoint_merged", False) and
                         touches_outer_boundary and
                         float(item_h) >= 0.50 * expected_diameter and
                         float(item_h) <= 2.40 * expected_diameter) or
                        (0.75 * expected_diameter <= float(item_w) <=
                         1.75 * expected_diameter and
                         0.65 * expected_diameter <= float(item_h) <=
                         1.75 * expected_diameter and
                         0.65 <= item_aspect <= 1.60 and
                         float(item.get("fill", 0.0)) >= 0.35)
                    )
                    if not endpoint_is_partial_ball:
                        continue
                mapped_cm = map_ball_to_cm(
                    float(item["x"]), plus_x, zero_x, minus_x
                )
                if mapped_cm is None:
                    continue
                if any(
                    abs(mapped_cm - fixed_cm) <=
                    BALL_ACQUIRE_FIXED_EXCLUSION_RADIUS_CM
                    for fixed_cm in BALL_ACQUIRE_FIXED_EXCLUSIONS_CM
                ):
                    continue
                filtered_candidates.append(item)
            ranked_candidates = filtered_candidates
            if len(ranked_candidates) > 1:
                acquisition_ranking = sorted(
                    ranked_candidates,
                    key=lambda item: item["score"],
                    reverse=True,
                )
                if (
                    float(acquisition_ranking[0]["score"]) -
                    float(acquisition_ranking[1]["score"]) <
                    BALL_ACQUIRE_SCORE_MARGIN
                ):
                    # An ambiguous first sighting is more likely to be two
                    # competing tube highlights than a trustworthy steel ball.
                    # Tracking remains permissive once a ball has passed the
                    # multi-frame acquisition gate.
                    ranked_candidates = []
        if tracking and ranked_candidates:
            best = max(
                ranked_candidates,
                key=lambda item: (
                    float(item["score"]) -
                    BALL_TRACK_DISTANCE_PENALTY *
                    min(
                        abs(float(item["x"]) - predicted_x) /
                        expected_diameter,
                        1.5,
                    )
                ),
            )
        else:
            best = (max(ranked_candidates, key=lambda item: item["score"])
                    if ranked_candidates else None)
        if tracking:
            threshold = BALL_TRACK_SCORE
        elif best is not None and best.get("source") == "dark":
            threshold = (BALL_ENDPOINT_DARK_ACQUIRE_SCORE
                         if best.get("endpoint_partial", False)
                         else BALL_DARK_ACQUIRE_SCORE)
        else:
            threshold = BALL_ACQUIRE_SCORE

        if ((best is None or best["score"] < threshold) and
                not out_of_window_background):
            template_candidate = (
                self._template_track(
                    rotated,
                    x_left,
                    x_right,
                    axis_y,
                    axis_slope,
                    span,
                    expected_diameter,
                )
                if (tracking and not tracking_near_marker and
                    allow_fallbacks) else None
            )
            if template_candidate is not None:
                best = template_candidate

        if ((best is None or best["score"] < threshold) and
                not out_of_window_background):
            optical_candidate = (
                self._optical_flow_track(
                    rotated,
                    axis_y,
                    axis_slope,
                    span,
                    expected_diameter,
                )
                if (tracking and not tracking_near_marker and
                    allow_fallbacks) else None
            )
            if optical_candidate is not None:
                best = optical_candidate

        if (BALL_ENABLE_LOCAL_SPECULAR_FALLBACK and
                (best is None or best["score"] < threshold) and
                not out_of_window_background):
            local_candidate = (
                self._local_specular_track(
                    rotated,
                    x_left,
                    x_right,
                    axis_y,
                    axis_slope,
                    span,
                    expected_diameter,
                )
                if (tracking and not tracking_near_marker and
                    allow_fallbacks) else None
            )
            if (local_candidate is not None and
                    local_candidate["score"] >= BALL_TRACK_SCORE and
                    self.local_streak < BALL_MAX_CONSECUTIVE_LOCAL_FRAMES):
                best = local_candidate

        if (BALL_ENABLE_HOUGH_FALLBACK and
                (best is None or best["score"] < threshold) and
                not out_of_window_background):
            hough_candidate = (self._hough_fallback(
                rotated,
                x_left,
                x_right,
                axis_y,
                axis_slope,
                expected_diameter,
            ) if (
                allow_fallbacks and
                (((tracking and not tracking_near_marker) and
                  self.fallback_streak < BALL_MAX_CONSECUTIVE_FALLBACK_FRAMES) or
                 not tracking)
            ) else None)
            if not tracking and hough_candidate is not None:
                hough_x = float(hough_candidate["x"])
                acquire_guard = span * BALL_ACQUIRE_EDGE_GUARD_RATIO
                mapped_cm = map_ball_to_cm(
                    hough_x, plus_x, zero_x, minus_x
                )
                fixed_hardware = (
                    mapped_cm is None or any(
                        abs(mapped_cm - fixed_cm) <=
                        BALL_ACQUIRE_FIXED_EXCLUSION_RADIUS_CM
                        for fixed_cm in BALL_ACQUIRE_FIXED_EXCLUSIONS_CM
                    )
                )
                if (hough_candidate["score"] < BALL_HOUGH_ACQUIRE_SCORE or
                        hough_x < plus_x + acquire_guard or
                        hough_x > minus_x - acquire_guard or
                        fixed_hardware):
                    hough_candidate = None
            best = hough_candidate
        if best is None:
            self._clear_pending()
            self.lost_frames += 1
            if self.lost_frames > BALL_HOLD_FRAMES:
                self.reacquire_side = 0
                self.last_x = None
                self.last_y = None
            return None

        if (best.get("fallback", False) or
                best.get("endpoint_merged", False)):
            refined_x = float(best["x"])
            refined_y = float(best["y"])
        else:
            # 低饱和区域可能只覆盖钢球的一部分，bbox 中心通常比像素重心更接近球心。
            bx, by, bw, bh = best["bbox"]
            bbox_center_x = bx + bw * 0.5
            bbox_center_y = by + bh * 0.5
            refined_x = best["x"] * 0.45 + bbox_center_x * 0.55
            refined_y = best["y"] * 0.45 + bbox_center_y * 0.55

        if self.last_x is not None and self.lost_frames <= BALL_HOLD_FRAMES:
            predicted_x, tracking_jump_limit = self._tracking_window(span)
            jump = abs(refined_x - predicted_x)
            # High score alone does not make a discontinuous jump physical:
            # the orange tube contains several strong, circular reflections.
            # Reject every over-limit jump while tracking; after the normal
            # lost-frame window the detector is free to acquire anywhere.
            jump_limit = (BALL_FALLBACK_MAX_JUMP_PX
                          if best.get("fallback", False)
                          else tracking_jump_limit)
            if jump > jump_limit:
                self._clear_pending()
                self.lost_frames += 1
                if self.lost_frames > BALL_HOLD_FRAMES:
                    self.reacquire_side = 0
                    self.last_x = None
                    self.last_y = None
                return None

            # A compact fixed highlight can be geometrically indistinguishable
            # from the polished ball for one frame.  Reject candidates whose
            # measured velocity would require an impossible acceleration.  A
            # stricter directional guard is applied near calibration strips,
            # where static circular/rectangular features are expected.
            if self.last_accept_ms is not None:
                dt_s = clamp(
                    elapsed_ms(self.current_ms, self.last_accept_ms) / 1000.0,
                    0.005,
                    BALL_TRACK_MAX_DT_S,
                )
                px_per_mm = max(0.5, float(span) / 200.0)
                instant_velocity_px_s = (
                    float(refined_x) - float(self.last_x)
                ) / dt_s
                acceleration_slack_px_s = (
                    BALL_TRACK_MAX_ACCEL_MM_S2 * dt_s +
                    BALL_TRACK_VELOCITY_SLACK_MM_S
                ) * px_per_mm
                marker_motion_guard_px_s = (
                    BALL_MARKER_MOTION_GUARD_SPEED_MM_S * px_per_mm
                )
                near_marker = any(
                    abs(float(refined_x) - float(marker_x)) <=
                    marker_artifact_radius
                    for marker_x in (plus_x, zero_x, minus_x)
                )
                velocity_reversal = (
                    abs(self.motion_x) > marker_motion_guard_px_s and
                    (instant_velocity_px_s * self.motion_x <= 0.0 or
                     abs(instant_velocity_px_s) <
                     BALL_MARKER_MOTION_MIN_FRACTION * abs(self.motion_x))
                )
                # A commanded catch can bring the real ball to rest directly
                # over a marker.  In that case the velocity EMA still carries
                # the pre-catch motion for several frames, so an unconditional
                # reversal gate falsely rejects the same strong ball candidate
                # forever.  Keep rejecting weak marker-like candidates, while
                # allowing a candidate which independently clears the stricter
                # cold-acquire appearance/geometry score to update the tracker.
                weak_marker_candidate = best["score"] < BALL_ACQUIRE_SCORE
                if (abs(instant_velocity_px_s - self.motion_x) >
                        acceleration_slack_px_s or
                        (near_marker and velocity_reversal and
                         weak_marker_candidate)):
                    self._clear_pending()
                    self.lost_frames += 1
                    if self.lost_frames > BALL_HOLD_FRAMES:
                        self.reacquire_side = 0
                        self.last_x = None
                        self.last_y = None
                    return None

        if not tracking:
            if (self.pending_x is not None and
                    abs(refined_x - self.pending_x) <= BALL_ACQUIRE_CONFIRM_JUMP_PX and
                    abs(refined_y - self.pending_y) <= 0.60 * expected_diameter):
                self.pending_count += 1
                self.pending_x = 0.5 * self.pending_x + 0.5 * refined_x
                self.pending_y = 0.5 * self.pending_y + 0.5 * refined_y
            else:
                self.pending_x = float(refined_x)
                self.pending_y = float(refined_y)
                self.pending_count = 1
            acquire_confirm_frames = (
                BALL_ENDPOINT_ACQUIRE_CONFIRM_FRAMES
                if best.get("endpoint_partial", False)
                else BALL_ACQUIRE_CONFIRM_FRAMES
            )
            if self.pending_count < acquire_confirm_frames:
                return None
            refined_x = float(self.pending_x)
            refined_y = float(self.pending_y)
            self._clear_pending()

        if self.last_x is not None and self.last_accept_ms is not None:
            dt_s = clamp(
                elapsed_ms(self.current_ms, self.last_accept_ms) / 1000.0,
                0.005,
                BALL_TRACK_MAX_DT_S,
            )
            instant_motion_x = (
                (float(refined_x) - float(self.last_x)) /
                dt_s
            )
            max_speed_px_s = (
                BALL_TRACK_MAX_SPEED_MM_S * max(0.5, span / 200.0)
            )
            instant_motion_x = clamp(
                instant_motion_x,
                -max_speed_px_s,
                max_speed_px_s,
            )
            self.motion_x = 0.55 * self.motion_x + 0.45 * instant_motion_x
        else:
            self.motion_x = 0.0
        self.last_x = float(refined_x)
        self.last_y = float(refined_y)
        self.last_accept_ms = self.current_ms
        self.reacquire_side = 0
        self.last_gray = cv2.cvtColor(rotated, cv2.COLOR_BGR2GRAY).copy()
        if best.get("source") != "template":
            template_radius = max(5, int(round(expected_diameter * 0.72)))
            template_left = max(0, int(round(refined_x)) - template_radius)
            template_right = min(
                self.last_gray.shape[1],
                int(round(refined_x)) + template_radius + 1,
            )
            template_top = max(0, int(round(refined_y)) - template_radius)
            template_bottom = min(
                self.last_gray.shape[0],
                int(round(refined_y)) + template_radius + 1,
            )
            template = self.last_gray[
                template_top:template_bottom,
                template_left:template_right,
            ]
            if (template.shape[0] == 2 * template_radius + 1 and
                    template.shape[1] == 2 * template_radius + 1):
                self.ball_template = template.copy()
        self.lost_frames = 0
        if best.get("fallback", False):
            self.fallback_streak += 1
        else:
            self.fallback_streak = 0
        if best.get("source") == "local_specular":
            self.local_streak += 1
        else:
            self.local_streak = 0
        if best.get("source") == "template":
            self.template_streak += 1
        else:
            self.template_streak = 0

        quality = int(clamp((best["score"] - BALL_TRACK_SCORE) * 35.0 + 50.0, 0, 99))
        best["x"] = float(refined_x)
        best["y"] = float(refined_y)
        best["quality"] = quality
        return best


# ============================================================
# 5. 三点一维透视映射
# ============================================================
def map_ball_to_cm(ball_x, plus_x, zero_x, minus_x):
    """以图像左、0、图像右三点求射影映射并应用当前物理坐标方向。"""
    q_plus = float(plus_x) - float(zero_x)
    q_minus = float(minus_x) - float(zero_x)
    if abs(q_plus) < 5.0 or abs(q_minus) < 5.0:
        return None

    c = -0.5 * (1.0 / q_minus + 1.0 / q_plus)
    a = 10.0 * c + 10.0 / q_plus
    q = float(ball_x) - float(zero_x)
    denominator = c * q + 1.0
    if abs(denominator) < 1e-5:
        return None
    return IMAGE_LEFT_POSITION_SIGN * a * q / denominator


# ============================================================
# 6. 补光灯与 UART
# ============================================================
def init_fill_light():
    if not FILL_LIGHT_ENABLED:
        print("FILL_LIGHT disabled")
        return None
    try:
        err.check_raise(
            pinmap.set_pin_function(FILL_LIGHT_PIN, FILL_LIGHT_GPIO),
            "fill light pin map failed",
        )
        light = gpio.GPIO(FILL_LIGHT_GPIO, gpio.Mode.OUT)
        light.value(1)
        print("FILL_LIGHT on:", FILL_LIGHT_PIN, FILL_LIGHT_GPIO)
        return light
    except Exception as exception:
        print("FILL_LIGHT init failed:", exception)
        return None


def init_uart():
    if not ENABLE_UART:
        return None
    try:
        err.check_raise(
            pinmap.set_pin_function(UART_TX_PIN, UART_TX_FUNCTION),
            "UART TX pin map failed",
        )
        err.check_raise(
            pinmap.set_pin_function(UART_RX_PIN, UART_RX_FUNCTION),
            "UART RX pin map failed",
        )
        serial = uart.UART(UART_DEVICE, UART_BAUD)
        serial.write_str("$BOOT,PLASTIC_MARKER_TRACKER_V3\r\n")
        print("UART ready:", UART_DEVICE, UART_BAUD)
        return serial
    except Exception as exception:
        print("UART init failed:", exception)
        return None


# ============================================================
# 7. 绘制
# ============================================================
def draw_overlay(
    frame,
    marker_state,
    ball_detector,
    ball,
    valid,
    filtered_cm,
    raw_cm,
    velocity_cm_s,
    fps,
    process_ms,
):
    if valid:
        cv2.putText(
            frame,
            "x={:+.2f}cm raw={:+.2f} Q={:02d}".format(
                filtered_cm,
                raw_cm,
                ball["quality"],
            ),
            (6, 17),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            (0, 255, 0),
            1,
            cv2.LINE_AA,
        )
        cv2.putText(
            frame,
            "v={:+.1f}cm/s {} FPS:{:.1f} P:{:.1f}ms".format(
                velocity_cm_s,
                marker_state["mode"],
                fps,
                process_ms,
            ),
            (6, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            (0, 255, 255),
            1,
            cv2.LINE_AA,
        )
    else:
        if ball is not None:
            text = "BAD x={:.1f} y={:.1f} {}".format(
                ball["x"], ball["y"], ball.get("source", "neutral")
            )
        elif marker_state is not None:
            text = "SEARCH R{} D{} A{} C{} H{}".format(
                len(ball_detector.debug_raw),
                len(ball_detector.debug_dark),
                len(ball_detector.debug_appearance),
                len(ball_detector.debug_candidates),
                len(ball_detector.debug_hough),
            )
        else:
            text = "MARKER SEARCH"
        cv2.putText(
            frame,
            "{} FPS:{:.1f} P:{:.1f}ms".format(text, fps, process_ms),
            (6, 18),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.47,
            (0, 0, 255),
            1,
            cv2.LINE_AA,
        )
        if ball is not None:
            cv2.putText(
                frame,
                "CAND x={:.1f} y={:.1f} src={} C={:.0f} D={:.2f}".format(
                    ball["x"],
                    ball["y"],
                    ball.get("source", "neutral"),
                    ball.get("contrast", 0.0),
                    ball.get("dark_fraction", 0.0),
                ),
                (6, 37),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.38,
                (0, 255, 255),
                1,
                cv2.LINE_AA,
            )

    if marker_state is None:
        return

    inverse = marker_state["inverse"]
    plus_x, zero_x, minus_x = marker_state["marker_x"]
    axis_y = marker_state["axis_y"]
    axis_slope = float(marker_state.get("axis_slope", 0.0))
    axis_origin_x = marker_state["rotated"].shape[1] * 0.5

    def local_axis_y(x):
        return axis_y + axis_slope * (float(x) - axis_origin_x)

    if DRAW_LEVEL >= 2:
        plus_point = affine_point(inverse, (plus_x, local_axis_y(plus_x)))
        zero_point = affine_point(inverse, (zero_x, local_axis_y(zero_x)))
        minus_point = affine_point(inverse, (minus_x, local_axis_y(minus_x)))
        cv2.line(
            frame,
            point_int(plus_point),
            point_int(minus_point),
            (0, 255, 255),
            1,
            cv2.LINE_AA,
        )
        for point, color, label in (
            (
                plus_point,
                (255, 0, 0),
                "{:+.0f}".format(10.0 * IMAGE_LEFT_POSITION_SIGN),
            ),
            (zero_point, (0, 255, 255), "0"),
            (
                minus_point,
                (0, 0, 255),
                "{:+.0f}".format(-10.0 * IMAGE_LEFT_POSITION_SIGN),
            ),
        ):
            cv2.circle(frame, point_int(point), 4, color, 2, cv2.LINE_AA)
            cv2.putText(
                frame,
                label,
                (point_int(point)[0] - 10, point_int(point)[1] - 7),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.36,
                color,
                1,
                cv2.LINE_AA,
            )

    if DRAW_LEVEL >= 1 and ball is not None:
        ball_point = affine_point(inverse, (ball["x"], ball["y"]))
        radius = max(4, int(round(ball["diameter"] * 0.5)))
        cv2.circle(frame, point_int(ball_point), radius, (0, 255, 0), 1, cv2.LINE_AA)
        cv2.circle(frame, point_int(ball_point), 2, (0, 0, 255), -1, cv2.LINE_AA)

    if ball is None:
        cv2.putText(
            frame,
            "R{} D{} A{} C{} H{}".format(
                len(ball_detector.debug_raw),
                len(ball_detector.debug_dark),
                len(ball_detector.debug_appearance),
                len(ball_detector.debug_candidates),
                len(ball_detector.debug_hough),
            ),
            (6, 37),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            (0, 255, 255),
            1,
            cv2.LINE_AA,
        )
        # Temporary on-device diagnostics: magenta marks geometry-qualified
        # appearance components, and cyan marks candidates that passed the
        # appearance gates.  Values are C=local contrast and D=dark fraction.
        for item in ball_detector.debug_appearance[:8]:
            px, py = float(item[0]), float(item[1])
            contrast = float(item[5])
            dark_fraction = float(item[6])
            point = point_int(affine_point(inverse, (px, py)))
            cv2.circle(frame, point, 4, (255, 0, 255), 1, cv2.LINE_AA)
            cv2.putText(
                frame,
                "{:.0f}/{:.2f}".format(contrast, dark_fraction),
                (point[0] + 4, point[1] - 3),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.28,
                (255, 0, 255),
                1,
                cv2.LINE_AA,
            )
        for item in ball_detector.debug_candidates[:8]:
            point = point_int(affine_point(inverse, (float(item[1]), float(item[2]))))
            cv2.circle(frame, point, 6, (255, 255, 0), 1, cv2.LINE_AA)
        for prefix, items, color in (
            ("N", ball_detector.debug_raw, (0, 255, 255)),
            ("D", ball_detector.debug_dark, (255, 255, 0)),
        ):
            for item in items[:16]:
                x, y, w, h = [int(v) for v in item[:4]]
                point = point_int(affine_point(
                    inverse,
                    (x + 0.5 * w, y + 0.5 * h),
                ))
                cv2.circle(frame, point, 2, color, -1, cv2.LINE_AA)
                cv2.putText(
                    frame,
                    "{}{},{}".format(prefix, w, h),
                    (point[0] + 2, point[1] + 7),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.24,
                    color,
                    1,
                    cv2.LINE_AA,
                )


# ============================================================
# 8. 主程序
# ============================================================
def main():
    # 保持对象存活到视觉进程退出；先开灯，再让相机丢弃起始曝光帧。
    fill_light = init_fill_light()
    cam = camera.Camera(
        FRAME_W,
        FRAME_H,
        image.Format.FMT_BGR888,
        fps=CAMERA_FPS,
        buff_num=CAMERA_BUFFER_NUM,
    )
    try:
        cam.exp_mode(camera.AeMode.Manual)
        actual_exposure_us = cam.exposure(CAMERA_MANUAL_EXPOSURE_US)
        actual_gain = cam.gain(CAMERA_MANUAL_GAIN)
        print(
            "CAMERA exposure_us={} gain={} mode={}".format(
                actual_exposure_us,
                actual_gain,
                cam.exp_mode(),
            )
        )
    except Exception as exception:
        print("Manual camera exposure failed:", exception)
    cam.skip_frames(20)

    disp = None
    if ENABLE_PREVIEW:
        try:
            display.set_trans_image_quality(PREVIEW_JPEG_QUALITY)
        except Exception as exception:
            print("Set preview quality failed:", exception)
        disp = display.Display()

    serial = init_uart()
    print("VISION_BUILD", VISION_BUILD, "fill_light={}".format(
        int(fill_light is not None)
    ))
    marker_tracker = PlasticMarkerTracker()
    ball_detector = BallDetector()
    position_median = RollingMedianFilter(5)
    position_filter = AlphaBetaFilter(POSITION_ALPHA, POSITION_BETA)

    last_frame_ms = time.ticks_ms()
    last_uart_ms = 0
    last_print_ms = 0
    frame_index = 0
    marker_available_last_frame = False
    fps_ema = 0.0
    process_ms_ema = 0.0
    debug_ball_seen = False
    debug_loss_streak = 0
    diagnostic_frames = 0
    diagnostic_valid_frames = 0
    diagnostic_uart_packets = 0
    diagnostic_max_frame_gap_ms = 0
    diagnostic_max_process_ms = 0
    diagnostic_max_uart_ms = 0
    diagnostic_max_preview_ms = 0

    while not app.need_exit():
        image_maix = cam.read()
        if image_maix is None:
            time.sleep_ms(1)
            continue

        frame_start_ms = time.ticks_ms()
        frame = image.image2cv(image_maix, ensure_bgr=False, copy=False)
        now_ms = frame_start_ms
        frame_index += 1

        dt_ms = elapsed_ms(now_ms, last_frame_ms)
        last_frame_ms = now_ms
        diagnostic_frames += 1
        diagnostic_max_frame_gap_ms = max(diagnostic_max_frame_gap_ms, dt_ms)
        if dt_ms > 0:
            instant_fps = 1000.0 / float(dt_ms)
            fps_ema = instant_fps if fps_ema <= 0 else fps_ema * 0.90 + instant_fps * 0.10

        # FIXED_MARKER_X describes a camera rigidly mounted to the chassis.
        # Re-scan the orange tube only until the startup median has locked its
        # axis.  _find_tube_axis() takes 170--350 ms on this MaixCAM2; running
        # it every six frames created periodic UART gaps and extra heat even
        # though update() discarded every later axis result in favour of the
        # already locked fixed_axis_y.  A service/camera restart deliberately
        # performs the seven-sample lock again, so installation or lighting
        # changes are still handled without a scene-specific runtime search.
        if (marker_tracker.last_state is None or
                (FIXED_MARKER_X is not None and
                 marker_tracker.fixed_axis_y is None) or
                (FIXED_MARKER_X is None and
                 frame_index % MARKER_UPDATE_EVERY_N == 1)):
            marker_state = marker_tracker.update(frame)
        else:
            marker_state = marker_tracker.reuse_verified_geometry(frame)
        marker_available_now = marker_state is not None
        if marker_available_now != marker_available_last_frame:
            ball_detector.reset()
            position_median.reset()
            position_filter.reset()
        marker_available_last_frame = marker_available_now

        ball = None
        raw_cm = 0.0
        filtered_cm = position_filter.x if position_filter.valid else 0.0
        velocity_cm_s = position_filter.v if position_filter.valid else 0.0
        position_valid = False

        if marker_state is not None:
            ball = ball_detector.detect(marker_state, now_ms)
            if ball is not None:
                plus_x, zero_x, minus_x = marker_state["marker_x"]
                mapped = map_ball_to_cm(ball["x"], plus_x, zero_x, minus_x)
                if (mapped is not None and
                        -BALL_VALID_POSITION_LIMIT_CM <= mapped <=
                        BALL_VALID_POSITION_LIMIT_CM):
                    # End-pocket centres sit beyond the calibrated strips.
                    # Report the full validated corridor so motion out of a
                    # pocket is visible immediately to the endpoint controller.
                    raw_cm = clamp(
                        float(mapped),
                        -BALL_REPORTED_POSITION_LIMIT_CM,
                        BALL_REPORTED_POSITION_LIMIT_CM,
                    )
                    median_cm = position_median.update(raw_cm)
                    filtered_cm, velocity_cm_s = position_filter.update(
                        median_cm, now_ms
                    )
                    position_valid = True
                elif mapped is not None:
                    # Outside the verified blocked-end physical corridor, do
                    # not let a fixed chassis structure maintain the tracker.
                    ball_detector.reset()
        else:
            ball_detector.reset()
            position_median.reset()

        if position_valid:
            debug_ball_seen = True
            debug_loss_streak = 0
            diagnostic_valid_frames += 1
        elif debug_ball_seen:
            debug_loss_streak += 1
            if (DEBUG_LOSS_PREFIX and
                    debug_loss_streak in (1, 3, 8, 16)):
                loss_tag = "{}-loss-{:02d}".format(
                    DEBUG_LOSS_PREFIX,
                    debug_loss_streak,
                )
                cv2.imwrite(loss_tag + "-raw.jpg", frame.copy())
                overlay = frame.copy()
                draw_overlay(
                    overlay,
                    marker_state,
                    ball_detector,
                    ball,
                    position_valid,
                    filtered_cm,
                    raw_cm,
                    velocity_cm_s,
                    fps_ema,
                    process_ms_ema,
                )
                cv2.imwrite(loss_tag + "-overlay.jpg", overlay)
                print("DEBUG_LOSS_SNAPSHOT", loss_tag)

        process_ms = float(elapsed_ms(time.ticks_ms(), frame_start_ms))
        diagnostic_max_process_ms = max(
            diagnostic_max_process_ms,
            int(round(process_ms)),
        )
        process_ms_ema = (
            process_ms
            if process_ms_ema <= 0
            else process_ms_ema * 0.90 + process_ms * 0.10
        )

        if DEBUG_SNAPSHOT_PREFIX and frame_index == 30:
            print(
                "DEBUG_SAMPLE ball={} marker_x={} raw_cm={:.3f} "
                "filtered_cm={:.3f} velocity_cm_s={:.3f}".format(
                    None if ball is None else {
                        "x": round(float(ball["x"]), 3),
                        "y": round(float(ball["y"]), 3),
                        "source": ball.get("source", "unknown"),
                        "quality": int(ball.get("quality", 0)),
                    },
                    None if marker_state is None else tuple(
                        round(float(value), 3)
                        for value in marker_state["marker_x"]
                    ),
                    raw_cm,
                    filtered_cm,
                    velocity_cm_s,
                )
            )
            cv2.imwrite(DEBUG_SNAPSHOT_PREFIX + "-raw.jpg", frame.copy())
            draw_overlay(
                frame,
                marker_state,
                ball_detector,
                ball,
                position_valid,
                filtered_cm,
                raw_cm,
                velocity_cm_s,
                fps_ema,
                process_ms_ema,
            )
            cv2.imwrite(DEBUG_SNAPSHOT_PREFIX + "-overlay.jpg", frame)
            print("DEBUG_SNAPSHOT", DEBUG_SNAPSHOT_PREFIX)
            cam.close()
            return

        if serial is not None and elapsed_ms(now_ms, last_uart_ms) >= UART_SEND_PERIOD_MS:
            last_uart_ms = now_ms
            uart_start_ms = time.ticks_ms()
            try:
                if position_valid:
                    serial.write_str(
                        "$B,{:.2f},1,{:.2f}\r\n".format(
                            filtered_cm * 10.0,
                            velocity_cm_s * 10.0,
                        )
                    )
                else:
                    serial.write_str("$B,0,0,0\r\n")
                diagnostic_uart_packets += 1
            except Exception as exception:
                print("UART write failed:", exception)
                serial = None
            diagnostic_max_uart_ms = max(
                diagnostic_max_uart_ms,
                elapsed_ms(time.ticks_ms(), uart_start_ms),
            )

        if ENABLE_PREVIEW and frame_index % PREVIEW_EVERY_N == 0:
            preview_start_ms = time.ticks_ms()
            draw_overlay(
                frame,
                marker_state,
                ball_detector,
                ball,
                position_valid,
                filtered_cm,
                raw_cm,
                velocity_cm_s,
                fps_ema,
                process_ms_ema,
            )
            # frame 与 image_maix 共用底层缓冲区，直接显示，避免再次 resize/cv2image。
            disp.show(image_maix)
            diagnostic_max_preview_ms = max(
                diagnostic_max_preview_ms,
                elapsed_ms(time.ticks_ms(), preview_start_ms),
            )

        if elapsed_ms(now_ms, last_print_ms) >= PRINT_PERIOD_MS:
            last_print_ms = now_ms
            try:
                with open("/sys/class/thermal/thermal_zone0/temp", "r") as thermal_file:
                    diagnostic_temperature_mc = int(thermal_file.read().strip())
            except Exception:
                diagnostic_temperature_mc = -1
            print(
                "RUNTIME frames={} valid={} uart={} gapmax={}ms procmax={}ms "
                "uartmax={}ms previewmax={}ms temp_mc={}".format(
                    diagnostic_frames,
                    diagnostic_valid_frames,
                    diagnostic_uart_packets,
                    diagnostic_max_frame_gap_ms,
                    diagnostic_max_process_ms,
                    diagnostic_max_uart_ms,
                    diagnostic_max_preview_ms,
                    diagnostic_temperature_mc,
                )
            )
            diagnostic_frames = 0
            diagnostic_valid_frames = 0
            diagnostic_uart_packets = 0
            diagnostic_max_frame_gap_ms = 0
            diagnostic_max_process_ms = 0
            diagnostic_max_uart_ms = 0
            diagnostic_max_preview_ms = 0
            if position_valid:
                print(
                    "BALL x={:+.3f} raw={:+.3f} v={:+.2f} Q={} "
                    "C={:.1f} D={:.2f} F={} xy=({:.1f},{:.1f}) axis={:.1f}/{:.3f} "
                    "box={} mode={} FPS={:.1f} "
                    "PROC={:.1f}ms markers=({:.1f},{:.1f},{:.1f})".format(
                        filtered_cm,
                        raw_cm,
                        velocity_cm_s,
                        ball["quality"],
                        ball.get("contrast", 0.0),
                        ball.get("dark_fraction", 0.0),
                        int(ball.get("fallback", False)),
                        ball["x"],
                        ball["y"],
                        marker_state["axis_y"],
                        marker_state.get("axis_slope", 0.0),
                        ball["bbox"],
                        marker_state["mode"],
                        fps_ema,
                        process_ms_ema,
                        marker_state["marker_x"][0],
                        marker_state["marker_x"][1],
                        marker_state["marker_x"][2],
                    )
                )
            else:
                print(
                    "SEARCH marker={} ball={} selected={} groups={} comps={} "
                    "pending={} last_x={} lost={} FPS={:.1f} PROC={:.1f}ms".format(
                        marker_state is not None,
                        ball is not None,
                        marker_tracker.debug_selected,
                        marker_tracker.debug_groups,
                        marker_tracker.debug_components,
                        ball_detector.pending_count,
                        (None if ball_detector.last_x is None else
                         round(float(ball_detector.last_x), 1)),
                        ball_detector.lost_frames,
                        fps_ema,
                        process_ms_ema,
                    )
                )
                if marker_state is not None and ball is None:
                    print(
                        "BALL_DEBUG raw={} dark={} appearance={} candidates={} hough={}".format(
                            ball_detector.debug_raw,
                            ball_detector.debug_dark,
                            ball_detector.debug_appearance,
                            ball_detector.debug_candidates,
                            ball_detector.debug_hough,
                        )
                    )

    cam.close()


if __name__ == "__main__":
    main()
