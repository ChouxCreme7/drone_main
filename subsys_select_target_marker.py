import cv2
import numpy

from parameters import RED, BLUE, RAD2DEG, DRONE_POS, Distance, Angle, ScreenPosition
from subsys_markers_detected import DetectedMarkersStatus
from typing import List


class MarkerStatus:
    """
    Contains data about the marker that is selected as the target to be reached
    """

    id: int = -1
    corners: List[ScreenPosition] = []

    # Origin axis
    center_pt: ScreenPosition = ScreenPosition((0, 0))
    # Horizontal axis
    top_pt: ScreenPosition = ScreenPosition((0, 0))
    bottom_pt: ScreenPosition = ScreenPosition((0, 0))
    # Vertical axis
    left_pt: ScreenPosition = ScreenPosition((0, 0))
    right_pt: ScreenPosition = ScreenPosition((0, 0))

    # Horizontal angle
    h_angle: Angle = Angle(0)
    # Vertical angle
    v_angle: Angle = Angle(0)
    # angle and distance between marker and drone
    m_angle: Angle = Angle(0)
    m_distance: Distance = Distance(0)

    height: Distance = Distance(0)
    width: Distance = Distance(0)

    @classmethod
    def reset(cls):
        cls.id = -1
        cls.corners = []
        cls.center_pt = ScreenPosition((0, 0))
        cls.top_pt = ScreenPosition((0, 0))
        cls.bottom_pt = ScreenPosition((0, 0))
        cls.left_pt = ScreenPosition((0, 0))
        cls.right_pt = ScreenPosition((0, 0))
        cls.h_angle = Angle(0)
        cls.v_angle = Angle(0)
        cls.m_angle = Angle(0)
        cls.m_distance = Distance(0)
        cls.height = Distance(0)
        cls.width = Distance(0)

    @classmethod
    def __get_dict__(cls) -> dict:
        ms: dict = {'id': cls.id,
                    'H_angle': int(cls.h_angle * RAD2DEG),
                    'v_angle': int(cls.v_angle * RAD2DEG),
                    'm_angle': int(cls.m_angle * RAD2DEG),
                    'm_distance': cls.m_distance,
                    'm_height': cls.height,
                    'm_width': cls.width}
        return ms


class SelectTargetMarker:
    """
    Selects the marker to reach first from the list of markers detected by the Tello onboard camera,
    then returns the corresponding MarkerStatus class filled with the position of the Tello relatively
    to this marker
    """
    marker_pos: ScreenPosition = (0.0, 0.0)
    offset: tuple = (0, 0)

    @classmethod
    def setup(cls):
        MarkerStatus.reset()

    @classmethod
    def run(cls, frame: numpy.ndarray, markers: type(DetectedMarkersStatus),
            offset: tuple = (0, 0)) -> type(MarkerStatus):

        target_marker_id, corners = cls._get_marker_with_min_id(markers)
        if target_marker_id == -1:
            MarkerStatus.reset()
            return MarkerStatus

        br, bl, tl, tr = corners[0], corners[1], corners[2], corners[3]
        center_pt = cls._get_midpoint([br, bl, tl, tr])
        # get symmetry axes
        left_pt = cls._get_midpoint([bl, tl])
        right_pt = cls._get_midpoint([br, tr])
        bottom_pt = cls._get_midpoint([br, bl])
        top_pt = cls._get_midpoint([tl, tr])

        height = cls._length_segment(bottom_pt, top_pt)
        width = cls._length_segment(left_pt, right_pt)

        h_angle = cls._angle_between(left_pt, right_pt)
        v_angle = cls._angle_between(top_pt, bottom_pt, vertical=True)

        cls.offset = (int(offset[0] * width), int(offset[1] * height))
        cls.marker_pos = ScreenPosition((center_pt[0] + cls.offset[0],
                                         center_pt[1] + cls.offset[1]))
        # DRONE_POS is a tuple (x, y) that represents the position of the UAV on the pygame display
        m_angle = cls._angle_between(DRONE_POS, cls.marker_pos, vertical=True)
        m_distance = cls._length_segment(DRONE_POS, cls.marker_pos)

        cls.draw(frame)

        # update output
        MarkerStatus.id = target_marker_id
        MarkerStatus.corners = corners
        MarkerStatus.center_pt = center_pt
        MarkerStatus.left_pt = left_pt
        MarkerStatus.right_pt = right_pt
        MarkerStatus.bottom_pt = bottom_pt
        MarkerStatus.top_pt = top_pt
        MarkerStatus.h_angle = h_angle
        MarkerStatus.v_angle = v_angle
        MarkerStatus.m_angle = m_angle
        MarkerStatus.m_distance = m_distance
        MarkerStatus.height = height
        MarkerStatus.width = width
        return MarkerStatus

    @staticmethod
    def _get_marker_with_min_id(markers: DetectedMarkersStatus) -> (int, List[ScreenPosition]):
        target_id = -1
        target_corners = []

        if markers.ids is None:
            return target_id, target_corners

        for i in range(len(markers.ids)):
            marker_id = markers.ids[i][0]
            if marker_id < target_id or target_id == -1:
                target_id = marker_id
                target_corners = markers.corners[i][0]

        return target_id, target_corners

    @staticmethod
    def _get_midpoint(corners: List[ScreenPosition]) -> ScreenPosition:
        # corners = [p1,p2,p3,p4] with pi = (xi, yi)
        xc = yc = 0
        n = len(corners)
        for x, y in corners:
            xc += x
            yc += y
        xc = int(xc / n)
        yc = int(yc / n)
        midpoint = (xc, yc)
        return ScreenPosition(midpoint)

    @staticmethod
    def _angle_between(p1: ScreenPosition, p2: ScreenPosition, vertical: bool = False) -> Angle:
        dx = p1[0] - p2[0]
        dy = p1[1] - p2[1]
        if not vertical:  # Angle between horizontal axis and segment (p1,p2)
            alpha = numpy.arctan(-dy / (dx + 0.000001))
            return Angle(alpha)
        else:  # Angle between vertical axis and segment (p1,p2)
            beta = numpy.arctan(-dx / (dy + 0.000001))
            return Angle(beta)

    @staticmethod
    def _length_segment(p1: ScreenPosition, p2: ScreenPosition) -> Distance:
        length = numpy.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)
        return Distance(length)

    @classmethod
    def draw(cls, frame: numpy.ndarray):
        if MarkerStatus.id == -1:
            return
        cv2.aruco.drawDetectedMarkers(frame,
                                      numpy.array([[MarkerStatus.corners]]),
                                      numpy.array([[MarkerStatus.id]]),
                                      borderColor=RED)
        cv2.line(frame,
                 MarkerStatus.top_pt,
                 MarkerStatus.bottom_pt,
                 RED, 2)
        cv2.line(frame,
                 MarkerStatus.left_pt,
                 MarkerStatus.right_pt,
                 RED, 2)

        top_pt_with_offset = tuple(numpy.array(MarkerStatus.top_pt) + numpy.array(cls.offset))
        bottom_pt_with_offset = tuple(numpy.array(MarkerStatus.bottom_pt) + numpy.array(cls.offset))
        left_pt_with_offset = tuple(numpy.array(MarkerStatus.left_pt) + numpy.array(cls.offset))
        right_pt_with_offset = tuple(numpy.array(MarkerStatus.right_pt) + numpy.array(cls.offset))

        cv2.line(frame,
                 top_pt_with_offset,
                 bottom_pt_with_offset,
                 RED, 2)
        cv2.line(frame,
                 left_pt_with_offset,
                 right_pt_with_offset,
                 RED, 2)

        if DRONE_POS[0] != 0:
            cv2.line(frame,
                     DRONE_POS,
                     cls.marker_pos,
                     BLUE, 2)
