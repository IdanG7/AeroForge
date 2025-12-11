"""Norfair tracker wrapper for AeroForge."""

import numpy as np
from norfair import Detection, Tracker
from typing import List, Dict, Any, Optional, Callable
import logging

logger = logging.getLogger(__name__)


class NorfairTrackerWrapper:
    """Wraps Norfair tracker and handles format conversion."""

    def __init__(self, config: Dict[str, Any]):
        """
        Initialize Norfair tracker with configuration.

        Args:
            config: Tracker configuration dict
        """
        self.config = config

        # Extract Norfair parameters
        distance_function = config.get("distance_function", "euclidean")
        distance_threshold = config.get("distance_threshold", 30)
        hit_counter_max = config.get("hit_counter_max", 15)
        initialization_delay = config.get("initialization_delay", 0)
        pointwise_hit_counter_max = config.get("pointwise_hit_counter_max", 4)
        detection_threshold = config.get("detection_threshold", 0.0)
        past_detections_length = config.get("past_detections_length", 4)

        # Create distance function
        if distance_function == "euclidean":
            dist_func = self._euclidean_distance
        elif distance_function == "iou":
            dist_func = self._iou_distance
        else:
            logger.warning(f"Unknown distance function: {distance_function}, using euclidean")
            dist_func = self._euclidean_distance

        # Initialize Norfair tracker
        self.tracker = Tracker(
            distance_function=dist_func,
            distance_threshold=distance_threshold,
            hit_counter_max=hit_counter_max,
            initialization_delay=initialization_delay,
            pointwise_hit_counter_max=pointwise_hit_counter_max,
            detection_threshold=detection_threshold,
            past_detections_length=past_detections_length
        )

        logger.info(f"Norfair tracker initialized with distance_function={distance_function}, "
                   f"distance_threshold={distance_threshold}")

    def update(self, dt: float, detections: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """
        Update tracker with new detections and return tracked detections.

        Args:
            dt: Time delta since last update (seconds)
            detections: List of detection dicts from AeroForge

        Returns:
            List of tracked detection dicts in AeroForge format
        """
        # Convert AeroForge detections to Norfair format
        norfair_detections = [self._aeroforge_to_norfair(det) for det in detections]

        # Update Norfair tracker
        tracked_objects = self.tracker.update(detections=norfair_detections, period=int(dt * 1000))

        # Convert Norfair tracked objects back to AeroForge format
        tracked_detections = [self._norfair_to_aeroforge(obj) for obj in tracked_objects]

        logger.debug(f"Tracked {len(tracked_detections)} objects from {len(detections)} detections")

        return tracked_detections

    def _aeroforge_to_norfair(self, detection: Dict[str, Any]) -> Detection:
        """
        Convert AeroForge detection to Norfair format.

        Norfair works with points, so we convert bbox to 5 points:
        - Centroid (center point)
        - 4 corners (top-left, top-right, bottom-right, bottom-left)

        Args:
            detection: AeroForge detection dict

        Returns:
            Norfair Detection object
        """
        x, y, w, h = detection["bbox"]
        cx, cy = detection["centroid"]
        score = detection["score"]

        # Create numpy array of points: [centroid, TL, TR, BR, BL]
        points = np.array([
            [cx, cy],           # centroid
            [x, y],             # top-left
            [x + w, y],         # top-right
            [x + w, y + h],     # bottom-right
            [x, y + h]          # bottom-left
        ], dtype=np.float32)

        # All points have the same score
        scores = np.array([score] * 5, dtype=np.float32)

        return Detection(
            points=points,
            scores=scores,
            data={"original_id": detection["id"]}
        )

    def _norfair_to_aeroforge(self, tracked_obj) -> Dict[str, Any]:
        """
        Convert Norfair tracked object back to AeroForge format.

        Args:
            tracked_obj: Norfair TrackedObject

        Returns:
            AeroForge detection dict
        """
        # Extract points (estimated state from Kalman filter)
        points = tracked_obj.estimate  # shape: (5, 2)

        # First point is centroid
        centroid = points[0]

        # Reconstruct bbox from corners (points 1-4)
        corners = points[1:5]
        x_min = float(corners[:, 0].min())
        y_min = float(corners[:, 1].min())
        x_max = float(corners[:, 0].max())
        y_max = float(corners[:, 1].max())

        bbox = [
            int(x_min),
            int(y_min),
            int(x_max - x_min),
            int(y_max - y_min)
        ]

        # Get score from last detection
        if tracked_obj.last_detection is not None:
            score = float(np.mean(tracked_obj.last_detection.scores))
        else:
            score = 1.0

        return {
            "id": int(tracked_obj.id),  # Norfair's tracking ID
            "bbox": bbox,
            "centroid": [float(centroid[0]), float(centroid[1])],
            "score": score
        }

    @staticmethod
    def _euclidean_distance(detection, tracked_object):
        """Euclidean distance between detection and tracked object centroids."""
        # Use only the centroid (first point)
        return np.linalg.norm(detection.points[0] - tracked_object.estimate[0])

    @staticmethod
    def _iou_distance(detection, tracked_object):
        """IoU-based distance between detection and tracked object bboxes."""
        # Reconstruct bboxes from corners (points 1-4)
        def points_to_bbox(points):
            corners = points[1:5]
            x_min = corners[:, 0].min()
            y_min = corners[:, 1].min()
            x_max = corners[:, 0].max()
            y_max = corners[:, 1].max()
            return [x_min, y_min, x_max, y_max]

        bbox1 = points_to_bbox(detection.points)
        bbox2 = points_to_bbox(tracked_object.estimate)

        # Calculate IoU
        x1_min, y1_min, x1_max, y1_max = bbox1
        x2_min, y2_min, x2_max, y2_max = bbox2

        # Intersection
        x_inter_min = max(x1_min, x2_min)
        y_inter_min = max(y1_min, y2_min)
        x_inter_max = min(x1_max, x2_max)
        y_inter_max = min(y1_max, y2_max)

        inter_area = max(0, x_inter_max - x_inter_min) * max(0, y_inter_max - y_inter_min)

        # Union
        bbox1_area = (x1_max - x1_min) * (y1_max - y1_min)
        bbox2_area = (x2_max - x2_min) * (y2_max - y2_min)
        union_area = bbox1_area + bbox2_area - inter_area

        if union_area == 0:
            return 1.0  # Maximum distance

        iou = inter_area / union_area
        return 1.0 - iou  # Convert IoU to distance
