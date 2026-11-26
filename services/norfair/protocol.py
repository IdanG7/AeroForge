"""MessagePack protocol for AeroForge ↔ Norfair communication."""

import msgpack
from typing import Dict, List, Any, Optional
import logging

logger = logging.getLogger(__name__)


class ProtocolError(Exception):
    """Protocol-related errors."""
    pass


class Protocol:
    """Handles MessagePack serialization/deserialization for the IPC protocol."""

    PROTOCOL_VERSION = 1

    @staticmethod
    def serialize_response(success: bool, detections: Optional[List[Dict[str, Any]]] = None,
                          error: Optional[str] = None) -> bytes:
        """
        Serialize a response message to MessagePack binary.

        Args:
            success: Whether the request was successful
            detections: List of detection dicts (if successful)
            error: Error message (if not successful)

        Returns:
            MessagePack encoded bytes
        """
        response = {
            "version": Protocol.PROTOCOL_VERSION,
            "success": success
        }

        if success and detections is not None:
            response["detections"] = detections
        elif not success and error is not None:
            response["error"] = error

        return msgpack.packb(response, use_bin_type=True)

    @staticmethod
    def deserialize_request(data: bytes) -> Dict[str, Any]:
        """
        Deserialize a request message from MessagePack binary.

        Args:
            data: MessagePack encoded bytes

        Returns:
            Parsed request dict

        Raises:
            ProtocolError: If message format is invalid
        """
        try:
            request = msgpack.unpackb(data, raw=False)
        except Exception as e:
            raise ProtocolError(f"Failed to deserialize MessagePack: {e}")

        if not isinstance(request, dict):
            raise ProtocolError("Request must be a dict")

        # Validate version (optional field for backwards compatibility)
        version = request.get("version", 1)
        if version != Protocol.PROTOCOL_VERSION:
            logger.warning(f"Protocol version mismatch: got {version}, expected {Protocol.PROTOCOL_VERSION}")

        # Validate required fields
        if "cmd" not in request:
            raise ProtocolError("Missing 'cmd' field in request")

        if request["cmd"] == "update":
            if "dt" not in request:
                raise ProtocolError("Missing 'dt' field in update request")
            if "detections" not in request:
                raise ProtocolError("Missing 'detections' field in update request")

        return request

    @staticmethod
    def validate_detection(detection: Dict[str, Any]) -> bool:
        """
        Validate a detection dict has required fields.

        Args:
            detection: Detection dict to validate

        Returns:
            True if valid, False otherwise
        """
        required_fields = ["id", "bbox", "centroid", "score"]

        if not all(field in detection for field in required_fields):
            return False

        # Validate bbox format [x, y, w, h]
        bbox = detection["bbox"]
        if not isinstance(bbox, (list, tuple)) or len(bbox) != 4:
            return False

        # Validate centroid format [cx, cy]
        centroid = detection["centroid"]
        if not isinstance(centroid, (list, tuple)) or len(centroid) != 2:
            return False

        # Validate score is a number
        if not isinstance(detection["score"], (int, float)):
            return False

        return True
