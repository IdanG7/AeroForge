#!/usr/bin/env python3
"""Norfair tracking service for AeroForge.

This service runs a Unix domain socket server that receives detection data
from the AeroForge C++ application, tracks objects using Norfair, and returns
tracked detections.
"""

import socket
import os
import sys
import argparse
import logging
import signal
from pathlib import Path
import yaml

from protocol import Protocol, ProtocolError
from tracker import NorfairTrackerWrapper

# Global shutdown flag
shutdown_requested = False


def signal_handler(signum, frame):
    """Handle shutdown signals gracefully."""
    global shutdown_requested
    logging.info(f"Received signal {signum}, shutting down...")
    shutdown_requested = True


class NorfairService:
    """Unix domain socket server for Norfair tracking."""

    def __init__(self, socket_path: str, tracker_config: dict):
        """
        Initialize the Norfair service.

        Args:
            socket_path: Path to Unix domain socket
            tracker_config: Norfair tracker configuration
        """
        self.socket_path = socket_path
        self.tracker = NorfairTrackerWrapper(tracker_config)
        self.protocol = Protocol()
        self.server_socket = None

        logging.info(f"Initialized Norfair service with socket: {socket_path}")

    def start(self):
        """Start the socket server and begin handling requests."""
        # Remove socket file if it exists
        if os.path.exists(self.socket_path):
            os.remove(self.socket_path)
            logging.info(f"Removed existing socket file: {self.socket_path}")

        # Create Unix domain socket
        self.server_socket = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.server_socket.bind(self.socket_path)
        self.server_socket.listen(1)

        # Set socket permissions (allow owner read/write)
        os.chmod(self.socket_path, 0o600)

        logging.info(f"Listening on {self.socket_path}")

        try:
            while not shutdown_requested:
                logging.info("Waiting for connection...")
                try:
                    # Accept connection with timeout
                    self.server_socket.settimeout(1.0)
                    client_socket, _ = self.server_socket.accept()
                    logging.info("Client connected")

                    # Handle this client until disconnection
                    self._handle_client(client_socket)

                except socket.timeout:
                    continue  # Check shutdown flag
                except Exception as e:
                    if not shutdown_requested:
                        logging.error(f"Error accepting connection: {e}")

        finally:
            self._cleanup()

    def _handle_client(self, client_socket):
        """
        Handle requests from a connected client.

        Args:
            client_socket: Client socket connection
        """
        try:
            while not shutdown_requested:
                # Receive message length (4 bytes, big-endian)
                length_bytes = self._recv_exact(client_socket, 4)
                if not length_bytes:
                    logging.info("Client disconnected")
                    break

                msg_length = int.from_bytes(length_bytes, byteorder='big')

                # Validate message length (max 1MB for safety)
                if msg_length <= 0 or msg_length > 1024 * 1024:
                    logging.error(f"Invalid message length: {msg_length}")
                    break

                # Receive message payload
                msg_data = self._recv_exact(client_socket, msg_length)
                if not msg_data:
                    logging.error("Failed to receive complete message")
                    break

                # Process request and send response
                response = self._process_request(msg_data)
                self._send_response(client_socket, response)

        except Exception as e:
            logging.error(f"Error handling client: {e}", exc_info=True)
        finally:
            client_socket.close()
            logging.info("Client connection closed")

    def _recv_exact(self, sock, n):
        """
        Receive exactly n bytes from socket.

        Args:
            sock: Socket to receive from
            n: Number of bytes to receive

        Returns:
            Received bytes or None if connection closed
        """
        data = bytearray()
        while len(data) < n:
            try:
                packet = sock.recv(n - len(data))
                if not packet:
                    return None  # Connection closed
                data.extend(packet)
            except socket.timeout:
                if shutdown_requested:
                    return None
                continue
        return bytes(data)

    def _process_request(self, msg_data: bytes) -> bytes:
        """
        Process a request message and generate response.

        Args:
            msg_data: Raw message bytes

        Returns:
            Response message bytes
        """
        try:
            # Deserialize request
            request = self.protocol.deserialize_request(msg_data)

            cmd = request["cmd"]

            if cmd == "update":
                # Extract parameters
                dt = request["dt"]
                detections = request["detections"]

                # Validate detections
                for det in detections:
                    if not self.protocol.validate_detection(det):
                        return self.protocol.serialize_response(
                            success=False,
                            error="Invalid detection format"
                        )

                # Update tracker
                tracked_detections = self.tracker.update(dt, detections)

                # Return successful response
                return self.protocol.serialize_response(
                    success=True,
                    detections=tracked_detections
                )

            elif cmd == "ping":
                # Health check
                return self.protocol.serialize_response(success=True, detections=[])

            else:
                return self.protocol.serialize_response(
                    success=False,
                    error=f"Unknown command: {cmd}"
                )

        except ProtocolError as e:
            logging.error(f"Protocol error: {e}")
            return self.protocol.serialize_response(success=False, error=str(e))
        except Exception as e:
            logging.error(f"Error processing request: {e}", exc_info=True)
            return self.protocol.serialize_response(success=False, error=str(e))

    def _send_response(self, sock, response: bytes):
        """
        Send response message to client.

        Args:
            sock: Client socket
            response: Response message bytes
        """
        # Send message length (4 bytes, big-endian)
        length_bytes = len(response).to_bytes(4, byteorder='big')
        sock.sendall(length_bytes)

        # Send message payload
        sock.sendall(response)

    def _cleanup(self):
        """Clean up resources on shutdown."""
        if self.server_socket:
            self.server_socket.close()

        if os.path.exists(self.socket_path):
            os.remove(self.socket_path)
            logging.info(f"Removed socket file: {self.socket_path}")

        logging.info("Service shut down cleanly")


def load_config(config_path: str) -> dict:
    """Load configuration from YAML file."""
    with open(config_path, 'r') as f:
        return yaml.safe_load(f)


def setup_logging(level: str):
    """Configure logging."""
    log_format = "%(asctime)s - %(name)s - %(levelname)s - %(message)s"
    logging.basicConfig(
        level=getattr(logging, level.upper()),
        format=log_format,
        handlers=[
            logging.StreamHandler(sys.stdout)
        ]
    )


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(description="Norfair tracking service for AeroForge")
    parser.add_argument(
        "--config",
        type=str,
        default="config.yaml",
        help="Path to configuration file (default: config.yaml)"
    )
    args = parser.parse_args()

    # Load configuration
    config = load_config(args.config)

    # Setup logging
    log_level = config.get("logging", {}).get("level", "INFO")
    setup_logging(log_level)

    # Register signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # Extract configuration
    socket_config = config.get("socket", {})
    tracker_config = config.get("tracker", {})

    socket_path = socket_config.get("path", "/tmp/aeroforge_norfair.sock")

    # Create and start service
    service = NorfairService(socket_path, tracker_config)

    logging.info("Starting Norfair service...")
    logging.info("Press Ctrl+C to stop")

    try:
        service.start()
    except Exception as e:
        logging.error(f"Fatal error: {e}", exc_info=True)
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
