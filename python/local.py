import os
from pathlib import Path

class EnvironmentSettings:
    def __init__(self):
        # Base directory for pytracking
        self.workspace_dir = str(Path.home() / 'pytracking')

        # Directory for pre-trained networks
        self.network_path = os.path.join(self.workspace_dir, 'pytracking', 'networks')

        # Directory for results (tracking outputs, etc.)
        self.results_path = os.path.join(self.workspace_dir, 'pytracking', 'tracking_results')

        # Directory for segmentation results
        self.segmentation_path = os.path.join(self.workspace_dir, 'pytracking', 'segmentation_results')

        # Datasets directory (not needed for basic tracking)
        # Only needed if you want to run benchmark evaluations
        self.got10k_path = ''
        self.lasot_path = ''
        self.trackingnet_path = ''
        self.coco_path = ''
        self.lvis_path = ''
        self.youtubevos_dir = ''
        self.davis_dir = ''

        # Create directories if they don't exist
        os.makedirs(self.network_path, exist_ok=True)
        os.makedirs(self.results_path, exist_ok=True)
        os.makedirs(self.segmentation_path, exist_ok=True)
