"""
Track loading and trajectory parsing module for kinematic bicycle simulation.
Pure Python module without mandatory ROS dependencies for easy unit testing.
"""

import csv
import math
import os

try:
    from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
except ImportError:
    get_package_share_directory = None

    class PackageNotFoundError(Exception):
        pass


DEFAULT_TRACK_FILE = 'centerline_0.csv'


class Track:
    """Represents a racetrack loaded from a CSV file."""

    def __init__(self, track_file=None, trajectory_type='centerline', close_loop=True):
        self.track_file = track_file or DEFAULT_TRACK_FILE
        self.trajectory_type = trajectory_type  # 'centerline', 'sp', 'iqp'
        self.close_loop = close_loop

        self.file_path = self.resolve_track_path(self.track_file)
        if not self.file_path or not os.path.isfile(self.file_path):
            raise FileNotFoundError(f"Track file '{self.track_file}' could not be resolved.")

        if not self.file_path.lower().endswith('.csv'):
            raise ValueError(f"Track file '{self.track_file}' must be a CSV file.")

        self.raw_data = {}
        self.waypoints = []
        self._load_csv_track()

    @staticmethod
    def get_search_paths(file_name):
        """Returns candidate paths to search for the track file."""
        candidates = []
        if os.path.isabs(file_name):
            return [file_name]

        # 1. Package share directory (installed)
        if get_package_share_directory is not None:
            for pkg in ('track_environment', 'kinematic_bicycle'):
                try:
                    share_dir = get_package_share_directory(pkg)
                    candidates.append(os.path.join(share_dir, 'tracks', file_name))
                    candidates.append(os.path.join(share_dir, file_name))
                except PackageNotFoundError:
                    pass

        # 2. Module relative directory (source tree)
        module_dir = os.path.dirname(os.path.abspath(__file__))
        package_root = os.path.abspath(os.path.join(module_dir, '..'))
        candidates.append(os.path.join(package_root, 'tracks', file_name))
        candidates.append(os.path.join(module_dir, 'tracks', file_name))
        candidates.append(os.path.join(module_dir, file_name))

        cwd = os.getcwd()
        candidates.append(os.path.join(cwd, 'tracks', file_name))
        candidates.append(os.path.join(cwd, file_name))
        candidates.append(os.path.join(cwd, 'src', 'track_environment', 'tracks', file_name))
        candidates.append(os.path.join(cwd, 'src', 'kinematic_bicycle', 'tracks', file_name))
        return candidates

    @classmethod
    def resolve_track_path(cls, file_name):
        """Finds the first existing candidate file path."""
        for path in cls.get_search_paths(file_name):
            if os.path.isfile(path):
                return path
        return None

    def _load_csv_track(self):
        """Extracts waypoints from a CSV file and trackbounds from random_track0.csv."""
        with open(self.file_path, 'r') as f:
            reader = csv.DictReader(f)
            if not reader.fieldnames or not {'x', 'y'}.issubset(reader.fieldnames):
                raise ValueError(
                    f"Track CSV '{self.track_file}' must contain 'x' and 'y' columns."
                )
            for row in reader:
                x = float(row.get('x', 0.0))
                y = float(row.get('y', 0.0))
                psi = float(row.get('psi', 0.0) if row.get('psi') else 0.0)
                kappa = float(row.get('kappa', 0.0) if row.get('kappa') else 0.0)
                s = float(row.get('s', 0.0) if row.get('s') else 0.0)
                vx = float(row.get('vx', 0.0) if row.get('vx') else 0.0)
                ax = float(row.get('ax', 0.0) if row.get('ax') else 0.0)

                self.waypoints.append({
                    'x': x,
                    'y': y,
                    'psi': psi,
                    'kappa': kappa,
                    's': s,
                    'vx': vx,
                    'ax': ax
                })

            if len(self.waypoints) < 2:
                raise ValueError(
                    f"Track CSV '{self.track_file}' contains fewer than 2 waypoints."
                )

        # Calculate or refine headings if missing/zero
        n = len(self.waypoints)
        for i in range(n):
            if self.waypoints[i]['psi'] == 0.0:
                next_i = (i + 1) % n
                dx = self.waypoints[next_i]['x'] - self.waypoints[i]['x']
                dy = self.waypoints[next_i]['y'] - self.waypoints[i]['y']
                self.waypoints[i]['psi'] = math.atan2(dy, dx)

        # Close the loop if requested
        if self.close_loop and n > 0:
            first = self.waypoints[0].copy()
            last = self.waypoints[-1]
            dist_close = math.hypot(first['x'] - last['x'], first['y'] - last['y'])
            if dist_close > 1e-4:
                first['s'] = last['s'] + dist_close
                self.waypoints.append(first)

        # Load boundaries from random_track0.csv
        bounds_file = self.resolve_track_path('random_track0.csv')
        self.trackbounds_markers = []
        if bounds_file and os.path.isfile(bounds_file):
            with open(bounds_file, 'r') as f:
                reader = csv.reader(f)
                marker_id = 0
                for row in reader:
                    if len(row) < 3:
                        continue
                    color_name = row[0]
                    try:
                        cx = float(row[1])
                        cy = float(row[2])
                    except ValueError:
                        continue

                    r, g, b = 1.0, 1.0, 1.0
                    if 'blue' in color_name:
                        r, g, b = 0.0, 0.0, 1.0
                    elif 'yellow' in color_name:
                        r, g, b = 1.0, 1.0, 0.0
                    elif 'orange' in color_name:
                        r, g, b = 1.0, 0.5, 0.0

                    marker = {
                        "header": {"frame_id": "map"},
                        "id": marker_id,
                        "type": 2,  # SPHERE
                        "pose": {
                            "position": {"x": cx, "y": cy, "z": 0.0},
                            "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
                        },
                        "scale": {"x": 0.2, "y": 0.2, "z": 0.2},
                        "color": {"r": r, "g": g, "b": b, "a": 1.0}
                    }
                    self.trackbounds_markers.append(marker)
                    marker_id += 1

    @property
    def x(self):
        return [wp['x'] for wp in self.waypoints]

    @property
    def y(self):
        return [wp['y'] for wp in self.waypoints]

    @property
    def psi(self):
        return [wp['psi'] for wp in self.waypoints]

    @property
    def vx(self):
        return [wp['vx'] for wp in self.waypoints]

    @property
    def kappa(self):
        return [wp['kappa'] for wp in self.waypoints]

    @property
    def total_length(self):
        """Calculates total perimeter along the waypoints."""
        total = 0.0
        for i in range(len(self.waypoints) - 1):
            total += math.hypot(
                self.waypoints[i + 1]['x'] - self.waypoints[i]['x'],
                self.waypoints[i + 1]['y'] - self.waypoints[i]['y']
            )
        return total

    @property
    def start_pose(self):
        """Returns (x, y, psi) of the start waypoint."""
        if not self.waypoints:
            return 0.0, 0.0, 0.0
        return self.waypoints[0]['x'], self.waypoints[0]['y'], self.waypoints[0]['psi']
