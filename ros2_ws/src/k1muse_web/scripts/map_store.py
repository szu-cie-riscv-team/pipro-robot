"""Read the newest complete C04 PGM map and its matching C07 point file."""

from dataclasses import dataclass
import hashlib
import math
import os
from pathlib import Path
import tempfile

import yaml


MAX_PIXELS = 16_000_000
MAX_POINTS = 100


@dataclass(frozen=True)
class SavedMap:
    yaml_path: Path
    map_id: str
    width: int
    height: int
    resolution: float
    origin: tuple
    pixels: bytes  # PGM image rows run from top to bottom.
    file_signature: tuple = ()

    @property
    def points_path(self):
        return self.yaml_path.with_name(
            f'{self.yaml_path.stem}.{self.map_id[:12]}.points.yaml')


def _pgm(raw):
    if not raw.startswith(b'P5'):
        raise ValueError('only binary P5 PGM maps are supported')
    pos = 2
    tokens = []
    for _ in range(3):
        while pos < len(raw):
            if raw[pos] == 35:  # # comment
                end = raw.find(b'\n', pos)
                pos = len(raw) if end < 0 else end + 1
            elif raw[pos] in b' \t\r\n':
                pos += 1
            else:
                break
        start = pos
        while pos < len(raw) and raw[pos] not in b' \t\r\n#':
            pos += 1
        if start == pos:
            raise ValueError('incomplete PGM header')
        tokens.append(int(raw[start:pos]))
    width, height, maximum = tokens
    if width <= 0 or height <= 0 or width * height > MAX_PIXELS or maximum != 255:
        raise ValueError('unsupported PGM size or depth')
    if pos >= len(raw) or raw[pos] not in b' \t\r\n':
        raise ValueError('missing PGM separator')
    pos += 2 if raw[pos:pos + 2] == b'\r\n' else 1
    pixels = raw[pos:]
    if len(pixels) != width * height:
        raise ValueError('incomplete PGM image')
    return width, height, pixels


def _interpretation(document):
    """Normalize the PGM interpretation used by the fixed Nav2 map loader."""
    mode = str(document.get('mode', 'trinary')).lower()
    if mode not in ('trinary', 'scale', 'raw'):
        raise ValueError('unsupported map mode')
    value = str(document['negate']).lower()
    negate = value == 'true' if value in ('true', 'false') else bool(int(value))
    occupied, free = (float(document[key]) for key in ('occupied_thresh', 'free_thresh'))
    if not (math.isfinite(occupied) and math.isfinite(free) and 0 <= free < occupied <= 1):
        raise ValueError('invalid map thresholds')
    return mode, negate, occupied, free


def _map_id(pixels, width, height, resolution, origin, interpretation):
    identity = hashlib.sha256(pixels)
    identity.update(repr((width, height, resolution, origin)).encode('ascii'))
    # Preserve existing sidecars for the project's map_saver defaults only.
    # Never fall back to a legacy ID for another interpretation.
    if interpretation != ('trinary', False, 0.65, 0.25):
        identity.update(b'\0k1-map-interpretation-v1\0')
        identity.update(repr(interpretation).encode('ascii'))
    return identity.hexdigest()


def _candidate(yaml_path):
    document = yaml.safe_load(yaml_path.read_text(encoding='utf-8'))
    if not isinstance(document, dict):
        raise ValueError('invalid map YAML')
    image = document.get('image')
    if not isinstance(image, str) or not image.lower().endswith('.pgm'):
        raise ValueError('map image must be PGM')
    interpretation = _interpretation(document)
    image_path = (yaml_path.parent / image).resolve()
    resolution = float(document['resolution'])
    origin = tuple(float(value) for value in document['origin'])
    if (len(origin) != 3 or not math.isfinite(resolution) or resolution <= 0 or
            not all(math.isfinite(value) for value in origin)):
        raise ValueError('invalid map geometry')
    yaml_stat, image_stat = yaml_path.stat(), image_path.stat()
    stamp = max(yaml_stat.st_mtime_ns, image_stat.st_mtime_ns)
    signature = (str(yaml_path.resolve()), str(image_path),
                 *((stat.st_ino, stat.st_size, stat.st_mtime_ns, stat.st_ctime_ns)
                   for stat in (yaml_stat, image_stat)))
    return stamp, image_path, resolution, origin, signature, interpretation


def latest_map(directory, cached=None):
    directory = Path(directory)
    candidates = []
    for yaml_path in directory.glob('*.yaml'):
        if '.points.' in yaml_path.name:
            continue
        try:
            stamp, image_path, resolution, origin, signature, interpretation = _candidate(yaml_path)
            candidates.append((stamp, yaml_path.name, yaml_path, image_path,
                               resolution, origin, signature, interpretation))
        except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError):
            continue
    for _, _, yaml_path, image_path, resolution, origin, signature, interpretation in sorted(
            candidates, reverse=True):
        try:
            if cached is not None and cached.file_signature == signature:
                return cached
            width, height, pixels = _pgm(image_path.read_bytes())
            identity = _map_id(pixels, width, height, resolution, origin, interpretation)
            return SavedMap(yaml_path, identity, width, height,
                            resolution, origin, pixels, signature)
        except (OSError, TypeError, ValueError):
            continue
    raise FileNotFoundError('no complete saved PGM map')


def _validated_points(points):
    if not isinstance(points, dict) or len(points) > MAX_POINTS:
        raise ValueError('invalid point list')
    clean = {}
    for name, pose in points.items():
        if not isinstance(name, str) or not 1 <= len(name) <= 64:
            raise ValueError('invalid point name')
        if not isinstance(pose, dict):
            raise ValueError('invalid point pose')
        try:
            values = {key: float(pose[key]) for key in ('x', 'y', 'yaw')}
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError('invalid point pose') from exc
        if not all(math.isfinite(value) for value in values.values()):
            raise ValueError('non-finite point pose')
        clean[name] = values
    return clean


def load_points(saved_map):
    path = saved_map.points_path
    if not path.exists():
        return {}
    document = yaml.safe_load(path.read_text(encoding='utf-8'))
    if not isinstance(document, dict) or document.get('map_id') != saved_map.map_id:
        raise ValueError('point file does not match map revision')
    return _validated_points(document.get('tasks'))


def save_points(saved_map, map_id, points):
    if map_id != saved_map.map_id:
        raise ValueError('map changed; refresh before saving points')
    clean = _validated_points(points)
    path = saved_map.points_path
    document = {'map_id': saved_map.map_id, 'tasks': clean}
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(
                mode='w', encoding='utf-8', dir=path.parent,
                prefix='.' + path.name + '.', suffix='.tmp',
                delete=False) as file:
            temporary = Path(file.name)
            yaml.safe_dump(document, file, allow_unicode=True, sort_keys=True)
            file.flush()
            os.fsync(file.fileno())
        os.replace(temporary, path)
    finally:
        if temporary is not None and temporary.exists():
            temporary.unlink()
    return clean
