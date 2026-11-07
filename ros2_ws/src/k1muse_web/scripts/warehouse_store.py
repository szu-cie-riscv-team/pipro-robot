"""Warehouse validation and one-shot map-sidecar persistence (P01/P03).

No ROS imports, schema framework, background work or calibration defaults.
"""

import json
import math
import os
from pathlib import Path
import re
import tempfile

from map_store import load_points


MAX_SAFE_INTEGER = 9007199254740991
MAX_WAREHOUSE_BYTES = 65536
IDENTITY = re.compile(r'[A-Za-z0-9_-]{1,32}\Z')
MAP_ID = re.compile(r'[0-9a-f]{64}\Z')


def _invalid(field):
    raise ValueError('warehouse_invalid: ' + field)


def _keys(value, names):
    if not isinstance(value, dict) or set(value) != set(names):
        _invalid('object fields: ' + ','.join(names))


def _string(value):
    if not isinstance(value, str):
        _invalid('string')
    # UTF-8 payloads cannot contain isolated JSON surrogate escapes.
    try:
        value.encode('utf-8')
    except UnicodeEncodeError:
        _invalid('UTF-8 string')
    return value


def _identity(value):
    if not IDENTITY.fullmatch(_string(value)):
        _invalid('identity')


def _number(value, positive=False):
    if (type(value) not in (int, float) or not math.isfinite(value) or
            (positive and value <= 0)):
        _invalid('finite number/range')
    return value


def _integer(value, minimum, maximum):
    _number(value)
    if value != math.floor(value) or not minimum <= value <= maximum:
        _invalid('integer/range')


def _pose(value):
    _keys(value, ('x_m', 'y_m', 'yaw_rad'))
    for number in value.values():
        _number(number)


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            _invalid('duplicate key: ' + key)
        result[key] = value
    return result


def parse_warehouse_document(text):
    """Parse strict JSON before duplicate object keys are lost."""
    document = json.loads(text, object_pairs_hook=_unique_object,
                          parse_constant=lambda value: _invalid(value))
    return validate_warehouse_document(document)


def validate_warehouse_document(document):
    """Validate source data; unfinished fetch values stay null, never defaulted.

    Returns the original document without sorting, renumbering or mutating it.
    A caller reading raw JSON must use parse_warehouse_document first.
    """
    _keys(document, ('schema_version', 'revision', 'map_id', 'layout'))
    if document['schema_version'] == 1:
        _invalid('schema_v1_requires_web_migration')
    _integer(document['schema_version'], 2, 2)
    _integer(document['revision'], 0, MAX_SAFE_INTEGER)
    if not MAP_ID.fullmatch(_string(document['map_id'])):
        _invalid('map_id')
    layout = document['layout']
    _keys(layout, ('anchor', 'shelves'))
    if not isinstance(layout['shelves'], list):
        _invalid('shelves')
    shelf_ids, qr_refs = set(), set()
    for shelf in layout['shelves']:
        _keys(shelf, ('id', 'name', 'width_m', 'depth_m', 'pose', 'levels'))
        _identity(shelf['id'])
        if shelf['id'] in shelf_ids:
            _invalid('duplicate shelf id')
        shelf_ids.add(shelf['id'])
        _string(shelf['name'])
        _number(shelf['width_m'], positive=True)
        _number(shelf['depth_m'], positive=True)
        _pose(shelf['pose'])
        if not isinstance(shelf['levels'], list):
            _invalid('levels')
        layers = set()
        for level in shelf['levels']:
            _keys(level, ('layer', 'height_m', 'drug', 'qr', 'fetch'))
            _integer(level['layer'], 1, 2147483647)
            if level['layer'] in layers:
                _invalid('duplicate layer')
            layers.add(level['layer'])
            _number(level['height_m'], positive=True)
            _string(level['drug'])
            qr = level['qr']
            _keys(qr, ('left', 'right'))
            for side in ('left', 'right'):
                marker = qr[side]
                if marker is None:
                    continue
                _keys(marker, ('x_m', 'z_m', 'marker_id') if 'marker_id' in marker else ('x_m', 'z_m'))
                if 'marker_id' in marker and marker['marker_id'] is not None:
                    if type(marker['marker_id']) is not int:
                        _invalid('marker_id integer')
                    _integer(marker['marker_id'], 0, 49)
                _number(marker['x_m'])
                _number(marker['z_m'], positive=True)
                if side == 'left':
                    qr_refs.add((shelf['id'], level['layer']))
            if qr['left'] and qr['right'] and qr['left']['x_m'] >= qr['right']['x_m']:
                _invalid('QR left/right order')
            fetch = level['fetch']
            if fetch is None:
                continue
            _keys(fetch, ('class_id', 'grasp', 'roi', 'search_min_m', 'search_max_m'))
            for name in ('class_id', 'grasp'):
                if fetch[name] is not None:
                    _string(fetch[name])
            if fetch['grasp'] not in (None, '', 'front', 'top'):
                _invalid('grasp')
            roi = fetch['roi']
            if roi is not None:
                if not isinstance(roi, list) or len(roi) != 4:
                    _invalid('roi')
                for value in roi:
                    _number(value)
                    if not 0 <= value <= 1:
                        _invalid('roi range')
                if not (roi[0] < roi[2] and roi[1] < roi[3]):
                    _invalid('roi order')
            for name in ('search_min_m', 'search_max_m'):
                if fetch[name] is not None:
                    _number(fetch[name])
                    if not 0 <= fetch[name] <= shelf['width_m']:
                        _invalid('search range')
            lo, hi = fetch['search_min_m'], fetch['search_max_m']
            if lo is not None and hi is not None and lo > hi:
                _invalid('search order')
    anchor = layout['anchor']
    if anchor is not None:
        _keys(anchor, ('shelf_id', 'layer', 'point_name', 'point_pose', 'camera_distance_m'))
        _identity(anchor['shelf_id'])
        _integer(anchor['layer'], 1, 2147483647)
        if not _string(anchor['point_name']):
            _invalid('point_name')
        _pose(anchor['point_pose'])
        _number(anchor['camera_distance_m'], positive=True)
        if (anchor['shelf_id'], anchor['layer']) not in qr_refs:
            _invalid('anchor QR reference')
    return document


def _migrate_v1(document):
    """Editor-only conversion. Right markers remain explicitly unconfigured."""
    _keys(document, ('schema_version', 'revision', 'map_id', 'layout'))
    if document['schema_version'] != 1:
        return validate_warehouse_document(document)
    _integer(document['schema_version'], 1, 1)
    for shelf in document['layout']['shelves']:
        for level in shelf['levels']:
            old = level['qr']
            if old is not None:
                _keys(old, ('cell', 'x_m', 'z_m'))
                _identity(old['cell'])
            level['qr'] = {'left': {'x_m': old['x_m'], 'z_m': old['z_m']} if old else None,
                           'right': None}
            fetch = level['fetch']
            if fetch is not None:
                _keys(fetch, ('class_id', 'color', 'grasp', 'roi', 'search_min_m', 'search_max_m'))
                if fetch['color'] is not None:
                    _string(fetch['color'])
                del fetch['color']
    document['schema_version'] = 2
    return validate_warehouse_document(document)


def parse_warehouse_arguments(text):
    if len(text.encode('utf-8')) > MAX_WAREHOUSE_BYTES:
        raise ValueError('warehouse_too_large')
    try:
        return json.loads(text, object_pairs_hook=_unique_object,
                          parse_constant=lambda value: _invalid(value))
    except (ValueError, UnicodeError) as error:
        raise ValueError('warehouse_invalid: ' + str(error)) from error


def warehouse_path(saved_map):
    return saved_map.yaml_path.with_name(
        f'{saved_map.yaml_path.stem}.{saved_map.map_id[:12]}.warehouse.json')


def load_warehouse(saved_map, for_edit=False):
    try:
        with warehouse_path(saved_map).open('rb') as file:
            raw = file.read(MAX_WAREHOUSE_BYTES + 1)
    except FileNotFoundError:
        return None
    except OSError as error:
        raise ValueError('warehouse_io_error: ' + str(error)) from error
    if len(raw) > MAX_WAREHOUSE_BYTES:
        raise ValueError('warehouse_too_large')
    try:
        document = json.loads(raw.decode('utf-8'), object_pairs_hook=_unique_object,
                              parse_constant=lambda value: _invalid(value))
        legacy = isinstance(document, dict) and document.get('schema_version') == 1
        document = _migrate_v1(document) if for_edit else validate_warehouse_document(document)
    except (ValueError, UnicodeError) as error:
        raise ValueError('warehouse_invalid: ' + str(error)) from error
    if document['map_id'] != saved_map.map_id:
        raise ValueError('map_changed')
    return (document, legacy) if for_edit else document


def warehouse_get(saved_map):
    loaded = load_warehouse(saved_map, for_edit=True)
    document, legacy = loaded if loaded else (None, False)
    try:
        points = load_points(saved_map)
    except (OSError, ValueError) as error:
        raise ValueError('warehouse_io_error: ' + str(error)) from error
    return {'map_id': saved_map.map_id, 'document': document, 'migration_required': legacy,
            'synced_revision': document['revision'] if document else 0, 'points': points}


def warehouse_sync(saved_map, base_revision, document):
    _integer(base_revision, 0, MAX_SAFE_INTEGER - 1)
    _keys(document, ('schema_version', 'map_id', 'layout'))
    if document['map_id'] != saved_map.map_id:
        raise ValueError('map_changed')
    loaded = load_warehouse(saved_map, for_edit=True)
    current, legacy = loaded if loaded else (None, False)
    if base_revision != (current['revision'] if current else 0):
        raise ValueError('warehouse_revision_changed')
    candidate = dict(document, revision=base_revision + 1)
    validate_warehouse_document(candidate)
    anchor = candidate['layout']['anchor']
    if anchor is not None:
        try:
            saved = load_points(saved_map).get(anchor['point_name'])
        except (OSError, ValueError) as error:
            raise ValueError('warehouse_io_error: ' + str(error)) from error
        expected = anchor['point_pose']
        if (saved is None or abs(saved['x'] - expected['x_m']) > 1e-9 or
                abs(saved['y'] - expected['y_m']) > 1e-9 or
                abs(math.remainder(saved['yaw'] - expected['yaw_rad'], 2 * math.pi)) > 1e-9):
            raise ValueError('warehouse_anchor_changed')
    raw = (json.dumps(candidate, ensure_ascii=False, allow_nan=False,
                      separators=(',', ':')) + '\n').encode('utf-8')
    if len(raw) > MAX_WAREHOUSE_BYTES:
        raise ValueError('warehouse_too_large')
    path, temporary = warehouse_path(saved_map), None
    try:
        if legacy:
            # Preserve original schema-1 bytes before the explicit sync replaces them.
            backup = path.with_name(path.name + '.v1-r' + str(base_revision) + '.bak')
            original = path.read_bytes()
            if backup.exists():
                if backup.read_bytes() != original:
                    raise ValueError('warehouse_migration_backup_conflict')
            else:
                with backup.open('xb') as file:
                    file.write(original)
                    file.flush()
                    os.fsync(file.fileno())
        with tempfile.NamedTemporaryFile(mode='wb', dir=path.parent,
                                         prefix='.' + path.name + '.', suffix='.tmp',
                                         delete=False) as file:
            temporary = Path(file.name)
            file.write(raw)
            file.flush()
            os.fsync(file.fileno())
        os.replace(temporary, path)
    except OSError as error:
        raise ValueError('warehouse_io_error: ' + str(error)) from error
    finally:
        if temporary is not None and temporary.exists():
            temporary.unlink()
    return {'map_id': saved_map.map_id, 'synced_revision': candidate['revision'],
            'restart_required': True}
