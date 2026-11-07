#!/usr/bin/env python3
"""One request per process. Use shared map identity and matching sidecars."""
import json
from pathlib import Path
import re
import sys
from map_store import SavedMap, _candidate, _pgm, _map_id, load_points, save_points
from warehouse_store import parse_warehouse_arguments, warehouse_get, warehouse_sync, _keys


def read_map(path):
    _, image, resolution, origin, signature, interpretation = _candidate(path)
    width, height, pixels = _pgm(image.read_bytes())
    identity = _map_id(pixels, width, height, resolution, origin, interpretation)
    return SavedMap(path, identity, width, height,
                    resolution, origin, pixels, signature)


def maps(directory):
    result = []
    for path in sorted(Path(directory).glob('*.yaml')):
        if '.points.' in path.name:
            continue
        try:
            result.append(read_map(path))
        except Exception:
            continue  # An incomplete map is not selectable.
    return result


def execute(directory, operation, args):
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    if operation == 'save_target':
        name = args['name']
        if not isinstance(name, str) or not re.fullmatch(r'[\w-]{1,64}', name):
            raise ValueError('地图名称限 1～64 个字母、数字、中文、下划线或横线')
        target = directory / name
        if not args.get('overwrite', False) and any(
                target.with_suffix(suffix).exists() for suffix in ('.yaml', '.pgm')):
            raise ValueError('地图已存在，需要明确勾选覆盖')
        return {'target': str(target)}
    if operation == 'saved_map':
        saved = read_map(directory / (args['name'] + '.yaml'))
        try:
            if args.get('initialize_mapping_origin') is True and not saved.points_path.exists():
                save_points(saved, saved.map_id, {'建图起点': {'x': 0., 'y': 0., 'yaw': 0.}})
            tasks = load_points(saved)
        except Exception as error:
            raise ValueError(f'地图已保存、标点未完成: {error}') from error
        return {'map_id': saved.map_id, 'name': saved.yaml_path.stem, 'tasks': tasks}
    available = maps(directory)
    if operation == 'list_maps':
        return {'maps': [{'name': m.yaml_path.stem, 'map_id': m.map_id,
                          'path': str(m.yaml_path), 'resolution': m.resolution,
                          'width': m.width, 'height': m.height} for m in available]}
    selected = next((m for m in available if m.map_id == args['map_id']), None)
    if selected is None:
        raise ValueError('map_changed: 地图不存在或内容已改变，请刷新地图列表')
    if operation == 'warehouse_get':
        _keys(args, ('map_id',))
        return warehouse_get(selected)
    if operation == 'warehouse_sync':
        _keys(args, ('map_id', 'base_revision', 'document'))
        return warehouse_sync(selected, args['base_revision'], args['document'])
    if operation == 'select_map':
        return {'map_id': selected.map_id, 'path': str(selected.yaml_path),
                'tasks': load_points(selected)}
    if operation == 'get_points':
        return {'map_id': selected.map_id, 'tasks': load_points(selected)}
    if operation == 'save_points':
        return {'map_id': selected.map_id, 'tasks': save_points(
            selected, args['map_id'], args['tasks'])}
    raise ValueError('unsupported file operation')


if __name__ == '__main__':
    try:
        args = (parse_warehouse_arguments(sys.argv[3]) if sys.argv[2] in
                ('warehouse_get', 'warehouse_sync') else json.loads(sys.argv[3]))
        print(json.dumps(execute(sys.argv[1], sys.argv[2], args),
                         ensure_ascii=False, allow_nan=False, separators=(',', ':')))
    except Exception as error:
        print(json.dumps({'error': str(error)}, ensure_ascii=False))
        sys.exit(1)
