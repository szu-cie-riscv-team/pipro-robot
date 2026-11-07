#!/usr/bin/env python3
"""One explicit catalog read/write; no ROS, model, map or background work."""
import json
import os
from pathlib import Path
import sys
import tempfile

import yaml


def unique(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError('drug_catalog_invalid: duplicate key')
        result[key] = value
    return result


class CatalogLoader(yaml.SafeLoader):
    pass


def mapping(loader, node):
    return unique((loader.construct_object(k), loader.construct_object(v))
                  for k, v in node.value)


CatalogLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, mapping)


def normalize(document):
    if not isinstance(document, dict) or set(document) != {'drugs'} or not isinstance(document['drugs'], list):
        raise ValueError('drug_catalog_invalid: drugs array required')
    owners, result = {}, []

    def name(value):
        if not isinstance(value, str) or not value.strip(' \t\r\n\f\v'):
            raise ValueError('drug_catalog_invalid: nonempty string required')
        return value.strip(' \t\r\n\f\v')

    for item in document['drugs']:
        if not isinstance(item, dict) or set(item) not in ({'name'}, {'name', 'aliases'}):
            raise ValueError('drug_catalog_invalid: name/aliases required')
        canonical = name(item['name'])
        if canonical in owners:
            raise ValueError('drug_catalog_invalid: name conflict')
        owners[canonical] = canonical
        aliases = item.get('aliases', [])
        if not isinstance(aliases, list):
            raise ValueError('drug_catalog_invalid: aliases array required')
        row = {'name': canonical, 'aliases': []}
        for value in aliases:
            alias = name(value)
            if alias in owners:
                if owners[alias] != canonical:
                    raise ValueError('drug_catalog_invalid: alias conflict')
                continue
            owners[alias] = canonical
            row['aliases'].append(alias)
        result.append(row)
    return result


def load_catalog(path, default):
    source = Path(path)
    if not source.exists():
        source = Path(default)
    with source.open('rb') as stream:
        raw = stream.read(65537)
    if len(raw) > 65536:
        raise ValueError('drug_catalog_too_large')
    return normalize(yaml.load(raw.decode('utf-8'), Loader=CatalogLoader))


def save_catalog(path, document):
    drugs = normalize(document)
    content = yaml.safe_dump({'drugs': drugs}, allow_unicode=True, sort_keys=False).encode('utf-8')
    if len(content) > 65536:
        raise ValueError('drug_catalog_too_large')
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(dir=target.parent, prefix='.' + target.name, delete=False) as stream:
            temporary = stream.name
            stream.write(content)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, target)
    finally:
        if temporary and os.path.exists(temporary):
            os.unlink(temporary)
    return {'drugs': drugs}


def execute(path, default, operation, args):
    if operation == 'drug_catalog_get' and args == {}:
        try:
            return {'drugs': load_catalog(path, default), 'error': ''}
        except Exception as error:
            return {'drugs': None, 'error': str(error)}
    if operation == 'drug_catalog_save':
        return save_catalog(path, args)
    raise ValueError('unsupported catalog operation')


if __name__ == '__main__':
    try:
        if len(sys.argv[4].encode('utf-8')) > 65536:
            raise ValueError('drug_catalog_too_large')
        args = json.loads(sys.argv[4], object_pairs_hook=unique)
        result = execute(sys.argv[1], sys.argv[2], sys.argv[3], args)
        print(json.dumps(result, ensure_ascii=False, allow_nan=False))
    except Exception as error:
        print(json.dumps({'error': str(error)}, ensure_ascii=False))
        sys.exit(1)
