#!/usr/bin/env python3
import os.path
import yaml

try:
    from yaml import CLoader as Loader, CDumper as Dumper
except ImportError:
    from yaml import Loader, Dumper


def get_app_status():
    data = {}

    if os.path.exists('status.yaml'):
        with open('status.yaml', 'r') as file:
            contents = '\n'.join(file.readlines())
            data = yaml.load(contents, Loader=Loader)

    return data


def get_core_status():
    data = {}
    with open('lib/core/status.yaml', 'r') as file:
        contents = '\n'.join(file.readlines())
        data = yaml.load(contents, Loader=Loader)

    return data


def extract_names(data, names, prefix):
    if isinstance(data, list):
        for item in data:
            names.append(f"{prefix}_{item.upper()}")
    else:
        for key in data.keys():
            key_part = key.upper()
            new_prefix = prefix + "_" + key_part
            extract_names(data[key], names, new_prefix)


def gen_statuses(data: dict):
    statuses = []
    counter = 0

    core_status: dict = data['core_status']
    print(core_status)
    names = []
    for key in core_status.keys():
        key_part = 'STATUS_' + key.upper()
        names = []
        extract_names(core_status[key], names, key_part)
    print(names)

if __name__ == "__main__":
    app_status = get_app_status()
    core_status = get_core_status()
    statuses = gen_statuses(app_status | core_status)
