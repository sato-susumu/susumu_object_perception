#!/usr/bin/env python3
"""Import two BSD-licensed VerifAI Webots city worlds for outdoor SLAM tests.

The upstream worlds are old Webots R2018b OSM-importer worlds. This importer
keeps their city layout, adds the EXTERNPROTO declarations required by current
Webots, removes old Scenic controllers, strips Road fields that R2025a skips,
and appends this package's TurtleBot3Burger sensor stack.
"""

import argparse
import os
import re
import urllib.request


VERIFAI_COMMIT = '6f4b8ee93af908d337b1fb136951aaacbed535fc'
BASE = (
    'https://raw.githubusercontent.com/BerkeleyLearnVerify/VerifAI/'
    f'{VERIFAI_COMMIT}'
)

WORLDS = {
    'outdoor_urban_shattuck.wbt': {
        'url': f'{BASE}/examples/webots/worlds/shattuck_build.wbt',
        'source': 'examples/webots/worlds/shattuck_build.wbt',
        'translation': (-6.0, 0.0, -110.0),
        'rotation': (0.0, 1.0, 0.0, 1.5708),
        'description': 'Downtown Berkeley/Shattuck OSM city blocks.',
    },
    'outdoor_urban_intersection.wbt': {
        'url': f'{BASE}/examples/webots/worlds/scenic_intersection.wbt',
        'source': 'examples/webots/worlds/scenic_intersection.wbt',
        'translation': (-28.0, 0.0, 25.0),
        'rotation': (0.0, 1.0, 0.0, 0.0),
        'description': 'Dense OSM urban intersection used by VerifAI Scenic.',
    },
}

LICENSE_URL = f'{BASE}/LICENSE'

EXTERNPROTOS = [
    'EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/backgrounds/protos/TexturedBackground.proto"',
    'EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/backgrounds/protos/TexturedBackgroundLight.proto"',
    'EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/floors/protos/Floor.proto"',
    'EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/road/protos/Crossroad.proto"',
    'EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/road/protos/Road.proto"',
    'EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/traffic/protos/PedestrianCrossing.proto"',
    'EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/traffic/protos/GenericTrafficLight.proto"',
    'EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/traffic/protos/TrafficCone.proto"',
    'EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/buildings/protos/SimpleBuilding.proto"',
    'EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/trees/protos/SimpleTree.proto"',
    'EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/develop/projects/robots/robotis/turtlebot/protos/TurtleBot3Burger.proto"',
]


def read_url(url):
    with urllib.request.urlopen(url, timeout=30) as response:
        return response.read().decode('utf-8')


def node_body(text, start):
    depth = 0
    for i in range(start, len(text)):
        ch = text[i]
        if ch == '{':
            depth += 1
        elif ch == '}':
            depth -= 1
            if depth == 0:
                return text[start:i + 1]
    return text[start:]


def remove_nodes(text, pattern):
    out = []
    pos = 0
    for match in re.finditer(pattern, text, re.M):
        start = match.start()
        brace = text.find('{', match.end() - 1)
        if brace < 0:
            continue
        out.append(text[pos:start])
        body = node_body(text, brace)
        pos = brace + len(body)
    out.append(text[pos:])
    return ''.join(out)


def strip_unsupported_road_fields(text):
    # R2025a Road skips these legacy fields. Removing them keeps loader logs
    # readable and makes failures easier to spot.
    text = re.sub(r'\n\s*dashedLine\s*\[[^\]]*\]', '', text, flags=re.S)
    text = re.sub(r'\n\s*texture\s*\[[^\]]*\]', '', text, flags=re.S)
    return text


def add_externprotos(text):
    lines = text.splitlines()
    header = lines[0]
    rest = '\n'.join(lines[1:]).lstrip('\n')
    return header + '\n\n' + '\n'.join(EXTERNPROTOS) + '\n\n' + rest + '\n'


def turtlebot_block(name, spec):
    tx, ty, tz = spec['translation']
    rx, ry, rz, ra = spec['rotation']
    return f'''

# === TurtleBot3 + MID360-compatible sensor stack (susumu_object_perception) ===
# Source world: BerkeleyLearnVerify/VerifAI {VERIFAI_COMMIT}
# Upstream path: {spec['source']}
# Description: {spec['description']}
TurtleBot3Burger {{
  translation {tx:.3f} {ty:.3f} {tz:.3f}
  rotation {rx:.6f} {ry:.6f} {rz:.6f} {ra:.6f}
  controller "<extern>"
  controllerArgs [ "" ]
  extensionSlot [
    Solid {{ name "imu_link" }}
    GPS {{ }}
    InertialUnit {{ name "inertial_unit" }}
    Camera {{
      translation 0 0 0.75
      rotation 0 1 0 1.5708
      name "omni_camera"
      fieldOfView 6.283185
      width 2048
      height 1024
      projection "cylindrical"
      antiAliasing TRUE
    }}
    Camera {{ translation 0.05 0 0.10 rotation 0 1 0 -0.15 name "camera" width 1920 height 1080 fieldOfView 1.02974 }}
    Lidar {{
      translation 0 0 0.20
      name "lidar3d"
      horizontalResolution 720
      fieldOfView 6.283185
      verticalFieldOfView 1.029744
      tiltAngle 0
      numberOfLayers 28
      near 0.1
      minRange 0.1
      maxRange 40.0
      type "rotating"
      pointCloud TRUE
    }}
  ]
}}
'''


def convert_world(raw, name, spec):
    text = raw.replace('\r\n', '\n')
    text = add_externprotos(text)
    text = strip_unsupported_road_fields(text)
    # Remove old Scenic/vehicle controllers. Static traffic cones and buildings
    # remain; ROS 2 controls only the added TurtleBot.
    text = remove_nodes(text, r'^DEF\s+\w+\s+(?:ToyotaPrius|BmwX5)\s*\{')
    text = remove_nodes(text, r'^Supervisor\s*\{')
    text = remove_nodes(text, r'^GenericTrafficLight\s*\{')
    text = text.rstrip() + turtlebot_block(name, spec)
    return text


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--output-dir',
        default=os.path.join(os.getcwd(), 'webots_worlds'),
        help='directory where converted .wbt files are written')
    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)
    for name, spec in WORLDS.items():
        raw = read_url(spec['url'])
        converted = convert_world(raw, name, spec)
        out_path = os.path.join(args.output_dir, name)
        with open(out_path, 'w', encoding='utf-8') as f:
            f.write(converted)
        print(f'wrote {out_path} ({converted.count(chr(10)) + 1} lines)')

    notice = (
        'VerifAI Webots world notice\n'
        '============================\n\n'
        f'Imported from BerkeleyLearnVerify/VerifAI commit {VERIFAI_COMMIT}.\n'
        'Source files:\n'
        '- examples/webots/worlds/shattuck_build.wbt\n'
        '- examples/webots/worlds/scenic_intersection.wbt\n\n'
        'Local changes are generated by scripts/import_verifai_outdoor_worlds.py:\n'
        '- add R2025a EXTERNPROTO declarations\n'
        '- remove old Scenic vehicle/Supervisor controllers\n'
        '- remove legacy Road dashedLine/texture fields skipped by R2025a\n'
        '- append TurtleBot3Burger with this package sensor stack\n\n'
    )
    license_text = read_url(LICENSE_URL)
    notice_path = os.path.join(args.output_dir, 'VERIFAI_BSD_LICENSE.txt')
    with open(notice_path, 'w', encoding='utf-8') as f:
        f.write(notice)
        f.write(license_text)
        if not license_text.endswith('\n'):
            f.write('\n')
    print(f'wrote {notice_path}')


if __name__ == '__main__':
    main()
