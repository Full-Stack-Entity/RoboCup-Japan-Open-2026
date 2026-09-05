import json

from handyman_map_tools.import_unity_maps import import_layouts
from handyman_map_tools.occupancy_map import OccupancyMap


def _write_export(root):
    layout = root / 'LayoutA'
    layout.mkdir(parents=True)
    width = height = 120
    pixels = bytearray([254] * (width * height))
    for row in range(height):
        for column in range(width):
            if row in (0, height - 1) or column in (0, width - 1):
                pixels[row * width + column] = 0
    (layout / 'map.pgm').write_bytes(
        f'P5\n{width} {height}\n255\n'.encode('ascii') + pixels
    )
    (layout / 'map.yaml').write_text(
        'image: map.pgm\nresolution: 0.1\norigin: [0, 0, 0]\n'
        'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n',
        encoding='utf-8',
    )
    room_bounds = {
        'living_room': (1.0, 1.0, 5.0, 5.0),
        'kitchen': (6.0, 1.0, 10.0, 5.0),
        'bedroom': (1.0, 6.0, 5.0, 10.0),
        'lobby': (6.0, 6.0, 10.0, 10.0),
    }
    rooms = []
    for name, (x0, y0, x1, y1) in room_bounds.items():
        rooms.append({
            'id': name,
            'polygon': [
                {'x': x0, 'y': y0}, {'x': x1, 'y': y0},
                {'x': x1, 'y': y1}, {'x': x0, 'y': y1},
            ],
        })
    document = {
        'schema_version': 1,
        'environment': 'LayoutA',
        'coordinate_system': 'ros_map',
        'rooms': rooms,
        'destinations': [{
            'id': 'table#living_room',
            'semantic_class': 'table',
            'room': 'living_room',
            'pose': {'x': 3.0, 'y': 3.0, 'yaw': 0.0},
            'axis_aligned_bounds': {
                'size': {'x': 0.5, 'y': 0.5},
            },
        }],
        'object_spawn_candidates': [{
            'id': 'candidate01',
            'room': 'living_room',
            'pose': {'x': 2.0, 'y': 2.0, 'yaw': 0.0},
        }],
        'robot_initial_pose': {'x': 2.0, 'y': 2.0, 'yaw': 0.0},
        'messages': [],
    }
    (layout / 'semantic_map.json').write_text(
        json.dumps(document), encoding='utf-8'
    )


def test_occupancy_map_coordinate_round_trip(tmp_path):
    _write_export(tmp_path)
    grid = OccupancyMap(tmp_path / 'LayoutA' / 'map.yaml')
    index = grid.world_to_index(2.05, 3.05)
    assert grid.is_free(index)
    x, y = grid.index_to_world(index)
    assert abs(x - 2.05) < 1e-6
    assert abs(y - 3.05) < 1e-6


def test_import_generates_package_maps_and_navigation_poses(tmp_path):
    export_root = tmp_path / 'export'
    package_root = tmp_path / 'handyman_rebuild_ros2'
    package_root.mkdir()
    (package_root / 'package.xml').write_text('<package/>', encoding='utf-8')
    _write_export(export_root)

    report = import_layouts(export_root, package_root, ('LayoutA',))

    generated = (
        package_root / 'config' / 'environments' / 'layout_a.yaml'
    ).read_text(encoding='utf-8')
    assert 'internal_name: LayoutA' in generated
    assert 'package://handyman_rebuild_ros2/maps/LayoutA/map.yaml' in generated
    assert generated.count('search_points:') == 4
    assert 'room: living_room' in generated
    assert (package_root / 'maps' / 'LayoutA' / 'map.pgm').is_file()
    layout_report = report['layouts'][0]
    assert layout_report['rooms_disconnected_from_robot'] == []
    assert layout_report['destination_approach_counts'][
        'table#living_room'
    ] >= 2
