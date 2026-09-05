"""Generate competition environment configuration from Unity exports."""

import argparse
from collections import defaultdict
import json
import math
from pathlib import Path
import shutil
import sys
from typing import Any, Dict, List, Sequence, Tuple

from .occupancy_map import OccupancyMap
from .semantic_map import load, require_valid


Pose = Tuple[float, float, float]
Point = Tuple[float, float]
DEFAULT_LAYOUTS = ('LayoutA', 'LayoutB', 'LayoutC', 'LayoutD')


def _number(value: float) -> str:
    value = 0.0 if abs(value) < 0.0000005 else value
    return f'{value:.6f}'.rstrip('0').rstrip('.')


def _pose_yaml(pose: Pose, room: str = '') -> str:
    fields = []
    if room:
        fields.append(f'room: {room}')
    fields.extend((
        f'x: {_number(pose[0])}',
        f'y: {_number(pose[1])}',
        f'yaw: {_number(pose[2])}',
    ))
    return '{' + ', '.join(fields) + '}'


def _polygon(room: Dict[str, Any]) -> List[Point]:
    return [(float(item['x']), float(item['y'])) for item in room['polygon']]


def _center(points: Sequence[Point]) -> Point:
    return (
        sum(point[0] for point in points) / len(points),
        sum(point[1] for point in points) / len(points),
    )


def _inset_polygon(points: Sequence[Point], distance: float = 0.05) -> List[Point]:
    """Move vertices slightly toward the center to remove trigger overlap."""
    center = _center(points)
    result = []
    for point in points:
        length = _distance(point, center)
        if length <= distance:
            raise ValueError('room polygon is too small to inset safely')
        amount = distance / length
        result.append((
            point[0] + (center[0] - point[0]) * amount,
            point[1] + (center[1] - point[1]) * amount,
        ))
    return result


def _distance(left: Point, right: Point) -> float:
    return math.hypot(left[0] - right[0], left[1] - right[1])


def _boundary_distance(point: Point, polygon: Sequence[Point]) -> float:
    best = math.inf
    previous = len(polygon) - 1
    for current, end in enumerate(polygon):
        start = polygon[previous]
        delta_x = end[0] - start[0]
        delta_y = end[1] - start[1]
        length_squared = delta_x * delta_x + delta_y * delta_y
        if length_squared == 0.0:
            nearest = start
        else:
            amount = max(0.0, min(1.0, (
                (point[0] - start[0]) * delta_x
                + (point[1] - start[1]) * delta_y
            ) / length_squared))
            nearest = (
                start[0] + amount * delta_x,
                start[1] + amount * delta_y,
            )
        best = min(best, _distance(point, nearest))
        previous = current
    return best


def _search_poses(
    grid: OccupancyMap,
    polygon: Sequence[Point],
    count: int = 4,
) -> List[Pose]:
    candidates = [
        item for item in grid.safe_cells_in_polygon(polygon, 0.35)
        if _boundary_distance(grid.index_to_world(item), polygon) >= 0.25
    ]
    if not candidates:
        candidates = [
            item
            for item in grid.safe_cells_in_polygon(polygon, 0.20, 0.10)
            if _boundary_distance(grid.index_to_world(item), polygon) >= 0.10
        ]
    if not candidates:
        raise ValueError('room contains no collision-safe free search point')
    room_center = _center(polygon)
    positions = {index: grid.index_to_world(index) for index in candidates}
    first = max(
        candidates,
        key=lambda item: (
            grid.clearance(item) - 0.05 * _distance(positions[item], room_center)
        ),
    )
    selected = [first]
    while len(selected) < count:
        remaining = [item for item in candidates if item not in selected]
        if not remaining:
            break
        next_item = max(
            remaining,
            key=lambda item: (
                min(
                    _distance(positions[item], positions[chosen])
                    for chosen in selected
                ),
                grid.clearance(item),
            ),
        )
        separation = min(
            _distance(positions[next_item], positions[chosen])
            for chosen in selected
        )
        if separation < 0.80:
            break
        selected.append(next_item)
    result = []
    for index in selected:
        x, y = positions[index]
        yaw = math.atan2(room_center[1] - y, room_center[0] - x)
        result.append((x, y, yaw))
    return result


def _approach_poses(
    grid: OccupancyMap,
    polygon: Sequence[Point],
    destination: Dict[str, Any],
    count: int = 3,
) -> List[Pose]:
    target = (
        float(destination['pose']['x']),
        float(destination['pose']['y']),
    )
    bounds = destination.get('axis_aligned_bounds') or {}
    size = bounds.get('size') or {}
    object_radius = 0.5 * max(
        abs(float(size.get('x', 0.0))),
        abs(float(size.get('y', 0.0))),
    )
    preferred = max(0.65, object_radius + 0.50)
    minimum = max(0.45, object_radius + 0.30)
    maximum = preferred + 0.65
    candidates = [
        item
        for item in grid.safe_cells_in_polygon(polygon, 0.35, 0.10)
        if _boundary_distance(grid.index_to_world(item), polygon) >= 0.10
    ]
    candidates = [
        index for index in candidates
        if minimum <= _distance(grid.index_to_world(index), target) <= maximum
    ]
    if not candidates:
        candidates = [
            item
            for item in grid.safe_cells_in_polygon(polygon, 0.20, 0.10)
            if _boundary_distance(grid.index_to_world(item), polygon) >= 0.05
        ]
        candidates = [
            index for index in candidates
            if 0.35 <= _distance(grid.index_to_world(index), target)
            <= maximum + 0.50
        ]
    if not candidates:
        raise ValueError(
            f"destination {destination['id']!r} has no safe approach pose"
        )
    candidates.sort(key=lambda item: (
        abs(_distance(grid.index_to_world(item), target) - preferred),
        -grid.clearance(item),
    ))
    selected: List[int] = []
    selected_angles: List[float] = []
    for index in candidates:
        point = grid.index_to_world(index)
        angle = math.atan2(point[1] - target[1], point[0] - target[0])
        if all(
            abs(math.atan2(math.sin(angle - old), math.cos(angle - old)))
            >= math.radians(55.0)
            for old in selected_angles
        ):
            selected.append(index)
            selected_angles.append(angle)
            if len(selected) == count:
                break
    if len(selected) == 1:
        first_point = grid.index_to_world(selected[0])
        alternatives = [
            item for item in candidates
            if _distance(grid.index_to_world(item), first_point) >= 0.25
        ]
        if alternatives:
            selected.append(max(
                alternatives,
                key=lambda item: _distance(
                    grid.index_to_world(item), first_point
                ),
            ))
    result = []
    for index in selected:
        x, y = grid.index_to_world(index)
        result.append((x, y, math.atan2(target[1] - y, target[0] - x)))
    return result


def _safe_initial_pose(
    grid: OccupancyMap,
    raw_pose: Dict[str, Any],
    rooms: Sequence[Dict[str, Any]],
) -> Tuple[Pose, bool]:
    original = (
        float(raw_pose['x']),
        float(raw_pose['y']),
        float(raw_pose['yaw']),
    )
    original_index = grid.world_to_index(original[0], original[1])
    if grid.clearance(original_index) >= 0.25:
        return original, False
    candidates = []
    for room in rooms:
        candidates.extend(grid.safe_cells_in_polygon(_polygon(room), 0.25, 0.10))
    if not candidates:
        raise ValueError('no safe replacement for robot_initial_pose')
    nearest = min(
        candidates,
        key=lambda item: _distance(
            grid.index_to_world(item), (original[0], original[1])
        ),
    )
    x, y = grid.index_to_world(nearest)
    return (x, y, original[2]), True


def _render_environment(
    layout: str,
    document: Dict[str, Any],
    grid: OccupancyMap,
) -> Tuple[str, Dict[str, Any]]:
    rooms = {room['id']: room for room in document['rooms']}
    initial_pose, initial_snapped = _safe_initial_pose(
        grid, document['robot_initial_pose'], document['rooms']
    )
    lines = [
        f'internal_name: {document["environment"]}',
        'map: package://handyman_rebuild_ros2/maps/'
        f'{layout}/map.yaml',
        f'initial_pose: {_pose_yaml(initial_pose)}',
        'rooms:',
    ]
    room_components = {}
    search_poses_by_room = {}
    for room_name in sorted(rooms):
        polygon = _polygon(rooms[room_name])
        config_polygon = _inset_polygon(polygon)
        points = ', '.join(
            f'[{_number(point[0])}, {_number(point[1])}]'
            for point in config_polygon
        )
        search_poses = _search_poses(grid, polygon)
        search_poses_by_room[room_name] = search_poses
        lines.extend((
            f'  {room_name}:',
            f'    region: [{points}]',
            '    search_points:',
        ))
        lines.extend(f'      - {_pose_yaml(pose)}' for pose in search_poses)
        room_components[room_name] = grid.component(
            search_poses[0][0], search_poses[0][1]
        )

    grouped = defaultdict(list)
    approach_counts = {}
    for destination in document['destinations']:
        room_name = destination['room']
        poses = _approach_poses(
            grid, _polygon(rooms[room_name]), destination
        )
        semantic_class = destination['semantic_class']
        grouped[semantic_class].extend((room_name, pose) for pose in poses)
        approach_counts[destination['id']] = len(poses)
    lines.append('destinations:')
    for name in sorted(grouped):
        lines.append(f'  {name}:')
        for room_name, pose in grouped[name]:
            lines.append(f'    - {_pose_yaml(pose, room_name)}')

    robot_component = grid.component(initial_pose[0], initial_pose[1])
    disconnected = sorted(
        room for room, component in room_components.items()
        if component >= 0 and component != robot_component
    )
    report = {
        'layout': layout,
        'environment': document['environment'],
        'grid': {
            'width': grid.width,
            'height': grid.height,
            'resolution': grid.resolution,
            'free_components': sorted(grid.component_sizes, reverse=True),
        },
        'initial_pose_snapped': initial_snapped,
        'room_region_inset_m': 0.05,
        'robot_component': robot_component,
        'room_components': room_components,
        'rooms_disconnected_from_robot': disconnected,
        'search_point_counts': {
            name: len(search_poses_by_room[name]) for name in rooms
        },
        'destination_approach_counts': approach_counts,
    }
    return '\n'.join(lines) + '\n', report


def import_layouts(
    export_root: Path,
    package_root: Path,
    layouts: Sequence[str] = DEFAULT_LAYOUTS,
) -> Dict[str, Any]:
    export_root = Path(export_root).expanduser().resolve()
    package_root = Path(package_root).expanduser().resolve()
    config_root = package_root / 'config' / 'environments'
    maps_root = package_root / 'maps'
    if not (package_root / 'package.xml').is_file():
        raise ValueError(f'not a ROS package directory: {package_root}')
    config_root.mkdir(parents=True, exist_ok=True)
    maps_root.mkdir(parents=True, exist_ok=True)
    reports = []
    for layout in layouts:
        source = export_root / layout
        required = ('map.pgm', 'map.yaml', 'semantic_map.json')
        missing = [name for name in required if not (source / name).is_file()]
        if missing:
            raise ValueError(f'{layout} is missing: {", ".join(missing)}')
        document = load(str(source / 'semantic_map.json'))
        require_valid(document)
        if document['environment'] != layout:
            raise ValueError(
                f'{layout} contains environment {document["environment"]!r}'
            )
        grid = OccupancyMap(source / 'map.yaml')
        yaml_text, report = _render_environment(layout, document, grid)
        target = maps_root / layout
        target.mkdir(parents=True, exist_ok=True)
        for name in ('map.pgm', 'map.yaml', 'semantic_map.json', 'preview.png'):
            source_file = source / name
            if source_file.is_file():
                shutil.copy2(source_file, target / name)
        config_name = f'{layout[:6].lower()}_{layout[6:].lower()}.yaml'
        (config_root / config_name).write_text(yaml_text, encoding='utf-8')
        reports.append(report)
    aggregate = {
        'schema_version': 1,
        'source': str(export_root),
        'layouts': reports,
    }
    (maps_root / 'import_report.json').write_text(
        json.dumps(aggregate, indent=2, ensure_ascii=False) + '\n',
        encoding='utf-8',
    )
    return aggregate


def main(argv: Sequence[str] = None) -> int:
    parser = argparse.ArgumentParser(
        description='Import Unity Handyman maps into handyman_rebuild_ros2.'
    )
    parser.add_argument('export_root', help='directory containing LayoutA-D')
    parser.add_argument('package_root', help='handyman_rebuild_ros2 source dir')
    parser.add_argument(
        '--layouts', nargs='+', default=list(DEFAULT_LAYOUTS),
        help='layout directories to import (default: LayoutA LayoutB LayoutC LayoutD)',
    )
    args = parser.parse_args(argv)
    try:
        report = import_layouts(
            Path(args.export_root), Path(args.package_root), args.layouts
        )
    except (OSError, ValueError) as error:
        print(f'ERROR: {error}', file=sys.stderr)
        return 1
    for item in report['layouts']:
        disconnected = item['rooms_disconnected_from_robot']
        suffix = (
            ' disconnected rooms: ' + ', '.join(disconnected)
            if disconnected else ' all room centers connected'
        )
        print(
            f"Imported {item['layout']}: "
            f"{item['grid']['width']}x{item['grid']['height']};{suffix}"
        )
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
