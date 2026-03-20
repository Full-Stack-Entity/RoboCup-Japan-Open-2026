#!/usr/bin/env python3

import argparse
import random
import shutil
from pathlib import Path


IMAGE_EXTENSIONS = {'.jpg', '.jpeg', '.png', '.bmp'}
SPLITS = ('train', 'val', 'test')


def parse_args():
    parser = argparse.ArgumentParser(
        description='Split raw capture sessions into YOLO train/val/test.'
    )
    parser.add_argument(
        '--raw-dir',
        default='training/handyman_yolo/data/raw',
        help='Directory containing per-session image/label folders.',
    )
    parser.add_argument(
        '--output-root',
        default='training/handyman_yolo/data',
        help='Dataset root containing images/ and labels/ split folders.',
    )
    parser.add_argument('--train-ratio', type=float, default=0.7)
    parser.add_argument('--val-ratio', type=float, default=0.2)
    parser.add_argument('--test-ratio', type=float, default=0.1)
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument(
        '--clean',
        action='store_true',
        help='Delete existing generated files in split directories first.',
    )
    return parser.parse_args()


def list_sessions(raw_dir: Path):
    sessions = []
    for session_dir in sorted(raw_dir.iterdir()):
        if not session_dir.is_dir():
            continue
        images_dir = session_dir / 'images'
        if not images_dir.is_dir():
            continue
        sessions.append(session_dir)
    return sessions


def split_sessions(sessions, train_ratio, val_ratio, test_ratio, seed):
    total = train_ratio + val_ratio + test_ratio
    if total <= 0:
        raise SystemExit('Split ratios must sum to a positive value.')

    normalized = [
        train_ratio / total,
        val_ratio / total,
        test_ratio / total,
    ]

    sessions = list(sessions)
    random.Random(seed).shuffle(sessions)

    n = len(sessions)
    n_train = int(round(n * normalized[0]))
    n_val = int(round(n * normalized[1]))
    n_train = min(n_train, n)
    n_val = min(n_val, n - n_train)
    n_test = n - n_train - n_val

    return {
        'train': sessions[:n_train],
        'val': sessions[n_train:n_train + n_val],
        'test': sessions[n_train + n_val:n_train + n_val + n_test],
    }


def clear_split_dir(path: Path):
    for child in path.iterdir():
        if child.name == '.gitkeep':
            continue
        if child.is_dir():
            shutil.rmtree(child)
        else:
            child.unlink()


def prepare_output_dirs(output_root: Path, clean: bool):
    for group in ('images', 'labels'):
        for split in SPLITS:
            split_dir = output_root / group / split
            split_dir.mkdir(parents=True, exist_ok=True)
            existing = [p for p in split_dir.iterdir() if p.name != '.gitkeep']
            if existing and not clean:
                raise SystemExit(
                    f'Split directory is not empty: {split_dir}\n'
                    'Re-run with --clean to regenerate the split.'
                )
            if clean:
                clear_split_dir(split_dir)


def copy_session(session_dir: Path, output_root: Path, split: str):
    images_dir = session_dir / 'images'
    labels_dir = session_dir / 'labels'

    image_paths = sorted(
        path for path in images_dir.iterdir()
        if path.is_file() and path.suffix.lower() in IMAGE_EXTENSIONS
    )

    copied = 0
    for image_path in image_paths:
        stem = image_path.stem
        dst_image = output_root / 'images' / split / image_path.name
        shutil.copy2(image_path, dst_image)

        src_label = labels_dir / f'{stem}.txt'
        dst_label = output_root / 'labels' / split / f'{stem}.txt'
        if src_label.is_file():
            shutil.copy2(src_label, dst_label)
        else:
            dst_label.write_text('', encoding='utf-8')

        copied += 1
    return copied


def main():
    args = parse_args()

    raw_dir = Path(args.raw_dir)
    output_root = Path(args.output_root)
    if not raw_dir.is_dir():
        raise SystemExit(f'Raw sessions directory does not exist: {raw_dir}')

    sessions = list_sessions(raw_dir)
    if not sessions:
        raise SystemExit(f'No valid sessions found under: {raw_dir}')

    split_map = split_sessions(
        sessions=sessions,
        train_ratio=args.train_ratio,
        val_ratio=args.val_ratio,
        test_ratio=args.test_ratio,
        seed=args.seed,
    )
    prepare_output_dirs(output_root, clean=args.clean)

    print('Session split:')
    for split in SPLITS:
        names = [session.name for session in split_map[split]]
        print(f'  {split}: {len(names)} session(s) -> {names}')

    print('\nCopying files:')
    for split in SPLITS:
        image_count = 0
        for session_dir in split_map[split]:
            image_count += copy_session(session_dir, output_root, split)
        print(f'  {split}: {image_count} image(s)')


if __name__ == '__main__':
    main()
