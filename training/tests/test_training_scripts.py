import importlib.util
import json
import sys
import tempfile
import types
import unittest
from pathlib import Path
from unittest import mock


TRAINING_ROOT = Path(__file__).resolve().parents[1]


def load_module(module_name, relative_path, injected_modules=None):
    module_path = TRAINING_ROOT / relative_path
    if not module_path.is_file():
        raise FileNotFoundError(module_path)

    spec = importlib.util.spec_from_file_location(module_name, module_path)
    module = importlib.util.module_from_spec(spec)
    injected_modules = injected_modules or {}
    with mock.patch.dict(sys.modules, injected_modules, clear=False):
        spec.loader.exec_module(module)
    return module


class SplitDatasetTests(unittest.TestCase):
    def test_split_dataset_uses_flat_raw_export_layout(self):
        split_module = load_module(
            'split_dataset',
            Path('scripts/split_dataset.py'),
        )

        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            raw_dir = root / 'raw'
            output_root = root / 'output'
            canonical_classes = root / 'classes.txt'

            (raw_dir / 'images').mkdir(parents=True)
            (raw_dir / 'labels').mkdir(parents=True)
            canonical_classes.write_text('apple\nspray_bottle\nsugar\n', encoding='utf-8')
            (raw_dir / 'classes.txt').write_text(
                canonical_classes.read_text(encoding='utf-8'),
                encoding='utf-8',
            )
            (raw_dir / 'notes.json').write_text(
                json.dumps({'categories': [{'id': '0', 'name': 'apple'}]}),
                encoding='utf-8',
            )

            for name in ('a.jpg', 'b.jpg', 'c.png', 'd.jpg'):
                (raw_dir / 'images' / name).write_bytes(b'image')

            (raw_dir / 'labels' / 'a.txt').write_text('0 0.5 0.5 0.2 0.2\n', encoding='utf-8')
            (raw_dir / 'labels' / 'c.txt').write_text('2 0.4 0.4 0.1 0.1\n', encoding='utf-8')

            split_module.main([
                '--raw-dir', str(raw_dir),
                '--output-root', str(output_root),
                '--classes-file', str(canonical_classes),
                '--train-ratio', '0.5',
                '--val-ratio', '0.25',
                '--test-ratio', '0.25',
                '--seed', '7',
            ])

            split_images = list(output_root.glob('images/*/*'))
            split_labels = list(output_root.glob('labels/*/*'))
            self.assertEqual(4, len(split_images))
            self.assertEqual(4, len(split_labels))
            self.assertEqual(2, len(list((output_root / 'images' / 'train').iterdir())))
            self.assertEqual(1, len(list((output_root / 'images' / 'val').iterdir())))
            self.assertEqual(1, len(list((output_root / 'images' / 'test').iterdir())))

            all_label_names = {path.name for path in split_labels}
            self.assertEqual({'a.txt', 'b.txt', 'c.txt', 'd.txt'}, all_label_names)
            copied_empty = [
                path for path in split_labels
                if path.name in {'b.txt', 'd.txt'} and path.read_text(encoding='utf-8') == ''
            ]
            self.assertEqual(2, len(copied_empty))

    def test_split_dataset_rejects_mismatched_class_lists(self):
        split_module = load_module(
            'split_dataset_mismatch',
            Path('scripts/split_dataset.py'),
        )

        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            raw_dir = root / 'raw'
            output_root = root / 'output'
            canonical_classes = root / 'classes.txt'

            (raw_dir / 'images').mkdir(parents=True)
            (raw_dir / 'labels').mkdir(parents=True)
            (raw_dir / 'images' / 'a.jpg').write_bytes(b'image')
            (raw_dir / 'classes.txt').write_text('Sugar\n', encoding='utf-8')
            canonical_classes.write_text('sugar\n', encoding='utf-8')

            with self.assertRaises(SystemExit) as ctx:
                split_module.main([
                    '--raw-dir', str(raw_dir),
                    '--output-root', str(output_root),
                    '--classes-file', str(canonical_classes),
                ])

            self.assertIn('Class list mismatch', str(ctx.exception))


class TrainScriptTests(unittest.TestCase):
    def test_train_script_accepts_default_yolo26_alias(self):
        class FakeYOLO:
            last_model = None
            last_kwargs = None

            def __init__(self, model_spec):
                FakeYOLO.last_model = model_spec

            def train(self, **kwargs):
                FakeYOLO.last_kwargs = kwargs
                return {'ok': True}

        fake_ultralytics = types.SimpleNamespace(YOLO=FakeYOLO)
        train_module = load_module(
            'train_script',
            Path('scripts/train.py'),
        )

        with tempfile.TemporaryDirectory() as tmpdir:
            root = Path(tmpdir)
            data_path = root / 'dataset.yaml'
            project_dir = root / 'runs'
            data_path.write_text('path: dataset\ntrain: images/train\nval: images/val\n', encoding='utf-8')

            with mock.patch.dict(sys.modules, {'ultralytics': fake_ultralytics}, clear=False):
                train_module.main([
                    '--data', str(data_path),
                    '--project', str(project_dir),
                    '--name', 'demo',
                ])

            self.assertEqual('yolo26n.pt', FakeYOLO.last_model)
            self.assertEqual(str(data_path.resolve()), FakeYOLO.last_kwargs['data'])
            self.assertEqual(str(project_dir.resolve()), FakeYOLO.last_kwargs['project'])
            self.assertEqual('demo', FakeYOLO.last_kwargs['name'])


class ExtractFramesTests(unittest.TestCase):
    def test_extract_frames_parser_uses_flat_raw_output_root(self):
        fake_cv2 = types.SimpleNamespace()
        extract_module = load_module(
            'extract_frames',
            Path('scripts/extract_frames.py'),
            injected_modules={'cv2': fake_cv2},
        )

        args = extract_module.parse_args(['--video', 'clip.mp4'])
        self.assertEqual(['clip.mp4'], args.video)
        self.assertEqual('training/data/raw', args.output_root)
        self.assertFalse(hasattr(args, 'session'))


if __name__ == '__main__':
    unittest.main()
