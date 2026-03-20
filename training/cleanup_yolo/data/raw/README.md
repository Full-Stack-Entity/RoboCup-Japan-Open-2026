# Raw Capture Sessions

Put each capture session in its own subdirectory:

```text
data/raw/
  20260318_soysauce_scan_01/
    images/
      frame_000001.jpg
      frame_000002.jpg
    labels/
      frame_000001.txt
      frame_000002.txt
```

Notes:

- keep one short recording or one stable scene setup per session
- do not mix unrelated scenes into the same session
- `split_dataset.py` splits by session, not by individual frame
- negative images are allowed; use an empty `.txt` label file
