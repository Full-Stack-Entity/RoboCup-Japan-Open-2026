#!/usr/bin/env python3
"""
将文件夹中的文件批量重命名为 1, 2, 3, ... 的脚本。
保留原始文件扩展名。
"""

import argparse
import os
from pathlib import Path


def rename_files_to_numbers(
    folder: str,
    start: int = 1,
    dry_run: bool = False,
    sort_by: str = "name",
) -> None:
    """
    将文件夹中的文件重命名为连续数字。

    Args:
        folder: 目标文件夹路径
        start: 起始数字，默认为 1
        dry_run: 若为 True，仅预览不实际执行
        sort_by: 排序方式，可选 "name"（按名称）或 "mtime"（按修改时间）
    """
    folder_path = Path(folder).resolve()
    if not folder_path.exists():
        raise FileNotFoundError(f"文件夹不存在: {folder_path}")
    if not folder_path.is_dir():
        raise NotADirectoryError(f"路径不是文件夹: {folder_path}")

    # 只获取文件，不包含子目录
    files = [f for f in folder_path.iterdir() if f.is_file()]

    if sort_by == "name":
        files.sort(key=lambda x: x.name)
    elif sort_by == "mtime":
        files.sort(key=lambda x: x.stat().st_mtime)
    else:
        files.sort(key=lambda x: x.name)

    if dry_run:
        print("【预览模式 - 不会实际重命名】\n")

    for i, file_path in enumerate(files):
        ext = file_path.suffix
        new_name = f"{start + i}{ext}"
        new_path = folder_path / new_name

        if file_path.name == new_name:
            if dry_run:
                print(f"  跳过（已是目标名）: {file_path.name}")
            continue

        # 检查目标是否已存在且不是当前文件
        if new_path.exists() and new_path != file_path:
            print(f"  警告: 目标已存在，跳过: {file_path.name} -> {new_name}")
            continue

        if dry_run:
            print(f"  {file_path.name}  ->  {new_name}")
        else:
            file_path.rename(new_path)
            print(f"  已重命名: {file_path.name}  ->  {new_name}")

    if not dry_run and files:
        print(f"\n完成，共重命名 {len(files)} 个文件。")


def main():
    parser = argparse.ArgumentParser(
        description="将文件夹中的文件批量重命名为 1, 2, 3, ..."
    )
    parser.add_argument(
        "folder",
        nargs="?",
        default=".",
        help="目标文件夹路径（默认当前目录）",
    )
    parser.add_argument(
        "-s", "--start",
        type=int,
        default=1,
        help="起始数字（默认: 1）",
    )
    parser.add_argument(
        "-n", "--dry-run",
        action="store_true",
        help="仅预览，不实际执行重命名",
    )
    parser.add_argument(
        "--sort",
        choices=["name", "mtime"],
        default="name",
        help="排序方式: name=按名称, mtime=按修改时间（默认: name）",
    )

    args = parser.parse_args()

    try:
        rename_files_to_numbers(
            folder=args.folder,
            start=args.start,
            dry_run=args.dry_run,
            sort_by=args.sort,
        )
    except (FileNotFoundError, NotADirectoryError) as e:
        print(f"错误: {e}")
        exit(1)


if __name__ == "__main__":
    main()
