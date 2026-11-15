#!/usr/bin/env python3
"""按 SHA256 清单校验并合并强化学习视频分卷（仅使用 Python 标准库）。

下载全部分卷后运行：
    python assemble_rl_videos.py --parts-dir "视频分卷所在文件夹"

生成原始 ZIP 后，用系统解压工具解压。已有输出仅在校验一致时保留，
不会覆盖内容不同的文件；合并失败时会清理本次创建的临时文件。
"""

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import sys
import tempfile


CHUNK_SIZE = 2 * 1024 * 1024


def validate_entry(entry, label):
    """Validate names before using manifest values as local paths."""
    if not isinstance(entry, dict):
        raise ValueError(f"{label} 清单项必须是对象。")
    name = entry.get("name")
    if (
        not isinstance(name, str)
        or not name
        or name in {".", ".."}
        or any(c in name for c in '/\\:\0')
        or Path(name).name != name
    ):
        raise ValueError(f"{label} 文件名无效，必须是不含目录的文件名。")
    size = entry.get("size")
    if type(size) is not int or size <= 0:
        raise ValueError(f"{label} 文件大小必须是正整数。")
    digest = entry.get("sha256")
    if not isinstance(digest, str) or re.fullmatch(r"[0-9a-fA-F]{64}", digest) is None:
        raise ValueError(f"{label} SHA256 格式无效。")
    return {"name": name, "size": size, "sha256": digest.lower()}


def load_manifest(path):
    with path.open("r", encoding="utf-8-sig") as source:
        manifest = json.load(source)
    if not isinstance(manifest, dict) or manifest.get("schema_version") != 1:
        raise ValueError("不支持的分卷清单版本。")
    archive = validate_entry(manifest.get("archive"), "原始 ZIP")
    raw_parts = manifest.get("parts")
    if not isinstance(raw_parts, list) or not raw_parts:
        raise ValueError("分卷清单为空或格式无效。")
    parts = [validate_entry(part, f"第 {i} 卷") for i, part in enumerate(raw_parts, 1)]
    if len({part["name"] for part in parts}) != len(parts):
        raise ValueError("分卷清单包含重复文件名。")
    if sum(part["size"] for part in parts) != archive["size"]:
        raise ValueError("分卷大小之和与原始 ZIP 大小不符。")
    return archive, parts


def matches_file(path, expected):
    if not path.is_file() or path.stat().st_size != expected["size"]:
        return False
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for data in iter(lambda: source.read(CHUNK_SIZE), b""):
            digest.update(data)
    return digest.hexdigest() == expected["sha256"]


def assemble(manifest_path, parts_dir, output=None):
    manifest_path = Path(manifest_path).resolve()
    parts_dir = Path(parts_dir).resolve()
    archive, parts = load_manifest(manifest_path)
    if not parts_dir.is_dir():
        raise ValueError(f"分卷文件夹不存在：{parts_dir}")
    output = Path(output).resolve() if output is not None else parts_dir / archive["name"]
    part_paths = [(parts_dir / part["name"]).resolve() for part in parts]
    if output == manifest_path or output in part_paths:
        raise ValueError("输出路径不能指向清单或输入分卷。")
    if output.exists():
        if matches_file(output, archive):
            print(f"已有 ZIP 大小和 SHA256 均一致：{output}")
            return output
        raise ValueError(f"输出文件已存在且校验不符，不予覆盖：{output}")
    for part, path in zip(parts, part_paths):
        if not path.is_file():
            raise ValueError(f"缺少分卷：{part['name']}。请下载全部分卷后重试。")
        if path.stat().st_size != part["size"]:
            raise ValueError(f"分卷大小不符：{part['name']}。请重新下载该分卷。")

    output.parent.mkdir(parents=True, exist_ok=True)
    temp_path = None
    try:
        descriptor, temp_name = tempfile.mkstemp(
            prefix=f".{archive['name']}.", suffix=".assembling", dir=output.parent
        )
        temp_path = Path(temp_name)
        total_size = 0
        total_digest = hashlib.sha256()
        with os.fdopen(descriptor, "wb") as destination:
            for index, (part, path) in enumerate(zip(parts, part_paths), 1):
                part_size = 0
                part_digest = hashlib.sha256()
                with path.open("rb") as source:
                    for data in iter(lambda: source.read(CHUNK_SIZE), b""):
                        destination.write(data)
                        part_digest.update(data)
                        total_digest.update(data)
                        part_size += len(data)
                        total_size += len(data)
                if part_size != part["size"] or part_digest.hexdigest() != part["sha256"]:
                    raise ValueError(f"分卷校验失败：{part['name']}。请重新下载该分卷。")
                print(f"已校验并合并 {index}/{len(parts)}：{part['name']}", flush=True)
        if total_size != archive["size"] or total_digest.hexdigest() != archive["sha256"]:
            raise ValueError("合并后 ZIP 整体校验失败，未生成最终文件。")
        # Recheck after the potentially lengthy merge; preserve any file created meanwhile.
        if output.exists():
            if matches_file(output, archive):
                print(f"已有 ZIP 大小和 SHA256 均一致：{output}")
                return output
            raise ValueError(f"合并期间输出路径出现其他文件，不予覆盖：{output}")
        os.replace(temp_path, output)
        temp_path = None
        print(f"合并完成，大小和 SHA256 均验证通过：{output}")
        return output
    finally:
        if temp_path is not None:
            temp_path.unlink(missing_ok=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--parts-dir", type=Path, default=Path.cwd(), help="全部分卷所在文件夹（默认当前目录）")
    parser.add_argument("--output", type=Path, help="输出 ZIP 路径（默认在分卷文件夹内使用原始 ZIP 文件名）")
    parser.add_argument(
        "--manifest", type=Path, default=Path(__file__).resolve().with_name("rl-video-parts.json"),
        help="SHA256 分卷清单（默认脚本同目录的 rl-video-parts.json）",
    )
    args = parser.parse_args()
    try:
        assemble(args.manifest, args.parts_dir, args.output)
    except (ValueError, OSError) as error:
        print(f"合并失败：{error}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("合并已取消，临时文件已清理。", file=sys.stderr)
        return 130
    return 0


if __name__ == "__main__":
    sys.exit(main())
