#!/usr/bin/env python3
"""
Convert a ROS2 bag from livox_ros_driver/msg/CustomMsg
to livox_ros_driver2/msg/CustomMsg.

The CDR binary data is unchanged — only the type name strings are updated
in the SQLite topics table, message_definitions table, and metadata.yaml.

Usage:
    python3 convert_livox_bag.py <input_bag_dir> <output_bag_dir>

Example:
    python3 convert_livox_bag.py 1 1_converted
"""

import argparse
import shutil
import sqlite3
import sys
from pathlib import Path

OLD_PKG = "livox_ros_driver"
NEW_PKG = "livox_ros_driver2"
OLD_TYPE = f"{OLD_PKG}/msg/CustomMsg"
NEW_TYPE = f"{NEW_PKG}/msg/CustomMsg"


def convert_bag(input_dir: Path, output_dir: Path) -> None:
    if output_dir.exists():
        print(f"[ERROR] Output directory already exists: {output_dir}")
        sys.exit(1)

    # --- 1. Copy the whole bag directory ---
    print(f"Copying {input_dir} → {output_dir} ...")
    shutil.copytree(input_dir, output_dir)

    # --- 2. Find the .db3 file inside the copy ---
    db3_files = list(output_dir.glob("*.db3"))
    if not db3_files:
        print("[ERROR] No .db3 file found in the bag directory.")
        sys.exit(1)
    db3_path = db3_files[0]
    print(f"Patching SQLite database: {db3_path}")

    con = sqlite3.connect(str(db3_path))
    cur = con.cursor()

    # --- 3. Update `topics` table ---
    cur.execute(
        "UPDATE topics SET type = ? WHERE type = ?",
        (NEW_TYPE, OLD_TYPE),
    )
    topics_updated = cur.rowcount
    print(f"  topics rows updated: {topics_updated}")

    # --- 4. Update `message_definitions` table ---
    # 4a. Rename the topic_type key
    cur.execute(
        "UPDATE message_definitions SET topic_type = ? WHERE topic_type = ?",
        (NEW_TYPE, OLD_TYPE),
    )
    defs_type_updated = cur.rowcount

    # 4b. Replace all occurrences of the old package name inside the
    #     encoded_message_definition text (covers the sub-message reference
    #     "livox_ros_driver/CustomPoint" and the MSG header line).
    cur.execute(
        "SELECT rowid, encoded_message_definition FROM message_definitions "
        "WHERE topic_type = ?",
        (NEW_TYPE,),
    )
    rows = cur.fetchall()
    defs_body_updated = 0
    for rowid, body in rows:
        new_body = body.replace(OLD_PKG + "/", NEW_PKG + "/")
        if new_body != body:
            cur.execute(
                "UPDATE message_definitions SET encoded_message_definition = ? "
                "WHERE rowid = ?",
                (new_body, rowid),
            )
            defs_body_updated += 1
    print(f"  message_definitions rows updated (type key): {defs_type_updated}")
    print(f"  message_definitions rows updated (body text): {defs_body_updated}")

    con.commit()
    con.close()

    # --- 5. Update metadata.yaml ---
    metadata_path = output_dir / "metadata.yaml"
    if not metadata_path.exists():
        print("[WARNING] metadata.yaml not found — skipping.")
    else:
        text = metadata_path.read_text()
        new_text = text.replace(OLD_TYPE, NEW_TYPE)
        metadata_path.write_text(new_text)
        changed = text.count(OLD_TYPE)
        print(f"  metadata.yaml replacements: {changed}")

    print()
    print("Done. Converted bag written to:")
    print(f"  {output_dir.resolve()}")
    print()
    print("Note: the type_description_hash is kept from the original bag.")
    print("Most ros2 bag play / subscriber workflows do not validate it,")
    print("but if you see hash-mismatch warnings you may need to regenerate it")
    print("with your ROS 2 installation's rosidl tooling.")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input_bag_dir", help="Source bag directory (e.g. 1)")
    parser.add_argument("output_bag_dir", help="Destination bag directory (e.g. 1_converted)")
    args = parser.parse_args()

    convert_bag(Path(args.input_bag_dir), Path(args.output_bag_dir))


if __name__ == "__main__":
    main()
