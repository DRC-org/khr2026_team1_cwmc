# Colcon のビルドが通らない問題に対処するためのパッチ
# https://github.com/micro-ROS/micro_ros_platformio/issues/188

import os
import shutil
from os.path import join, isfile
from SCons.Script import Import

Import("env")

project_libdeps_dir = env.subst("$PROJECT_LIBDEPS_DIR")
env_name = env.subst("$PIOENV")
project_dir = env.subst("$PROJECT_DIR")
micro_ros_dir = join(project_libdeps_dir, env_name, "micro_ros_platformio")

patches = [
    (
        join(project_dir, "patch", "library_builder_fixed.py"),
        join(micro_ros_dir, "microros_utils", "library_builder.py"),
        "library_builder.py",
    ),
    (
        join(project_dir, "patch", "extra_script_fixed.py"),
        join(micro_ros_dir, "extra_script.py"),
        "extra_script.py",
    ),
]


def apply_patches():
    for source_file, target_file, label in patches:
        # ターゲットが存在するか確認（ライブラリがまだ DL されていない場合はスキップ）
        if not isfile(target_file):
            print(f"Target file not found: {target_file}")
            print(f"Skipping patch for {label} (Library might not be installed yet).")
            continue

        if not isfile(source_file):
            print(f"Error: Source patch file not found at {source_file}")
            continue

        try:
            shutil.copyfile(source_file, target_file)
            print("--------------------------------------------------")
            print(f"micro-ROS Patch Applied: Replaced {label}")
            print("--------------------------------------------------")
        except Exception as e:
            print(f"Error applying patch for {label}: {e}")


apply_patches()
