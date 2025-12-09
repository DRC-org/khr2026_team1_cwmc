# Colcon のビルドが通らない問題に対処するためのパッチ
# https://github.com/micro-ROS/micro_ros_platformio/issues/188

import os
import shutil
from os.path import join, isfile
from SCons.Script import Import

Import("env")

source_file = join(env.subst("$PROJECT_DIR"), "patch", "library_builder_fixed.py")

project_libdeps_dir = env.subst("$PROJECT_LIBDEPS_DIR")
env_name = env.subst("$PIOENV")
target_file = join(
    project_libdeps_dir,
    env_name,
    "micro_ros_platformio",
    "microros_utils",
    "library_builder.py",
)


def overwrite_file():
    # ターゲットが存在するか確認（ライブラリがまだ DL されていない場合はスキップ）
    if not isfile(target_file):
        print(f"Target file not found: {target_file}")
        print("Skipping patch (Library might not be installed yet).")
        return

    # ソースファイルが存在するか確認
    if not isfile(source_file):
        print(f"Error: Source patch file not found at {source_file}")
        return

    try:
        shutil.copyfile(source_file, target_file)
        print("--------------------------------------------------")
        print(f"micro-ROS Patch Applied: Replaced library_builder.py")
        print("--------------------------------------------------")
    except Exception as e:
        print(f"Error applying patch: {e}")


overwrite_file()
