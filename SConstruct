#!/usr/bin/env python
import os
import sys

env = SConscript("godot-cpp/SConstruct")

sources = Glob("src/*.cpp")
env.Append(CPPPATH=["include"])

out_dir = "project/addons/vehicle_shapecast/bin"

lib_name = "vehicle_shapecast"
if env["platform"] == "linux":
    lib_name = f"lib{lib_name}.{env['platform']}.{env['target']}.{env['arch']}.so"
elif env["platform"] == "windows":
    lib_name = f"{lib_name}.{env['platform']}.{env['target']}.{env['arch']}.dll"
elif env["platform"] == "macos":
    lib_name = f"lib{lib_name}.{env['platform']}.{env['target']}.{env['arch']}.dylib"

library = env.SharedLibrary(
    target=os.path.join(out_dir, lib_name),
    source=sources,
)

Default(library)