#!/usr/bin/env python
import os

# Bindings are generated against the API dumped from the exact editor binary in
# ../Godot_v4.7.2-stable_win64.exe, so the extension can never drift from it.
custom_api = os.path.join("api", "extension_api.json")

env = SConscript("godot-cpp/SConstruct")

env.Append(CPPPATH=["src/"])
sources = Glob("src/*.cpp")

library = env.SharedLibrary(
    "bin/libpolislab{}{}".format(env["suffix"], env["SHLIBSUFFIX"]),
    source=sources,
)

Default(library)
