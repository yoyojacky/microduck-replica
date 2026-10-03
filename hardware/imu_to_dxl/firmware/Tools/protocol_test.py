#!/usr/bin/env python3
"""Build and RUN the portable C protocol tests using an existing host compiler.

No packages are installed. All generated files stay in Build/host-tests.
Examples:
  python Tools/protocol_test.py
  python Tools/protocol_test.py --cc gcc
  python Tools/protocol_test.py --vcvars "path/to/VC/Auxiliary/Build/vcvars64.bat"
"""
from __future__ import annotations

import argparse
import locale
import os
from pathlib import Path
import shutil
import subprocess
import sys


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", help="Existing native C compiler, e.g. gcc or clang")
    parser.add_argument("--vcvars", type=Path, help="Existing Visual Studio vcvars64.bat")
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    build = root / "Build" / "host-tests"
    build.mkdir(parents=True, exist_ok=True)
    env = os.environ.copy()
    # Keep temporary compiler products off the system disk too.
    env["TEMP"] = env["TMP"] = str(build)
    compiler = args.cc or next((shutil.which(x) for x in ("gcc", "clang", "cl")
                               if shutil.which(x)), None)
    vcvars = args.vcvars
    if vcvars:
        vcvars = vcvars.resolve(strict=True)
        if any(x in str(vcvars) for x in ('"', '\n', '\r', '%')):
            raise SystemExit("Unsupported character in vcvars path")
        setup = build / "capture-msvc-env.cmd"
        setup.write_text('@echo off\ncall "' + str(vcvars) + '" >nul\n'
                         'if errorlevel 1 exit /b 1\nset\n', encoding="utf-8")
        # vcvars 的 set 输出是本地代码页（中文系统是 GBK），按它解码，别让 stdout 变成 None
        result = subprocess.run([os.environ.get("COMSPEC", "cmd.exe"), "/d", "/c",
                                 str(setup)], env=env, capture_output=True, text=True,
                                encoding=locale.getpreferredencoding(False), errors="replace")
        if result.returncode:
            print(result.stdout + result.stderr, file=sys.stderr)
            return result.returncode
        for line in result.stdout.splitlines():
            if "=" in line and not line.startswith("="):
                key, value = line.split("=", 1)
                env[key] = value
        compiler = shutil.which("cl.exe", path=env.get("Path", env.get("PATH")))
    if not compiler:
        raise SystemExit("No native C compiler found. Provide --cc or --vcvars; nothing was installed.")
    exe = build / ("protocol_test.exe" if os.name == "nt" else "protocol_test")
    sources = [root / "Core/Src/protocol.c", root / "Tests/protocol_test.c"]
    is_msvc = Path(compiler).name.lower() in ("cl", "cl.exe")
    if is_msvc:
        command = [compiler, "/nologo", "/std:c11", "/W4", "/WX", "/Od", "/RTC1",
                   "/I" + str(root / "Core/Inc"), "/Fe:" + str(exe),
                   "/Fo" + str(build) + os.sep] + [str(x) for x in sources]
    else:
        command = [compiler, "-std=c11", "-Wall", "-Wextra", "-Werror", "-O0", "-g",
                   "-I", str(root / "Core/Inc"), *map(str, sources), "-o", str(exe)]
    print("Native compiler:", compiler, flush=True)
    subprocess.run(command, cwd=build, env=env, check=True)
    return subprocess.run([str(exe)], cwd=build, env=env).returncode


if __name__ == "__main__":
    raise SystemExit(main())
