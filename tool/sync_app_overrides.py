#!/usr/bin/env python3
"""Inject `dependency_overrides` path entries for every generated Dart package
into each app's pubspec.yaml — the step that makes custom src/ packages usable
by apps/ automatically.

For each package under rcldart_ws/dart/, ensure every app under rcldart_ws/apps/
has:

    dependency_overrides:
      <pkg>:
        path: ../../dart/<pkg>

Idempotent: existing entries are left untouched; only missing ones are added.
Text-based (no YAML reformatting) so it preserves your pubspec formatting.

Usage: tool/sync_app_overrides.py            # all apps
       tool/sync_app_overrides.py <app>...   # named apps only
"""
import os
import re
import sys

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DART = os.path.join(WS, "dart")
APPS = os.path.join(WS, "apps")


def generated_packages():
    pkgs = []
    for name in sorted(os.listdir(DART)):
        p = os.path.join(DART, name)
        if name.startswith("_"):
            continue
        if os.path.isfile(os.path.join(p, "pubspec.yaml")):
            pkgs.append(name)
    return pkgs


def ensure_overrides(pubspec_path, pkgs):
    with open(pubspec_path) as f:
        text = f.read()
    lines = text.splitlines()

    # Find the `dependency_overrides:` block (top-level key).
    start = None
    for i, ln in enumerate(lines):
        if ln.rstrip() == "dependency_overrides:":
            start = i
            break

    if start is None:
        # Append a fresh block.
        block = ["", "dependency_overrides:"]
        for pk in pkgs:
            block += [f"  {pk}:", f"    path: ../../dart/{pk}"]
        new = text.rstrip("\n") + "\n" + "\n".join(block) + "\n"
        added = list(pkgs)
    else:
        # Extent of the block: until the next top-level (col-0, non-blank) key.
        end = len(lines)
        for j in range(start + 1, len(lines)):
            if lines[j] and not lines[j][0].isspace():
                end = j
                break
        block_text = "\n".join(lines[start:end])
        added = []
        insert = []
        for pk in pkgs:
            # already declared in the block, in either two-line or inline
            # (`  pk: { path: ... }`) form?
            if re.search(rf"(?m)^\s{{2}}{re.escape(pk)}:(\s|$)", block_text):
                continue
            insert += [f"  {pk}:", f"    path: ../../dart/{pk}"]
            added.append(pk)
        if not added:
            return []
        lines[start + 1:start + 1] = insert
        new = "\n".join(lines) + ("\n" if text.endswith("\n") else "")

    with open(pubspec_path, "w") as f:
        f.write(new)
    return added


def main():
    pkgs = generated_packages()
    if not pkgs:
        print("no generated packages under dart/ — run gen first")
        return
    wanted = sys.argv[1:]
    for app in sorted(os.listdir(APPS)):
        if wanted and app not in wanted:
            continue
        pub = os.path.join(APPS, app, "pubspec.yaml")
        if not os.path.isfile(pub):
            continue
        added = ensure_overrides(pub, pkgs)
        print(f"{app}: +{len(added)} overrides" + (f" ({', '.join(added)})" if added else " (up to date)"))


if __name__ == "__main__":
    main()
