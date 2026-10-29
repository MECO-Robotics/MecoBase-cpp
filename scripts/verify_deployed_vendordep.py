#!/usr/bin/env python3
"""Verify the deployed Base vendordep descriptor, package, and checksums."""

from __future__ import annotations

import argparse
import hashlib
import json
import time
import urllib.error
import urllib.request


def fetch(url: str, attempts: int, delay: float) -> bytes:
    last_error: Exception | None = None
    for attempt in range(attempts):
        try:
            with urllib.request.urlopen(url, timeout=20) as response:
                if response.status != 200:
                    raise RuntimeError(f"HTTP {response.status}: {url}")
                return response.read()
        except (urllib.error.URLError, TimeoutError, RuntimeError) as error:
            last_error = error
            if attempt + 1 < attempts:
                time.sleep(delay)
    raise RuntimeError(f"Could not fetch deployed file {url}: {last_error}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--site-url", required=True, help="Pages root, with or without trailing slash")
    parser.add_argument("--attempts", type=int, default=12)
    parser.add_argument("--delay", type=float, default=10)
    args = parser.parse_args()
    site = args.site_url.rstrip("/")
    descriptor_bytes = fetch(f"{site}/MecoBase.json", args.attempts, args.delay)
    descriptor = json.loads(descriptor_bytes)
    source = fetch(descriptor["jsonUrl"], args.attempts, args.delay)
    if json.loads(source) != descriptor:
        raise SystemExit("The raw vendordep descriptor differs from the deployed Pages descriptor")
    if descriptor.get("version") != "2026.2.6":
        raise SystemExit(f"Unexpected deployed Base version: {descriptor.get('version')!r}")
    dependency = next((item for item in descriptor.get("cppDependencies", [])
                       if item.get("artifactId") == "meco-base-cpp"), None)
    if dependency is None or dependency.get("version") != descriptor["version"]:
        raise SystemExit("Deployed descriptor does not contain the matching Base Maven coordinate")
    checksums = json.loads(fetch(f"{site}/SHA256SUMS.json", args.attempts, args.delay)).get("artifacts", {})
    if not checksums:
        raise SystemExit("Deployed checksum manifest contains no artifacts")
    for relative_path, expected in checksums.items():
        if not relative_path.startswith("maven/"):
            continue
        contents = fetch(f"{site}/{relative_path}", args.attempts, args.delay)
        if hashlib.sha256(contents).hexdigest() != expected:
            raise SystemExit(f"Deployed artifact checksum mismatch: {relative_path}")
    maven = dependency["mavenUrls"][0].rstrip("/") if dependency.get("mavenUrls") else descriptor["mavenUrls"][-1].rstrip("/")
    path = f"com/mecorobotics/frc/meco-base-cpp/{dependency['version']}/meco-base-cpp-{dependency['version']}.pom"
    fetch(f"{maven}/{path}", args.attempts, args.delay)
    print(f"Verified deployed Base {dependency['version']} and {len(checksums)} checksummed Maven files")


if __name__ == "__main__":
    main()
