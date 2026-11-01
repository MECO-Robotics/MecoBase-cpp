#!/usr/bin/env python3
"""Verify the deployed Base vendordep descriptor, package, and checksums."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
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


def verify(site: str, expected_descriptor: dict, attempts: int = 12, delay: float = 10) -> tuple[str, int]:
    site = site.rstrip("/")
    descriptor = json.loads(fetch(f"{site}/MecoBase.json", attempts, delay))
    if descriptor != expected_descriptor:
        raise ValueError("The deployed descriptor does not match the release checkout")
    source = json.loads(fetch(descriptor["jsonUrl"], attempts, delay))
    if source != descriptor:
        raise ValueError("The raw vendordep descriptor differs from the deployed Pages descriptor")
    dependency = next((item for item in descriptor.get("cppDependencies", [])
                       if item.get("groupId") == "com.mecorobotics.frc"
                       and item.get("artifactId") == "meco-base-cpp"), None)
    if dependency is None or dependency.get("version") != descriptor.get("version"):
        raise ValueError("Deployed descriptor does not contain the matching Base Maven coordinate")
    checksums = json.loads(fetch(f"{site}/SHA256SUMS.json", attempts, delay)).get("artifacts", {})
    if not isinstance(checksums, dict):
        raise ValueError("Deployed checksum manifest has no artifact mapping")
    version = dependency["version"]
    prefix = f"maven/com/mecorobotics/frc/meco-base-cpp/{version}/meco-base-cpp-{version}"
    required = {f"{prefix}.pom", f"{prefix}-headers.zip", f"{prefix}-sources.zip"}
    for platform in dependency.get("binaryPlatforms", []):
        for variant in ("static", "staticdebug"):
            required.add(f"{prefix}-{platform}{variant}.zip")
    missing = required.difference(checksums)
    if missing:
        raise ValueError(f"Deployed checksum manifest lacks required Base artifacts: {sorted(missing)}")
    maven_count = 0
    for relative_path, expected in checksums.items():
        if not relative_path.startswith("maven/"):
            continue
        contents = fetch(f"{site}/{relative_path}", attempts, delay)
        if hashlib.sha256(contents).hexdigest() != expected:
            raise ValueError(f"Deployed artifact checksum mismatch: {relative_path}")
        maven_count += 1
    return version, maven_count


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--site-url", required=True, help="Pages root, with or without trailing slash")
    parser.add_argument("--attempts", type=int, default=12)
    parser.add_argument("--delay", type=float, default=10)
    args = parser.parse_args()
    descriptor_path = Path(__file__).resolve().parents[1] / "vendor/MecoBase.json"
    expected_descriptor = json.loads(descriptor_path.read_text(encoding="utf-8"))
    version, count = verify(args.site_url, expected_descriptor, args.attempts, args.delay)
    print(f"Verified deployed Base {version} and {count} checksummed Maven files")


if __name__ == "__main__":
    main()
