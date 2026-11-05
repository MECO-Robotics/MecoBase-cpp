#!/usr/bin/env python3
"""Verify the one-time 2026.2.14 Linux archive recovery run before reusing artifacts."""

import json
import os
import pathlib
import urllib.request


SOURCE_RUN = 42036443481
SOURCE_WORKFLOW_SHA = "447b84371ec0f5ce26754756c5b35fd97a43d302"
RELEASE_TAG_SHA = "447b84371ec0f5ce26754756c5b35fd97a43d302"
EXPECTED = {
    "studica-windows-maven-2026.0.0": "sha256:0605c4985e400aa236b2d5b649c3351369ec0327f723e575a40acd474ade22f7",
    "vendor-platform-linux": "sha256:dfd98339b6267e6bae25741a049348aa084c201744c8c9905521b9a98f97629c",
    "vendor-provenance-linux": "sha256:942f58af2188fa19d5622b2612768773a3f361328f06daf417165efb0a0e7400",
}


def api_json(path: str) -> dict:
    request = urllib.request.Request(
        f"https://api.github.com/repos/MECO-Robotics/MecoBase-cpp/{path}",
        headers={
            "Accept": "application/vnd.github+json",
            "Authorization": f"Bearer {os.environ['GITHUB_TOKEN']}",
            "X-GitHub-Api-Version": "2022-11-28",
        },
    )
    with urllib.request.urlopen(request, timeout=20) as response:
        return json.load(response)


if os.environ["REUSE_RUN_ID"] != str(SOURCE_RUN):
    raise SystemExit("This recovery manifest only authorizes run 42036443481")

run = api_json(f"actions/runs/{SOURCE_RUN}")
assert run["event"] == "workflow_dispatch"
assert run["workflow_id"] == 405037436
assert run["head_sha"] == SOURCE_WORKFLOW_SHA
assert run["conclusion"] == "cancelled"
assert run["status"] == "completed"
assert pathlib.Path(".git").exists()
checkout_sha = __import__("subprocess").check_output(
    ["git", "rev-parse", "HEAD"], text=True
).strip()
assert checkout_sha == RELEASE_TAG_SHA, (checkout_sha, RELEASE_TAG_SHA)

artifacts = api_json(f"actions/runs/{SOURCE_RUN}/artifacts")["artifacts"]
actual = {item["name"]: item["digest"] for item in artifacts if not item["expired"]}
for name, digest in EXPECTED.items():
    assert actual.get(name) == digest, (name, actual.get(name), digest)

print(
    f"Verified run {SOURCE_RUN} from workflow {SOURCE_WORKFLOW_SHA}; "
    f"release-tag checkout {checkout_sha}; {len(EXPECTED)} pinned artifact digests."
)
