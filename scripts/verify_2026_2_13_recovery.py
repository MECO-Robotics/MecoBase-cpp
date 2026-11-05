#!/usr/bin/env python3
"""Verify the one-time 2026.2.13 Linux archive recovery run before reusing artifacts."""

import json
import os
import pathlib
import urllib.request


SOURCE_RUN = 41962900942
SOURCE_WORKFLOW_SHA = "e73610d0fa4eea36bd2051de5d269b68a3bdde15"
RELEASE_TAG_SHA = "e73610d0fa4eea36bd2051de5d269b68a3bdde15"
EXPECTED = {
    "vendor-platform-linux": "sha256:543c8673fa5224e1657d9b1f1183c4de2d70982c5c5bb433d83068859a180024",
    "vendor-provenance-linux": "sha256:43322c576a2c3cfa7e3080a8175fb596af1cb0734faf1a5aa456500a4e186216",
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
    raise SystemExit("This recovery manifest only authorizes run 41962900942")

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
