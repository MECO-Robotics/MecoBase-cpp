#!/usr/bin/env python3
"""Verify the one-time 2026.2.12 archive recovery run before reusing artifacts."""

import json
import os
import pathlib
import urllib.request


SOURCE_RUN = 41533396868
SOURCE_WORKFLOW_SHA = "c1a96628f9e5c1fe15a0eacda477d9b77c3f591a"
RELEASE_TAG_SHA = "660eb3112d2edecb2d34efc5f0e19a75110212f1"
EXPECTED = {
    "meco-base-linuxathena-debug": "sha256:3923feaef2922c6de879a16ad9acc795f7e9e1a406ac3c948b36a03b47fc1f07",
    "meco-base-linuxathena-release": "sha256:71d0909f4a6b408620e7f78c5ad293e90cf0957ba36e0404761c0ba55003e037",
    "meco-base-linuxx86-64-debug": "sha256:997b43b4d50527a711ad42099e7ed5138e7b4bb92ca8aa6259c3677884a0f683",
    "meco-base-linuxx86-64-release": "sha256:627101e83ea3cd40ec4ee0b89f2e8e79ce55b5da6c5de4af781ec2dffdb8ff3c",
    "meco-base-windowsx86-64-debug": "sha256:f64ee355c08fcef5713b48f8090f755470a619a57a4a9cc0336247f50795a4ce",
    "meco-base-windowsx86-64-release": "sha256:c461be9e1cdd0dd0e23ccef820b1171dad4c0de54b5c158413e181bc1e1dfe82",
    "vendor-platform-linux": "sha256:b2db4b9e089b9fd150775bd43b4d08623461de39546b9b2f70c34740985f7a91",
    "vendor-provenance-linux": "sha256:7f3c82e78c4556a10bf9a795dabaf9099ae96b83a8c972bd8ac9a05e913c0b01",
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
    raise SystemExit("This recovery manifest only authorizes run 41533396868")

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
