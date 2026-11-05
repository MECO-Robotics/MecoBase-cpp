#!/usr/bin/env python3
"""Enforce one three-minute Windows recovery budget including queue time."""
from datetime import datetime, timezone
import json
import os
import urllib.request

run_id = os.environ['GITHUB_RUN_ID']
request = urllib.request.Request(
    f'https://api.github.com/repos/MECO-Robotics/MecoBase-cpp/actions/runs/{run_id}',
    headers={'Accept': 'application/vnd.github+json',
             'Authorization': f"Bearer {os.environ['GITHUB_TOKEN']}",
             'X-GitHub-Api-Version': '2022-11-28'})
with urllib.request.urlopen(request, timeout=10) as response:
    run = json.load(response)
created = datetime.fromisoformat(run['created_at'].replace('Z', '+00:00'))
elapsed = (datetime.now(timezone.utc) - created).total_seconds()
print(f'Windows recovery queue plus execution: {elapsed:.1f} seconds')
if elapsed < 0 or elapsed > 180:
    raise SystemExit('Windows recovery exceeded its three-minute total limit')
