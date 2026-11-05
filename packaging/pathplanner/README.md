# PathPlanner 2026.1.2 MECO patch: release preparation

This documents the published patched dependency. The patch is based on
PathPlanner tag `v2026.1.2`, commit `e02bbf3176588166e8fe5192ab8dea85f6d62f7a`.
It changes the segment index to use the already-clamped sample position and
adds a regression test. Retain PathPlanner's license and preserve attribution.

The proposed immutable Maven coordinate is
`com.pathplanner.lib:PathplannerLib-cpp:2026.1.2-meco1`. It must never replace
upstream `2026.1.2`. The MecoBase Pages deployment is a complete site replacement;
its existing publish job serializes deployments, retains all checksummed old
Maven paths, and verifies checksums. A standalone deployment would race and
could erase published packages. Integrate this staging into that existing job,
prior to finalization and `preserve_published_maven.py`; include every new
artifact in `SHA256SUMS.json` and test retention across a later release.

The `2026.1.2-meco1` coordinate has already been published for Linux,
roboRIO, and Windows. Its existing Maven artifacts are immutable and must be
retained when Pages is updated. The historical CI candidate workflow has been retired. Any future patched
coordinate must be built locally, use a new version, and declare only the
platforms actually packaged.
