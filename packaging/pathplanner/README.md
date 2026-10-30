# PathPlanner 2026.1.2 MECO patch: release preparation

This is a staging proposal, not a published dependency. The patch is based on
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

CI must check out the pinned commit, verify it, apply `sample-index.patch.gz` with `gzip -dc ... | git apply`,
build the upstream outputs for Linux x86-64, roboRIO and Windows x86-64 in
Debug and Release, run the regression, and collect headers plus all six static
archives. `stage.py --outputs <combined-outputs> --site <site-root>` checks
archive layout and rejects conflicting existing bytes. Run the normal finalizer
*after* staging so checksums include the new coordinate; then run the existing
preservation guard. A Windows archive has not yet been built or verified.

Before publishing, run the clean consumer matrix with a descriptor pointing to
the staged coordinate, then test the deployed coordinate and preservation of
all earlier published coordinates. Version and release the MecoBase descriptor
separately, listing only supported binary platforms. Do not change the current
2026.2.10 descriptor or its artifacts. Robot simulated scoring must be repeated
against the final published binaries. Local Linux and roboRIO builds and a
robot consumer build have passed; they do not establish Windows or deployment.

## Candidate workflow

`.github/workflows/pathplanner-candidate.yml` is deliberately non-publishing.
It builds the pinned source and patch, constructs a site candidate by fetching
and validating the current published checksum manifest and all retained Maven
coordinates, and runs clean consumers against a temporary descriptor pointing to
the candidate coordinate. The output site is an Actions artifact for review;
it is not attached to the existing Pages deployment. Publication must use the
existing serialized Pages job, refreshing and preserving the then-current site
inside that lock, and must verify deployed artifacts and consumers afterward.
A candidate snapshot alone must never be deployed because the live site can
change between staging and deployment. Existing coordinate conflicts fail closed.

The workflow is proposed but has not run on GitHub Actions. The Windows build,
Windows consumer, full CI matrix, deployment integration, and post-deployment
verification are therefore still required. A local staging exercise used the
upstream Windows archives as stand-ins to exercise assembly only; they are not
patched and must never be published as this coordinate.
