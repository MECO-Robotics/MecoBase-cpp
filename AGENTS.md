# Ownership and development

- This repository owns the reusable vendordep. Robot entry points, hardware maps, autos, and game behavior belong in consumers. See the [module map](README.md#modules) before editing.
- Preserve public headers and vendordep coordinates. Keep hardware/vendor calls in IO adapters; subsystems own goals and safety. Preserve Phoenix's native control timing and explicit physical units.
- Base owns deterministic motor simulation and the single-threaded simulation callback runtime. Jolt physics belongs in the optional MECO Simulation package. Replay substitutes recorded inputs without constructing hardware or physics; retain its recording shutdown/flush lifecycle.
- Use ordinary headers and source files; the DriveTarget named-module experiment was removed because its duplicate implementation and custom compiler plumbing did not serve production callers.
- Start with the desktop Debug command in [Build and validate](README.md#build-and-validate), then filter the installed test runner for the changed behavior. Broaden checks for shared/platform-sensitive changes. Cross-platform archives, packaging, and clean consumer validation belong at integration/release gates.
- Keep refactors scoped; prefer deleting duplication over adding frameworks. Update existing documentation when behavior or developer commands change.

# Test case guidance

Use a **less is more** philosophy: keep the smallest set of tests that provides meaningful confidence.

- Test observable behavior, important contracts, safety boundaries, and known regressions.
- Add a regression test when fixing a meaningful bug; make it fail for the original failure.
- Prefer a few representative cases and relevant boundaries over exhaustive combinations.
- Reuse existing fixtures and parameterize closely related cases instead of duplicating tests.
- Avoid tests that mirror implementation steps, assert incidental details, or merely repeat constants from production code.
- Skip new tests for documentation, formatting, and reversible low-impact changes unless they affect behavior.
- Use simulation or integration checks when command ownership, cancellation, units, or IO interactions are the actual concern; avoid elaborate mocks that bypass that concern.
- Keep assertions deterministic and tied to justified tolerances. Distinguish numerical simulation checks from hardware evidence.
- Run the checks relevant to the change. Broaden testing for shared behavior or unresolved risk; do not rerun unchanged checks without a reason.

Every test should protect a distinct behavior or failure mode. If its removal would not reduce confidence, consolidate or remove it. Test count and coverage percentage are not goals by themselves.
