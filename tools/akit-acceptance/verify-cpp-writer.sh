#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
output="${AKIT_ACCEPTANCE_OUTPUT:-${repo_root}/build/akit-acceptance/cpp-writer.wpilog}"
mkdir -p "$(dirname "${output}")"
output="$(realpath -m "${output}")"

(
  cd "${repo_root}"
  AKIT_ACCEPTANCE_CPP_OUTPUT="${output}" GTEST_FILTER='AdvantageKitAcceptanceFixture.*' \
    ./gradlew --no-daemon --console=plain runMecoBaseTestLinuxx86-64DebugGoogleTestExe
)

"${repo_root}/tools/akit-acceptance/run.sh" "${output}" \
  'ReplayInputs/Drive/backend=String' \
  'ReplayInputs/Drive/pose/0=Double' \
  'ReplayOutputs/Drive/voltage=Double'
