#!/usr/bin/env bash
set -euo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
project_root=$(cd -- "${script_dir}/.." && pwd)

test -s "${project_root}/debug/feasibility/feasibility_report.yaml"
test -s "${project_root}/debug/feasibility/feasibility_points.csv"
grep -q '^result:' "${project_root}/debug/feasibility/feasibility_report.yaml" || \
  grep -q '^test_results:' "${project_root}/debug/feasibility/feasibility_report.yaml"
echo "FastPlanner feasibility outputs are present."
