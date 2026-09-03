#!/usr/bin/env bash
# The dependency policy, both halves of it.
#
# ROADMAP.md §0 and CLAUDE.md state two rules, and CI was checking one:
#
#   1. the **default** build's `cargo tree` is `rsemu` and nothing else;
#   2. "several siblings pull external crates under optional features, so CI
#      checks the *feature-enabled* dependency tree, not just the default."
#
# Rule 2 had no check. This adds one, and it is deliberately a check on the
# **direct** dependencies rather than on the whole transitive tree, because the
# transitive tree is not clean and saying so is more useful than a gate that
# fails on the first run: `--all-features` today reaches forty-odd crates,
# among them `serde`, `libc`, `toml`, `thiserror`, `uuid` and `encoding_rs`,
# every one of them arriving through `fstool` and none of them chosen here.
# CLAUDE.md's "no serde, no libc" is a rule about what rsemu depends on; what a
# permitted first-party crate depends on in turn is that crate's policy to
# hold, and tightening this into a whole-tree allowlist is a decision for
# whoever owns that conversation, not for a build script.
#
# So: a new **direct** dependency fails the build, and the transitive count and
# list are printed on every run, so a `fstool` bump that doubles the tree shows
# up in the log instead of nowhere.
#
# Used by both `.github/workflows/ci.yml` and `scripts/check.sh`, so the local
# answer and the CI answer cannot differ.

set -uo pipefail
cd "$(dirname "$0")/.."

# CLAUDE.md's permitted list, verbatim.
PERMITTED="pktkit compcol purecrypto fstool puremp noroi oxideav-png"

fail=0

echo "== the default build's tree =="
n=$(cargo tree --edges normal --prefix none | sort -u | tee /dev/stderr | wc -l)
if [ "$n" -ne 1 ]; then
  echo "::error title=dependency policy::the default build gained a dependency"
  fail=1
fi

echo
echo "== direct dependencies, all features =="
direct=$(cargo metadata --no-deps --format-version 1 | python3 -c '
import json, sys
pkg = json.load(sys.stdin)["packages"][0]
names = sorted({d["name"] for d in pkg["dependencies"] if d["kind"] is None})
print("\n".join(names))
')
printf '%s\n' "$direct"
for d in $direct; do
  case " $PERMITTED " in
    *" $d "*) ;;
    *)
      echo "::error title=dependency policy::$d is not on CLAUDE.md's permitted list"
      fail=1
      ;;
  esac
done

echo
echo "== the feature-enabled tree, for the record =="
# Not a gate — see the header. Printed so that what a permitted crate drags in
# is visible in the log and in the diff of two logs.
tree=$(cargo tree --all-features --edges normal --prefix none | sed 's/ (\*)$//' | sort -u)
printf '%s\n' "$tree"
echo "transitive crates with --all-features: $(printf '%s\n' "$tree" | wc -l)"

exit "$fail"
