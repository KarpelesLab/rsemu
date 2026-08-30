#!/usr/bin/env bash
#
# Fetch the conformance corpora into a git-ignored directory.
#
# Corpora are downloaded at test time and never committed (CLAUDE.md, Testing).
# That is a licensing rule before it is a size one: some suites are copyleft and
# some have no licence at all, and executing one as an emulated guest is
# ordinary use while shipping it in this repository is redistribution.
# docs/testing/conformance-suites.md has the verified licence table.
#
# Idempotent: everything already present and verified is left alone. --force
# re-fetches.
#
# Dependencies: curl, git, and sha256sum or shasum. Nothing else — this runs on
# a CI image with no package installs.

set -euo pipefail

readonly SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
readonly DEST_ROOT="${RSEMU_TESTDATA:-${REPO_ROOT}/testdata}"

# ---------------------------------------------------------------------------
# Sources
# ---------------------------------------------------------------------------

readonly SST_REPO="https://github.com/SingleStepTests/65x02"

readonly NESTEST_ROM_URL="https://www.qmtpro.com/~nes/misc/nestest.nes"
readonly NESTEST_LOG_URL="https://www.qmtpro.com/~nes/misc/nestest.log"
# Both artefacts have been frozen for over a decade, so a mismatch means the
# wrong file, not a new release. Fatal.
readonly NESTEST_ROM_SHA="f67d55fd6b3cf0bad1cc85f1df0d739c65b53e79cecb7fea8f77ec0eadab0004"
readonly NESTEST_LOG_SHA="627c8e180b1a924dfa705c5dc6958fad7ab75a62de556173caf880ccc1337540"

readonly AC_BASE="https://raw.githubusercontent.com/100thCoin/AccuracyCoin/main"
# AccuracyCoin is actively developed and the ROM is rebuilt, so this is the
# build these runners were written against rather than a requirement. A
# mismatch warns; it does not fail.
readonly AC_ROM_SHA="b8aa2a6bbcf01a8839850d0b802a7a4e4bed6002adfe344729c234c6a0dd0647"

# ---------------------------------------------------------------------------
# Output
# ---------------------------------------------------------------------------

if [ -t 1 ]; then
	C_OK=$'\033[32m'; C_WARN=$'\033[33m'; C_ERR=$'\033[31m'; C_OFF=$'\033[0m'
else
	C_OK=''; C_WARN=''; C_ERR=''; C_OFF=''
fi

note() { printf '%s\n' "$*"; }
ok()   { printf '%s+%s %s\n' "$C_OK" "$C_OFF" "$*"; }
warn() { printf '%s!%s %s\n' "$C_WARN" "$C_OFF" "$*" >&2; }
die()  { printf '%serror%s %s\n' "$C_ERR" "$C_OFF" "$*" >&2; exit 1; }

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

need() { command -v "$1" >/dev/null 2>&1 || die "$1 is required but not on PATH"; }

sha256_of() {
	if command -v sha256sum >/dev/null 2>&1; then
		sha256sum "$1" | cut -d' ' -f1
	else
		shasum -a 256 "$1" | cut -d' ' -f1
	fi
}

# download <url> <dest>. Atomic: a partial transfer never lands as the real file.
download() {
	local url="$1" dest="$2" tmp
	tmp="${dest}.part"
	mkdir -p "$(dirname -- "$dest")"
	curl --fail --silent --show-error --location --retry 3 --retry-delay 2 \
		--output "$tmp" "$url" || { rm -f "$tmp"; die "could not download $url"; }
	mv -f "$tmp" "$dest"
}

# verify <file> <expected sha> <fatal|advisory>
verify() {
	local file="$1" want="$2" mode="$3" got
	got="$(sha256_of "$file")"
	if [ "$got" = "$want" ]; then
		return 0
	fi
	if [ "$mode" = fatal ]; then
		die "$(basename -- "$file") has the wrong checksum
    expected $want
    got      $got
  This is not the file the harness was written against. Delete it and retry."
	fi
	warn "$(basename -- "$file") differs from the build the runners were written against"
	warn "    expected $want"
	warn "    got      $got"
	warn "  Upstream may simply have released a new version. Re-read its release notes"
	warn "  before treating a new failure as a regression in this emulator."
	return 0
}

# fetch_verified <url> <dest> <sha> <fatal|advisory>
fetch_verified() {
	local url="$1" dest="$2" want="$3" mode="$4"
	if [ "$FORCE" = 0 ] && [ -f "$dest" ] && [ "$(sha256_of "$dest")" = "$want" ]; then
		ok "$(basename -- "$dest") already present and verified"
		return 0
	fi
	note "  downloading $(basename -- "$dest") ..."
	download "$url" "$dest"
	verify "$dest" "$want" "$mode"
	ok "$(basename -- "$dest") ($(wc -c <"$dest" | tr -d ' ') bytes)"
}

# Written beside every corpus so its terms travel with it.
write_notice() {
	local dir="$1" text="$2"
	mkdir -p "$dir"
	printf '%s\n' "$text" >"${dir}/PROVENANCE.txt"
}

# ---------------------------------------------------------------------------
# Suites
# ---------------------------------------------------------------------------

fetch_sst() {
	need git
	local dest="${DEST_ROOT}/sst-65x02"
	local -a want_dirs=()
	case "$SST_VARIANT" in
		nes6502) want_dirs=(nes6502) ;;
		6502)    want_dirs=(6502) ;;
		all)     want_dirs=(nes6502 6502 wdc65c02 rockwell65c02 synertek65c02) ;;
		*)       die "unknown --variant $SST_VARIANT (nes6502, 6502, all)" ;;
	esac

	# Subset mode: pull individual opcode files over HTTP. The full corpus is
	# gigabytes, and bringing a core up one addressing mode at a time does not
	# need it. Pairs with RSEMU_SST_OPCODES on the test side.
	if [ -n "$SST_OPCODES" ]; then
		need curl
		local variant opcode target count=0
		for variant in "${want_dirs[@]}"; do
			for opcode in $(printf '%s' "$SST_OPCODES" | tr ',' ' '); do
				target="${dest}/${variant}/v1/${opcode}.json"
				if [ "$FORCE" = 0 ] && [ -s "$target" ]; then
					continue
				fi
				download "${SST_REPO/github.com/raw.githubusercontent.com}/main/${variant}/v1/${opcode}.json" \
					"$target"
				count=$((count + 1))
			done
			download "${SST_REPO/github.com/raw.githubusercontent.com}/main/LICENSE" \
				"${dest}/LICENSE"
		done
		ok "fetched $count opcode file(s) into ${dest}"
		sst_notice "$dest"
		return 0
	fi

	if [ -d "${dest}/.git" ] && [ "$FORCE" = 0 ]; then
		note "  updating the existing checkout ..."
		git -C "$dest" sparse-checkout set --no-cone "${want_dirs[@]/#//}" /LICENSE /README.md
		git -C "$dest" pull --ff-only --depth 1 >/dev/null 2>&1 || \
			warn "could not update; using what is already checked out"
	else
		if [ -e "$dest" ]; then
			# A subset fetch (--opcodes) leaves a plain directory here. Cloning
			# into it would fail with a confusing message from git, so say what
			# is actually going on.
			[ "$FORCE" = 1 ] || die "${dest} exists but is not a git checkout
  It was probably created by --opcodes. Re-run with --force to replace it with
  a full sparse checkout, or keep using --opcodes."
			rm -rf "$dest"
		fi
		note "  cloning ${SST_REPO} (blobless, sparse: ${want_dirs[*]}) ..."
		note "  this is a large download; the nes6502 vectors alone are several hundred MB"
		git clone --depth 1 --filter=blob:none --sparse "$SST_REPO" "$dest"
		git -C "$dest" sparse-checkout set --no-cone "${want_dirs[@]/#//}" /LICENSE /README.md
	fi

	local variant count
	for variant in "${want_dirs[@]}"; do
		count=$(find "${dest}/${variant}/v1" -name '*.json' 2>/dev/null | wc -l | tr -d ' ')
		[ "$count" -gt 0 ] || die "${variant}: no vector files were checked out"
		ok "${variant}: ${count} opcode files"
	done
	[ -f "${dest}/LICENSE" ] || warn "the upstream LICENSE was not checked out"
	sst_notice "$dest"
}

sst_notice() {
	write_notice "$1" "SingleStepTests/65x02
${SST_REPO}

Licence: MIT. Redistributable with attribution — but this directory is
git-ignored anyway, because the corpus is gigabytes.

Consumed by tests/conformance/sst.rs."
}

fetch_nestest() {
	need curl
	local dest="${DEST_ROOT}/nestest"
	fetch_verified "$NESTEST_ROM_URL" "${dest}/nestest.nes" "$NESTEST_ROM_SHA" fatal
	fetch_verified "$NESTEST_LOG_URL" "${dest}/nestest.log" "$NESTEST_LOG_SHA" fatal
	write_notice "$dest" "nestest.nes and nestest.log
${NESTEST_ROM_URL}
${NESTEST_LOG_URL}

Licence: UNCLEAR. FETCH AND RUN ONLY — do not commit either file, do not
vendor them, do not attach them to a release. Running a ROM as an emulated
guest is ordinary use; redistributing it is not ours to do.

Consumed by tests/conformance/nestest.rs."
}

fetch_accuracycoin() {
	need curl
	local dest="${DEST_ROOT}/accuracycoin"
	fetch_verified "${AC_BASE}/AccuracyCoin.nes" "${dest}/AccuracyCoin.nes" \
		"$AC_ROM_SHA" advisory
	# The licence and the error-code tables travel with the ROM: a failure code
	# is unreadable without the README, and the runner prints codes.
	if [ "$FORCE" = 1 ] || [ ! -s "${dest}/LICENSE" ] || [ ! -s "${dest}/README.md" ]; then
		note "  downloading LICENSE and README.md ..."
		download "${AC_BASE}/LICENSE" "${dest}/LICENSE"
		download "${AC_BASE}/README.md" "${dest}/README.md"
	fi
	ok "AccuracyCoin licence and README present"
	write_notice "$dest" "AccuracyCoin
https://github.com/100thCoin/AccuracyCoin

Licence: MIT, (c) 2025 Chris Siebert. Redistributable with attribution; kept
here anyway so every corpus is fetched the same way.

README.md carries the per-test error-code tables. The runner prints the codes;
that file says what they mean.

Consumed by tests/conformance/accuracycoin.rs."
}

# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

usage() {
	cat <<'EOF'
usage: scripts/fetch-testdata.sh [options] [suite ...]

Suites:
  sst-65x02      SingleStepTests 6502 vectors  (MIT, redistributable)
  nestest        nestest.nes + nestest.log     (licence unclear, FETCH-ONLY)
  accuracycoin   AccuracyCoin.nes + README     (MIT, (c) 2025 Chris Siebert)

Options:
  --all               fetch every suite (the default when none is named)
  --force             re-fetch even if a verified copy is present
  --variant V         sst-65x02 only: nes6502 (default), 6502, or all
  --opcodes LIST      sst-65x02 only: fetch just these opcode files,
                      e.g. --opcodes a9,ad,b1. Pairs with RSEMU_SST_OPCODES.
  --list              print what is present and exit
  -h, --help          this text

Destination: $RSEMU_TESTDATA, or <repo>/testdata. Nothing here is ever
committed; see docs/testing/conformance-suites.md for the licence table.

Then:
  RSEMU_CONFORMANCE=1 cargo test --test conformance -- --nocapture
EOF
}

list_present() {
	note "corpus root: ${DEST_ROOT}"
	if [ ! -d "$DEST_ROOT" ]; then
		note "  (nothing fetched yet)"
		return 0
	fi
	local suite
	for suite in sst-65x02 nestest accuracycoin; do
		if [ -d "${DEST_ROOT}/${suite}" ]; then
			printf '  %-14s %s\n' "$suite" \
				"$(du -sh "${DEST_ROOT}/${suite}" 2>/dev/null | cut -f1) present"
		else
			printf '  %-14s missing\n' "$suite"
		fi
	done
}

FORCE=0
SST_VARIANT="nes6502"
SST_OPCODES=""
SUITES=()

while [ $# -gt 0 ]; do
	case "$1" in
		--all) SUITES=(sst-65x02 nestest accuracycoin) ;;
		--force) FORCE=1 ;;
		--variant) SST_VARIANT="${2:?--variant needs a value}"; shift ;;
		--opcodes) SST_OPCODES="${2:?--opcodes needs a value}"; shift ;;
		--list) list_present; exit 0 ;;
		-h|--help) usage; exit 0 ;;
		-*) die "unknown option $1 (try --help)" ;;
		*) SUITES+=("$1") ;;
	esac
	shift
done

[ ${#SUITES[@]} -eq 0 ] && SUITES=(sst-65x02 nestest accuracycoin)

mkdir -p "$DEST_ROOT"
# Belt and braces: even if the repository .gitignore is edited, nothing under
# the corpus root can be added by accident.
printf '# Conformance corpora: downloaded, never committed. See\n# docs/testing/conformance-suites.md for why.\n*\n' \
	>"${DEST_ROOT}/.gitignore"

note "corpus root: ${DEST_ROOT}"
for suite in "${SUITES[@]}"; do
	note ""
	note "== ${suite}"
	case "$suite" in
		sst-65x02|sst) fetch_sst ;;
		nestest) fetch_nestest ;;
		accuracycoin|coin) fetch_accuracycoin ;;
		*) die "unknown suite $suite (try --help)" ;;
	esac
done

note ""
ok "done. Run the suites with:"
note "    RSEMU_CONFORMANCE=1 cargo test --test conformance -- --nocapture"
