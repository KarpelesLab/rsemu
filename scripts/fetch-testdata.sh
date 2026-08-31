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
#
# One exception, and it is why `riscv-arch-test` is not in --all: that suite
# ships assembly rather than binaries, so it also needs clang and a RISC-V
# linker (lld, or the rust-lld a Rust toolchain already provides). It says so
# and stops if either is missing, rather than half-building a corpus.

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

# OpenSBI, the RISC-V supervisor binary interface firmware. BSD-2-Clause, so it
# may be read and used freely (ROADMAP.md §1) — it is fetched rather than
# committed only because binaries do not belong in this repository.
#
# Debian's package rather than an upstream release: upstream ships source
# tarballs, and building them needs a cross toolchain this script deliberately
# does not require. The .deb is a plain `ar` archive; `fw_jump.bin` inside it is
# already a flat image for 0x80000000.
readonly OPENSBI_DEB="https://deb.debian.org/debian/pool/main/o/opensbi/opensbi_1.6-1_all.deb"
readonly OPENSBI_DEB_SHA="dc4a43bd21a0ca11771ed8b19ee6fa8476d1d0f4976bd0b27c0357c699d27e1e"
readonly OPENSBI_MEMBER="./usr/lib/riscv64-linux-gnu/opensbi/generic/fw_jump.bin"

# A riscv64 Linux kernel, to boot on top of that firmware. The Debian
# installer's, because it is a bare `Image` with an EFI stub rather than a
# distribution package that has to be unpacked, and because it is rebuilt
# rarely enough to be a stable target.
#
# GPL-2.0. FETCH-ONLY, and that is the whole point of this script: running a
# GPL kernel as an emulated guest is ordinary use, while committing it here
# would be redistribution under its terms (ROADMAP.md §1).
readonly LINUX_IMAGE_URL="https://deb.debian.org/debian/dists/trixie/main/installer-riscv64/current/images/netboot/debian-installer/riscv64/linux"

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
# Game Boy
# ---------------------------------------------------------------------------
#
# Two suites, fetched together because a Game Boy bring-up wants both: blargg's
# ROMs are the quick gate and Gekkio's acceptance suite is the strict one. Both
# come out of c-sp/game-boy-test-roms, which is a *bundle* of the community's
# suites with each one's licence carried alongside it — that is the point of
# using it rather than five separate downloads.
readonly GB_BUNDLE_TAG="v7.0"
readonly GB_BUNDLE_URL="https://github.com/c-sp/game-boy-test-roms/releases/download/${GB_BUNDLE_TAG}/game-boy-test-roms-${GB_BUNDLE_TAG}.zip"

fetch_gameboy() {
	need curl
	need unzip
	local dest="${DEST_ROOT}/gb-bundle"
	local zip="${DEST_ROOT}/game-boy-test-roms-${GB_BUNDLE_TAG}.zip"

	if [ "$FORCE" = 1 ] || [ ! -d "${dest}/mooneye-test-suite" ]; then
		note "  downloading game-boy-test-roms ${GB_BUNDLE_TAG} ..."
		download "$GB_BUNDLE_URL" "$zip"
		rm -rf "$dest"
		mkdir -p "$dest"
		unzip -q -o "$zip" -d "$dest"
		rm -f "$zip"
	fi

	# The two suites the runners actually read, linked into stable directory
	# names so the environment variables below never mention a version.
	rm -rf "${DEST_ROOT}/gb-blargg" "${DEST_ROOT}/gb-mooneye"
	mkdir -p "${DEST_ROOT}/gb-blargg"
	# Only the individual `cpu_instrs` tests and `instr_timing`: the combined
	# `cpu_instrs.gb` is the same eleven tests again inside an MBC1 wrapper, and
	# running both doubles the time for no extra coverage.
	local found=0 rom
	while IFS= read -r rom; do
		cp "$rom" "${DEST_ROOT}/gb-blargg/"
		found=$((found + 1))
	done < <(find "${dest}/blargg" -path '*cpu_instrs/individual/*.gb' -o -name 'instr_timing.gb' 2>/dev/null)
	[ "$found" -gt 0 ] || warn "no blargg ROMs found in the bundle"
	ok "gb-blargg: ${found} ROMs"

	if [ -d "${dest}/mooneye-test-suite/acceptance" ]; then
		cp -r "${dest}/mooneye-test-suite/acceptance" "${DEST_ROOT}/gb-mooneye"
		ok "gb-mooneye: $(find "${DEST_ROOT}/gb-mooneye" -name '*.gb' | wc -l | tr -d ' ') ROMs"
	else
		warn "the bundle has no mooneye acceptance directory"
	fi

	write_notice "${DEST_ROOT}/gb-blargg" "blargg's Game Boy test ROMs
via https://github.com/c-sp/game-boy-test-roms (${GB_BUNDLE_TAG})

Licence: UNCLEAR. Shawn Hargreaves' (blargg's) ROMs have been passed around for
two decades without a stated licence. FETCH AND RUN ONLY — do not commit them,
do not vendor them, do not attach them to a release. Running one as an emulated
guest is ordinary use; redistributing it is not ours to do.

Consumed by RSEMU_GB_BLARGG_DIR in src/cpu/sm83/conformance.rs and
src/dev/gb/conformance.rs."

	write_notice "${DEST_ROOT}/gb-mooneye" "Gekkio/mooneye-test-suite
via https://github.com/c-sp/game-boy-test-roms (${GB_BUNDLE_TAG})

Licence: MIT, (c) Joonas Javanainen. Redistributable with attribution — but
this directory is git-ignored anyway, because every corpus is fetched the same
way. ROADMAP.md §12 names *mooneye-test-suite*, the suite, and not
*mooneye-gb*, which is the emulator: the suite is a fixture and the emulator is
somebody else's implementation, which §1 keeps us away from.

Consumed by RSEMU_GB_MOONEYE_DIR in src/dev/gb/conformance.rs."

	note ""
	note "  Then:"
	note "      RSEMU_CONFORMANCE=1 RSEMU_GB_BLARGG_DIR=${DEST_ROOT}/gb-blargg \\"
	note "      RSEMU_GB_MOONEYE_DIR=${DEST_ROOT}/gb-mooneye \\"
	note "      cargo test --release --all-features -- --nocapture conformance"
}

fetch_wozmon() {
	local dest="${DEST_ROOT}/apple1"
	local target="${dest}/wozmon.bin"

	if [ "$FORCE" = 0 ] && [ -s "$target" ]; then
		ok "wozmon.bin already present ($(wc -c <"$target" | tr -d ' ') bytes)"
		wozmon_notice "$dest"
		return 0
	fi

	# No default URL, and that is the point. The Woz Monitor's copyright status
	# is not clear — it has been passed around for decades, which is not a
	# licence — and there is no canonical, licence-stated download to hard-code.
	# rsemu will not pick a stranger's mirror on the user's behalf. Everything
	# rsemu ships works without this: `rsemu run apple1` boots RSMON, which is
	# ours and MIT (src/dev/apple1/monitor.rs).
	if [ -z "$WOZMON_URL" ]; then
		warn "no source for wozmon.bin, and this script will not guess one"
		note ""
		note "  The Woz Monitor's copyright status is unclear, so rsemu neither ships it"
		note "  nor picks a mirror for you. If you have a copy you may use, either:"
		note ""
		note "      cp /path/to/wozmon.bin ${target}"
		note "      scripts/fetch-testdata.sh --wozmon-url <url> wozmon"
		note ""
		note "  A 256-byte image whose last six bytes are the 6502's vectors is what"
		note "  the machine wants. Then:"
		note ""
		note "      RSEMU_APPLE1_ROM=${target} cargo test --all-features woz"
		note "      rsemu run apple1 --rom ${target}"
		note ""
		note "  docs/platforms/apple1.md has the long form."
		return 0
	fi

	need curl
	note "  downloading wozmon.bin from ${WOZMON_URL} ..."
	download "$WOZMON_URL" "$target"

	# No checksum to check against, so check the shape instead: 256 bytes, and a
	# reset vector that points into the page the ROM is decoded at. That catches
	# an HTML error page, an Intel-hex file, and a 4 KiB BASIC image, which are
	# the three things people actually end up with.
	local size
	size="$(wc -c <"$target" | tr -d ' ')"
	if [ "$size" -ne 256 ]; then
		rm -f "$target"
		die "that is ${size} bytes; the Apple 1's monitor socket holds 256"
	fi
	local vector
	vector="$(od -An -tx1 -j 252 -N 2 "$target" | tr -d ' \n')"
	case "$vector" in
		??ff) ok "wozmon.bin (256 bytes, reset vector \$FF${vector%ff})" ;;
		*)
			rm -f "$target"
			die "the reset vector at \$FFFC is \$${vector} — that is not an Apple 1 monitor ROM"
			;;
	esac
	wozmon_notice "$dest"
}

wozmon_notice() {
	write_notice "$1" "wozmon.bin — the Apple 1 monitor
Steve Wozniak, 1976. Source: whatever you pointed --wozmon-url at.

Licence: UNCLEAR. FETCH AND RUN ONLY — do not commit it, do not vendor it, do
not attach it to a release. Running it as an emulated guest is ordinary use;
redistributing it is not ours to do. This is the same rule nestest is under.

rsemu does not need it: \`rsemu run apple1\` boots RSMON, rsemu's own monitor,
which is MIT and committed (src/dev/apple1/monitor.rs).

Consumed by RSEMU_APPLE1_ROM in src/dev/apple1/tests.rs, and by
\`rsemu run apple1 --rom …\`."
}

fetch_opensbi() {
	need curl
	need tar
	need ar
	local dest="${DEST_ROOT}/riscv"
	local target="${dest}/fw_jump.bin"

	if [ "$FORCE" = 0 ] && [ -s "$target" ]; then
		ok "fw_jump.bin already present ($(wc -c <"$target" | tr -d ' ') bytes)"
		opensbi_notice "$dest"
		opensbi_hint "$target"
		return 0
	fi

	local work="${dest}/.unpack"
	rm -rf "$work"
	mkdir -p "$work"
	fetch_verified "$OPENSBI_DEB" "${work}/opensbi.deb" "$OPENSBI_DEB_SHA" advisory
	( cd "$work" && ar x opensbi.deb data.tar.xz ) || die "opensbi.deb is not an ar archive"
	tar -xf "${work}/data.tar.xz" -C "$work" "$OPENSBI_MEMBER" 2>/dev/null || \
		die "no ${OPENSBI_MEMBER} in the package; upstream may have moved it"
	mv -f "${work}/${OPENSBI_MEMBER}" "$target"
	rm -rf "$work"

	# Content check rather than a checksum of the firmware itself, since the
	# package may be rebuilt: every OpenSBI build carries its own banner, so
	# the string is in there. That catches an HTML error page, a truncated
	# download and the wrong member of the archive alike.
	local size
	size="$(wc -c <"$target" | tr -d ' ')"
	if [ "$size" -lt 32768 ] || ! grep -qa OpenSBI "$target"; then
		rm -f "$target"
		die "that is ${size} bytes and does not say OpenSBI anywhere; it is not the firmware"
	fi
	ok "fw_jump.bin (${size} bytes)"
	opensbi_notice "$dest"
	opensbi_hint "$target"
}

opensbi_notice() {
	write_notice "$1" "OpenSBI fw_jump.bin
${OPENSBI_DEB}
https://github.com/riscv-software-src/opensbi

Licence: BSD-2-Clause. Permissive, so it may be read as well as run — which is
unusual among the things in this directory and is why docs/platforms/riscv-virt.md
names it as the firmware for this board.

fw_jump.bin runs at 0x80000000 and hands control to 0x80200000 in S-mode. To
boot a kernel as well, concatenate: fw_jump.bin padded to 2 MiB, then a RISC-V
Image, and give the result to the one firmware slot.

Consumed by RSEMU_RISCV_FIRMWARE in src/dev/riscv/tests.rs, and by
\`rsemu run riscv-virt --media firmware=…\`."
}

opensbi_hint() {
	note ""
	note "  Boot it:"
	note "      RSEMU_RISCV_FIRMWARE=$1 \\"
	note "          cargo test --release --all-features riscv --lib -- --nocapture"
	note "      rsemu run riscv-virt --media firmware=$1"
}

linux_notice() {
	local dir="$1" text="$2"
	mkdir -p "$dir"
	printf '%s\n' "$text" >"${dir}/PROVENANCE-linux.txt"
}

# The EDK2 RISC-V build. Not downloaded: it ships in the distribution's `qemu`
# firmware package as `/usr/share/qemu/edk2-riscv-{code,vars}.fd`, and copying the
# file already on the machine beats guessing at a mirror URL. BSD-2-Clause-Patent,
# so it may be read as well as run; it is still not committed, because the rule
# is about the repository rather than about any one file.
fetch_edk2() {
	local dest="${DEST_ROOT}/riscv"
	local src="${RSEMU_EDK2_DIR:-/usr/share/qemu}"
	mkdir -p "$dest"

	if [ ! -r "${src}/edk2-riscv-code.fd" ]; then
		note "  no edk2-riscv-code.fd under ${src}"
		note "  install your distribution's qemu firmware package (Debian:"
		note "  qemu-efi-riscv64; elsewhere it comes with edk2/ovmf), or set"
		note "  RSEMU_EDK2_DIR to wherever the two .fd files are."
		return 0
	fi

	cp -f "${src}/edk2-riscv-code.fd" "${dest}/edk2-riscv-code.fd"
	# The variable store is copied rather than used in place because the whole
	# point of it is that the guest *writes* to it, and a run that scribbled on
	# the system's copy would be a surprise.
	cp -f "${src}/edk2-riscv-vars.fd" "${dest}/edk2-riscv-vars.fd"
	chmod u+w "${dest}/edk2-riscv-vars.fd"

	# Eight bytes bridging OpenSBI's compiled-in hand-off address to the flash
	# base: `lui t0, 0x20000` then `jr t0`, which leaves a0 and a1 alone. The
	# flash itself needs no trampoline; `fw_jump` does.
	printf '\xb7\x02\x00\x20\x67\x80\x02\x00' >"${dest}/tramp.bin"

	ok "edk2-riscv-code.fd ($(wc -c <"${dest}/edk2-riscv-code.fd" | tr -d ' ') bytes)"
	ok "edk2-riscv-vars.fd ($(wc -c <"${dest}/edk2-riscv-vars.fd" | tr -d ' ') bytes)"
	edk2_notice "$dest" "$src"
	edk2_hint "$dest"
}

edk2_notice() {
	# Its own file rather than PROVENANCE.txt, which the OpenSBI fetch owns:
	# both land in testdata/riscv and neither should erase the other.
	printf '%s\n' "EDK II for RISC-V (OvmfPkg/RiscVVirt), copied from ${2}
https://github.com/tianocore/edk2

Licence: BSD-2-Clause-Patent. Permissive, so it may be read as well as run.
Copied from the local qemu firmware package rather than downloaded; nothing
here is committed.

Consumed by RSEMU_RISCV_FLASH0 and RSEMU_RISCV_FLASH1 in src/dev/riscv/tests.rs.
docs/platforms/riscv-virt.md has the whole command line and says where it gets
to." >"${1}/PROVENANCE-edk2.txt"
}

edk2_hint() {
	local dest="$1"
	note ""
	note "  boot it with:"
	note "      RSEMU_RISCV_FIRMWARE=${dest}/fw_jump.bin \\"
	note "      RSEMU_RISCV_PAYLOAD=0x80200000:${dest}/tramp.bin \\"
	note "      RSEMU_RISCV_FLASH0=${dest}/edk2-riscv-code.fd \\"
	note "      RSEMU_RISCV_FLASH1=${dest}/edk2-riscv-vars.fd \\"
	note "      RSEMU_RISCV_FLASH1_OUT=${dest}/edk2-riscv-vars.fd \\"
	note "      RSEMU_RISCV_RAM=512M RSEMU_RISCV_QUANTA=6000000 \\"
	note "          cargo test --release --all-features firmware_from_the --lib -- --nocapture"
}

fetch_linux() {
	need curl
	local dest="${DEST_ROOT}/riscv"
	local target="${dest}/linux"
	mkdir -p "$dest"

	if [ "$FORCE" = 0 ] && [ -s "$target" ]; then
		ok "linux already present ($(wc -c <"$target" | tr -d ' ') bytes)"
	else
		note "  downloading linux (about 30 MiB) ..."
		download "$LINUX_IMAGE_URL" "$target"
	fi

	# No checksum: Debian rebuilds the installer kernel and pinning one would
	# make this script fail every point release. The header is the check that
	# matters anyway — a RISC-V `Image` carries "RISCV" at offset 0x30 and
	# "RSC\x05" at 0x38 (the boot-image header in Documentation/riscv), which
	# an HTML error page does not.
	local magic
	magic="$(dd if="$target" bs=1 skip=48 count=5 2>/dev/null || true)"
	if [ "$magic" != "RISCV" ]; then
		rm -f "$target"
		die "that is not a RISC-V Linux Image: no RISCV magic at offset 0x30"
	fi
	ok "linux ($(wc -c <"$target" | tr -d ' ') bytes)"
	# A separate file: this directory already holds OpenSBI's notice, and the
	# two licences are not the same one.
	linux_notice "$dest" "Debian installer riscv64 kernel
${LINUX_IMAGE_URL}

Licence: GPL-2.0. FETCH-ONLY — running it as an emulated guest is ordinary
use; committing it to this repository would be redistribution under its terms
(ROADMAP.md section 1).

A flat RISC-V \`Image\` with an EFI stub, loaded at 0x80200000, which is where
OpenSBI's fw_jump hands control on in S-mode.

Consumed by RSEMU_RISCV_PAYLOAD in src/dev/riscv/tests.rs."
	note ""
	note "  Boot it:"
	note "      RSEMU_RISCV_FIRMWARE=${dest}/fw_jump.bin \\"
	note "      RSEMU_RISCV_PAYLOAD=0x80200000:${target} \\"
	note "      RSEMU_RISCV_RAM=1G RSEMU_RISCV_QUANTA=4000000 \\"
	note "          cargo test --release --all-features firmware_from_the --lib -- --nocapture"
}

# ---------------------------------------------------------------------------
# riscv-arch-test
# ---------------------------------------------------------------------------
#
# The one suite here that is *built* rather than downloaded, because upstream
# ships assembly and nothing else. What lands in the corpus is a linked ELF per
# test plus the reference signature for it, which is everything
# tests/conformance/riscv.rs needs and nothing it does not.
#
# This replaces RISCOF (and its successor, the ACT4 framework), which wrap
# Python, Ruby, uv, mise and a UDB gem around two operations: decide which
# tests apply to the device under test and build them, then run a reference
# model to get the expected signatures. Both are done below, in shell, with no
# dependency a CI image does not already have except a compiler.
#
# Four inputs are ours and committed under scripts/riscv-arch-test:
# model_test.h (the DUT macros the suite asks every implementer to write),
# link.ld, and the two reference-model configuration overrides.

readonly ARCH_TEST_REPO="https://github.com/riscv-non-isa/riscv-arch-test"
# Pinned. 3.9.1 is the last tag of the RISCOF-era layout, where a test is a
# self-contained .S file under riscv-test-suite/rvNNi_m/<ext>/src. After it the
# repository was restructured around a framework that generates self-checking
# ELFs from a Sail run at build time, which needs Ruby, a UDB gem and mise;
# nothing about that produces a better measurement of this core.
readonly ARCH_TEST_TAG="3.9.1"

# The Sail RISC-V model, used as the reference. BSD-2-Clause — permissive, so
# it could be read; it is not, and does not need to be. It is run as a black
# box: give it an ELF, take the signature it writes.
#
# A prebuilt binary rather than a build from source: building Sail needs OCaml
# and a CMake toolchain, and none of that would change the answer.
readonly SAIL_RELEASE="2026-08-31-beaf449"
readonly SAIL_URL="https://github.com/riscv/sail-riscv/releases/download/${SAIL_RELEASE}/sail-riscv-Linux-x86_64.tar.gz"
# Advisory rather than fatal. A different build of the model is still a
# self-consistent reference — it regenerates every signature here — so a
# mismatch is worth saying out loud and not worth refusing. The version
# actually used is recorded in reference.txt, and the runner prints it.
readonly SAIL_SHA="61890dacb8dbf871941328e903990689caddea1918670d80e036c33f5192b1bd"

# The ISA strings the two configurations of rsemu's hart advertise, in the
# spelling riscv-arch-test's `check ISA:=regex(...)` clauses match against.
# These are `rsemu::cpu::riscv::Config::rv64gc` and `rv32gc` written out:
# RV64GC / RV32GC with machine, supervisor and user mode. Zicsr and Zifencei
# are named explicitly because `misa` has no bit for either and the suite's
# selectors ask for them by name.
readonly ARCH_TEST_ISA_RV64="RV64IMAFDCZicsr_Zifencei"
readonly ARCH_TEST_ISA_RV32="RV32IMAFDCZicsr_Zifencei"

# arch_test_isa <rv64|rv32>
arch_test_isa() {
	case "$1" in
		rv64) printf '%s' "$ARCH_TEST_ISA_RV64" ;;
		rv32) printf '%s' "$ARCH_TEST_ISA_RV32" ;;
		*) die "unknown architecture $1" ;;
	esac
}

# What ends up in the corpus's isa.txt, and what the runner prints. A reference
# signature only says anything about a hart that matches this string.
arch_test_isa_list() {
	local arch out=""
	for arch in $ARCH_TEST_ARCHS; do
		out="${out}${out:+, }$(arch_test_isa "$arch") (${arch})"
	done
	printf '%s' "$out"
}

# The extension directories that apply to that hart. Everything else in the
# suite — B, K, Zfh, Zfinx, Zicond, Zacas, Zcmop, Zimop, CMO, Svadu, Zfa and
# P_unratified — tests an extension rsemu does not implement, and building it
# would be asking the reference model what an instruction rsemu treats as
# illegal ought to return.
readonly ARCH_TEST_SUITES="I M A F D C Zifencei privilege"

# RV64 only, and this is a size decision rather than a coverage one. The RV32
# half of the suite is the same tests again for a 32-bit hart, and rsemu's core
# runs both from one `Config::xlen` — but rv32i_m/D alone is 313 MB of source,
# including four generated fused-multiply-add tests of 63 MB each, which take
# minutes apiece to assemble and produce signatures to match. riscv-tests
# already covers RV32 (its 409 binaries include the rv32ui, rv32um, rv32ua,
# rv32uf, rv32ud, rv32uc, rv32si and rv32mi families), so what would be bought
# here is a second opinion at a hundredfold the cost.
#
# tests/conformance/riscv.rs picks the hart's width from the ELF class, so
# adding rv32 back is this one word plus the disk to hold it.
readonly ARCH_TEST_ARCHS="rv64"

# Named DUT parameters the suite's selectors ask about, beyond the ISA string.
# `hw_data_misaligned_support` is the only one the selected directories use:
# rsemu performs misaligned loads and stores in hardware
# (`Config::misaligned`, default true), which is what the reference model is
# configured for too, so the tests written for the trapping variant are the
# ones that do not apply.
arch_test_param_ok() {
	case "$1" in
		hw_data_misaligned_support:=True) return 0 ;;
		hw_data_misaligned_support:=False) return 1 ;;
		# An unrecognised check is accepted rather than refused: refusing would
		# quietly shrink the corpus, and a test built for a parameter we do not
		# model shows up as a signature difference, which is loud.
		*) return 0 ;;
	esac
}

# arch_test_defines <file> <isa>
#
# RISCOF's test selection, in one function. Every test carries one or more
# `RVTEST_CASE(n,"...",name)` macros whose string holds `check` clauses saying
# which DUT the case is for and `def X=Y` clauses saying how to build it. The
# first case whose checks all pass wins, and its defines become -D flags. A
# test where no case passes is not for this hart and is not built.
arch_test_defines() {
	local file="$1" isa="$2"
	local line clause defs applicable regex body saved_ifs
	while IFS= read -r line; do
		defs=""
		applicable=1
		# Everything between the first and last double quote is the case string.
		body="$(printf '%s' "$line" | sed -e 's/^[^"]*"//' -e 's/"[^"]*$//')"
		saved_ifs="$IFS"
		IFS=';'
		for clause in $body; do
			IFS="$saved_ifs"
			clause="$(printf '%s' "$clause" |
				sed -e 's|^//||' -e 's/^[[:space:]]*//' -e 's/[[:space:]]*$//')"
			case "$clause" in
				"check ISA:=regex("*)
					regex="${clause#check ISA:=regex(}"
					regex="${regex%)}"
					printf '%s' "$isa" | grep -Eq "$regex" || applicable=0
					;;
				"check "*)
					arch_test_param_ok "${clause#check }" || applicable=0
					;;
				"def "*)
					defs="$defs -D${clause#def }"
					;;
			esac
			IFS=';'
		done
		IFS="$saved_ifs"
		if [ "$applicable" = 1 ]; then
			printf '%s\n' "$defs"
			return 0
		fi
	done < <(grep -h 'RVTEST_CASE(' "$file")
	return 1
}

# Find something that links RISC-V ELFs. clang can *assemble* for RISC-V out of
# the box but cannot link without a cross linker; lld can, and a Rust toolchain
# already ships one as rust-lld. Invoked through a symlink named ld.lld because
# that is how lld picks the ELF flavour, and how -fuse-ld=lld finds it.
arch_test_linker() {
	local bindir="$1" sysroot
	mkdir -p "$bindir"
	if command -v ld.lld >/dev/null 2>&1; then
		ln -sf "$(command -v ld.lld)" "${bindir}/ld.lld"
		return 0
	fi
	if command -v rustc >/dev/null 2>&1; then
		sysroot="$(rustc --print sysroot)"
		local lld
		lld="$(find "${sysroot}/lib/rustlib" -name rust-lld -type f 2>/dev/null | head -n1)"
		if [ -n "$lld" ]; then
			ln -sf "$lld" "${bindir}/ld.lld"
			return 0
		fi
	fi
	die "no RISC-V linker found
  This suite is built, not downloaded, so it needs a compiler and a linker.
  Either put lld on PATH (Debian: apt install lld; the binary is ld.lld), or
  install a Rust toolchain — rustup ships rust-lld, which this script will
  find and use."
}

# Everything that decides what the corpus should contain, hashed. Change any
# of it and the corpus is rebuilt; change none of it and a re-run is a no-op.
arch_test_stamp() {
	{
		printf '%s\n' "$ARCH_TEST_TAG" "$SAIL_RELEASE" "$ARCH_TEST_ARCHS" \
			"$ARCH_TEST_SUITES" "$(arch_test_isa_list)"
		cat "${SCRIPT_DIR}/riscv-arch-test/model_test.h" \
			"${SCRIPT_DIR}/riscv-arch-test/link.ld" \
			"${SCRIPT_DIR}/riscv-arch-test/sail-config.json" \
			"${SCRIPT_DIR}/riscv-arch-test/sail-config-rv32.json"
	} | sha256_of /dev/stdin
}

# arch_test_notbuilt <out> <test> <why> <log>
#
# A test that could not be built or referenced is written down rather than
# merely warned about. The runner prints the count of this file, so a corpus
# that is quietly missing a third of the D tests cannot pass for a complete
# one — which is the difference between a measurement and a number.
arch_test_notbuilt() {
	local out="$1" test="$2" why="$3" log="$4"
	warn "${test}: ${why}"
	sed -n 1,4p "$log" >&2
	printf '%s\t%s\n' "$test" "$why" >>"${out}/notbuilt.txt"
	AT_FAILED=$((AT_FAILED + 1))
}

# arch_test_build <arch> <suite-src-root> <sail> <out>
arch_test_build() {
	local arch="$1" src_root="$2" sail="$3" out="$4"
	local dir isa target march mabi xlen
	local -a sail_args
	isa="$(arch_test_isa "$arch")"
	dir="${arch}i_m"
	if [ "$arch" = rv64 ]; then
		target=riscv64-unknown-elf; march=rv64imafdc_zicsr_zifencei; mabi=lp64d; xlen=64
		sail_args=(--config-override="${SCRIPT_DIR}/riscv-arch-test/sail-config.json")
	else
		target=riscv32-unknown-elf; march=rv32imafdc_zicsr_zifencei; mabi=ilp32d; xlen=32
		sail_args=(--rv32
			--config-override="${SCRIPT_DIR}/riscv-arch-test/sail-config.json"
			--config-override="${SCRIPT_DIR}/riscv-arch-test/sail-config-rv32.json")
	fi

	local suite src name elf defs log
	log="${out}/.build.log"
	for suite in $ARCH_TEST_SUITES; do
		[ -d "${src_root}/${dir}/${suite}/src" ] || continue
		mkdir -p "${out}/elf/${arch}-${suite}" "${out}/ref/${arch}-${suite}"
		for src in "${src_root}/${dir}/${suite}/src/"*.S; do
			name="$(basename -- "$src" .S)"
			if ! defs="$(arch_test_defines "$src" "$isa")"; then
				printf '%s\n' "${arch}-${suite}/${name}" >>"${out}/skipped.txt"
				AT_SKIPPED=$((AT_SKIPPED + 1))
				continue
			fi
			elf="${out}/elf/${arch}-${suite}/${name}.elf"
			# -DFLEN=64: arch_test.h needs the float register width and has no
			# default for it. -Wl,--no-relax: linker relaxation would rewrite
			# the auipc/addi pairs the tests use to compute their own addresses,
			# and several of them measure exactly that.
			# shellcheck disable=SC2086
			if ! clang --target="$target" -march="$march" -mabi="$mabi" \
				-static -mcmodel=medany -fvisibility=hidden -nostdlib -nostartfiles \
				-fuse-ld=lld -Wl,-T,"${SCRIPT_DIR}/riscv-arch-test/link.ld" -Wl,--no-relax \
				-DXLEN=$xlen -DFLEN=64 $defs \
				-I "${src_root}/env" -I "${SCRIPT_DIR}/riscv-arch-test" \
				"$src" -o "$elf" >"$log" 2>&1
			then
				arch_test_notbuilt "$out" "${arch}-${suite}/${name}" "assembly failed" "$log"
				rm -f "$elf"
				continue
			fi
			# The reference. An ELF whose signature could not be generated is
			# deleted rather than left behind: the runner treats an image with
			# no reference as a broken corpus, which is exactly what it is.
			if ! "$sail" "${sail_args[@]}" \
				--test-signature="${out}/ref/${arch}-${suite}/${name}.sig" \
				--signature-granularity=4 --inst-limit=20000000 "$elf" >"$log" 2>&1
			then
				arch_test_notbuilt "$out" "${arch}-${suite}/${name}" \
					"the reference model would not run it" "$log"
				rm -f "$elf" "${out}/ref/${arch}-${suite}/${name}.sig"
				continue
			fi
			# SUCCESS is the model's word for "the test reached RVMODEL_HALT".
			# Without it the run hit the instruction limit, and the signature
			# on disk is a snapshot of an unfinished test — which would be
			# compared against rsemu's finished one and diffed as a failure of
			# this core. That is a wrong answer, so it is thrown away.
			if ! grep -q '^SUCCESS' "$log"; then
				arch_test_notbuilt "$out" "${arch}-${suite}/${name}" \
					"the reference model did not reach RVMODEL_HALT" "$log"
				rm -f "$elf" "${out}/ref/${arch}-${suite}/${name}.sig"
				continue
			fi
			AT_BUILT=$((AT_BUILT + 1))
		done
	done
	rm -f "$log"
}

fetch_arch_test() {
	need git
	need curl
	need tar
	command -v clang >/dev/null 2>&1 || die "clang is required to build riscv-arch-test
  The suite ships assembly, so something has to assemble it. Any clang that
  knows the riscv64 and riscv32 targets will do; check with
  \`clang -print-targets | grep riscv\`."

	local dest="${DEST_ROOT}/riscv-arch-test"
	local work="${dest}/.work"
	local src="${work}/suite"
	local sail_dir="${work}/sail"
	local bindir="${work}/bin"
	mkdir -p "$dest" "$work"

	# The suite itself. Blobless and sparse, narrowed to the extension
	# directories that apply to this hart: a full checkout at this tag is about
	# 800 MB, most of it the P_unratified, Zfinx, Zfh, K and B tests that will
	# never be built here.
	if [ "$FORCE" = 1 ] || [ ! -d "${src}/riscv-test-suite/env" ]; then
		note "  cloning ${ARCH_TEST_REPO} at ${ARCH_TEST_TAG} ..."
		rm -rf "$src"
		git clone --depth 1 --branch "$ARCH_TEST_TAG" --single-branch \
			--filter=blob:none --sparse "$ARCH_TEST_REPO" "$src" >/dev/null 2>&1 ||
			die "could not clone ${ARCH_TEST_REPO} at ${ARCH_TEST_TAG}"
		local -a sparse=(/COPYING.BSD /riscv-test-suite/env)
		local base suite
		for base in $ARCH_TEST_ARCHS; do
			for suite in $ARCH_TEST_SUITES; do
				sparse+=("/riscv-test-suite/${base}i_m/${suite}")
			done
		done
		git -C "$src" sparse-checkout set --no-cone "${sparse[@]}" >/dev/null 2>&1 ||
			die "could not narrow the checkout to the applicable test directories"
	fi
	[ -d "${src}/riscv-test-suite/env" ] ||
		die "${src} has no riscv-test-suite/env; the checkout is not what this script expects"
	ok "riscv-arch-test ${ARCH_TEST_TAG} checked out"

	# The reference model.
	if [ "$FORCE" = 1 ] || [ ! -x "${sail_dir}/bin/sail_riscv_sim" ]; then
		fetch_verified "$SAIL_URL" "${work}/sail.tar.gz" "$SAIL_SHA" advisory
		rm -rf "$sail_dir"
		mkdir -p "${work}/sail-unpack"
		tar -xzf "${work}/sail.tar.gz" -C "${work}/sail-unpack"
		# The tarball holds one top-level directory whose name carries the
		# platform; move whatever it is called into place.
		mv "${work}/sail-unpack"/*/ "$sail_dir"
		rm -rf "${work}/sail-unpack" "${work}/sail.tar.gz"
	fi
	[ -x "${sail_dir}/bin/sail_riscv_sim" ] ||
		die "the Sail release does not contain bin/sail_riscv_sim"
	local sail_version
	sail_version="$("${sail_dir}/bin/sail_riscv_sim" --version 2>&1 | head -n1)"
	ok "reference model: ${sail_version}"

	arch_test_linker "$bindir"
	export PATH="${bindir}:${PATH}"

	# All or nothing, keyed by a stamp over every input that can change what a
	# signature ought to be: the suite tag, the reference model release, the ISA
	# and directory selection, and the four DUT files. A half-rebuilt corpus —
	# some references from the old configuration, some from the new — is the one
	# failure this suite cannot survive, because a stale reference is a *wrong*
	# answer rather than a missing one. So the corpus is either current for this
	# stamp and left alone, or thrown away and rebuilt whole.
	local stamp
	stamp="$(arch_test_stamp)"
	if [ "$FORCE" = 0 ] && [ -d "${dest}/elf" ] &&
		[ "$(cat "${dest}/build-stamp.txt" 2>/dev/null)" = "$stamp" ]; then
		ok "corpus already built for this configuration ($(
			find "${dest}/elf" -name '*.elf' | wc -l | tr -d ' '
		) tests); --force rebuilds"
		arch_test_hint
		return 0
	fi
	rm -rf "${dest}/elf" "${dest}/ref" "${dest}/skipped.txt" \
		"${dest}/notbuilt.txt" "${dest}/build-stamp.txt"
	: >"${dest}/skipped.txt"
	: >"${dest}/notbuilt.txt"
	AT_BUILT=0
	AT_SKIPPED=0
	AT_FAILED=0
	note "  building and generating reference signatures (a few minutes) ..."
	local arch
	for arch in $ARCH_TEST_ARCHS; do
		arch_test_build "$arch" "${src}/riscv-test-suite" \
			"${sail_dir}/bin/sail_riscv_sim" "$dest"
	done

	[ "$AT_BUILT" -gt 0 ] || die "not one test was built; see the errors above"
	[ "$AT_FAILED" -eq 0 ] ||
		warn "${AT_FAILED} test(s) could not be built or referenced — the corpus is incomplete"
	ok "${AT_BUILT} test(s) built, ${AT_SKIPPED} not applicable to this hart"

	printf '%s\n' "$stamp" >"${dest}/build-stamp.txt"
	printf '%s %s\n' "riscv-arch-test" "$ARCH_TEST_TAG" >"${dest}/version.txt"
	printf '%s\n' "$(arch_test_isa_list)" >"${dest}/isa.txt"
	printf '%s, %s\n' "$sail_version" "$SAIL_RELEASE" >"${dest}/reference.txt"
	arch_test_notice "$dest"
	arch_test_hint
}

arch_test_notice() {
	write_notice "$1" "riscv-arch-test ${ARCH_TEST_TAG}
${ARCH_TEST_REPO}

Licence: BSD-3-Clause, (c) RISC-V International — verified against the
upstream COPYING.BSD and the SPDX-License-Identifier line at the head of every
test. Permissive, so it may be read as well as run, which is why
scripts/riscv-arch-test/model_test.h could be written from arch_test.h's
documented contract. It is still not committed: the rule is about this
repository, not about any one licence.

What is here is *built*, not downloaded. Each elf/<arch>-<ext>/<name>.elf was
assembled from the upstream .S with rsemu's own model_test.h and link.ld; each
ref/<arch>-<ext>/<name>.sig is the signature the Sail RISC-V model
(BSD-2-Clause, https://github.com/riscv/sail-riscv, release ${SAIL_RELEASE})
produced for it, with the model configured by
scripts/riscv-arch-test/sail-config.json to describe the same hart rsemu does.
Sail was run, never read.

skipped.txt lists the tests whose RVTEST_CASE selectors exclude this hart.

Consumed by tests/conformance/riscv.rs."
}

arch_test_hint() {
	note ""
	note "  Run it:"
	note "      RSEMU_CONFORMANCE=1 cargo test --release --all-features \\"
	note "          --test conformance -- --nocapture riscv"
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
  gameboy        blargg GB + mooneye acceptance (mixed; see the notices)
  wozmon         the Apple 1 monitor           (licence unclear, BRING YOUR OWN)
  opensbi        RISC-V firmware for riscv-virt (BSD-2-Clause, redistributable)
  riscv-arch-test  the RISC-V architectural certification tests (BSD-3-Clause).
                 Built rather than downloaded: needs clang and a RISC-V linker
                 (lld, or rustup's rust-lld), and fetches the Sail reference
                 model to generate the expected signatures. Takes a few
                 minutes, which is why --all leaves it out.
  edk2           UEFI for riscv-virt, copied from the local qemu firmware
                 package (BSD-2-Clause-Patent; nothing to download)
  linux          riscv64 Linux Image to boot on it (GPL-2.0, FETCH-ONLY)

Options:
  --all               fetch every suite (the default when none is named).
                      Excludes wozmon, which has no source to fetch from,
                      linux, which is 30 MiB nothing else needs, and
                      riscv-arch-test, which needs a compiler.
  --force             re-fetch even if a verified copy is present
  --wozmon-url URL    wozmon only: where to fetch a 256-byte monitor image
                      from. There is no default and there will not be one;
                      see docs/platforms/apple1.md.
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
	for suite in sst-65x02 nestest accuracycoin gb-blargg gb-mooneye apple1 riscv \
		riscv-arch-test; do
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
WOZMON_URL="${RSEMU_WOZMON_URL:-}"
SUITES=()

while [ $# -gt 0 ]; do
	case "$1" in
		--all) SUITES=(sst-65x02 nestest accuracycoin gameboy opensbi) ;;
		--force) FORCE=1 ;;
		--variant) SST_VARIANT="${2:?--variant needs a value}"; shift ;;
		--opcodes) SST_OPCODES="${2:?--opcodes needs a value}"; shift ;;
		--wozmon-url) WOZMON_URL="${2:?--wozmon-url needs a value}"; shift ;;
		--list) list_present; exit 0 ;;
		-h|--help) usage; exit 0 ;;
		-*) die "unknown option $1 (try --help)" ;;
		*) SUITES+=("$1") ;;
	esac
	shift
done

[ ${#SUITES[@]} -eq 0 ] && SUITES=(sst-65x02 nestest accuracycoin gameboy opensbi)

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
		gameboy|gb) fetch_gameboy ;;
		wozmon|apple1) fetch_wozmon ;;
		opensbi|riscv) fetch_opensbi ;;
		riscv-arch-test|arch-test|act) fetch_arch_test ;;
		edk2|uefi) fetch_edk2 ;;
		linux|kernel) fetch_linux ;;
		*) die "unknown suite $suite (try --help)" ;;
	esac
done

note ""
ok "done. Run the suites with:"
note "    RSEMU_CONFORMANCE=1 cargo test --test conformance -- --nocapture"
