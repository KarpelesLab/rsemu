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
  edk2           UEFI for riscv-virt, copied from the local qemu firmware
                 package (BSD-2-Clause-Patent; nothing to download)
  linux          riscv64 Linux Image to boot on it (GPL-2.0, FETCH-ONLY)

Options:
  --all               fetch every suite (the default when none is named).
                      Excludes wozmon, which has no source to fetch from,
                      and linux, which is 30 MiB nothing else needs.
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
	for suite in sst-65x02 nestest accuracycoin gb-blargg gb-mooneye apple1 riscv; do
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
		edk2|uefi) fetch_edk2 ;;
		linux|kernel) fetch_linux ;;
		*) die "unknown suite $suite (try --help)" ;;
	esac
done

note ""
ok "done. Run the suites with:"
note "    RSEMU_CONFORMANCE=1 cargo test --test conformance -- --nocapture"
