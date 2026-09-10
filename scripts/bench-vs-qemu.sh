#!/usr/bin/env bash
#
# The other half of ROADMAP.md phase 8's gate: rsemu against QEMU, wall clock,
# on the same guest doing the same work.
#
# Every performance number this project had before this script was
# self-referential -- callgrind against its own previous baseline. That is a
# fine instrument for attributing a change and a useless one for answering "is
# this fast?", and the answer has already gone wrong twice: `MAX_INSNS` was
# tuned against a cost that turned out to be in the runtime, and a "4% of a
# frame" figure quoted in two files priced a design nobody had built. An
# external reference is the fix, and the roadmap names which one.
#
# QEMU is GPLv2 and its source is permanently off limits (ROADMAP.md section 1,
# CLAUDE.md). **Running** it is not. This script starts it, reads what the
# guest prints on its serial line, and stops a clock -- black-box use of a
# program as a measuring instrument, which section 1 permits in the same
# sentence that forbids reading it. Nothing here was informed by how QEMU
# works, and nothing here may be.
#
# ---------------------------------------------------------------------------
# What it measures, and why it measures it that way
# ---------------------------------------------------------------------------
#
# "The same work" is the hard part of the comparison, because the two
# emulators have no common notion of time. rsemu is driven by `--for
# <virtual duration>`; QEMU without `-icount` has no virtual timeline at all.
# Equating the two would be a modelling decision, and a wrong one would silently
# become the answer.
#
# So neither is used. **The guest says when it is done.** Both emulators are
# handed the same kernel, the same initramfs, the same command line and a
# `/bench` init that prints a marker line before and after each phase of a
# fixed workload. The harness timestamps the markers on the *host* as they come
# out of the serial line, and stops the process at the last one. What is
# compared is host wall-clock seconds to drive one guest from reset to a fixed
# point in its own execution -- which needs no equivalence between the two
# timelines, and cannot be gamed by either side's idea of a second.
#
# The guest also prints a digest of what each phase produced, and a run whose
# digest disagrees with the others is refused. That is what makes it the *same*
# work rather than two programs with the same name.
#
# ---------------------------------------------------------------------------
# Measurement discipline, which this host demands
# ---------------------------------------------------------------------------
#
# `benches/a64_linux_boot.rs` says it plainly: the wall clock on a developer
# machine drifts several nanoseconds per instruction under concurrent load and
# whole seconds under a build. Three things follow, and all three are
# implemented here rather than recommended:
#
#   * Runs are **interleaved**, never A-then-B. One repetition runs every side
#     once, and the side that goes first rotates, so a machine that gets slower
#     over the afternoon costs each side the same.
#   * The reported figure is the **minimum** over repetitions, with the median
#     and the spread beside it. A minimum is the run least interfered with;
#     the spread is the harness reporting its own noise floor, and a ratio
#     quoted without it is not a measurement.
#   * The sides are **not** run concurrently. Two emulators on one machine
#     measure the memory system, not each other.
#
# ---------------------------------------------------------------------------
# Where the comparison is not fair, stated up front
# ---------------------------------------------------------------------------
#
#   * **Per-access cycle accounting.** rsemu always does it -- it is how the
#     bus charges time (ROADMAP.md 4.2) and it is about a fifth of its host
#     instructions. QEMU does it only under `-icount`. So QEMU is measured
#     BOTH ways and both ratios are published: `qemu` is what a person gets by
#     typing `qemu-system-...`, and `qemu-icount` is the nearer thing to what
#     rsemu is doing. Neither alone is the honest number.
#   * **Idle.** Under `-icount ...,sleep=off` and under rsemu, a guest that
#     waits on a timer costs no host time -- the scheduler moves virtual time
#     to the next event. Plain QEMU waits in real seconds. Every second the
#     guest spends asleep is therefore charged to plain QEMU and to nobody
#     else, which flatters rsemu. The workload is built to have almost no idle
#     in it for exactly this reason, and `boot` -- the one phase that has some
#     -- is reported separately so it can be discounted.
#   * **The boards are not the same board.** `arm64-virt` is close: one core, a
#     GICv2, a PL011, PSCI, two virtio-MMIO devices. `pc64` is not: it has no
#     PCI, no APIC and no ACPI, so QEMU is run on `microvm` (which also has
#     none of those) rather than on `pc`. It is still a different device set,
#     and the x86 leg is the weaker of the two comparisons because of it.
#   * **KVM is off on both sides**, explicitly (`-accel tcg`, and rsemu without
#     `--accel`). TCG against rsemu's JIT is the comparison the gate is about;
#     against KVM it would be a comparison of two host CPUs.
#   * **One vCPU on both sides.** rsemu's default threading is deterministic
#     and single-threaded; QEMU gets `-smp 1`.
#   * **Marker latency.** A marker is timestamped when the byte reaches this
#     process, not when the guest stored it, so each phase boundary carries one
#     UART's worth of latency on each side. It is milliseconds against phases
#     measured in seconds, but it is not zero, and it is why no phase here is
#     sized to finish in under about a second.
#
# ---------------------------------------------------------------------------
# Usage
# ---------------------------------------------------------------------------
#
#   scripts/fetch-testdata.sh arm64-linux arm64-initramfs x86-linux initramfs-x86
#   scripts/bench-vs-qemu.sh                       # both guests, 5 repetitions
#   scripts/bench-vs-qemu.sh --guest arm64 --reps 3
#   scripts/bench-vs-qemu.sh --engine interp       # the oracle, for scale
#   scripts/bench-vs-qemu.sh --sides rsemu,qemu    # skip the icount leg
#
# It skips loudly and exits 0 when QEMU or a fixture is absent, the way
# `check.sh`'s `crosshost` and `long` stages do; RSEMU_BENCH_REQUIRED=1 turns
# those skips into failures, for a runner that installed everything on purpose.
#
# ---------------------------------------------------------------------------
# One failure shape that is not a defect in anything
# ---------------------------------------------------------------------------
#
# **Do not edit this file while a run of it is in progress.** bash reads a
# script lazily, by byte offset, so inserting a line near the top while the run
# loop near the bottom is executing moves every offset after it; when the
# interpreter next seeks -- which is after the loop, at the report -- it lands
# mid-statement and dies with a syntax error on a line that is not wrong. It
# cost a complete five-repetition arm64 leg here: every sample was collected,
# the per-run totals reached stderr, and the report that would have aggregated
# them never ran. `check.sh`'s header records the same class of thing about
# shared target directories. If a run of this dies with a syntax error, look at
# the file's mtime before you look at the line it names.
#
# Dependencies: bash 5 (for $EPOCHREALTIME), cpio, and a qemu-system for each
# guest. No python, no bc.

set -uo pipefail

# Not decoration. `$EPOCHREALTIME` and `sort -g` both follow LC_NUMERIC, so in a
# comma-decimal locale the split in `us_of` finds no `.` and every duration
# comes out as garbage rather than as an error.
export LC_ALL=C

readonly SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
readonly TESTDATA="${RSEMU_TESTDATA:-${REPO_ROOT}/testdata}"

# The marker prefix the guest prints and this script greps for. Deliberately
# not a word that appears in kernel output.
readonly MARK='@@RSEMU-BENCH'

# The build the published figures are taken with. Every board and both lifted
# cores, so one binary runs every leg; `trace` is left out because a counter
# nothing reads is still a branch in the dispatch loop.
readonly FEATURES='cli,machine-arm64-virt,cpu-arm-a64-lift,jit,jit-x86,machine-pc64,cpu-x86-lift'

# ---------------------------------------------------------------------------
# Options
# ---------------------------------------------------------------------------

GUESTS=(arm64 x86)
SIDES=(rsemu qemu qemu-icount)
REPS=5
ENGINE=jit-host
ICOUNT='shift=0,sleep=off'
# Per run, host seconds. A side that does not reach the last marker inside this
# is recorded as a timeout rather than as a fast run.
TIMEOUT=1800
WORK=""
KEEP=0
RSEMU_BIN="${RSEMU_BIN:-}"

# The file's own header, which is the documentation. Printed by matching the
# comment block rather than by a line range, so it cannot go stale the next
# time a paragraph is added to it.
usage() {
	sed -n '2,/^[^#]/p' "${BASH_SOURCE[0]}" | sed '$d; s/^#\{1,\} \{0,1\}//'
	exit "${1:-0}"
}

while [ $# -gt 0 ]; do
	case "$1" in
	--guest) IFS=, read -r -a GUESTS <<<"$2"; shift 2 ;;
	--sides) IFS=, read -r -a SIDES <<<"$2"; shift 2 ;;
	--reps) REPS="$2"; shift 2 ;;
	--engine) ENGINE="$2"; shift 2 ;;
	--icount) ICOUNT="$2"; shift 2 ;;
	--timeout) TIMEOUT="$2"; shift 2 ;;
	--work) WORK="$2"; shift 2 ;;
	--keep) KEEP=1; shift ;;
	-h | --help) usage 0 ;;
	*) echo "bench-vs-qemu: unknown option $1" >&2; usage 2 ;;
	esac
done

[ "${GUESTS[0]}" = all ] && GUESTS=(arm64 x86)

# ---------------------------------------------------------------------------
# Saying nothing happened, loudly
# ---------------------------------------------------------------------------

SKIPPED=0

skip() {
	if [ -n "${RSEMU_BENCH_REQUIRED:-}" ]; then
		printf 'FAIL  %s -- RSEMU_BENCH_REQUIRED is set, so this had to run\n' "$1" >&2
		exit 1
	fi
	printf 'skip  %s\n' "$1" >&2
	SKIPPED=$((SKIPPED + 1))
}

die() { printf 'bench-vs-qemu: %s\n' "$1" >&2; exit 1; }

# ---------------------------------------------------------------------------
# The workload
# ---------------------------------------------------------------------------
#
# Four phases, and each is here because it exercises a different part of the
# translation pipeline. Sizes are chosen so that every phase takes something
# like a second under QEMU -- long enough that a marker's millisecond of UART
# latency does not matter, short enough that a whole repetition of the slower
# side stays in minutes.
#
#   boot   reset to `/init`. A kernel's early boot is the least synthetic
#          workload there is: MMU bring-up, exception vectors, the whole
#          initcall list, a few thousand distinct basic blocks executed once
#          each. It is the phase where translation *cost* shows up rather than
#          translated-code quality, and it is the only phase with idle in it.
#   hash   sha256 of 8 MiB already in memory. A tight integer kernel with no
#          branches worth predicting and no memory traffic outside L2 --
#          about as close to "how good is the generated code" as a real
#          program gets.
#   awk    an interpreter loop: 300 000 iterations of arithmetic through
#          busybox awk's own dispatch. Branchy, pointer-chasing, and it re-enters
#          the same handful of blocks millions of times, so it is where block
#          chaining and cross-block register allocation are worth something.
#   gzip   deflate over the busybox binary. Table lookups and unaligned
#          accesses at a rate the other three do not reach, which makes it the
#          phase that prices the software TLB -- 18% of the profile in
#          `benches/a64_linux_boot.rs`.
#
# The digest line at the end is what makes the comparison a comparison: it is
# the sha256 prefix of the hashed blob, the awk sum and the compressed size,
# and every side of every repetition must print the same one.
guest_init() {
	cat <<EOF
#!/bin/sh
# The workload half of scripts/bench-vs-qemu.sh. Runs as PID 1 out of the
# initramfs; \`rdinit=/bench\` on the kernel command line is what selects it.
/bin/busybox --install -s /bin 2>/dev/null
mount -t proc     proc     /proc 2>/dev/null
mount -t sysfs    sysfs    /sys  2>/dev/null
mount -t devtmpfs devtmpfs /dev  2>/dev/null
echo "${MARK} boot end"

# Untimed: building the input is host-visible work but it is I/O through the
# page cache, not the thing being measured.
dd if=/dev/zero of=/blob bs=65536 count=128 2>/dev/null

echo "${MARK} hash begin"
H=\$(sha256sum /blob | cut -c1-16)
echo "${MARK} hash end"

echo "${MARK} awk begin"
A=\$(awk 'BEGIN{s=0;for(i=0;i<300000;i++)s+=i%7;print s}')
echo "${MARK} awk end"

echo "${MARK} gzip begin"
G=\$(gzip -6 -c /bin/busybox | wc -c)
echo "${MARK} gzip end"

echo "${MARK} digest \$H/\$A/\$G"
echo "${MARK} all done"
# Both emulators are stopped by the harness at the marker above; this is only
# so that a hand-run of the same command line ends by itself.
poweroff -f 2>/dev/null
exec /bin/sh
EOF
}

readonly PHASES=(boot hash awk gzip total)

# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

# The initramfs each guest boots: whatever `fetch-testdata.sh` built, with one
# more file concatenated onto it.
#
# Concatenated rather than rebuilt, and under a new name rather than replacing
# `/init`, because both properties make this safe: the kernel's initramfs
# unpacker reads archives back to back, and a name nothing else uses cannot
# collide with what the fetched archive already contains. So the fixture stays
# exactly the one the fetch script's provenance file describes -- busybox and
# nothing else -- and what this script adds is 1 KiB of shell it wrote itself.
build_initramfs() {
	local base="$1" out="$2" dir="${WORK}/appendix"
	rm -rf "$dir"
	mkdir -p "$dir"
	guest_init >"${dir}/bench"
	chmod 755 "${dir}/bench"
	(cd "$dir" && printf 'bench\n' | cpio -o -H newc --quiet >"${WORK}/appendix.cpio") ||
		die "cpio would not build the appendix archive"
	cat "$base" "${WORK}/appendix.cpio" >"$out"
}

# ---------------------------------------------------------------------------
# The two command lines, per guest
# ---------------------------------------------------------------------------
#
# Written out in full rather than assembled from a table, because the whole
# claim of this script is that the two sides were given the same guest and the
# same command line, and a reader has to be able to check that by eye.

CMDLINE_ARM64='earlycon=pl011,0x9000000 console=ttyAMA0 rdinit=/bench'
# `cryptomgr.notests` is on both sides for the reason docs/platforms/pc64.md
# gives: the crypto self-tests are minutes of guest time that test the guest.
CMDLINE_X86='console=ttyS0,115200 earlyprintk=ttyS0,115200 nokaslr cryptomgr.notests rdinit=/bench'

# command_for <guest> <side> -> CMD array
#
# `--for` on the rsemu side is a bound, not a plan: the run is stopped at the
# last marker, and this only has to be more virtual time than the workload
# needs. `--capture <port>` is what makes it run unpaced.
command_for() {
	local guest="$1" side="$2"
	CMD=()
	case "$guest/$side" in
	arm64/rsemu)
		CMD=("$RSEMU_BIN" run arm64-virt
			--media "kernel=${KERNEL}" --media "initrd=${INITRD}"
			-p ram=1G -p "engine=${ENGINE}" -p "cmdline=${CMDLINE_ARM64}"
			--capture console --for 4000s)
		;;
	arm64/qemu | arm64/qemu-icount)
		CMD=(qemu-system-aarch64
			-machine virt -accel tcg -cpu cortex-a53
			-m 1G -smp 1 -display none -nodefaults -serial stdio
			-kernel "$KERNEL" -initrd "$INITRD" -append "$CMDLINE_ARM64")
		[ "$side" = qemu-icount ] && CMD+=(-icount "$ICOUNT")
		;;
	x86/rsemu)
		CMD=("$RSEMU_BIN" run pc64
			--media "kernel=${KERNEL}" --media "initrd=${INITRD}"
			-p "engine=${ENGINE}" -p "cmdline=${CMDLINE_X86}"
			--capture console --for 20000s)
		;;
	x86/qemu | x86/qemu-icount)
		# `microvm` rather than `pc`: no PCI, no ACPI, an i8259, an i8254, an
		# MC146818 and a 16550 -- which is `machines/pc64.machine`'s device
		# list. `auto-kernel-cmdline=off` matters, or QEMU appends arguments of
		# its own and the two sides stop having the same command line.
		CMD=(qemu-system-x86_64
			-machine microvm,acpi=off,pit=on,pic=on,rtc=on,isa-serial=on,auto-kernel-cmdline=off
			-accel tcg -cpu qemu64
			-m 257M -smp 1 -display none -nodefaults -serial stdio
			-kernel "$KERNEL" -initrd "$INITRD" -append "$CMDLINE_X86")
		[ "$side" = qemu-icount ] && CMD+=(-icount "$ICOUNT")
		;;
	*) die "no command for $guest/$side" ;;
	esac
}

# ---------------------------------------------------------------------------
# One run
# ---------------------------------------------------------------------------
#
# Start the process writing its serial line into a FIFO, read the FIFO in this
# shell -- a redirect rather than a pipeline, so the timings land in this
# shell's variables -- and stop the clock on each marker as the byte arrives.
#
# `$EPOCHREALTIME` rather than `date`: a fork per line of kernel output would
# be a bigger cost than several of the phases.
declare -A MARK_AT
RUN_STATUS=""

run_once() {
	local fifo="${WORK}/serial.fifo" log="$1"
	shift
	MARK_AT=()
	RUN_STATUS=ok
	rm -f "$fifo"
	mkfifo "$fifo" || die "cannot make a fifo in ${WORK}"

	local t0 pid line rest phase edge
	t0=$EPOCHREALTIME
	timeout -k 5 "$TIMEOUT" "$@" >"$fifo" 2>/dev/null &
	pid=$!
	MARK_AT[start]="$t0"

	while IFS= read -r line; do
		printf '%s\n' "$line"
		case "$line" in
		*"$MARK"*) ;;
		*) continue ;;
		esac
		local now=$EPOCHREALTIME
		line=${line%$'\r'}
		rest=${line#*"$MARK" }
		phase=${rest%% *}
		edge=${rest#* }
		case "$phase" in
		digest) DIGEST="$edge" ;;
		all) MARK_AT[total]="$now"; break ;;
		*) MARK_AT["${phase}.${edge}"]="$now" ;;
		esac
	done <"$fifo" >>"$log"

	kill -TERM "$pid" 2>/dev/null
	wait "$pid" 2>/dev/null
	rm -f "$fifo"
	[ -n "${MARK_AT[total]:-}" ] || RUN_STATUS=incomplete
}

# elapsed <a> <b> -> seconds, three decimals, without bc
#
# `$EPOCHREALTIME` is `<seconds>.<microseconds>` in the C locale. Split it and
# subtract in integer microseconds, because floating point in the time path is
# what `CLAUDE.md` spends a paragraph forbidding and there is no reason to make
# an exception for the harness that reports it.
us_of() { local t=$1; printf '%s%s' "${t%.*}" "${t#*.}"; }

elapsed() {
	local a b d
	a=$(us_of "$1")
	b=$(us_of "$2")
	d=$((b - a))
	printf '%d.%06d' $((d / 1000000)) $((d % 1000000))
}

# ---------------------------------------------------------------------------
# Collecting
# ---------------------------------------------------------------------------

# SAMPLES["<guest>/<side>/<phase>"] is a space-separated list of seconds.
declare -A SAMPLES
declare -A DIGESTS
declare -A FAILURES
DIGEST=""

record_run() {
	local guest="$1" side="$2"
	if [ "$RUN_STATUS" != ok ]; then
		FAILURES["$guest/$side"]=$((${FAILURES["$guest/$side"]:-0} + 1))
		return
	fi
	DIGESTS["$guest/$side"]="$DIGEST"
	local start="${MARK_AT[start]}"
	push "$guest/$side/boot" "$(elapsed "$start" "${MARK_AT[boot.end]}")"
	local p
	for p in hash awk gzip; do
		push "$guest/$side/$p" \
			"$(elapsed "${MARK_AT[${p}.begin]}" "${MARK_AT[${p}.end]}")"
	done
	push "$guest/$side/total" "$(elapsed "$start" "${MARK_AT[total]}")"
}

push() { SAMPLES["$1"]="${SAMPLES["$1"]:-} $2"; }

# min|median|max of a sample list, as three fields, or three dashes for a side
# that produced none.
#
# The empty case is checked here rather than left to awk's `NR == 0`, because
# `printf '%s\n'` with no arguments still emits one empty line and awk then
# formats it as `0.000` -- which came out of the ratio table as `0.00x` for a
# side `--sides` had excluded. A missing measurement must read as missing.
stats() {
	set -- $1
	[ $# -gt 0 ] || { printf '%s\n' '- - -'; return; }
	printf '%s\n' "$@" | sort -g | awk '
		{ v[NR] = $1 }
		END {
			if (NR == 0) { print "- - -"; exit }
			m = (NR % 2) ? v[(NR + 1) / 2] : (v[NR / 2] + v[NR / 2 + 1]) / 2
			printf "%.3f %.3f %.3f\n", v[1], m, v[NR]
		}'
}

# The ratio, with its `x`, or a bare dash for a side that did not run. Printed
# by one function rather than assembled at the call site, because a side that
# was skipped used to come out as `-x`, which reads as a number.
ratio_or_dash() {
	{ [ "$1" = "-" ] || [ "$2" = "-" ]; } && { printf '%s' -; return; }
	awk -v a="$1" -v b="$2" \
		'BEGIN { if (b <= 0) print "-"; else printf "%.2fx", a / b }'
}

# ---------------------------------------------------------------------------
# Setup
# ---------------------------------------------------------------------------

command -v cpio >/dev/null || die "cpio is needed to build the workload initramfs"

if [ -z "$WORK" ]; then
	WORK="${REPO_ROOT}/target/bench-vs-qemu"
fi
mkdir -p "$WORK" || die "cannot create $WORK"
trap '[ "$KEEP" = 1 ] || rm -f "${WORK}/serial.fifo"' EXIT

if [ -z "$RSEMU_BIN" ]; then
	RSEMU_BIN="${REPO_ROOT}/target/bench-vs-qemu/release/rsemu"
	if [ ! -x "$RSEMU_BIN" ]; then
		printf 'building rsemu --release --features %s\n' "$FEATURES" >&2
		CARGO_TARGET_DIR="${REPO_ROOT}/target/bench-vs-qemu" \
			cargo build --release --features "$FEATURES" --bin rsemu ||
			die "the rsemu build failed"
	fi
fi
[ -x "$RSEMU_BIN" ] || die "no rsemu binary at $RSEMU_BIN"

printf '# rsemu vs QEMU, wall clock on the same guest\n'
printf '# rsemu      %s (engine=%s)\n' "$("$RSEMU_BIN" --version | head -1)" "$ENGINE"
printf '# host       %s, %s cores, %s\n' \
	"$(awk -F': ' '/model name/ { print $2; exit }' /proc/cpuinfo)" \
	"$(nproc)" "$(uname -sr)"
printf '# governor   %s\n' \
	"$(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor 2>/dev/null || echo unknown)"
printf '# load       %s\n' "$(cut -d' ' -f1-3 /proc/loadavg)"
printf '# reps       %s, interleaved, starting side rotated\n' "$REPS"
printf '\n'

# ---------------------------------------------------------------------------
# Run
# ---------------------------------------------------------------------------

RAN_ANY=0

for guest in "${GUESTS[@]}"; do
	case "$guest" in
	arm64)
		QEMU=qemu-system-aarch64
		KERNEL="${RSEMU_ARM64_KERNEL:-${TESTDATA}/arm64/linux}"
		BASE_INITRD="${RSEMU_ARM64_INITRD:-${TESTDATA}/arm64/initramfs.cpio}"
		;;
	x86)
		QEMU=qemu-system-x86_64
		KERNEL="${RSEMU_X86_KERNEL:-${TESTDATA}/x86/bzImage}"
		BASE_INITRD="${RSEMU_X86_INITRD:-${TESTDATA}/x86/initramfs-x86.cpio}"
		;;
	*) die "no such guest: $guest (arm64, x86)" ;;
	esac

	if ! command -v "$QEMU" >/dev/null; then
		skip "$guest (no $QEMU on PATH)"
		continue
	fi
	if [ ! -s "$KERNEL" ]; then
		skip "$guest (no kernel at $KERNEL: scripts/fetch-testdata.sh ${guest}-linux)"
		continue
	fi
	if [ ! -s "$BASE_INITRD" ]; then
		skip "$guest (no initramfs at $BASE_INITRD: scripts/fetch-testdata.sh)"
		continue
	fi

	INITRD="${WORK}/bench-${guest}.cpio"
	build_initramfs "$BASE_INITRD" "$INITRD"
	RAN_ANY=1

	local_sides=("${SIDES[@]}")
	for ((rep = 0; rep < REPS; rep++)); do
		# Rotate which side goes first, so a host that drifts over the run
		# charges the drift to each side equally.
		n=${#local_sides[@]}
		for ((i = 0; i < n; i++)); do
			side="${local_sides[$(((i + rep) % n))]}"
			command_for "$guest" "$side"
			log="${WORK}/${guest}-${side}-${rep}.log"
			: >"$log"
			printf 'run %-6s %-12s rep %d/%d ... ' "$guest" "$side" \
				$((rep + 1)) "$REPS" >&2
			run_once "$log" "${CMD[@]}"
			if [ "$RUN_STATUS" = ok ]; then
				printf '%ss\n' "$(elapsed "${MARK_AT[start]}" "${MARK_AT[total]}")" >&2
			else
				printf 'DID NOT FINISH (see %s)\n' "$log" >&2
			fi
			record_run "$guest" "$side"
		done
	done
done

if [ "$RAN_ANY" = 0 ]; then
	printf '\nnothing was measured.\n' >&2
	exit 0
fi

# ---------------------------------------------------------------------------
# Report
# ---------------------------------------------------------------------------

printf '\n'
for guest in "${GUESTS[@]}"; do
	[ -n "${SAMPLES["$guest/${SIDES[0]}/total"]:-}" ] || continue

	printf '## %s\n\n' "$guest"

	# Every side must have run the same program. A digest that disagrees means
	# the two guests diverged, and a ratio between them would be a lie.
	base_digest=""
	for side in "${SIDES[@]}"; do
		d="${DIGESTS["$guest/$side"]:-}"
		[ -n "$d" ] || continue
		if [ -z "$base_digest" ]; then
			base_digest="$d"
		elif [ "$d" != "$base_digest" ]; then
			printf 'DIGEST MISMATCH: %s produced %s, expected %s\n' \
				"$side" "$d" "$base_digest" >&2
			exit 1
		fi
	done
	printf 'workload digest %s -- every side agreed\n\n' "$base_digest"

	printf '%-8s %-12s %9s %9s %9s %8s\n' \
		phase side 'min (s)' 'median' 'max' 'spread'
	for phase in "${PHASES[@]}"; do
		for side in "${SIDES[@]}"; do
			read -r mn md mx <<<"$(stats "${SAMPLES["$guest/$side/$phase"]:-}")"
			[ "$mn" = "-" ] && continue
			sp=$(awk -v a="$mn" -v b="$mx" \
				'BEGIN { if (a <= 0) print "-"; else printf "%.1f%%", 100 * (b - a) / a }')
			printf '%-8s %-12s %9s %9s %9s %8s\n' "$phase" "$side" "$mn" "$md" "$mx" "$sp"
		done
	done

	printf '\nthe ratio the gate is about -- rsemu wall clock divided by QEMU wall clock.\n'
	printf 'Greater than 1 means rsemu is slower. Both statistics, because on a loaded\n'
	printf 'host they disagree and publishing only the flattering one is the habit this\n'
	printf 'script exists to break: min/min is both sides at their least interfered\n'
	printf 'with, median/median is both sides on a typical afternoon.\n\n'
	printf '%-8s %14s %14s %14s %14s\n' phase \
		'min vs qemu' 'med vs qemu' 'min vs icount' 'med vs icount'
	for phase in "${PHASES[@]}"; do
		read -r r rmed _ <<<"$(stats "${SAMPLES["$guest/rsemu/$phase"]:-}")"
		read -r q qmed _ <<<"$(stats "${SAMPLES["$guest/qemu/$phase"]:-}")"
		read -r qi qimed _ <<<"$(stats "${SAMPLES["$guest/qemu-icount/$phase"]:-}")"
		[ "$r" = "-" ] && continue
		printf '%-8s %14s %14s %14s %14s\n' "$phase" \
			"$(ratio_or_dash "$r" "$q")" "$(ratio_or_dash "$rmed" "$qmed")" \
			"$(ratio_or_dash "$r" "$qi")" "$(ratio_or_dash "$rmed" "$qimed")"
	done
	printf '\n'

	for side in "${SIDES[@]}"; do
		n="${FAILURES["$guest/$side"]:-0}"
		[ "$n" = 0 ] || printf 'WARNING: %s/%s did not finish %d of %d runs\n' \
			"$guest" "$side" "$n" "$REPS"
	done
done

printf 'logs and the workload initramfs are under %s\n' "$WORK"
[ "$SKIPPED" = 0 ] || printf '%d leg(s) skipped\n' "$SKIPPED"
