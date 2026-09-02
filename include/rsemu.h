/*
 * rsemu -- the C ABI.
 *
 * GENERATED FILE. Do not edit: it is produced from `src/ffi/abi.rs` by
 * `src/ffi/header.rs`, and a test fails when the two disagree. To change the
 * ABI, change `abi.rs`; to update this file afterwards, run
 *
 *     RSEMU_UPDATE_HEADER=1 cargo test --features ffi header
 *
 * Build the library it declares with:
 *
 *     cargo rustc --lib --release --features ffi --crate-type staticlib
 *     cargo rustc --lib --release --features ffi --crate-type cdylib
 *
 * Conventions
 * -----------
 *
 * Status codes. Every fallible call returns rsemu_status: 0 is success and
 * every failure is negative. The numeric values are part of the ABI.
 *
 * Output buffers belong to the caller. A call that produces bytes takes
 * (out, out_len). *out_len is the capacity on entry and the length on return,
 * and it is written whether the call succeeded or not -- so passing a zero
 * capacity is a length query, and RSEMU_BUFFER_TOO_SMALL tells you how much to
 * allocate. rsemu never hands back a pointer you have to free.
 *
 * Bytes are pointer + length; identifiers are NUL-terminated. Media images,
 * machine descriptions and snapshots are (ptr, len) and need no terminator;
 * slot names, parameter names and machine names are const char *. (NULL, 0) is
 * a valid empty blob, (NULL, n > 0) is RSEMU_NULL_POINTER.
 *
 * Text is UTF-8 and is NOT NUL-terminated on the way out; *out_len carries the
 * length. Text on the way in is validated, and invalid UTF-8 is
 * RSEMU_INVALID_INPUT rather than a silent replacement character.
 *
 * Handles are integers, not pointers, and 0 is never valid. Ids are never
 * reused and nothing a caller invents is ever dereferenced, so a double free,
 * a use-after-free and a forged handle are all RSEMU_BAD_HANDLE rather than
 * undefined behaviour. Calls on one handle serialise; different handles are
 * independent, so no external mutex is needed.
 *
 * Panics do not cross this boundary. A caught panic is RSEMU_PANIC and the
 * machine it happened in is poisoned: only rsemu_last_error and rsemu_free
 * keep working on it. This requires an unwinding build; under
 * -C panic=abort a panic aborts the process instead.
 */

#ifndef RSEMU_H
#define RSEMU_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * A live object owned by rsemu: a configuration, or a machine.
 *
 * Deliberately an integer and not a pointer. `0` is never a valid handle, ids
 * are never reused, and no value a caller invents is ever dereferenced -- see
 * the module docs for why this ABI departs from the siblings here.
 */
typedef uint64_t rsemu_handle;

/*
 * The value of a handle that names nothing.
 */
#define RSEMU_INVALID_HANDLE ((rsemu_handle)0)

/*
 * The revision of this ABI. Bumped whenever a signature or a numeric value
 * changes meaning; compare it against `rsemu_abi_version` at startup to
 * catch a header that does not match the library it is linked against.
 */
#define RSEMU_ABI_VERSION ((uint32_t)1)

/*
 * Power-on reset: every register returns to its documented reset value.
 */
#define RSEMU_RESET_COLD ((int32_t)0)

/*
 * A reset-line pulse: battery-backed and always-on state survives.
 */
#define RSEMU_RESET_WARM ((int32_t)1)

/*
 * A bus-level reset, affecting only the devices on that bus.
 */
#define RSEMU_RESET_BUS ((int32_t)2)

/*
 * The result of a C ABI call. `0` is success; every failure is negative.
 *
 * The numeric values are part of the ABI and do not change. The first three
 * match `purecrypto` and `kataan` exactly; the rest are one code per
 * `Error` variant, plus one per
 * `BusError` variant so that a caller can retry a
 * busy access without matching on a message.
 *
 * The code says *which kind* of failure; `rsemu_last_error` says which one,
 * in the words rsemu would have printed to a terminal.
 */
typedef enum rsemu_status {
    /*
     * The call succeeded.
     */
    RSEMU_OK = 0,
    /*
     * A pointer argument was NULL where a value was required.
     */
    RSEMU_NULL_POINTER = -1,
    /*
     * The output buffer was too small; `*out_len` holds the length needed.
     */
    RSEMU_BUFFER_TOO_SMALL = -2,
    /*
     * An argument was out of range, or a string was not UTF-8.
     */
    RSEMU_INVALID_INPUT = -3,
    /*
     * The handle names nothing, or names the other kind of object. A double
     * free and a use-after-free both arrive here.
     */
    RSEMU_BAD_HANDLE = -4,
    /*
     * A machine description could not be parsed, resolved or validated. The
     * message carries `file:line:col` and a caret.
     */
    RSEMU_CONFIG = -5,
    /*
     * The description names a device class this build does not contain --
     * usually a Cargo feature that is off rather than a typo.
     */
    RSEMU_UNKNOWN_CLASS = -6,
    /*
     * A property was missing, of the wrong type, or out of range.
     */
    RSEMU_PROPERTY = -7,
    /*
     * A snapshot could not be written or restored.
     */
    RSEMU_STATE = -8,
    /*
     * A translation block was malformed. Always an rsemu bug.
     */
    RSEMU_IR = -9,
    /*
     * The operation is not implemented in this build yet.
     */
    RSEMU_UNIMPLEMENTED = -10,
    /*
     * Nothing is mapped at that guest address.
     */
    RSEMU_BUS_UNASSIGNED = -11,
    /*
     * The access width or alignment is not permitted there.
     */
    RSEMU_BUS_BAD_ACCESS = -12,
    /*
     * Something is mapped there and does not permit this access.
     */
    RSEMU_BUS_PROTECTED = -13,
    /*
     * The target was busy; the access may be retried.
     */
    RSEMU_BUS_RETRY = -14,
    /*
     * A panic was caught at the boundary. A machine that returns this is
     * poisoned: only `rsemu_last_error` and `rsemu_free` still work on it.
     */
    RSEMU_PANIC = -100,
} rsemu_status;

/*
 * Returns the ABI revision this library implements.
 *
 * Compare it with `RSEMU_ABI_VERSION` from the header. A mismatch means the
 * header and the library came from different builds, and nothing below this
 * line can be trusted.
 */
uint32_t rsemu_abi_version(void);

/*
 * Returns a NUL-terminated description of `status`, in static storage.
 *
 * The one string rsemu owns that a caller may hold on to. It must not be
 * freed, and it stays valid for the life of the process. An unrecognised code
 * answers "unknown status" rather than NULL, so a caller can print it
 * unconditionally.
 */
const char *rsemu_strerror(int32_t status);

/*
 * Writes a one-line description of how this library was configured.
 *
 * A machine is a feature set, so "which rsemu is this?" has a build-specific
 * answer and this is it: the version and the features that were enabled.
 *
 * Safety:
 *
 * `out_len` must point at a writable `size_t` holding the capacity of `out`,
 * and `out` must be writable for that many bytes.
 */
rsemu_status rsemu_build_info(char *out, size_t *out_len);

/*
 * Returns how many machines this build ships ready to run.
 *
 * Zero is a correct answer for a library built with `--features ffi` alone:
 * a machine is a feature set. A description passed to
 * `rsemu_config_machine` does not have to be one of these.
 */
uint32_t rsemu_catalog_count(void);

/*
 * Writes the name of catalog machine `index` -- what `rsemu run <name>` takes.
 *
 * Safety:
 *
 * As `rsemu_build_info`.
 */
rsemu_status rsemu_catalog_name(uint32_t index, char *out, size_t *out_len);

/*
 * Writes the name of media slot `slot` of catalog machine `index`.
 *
 * Slots are numbered from zero and an out-of-range `slot` is
 * `RSEMU_INVALID_INPUT`, so counting them is a loop rather than another call.
 * These are the names `rsemu_config_media` binds, and they are what
 * `--cart`, `--rom`, `--bios` and `--disk` spell on the command line.
 *
 * Safety:
 *
 * As `rsemu_build_info`.
 */
rsemu_status rsemu_catalog_media(
    uint32_t index,
    uint32_t slot,
    char *out,
    size_t *out_len
);

/*
 * Creates an empty machine configuration and returns its handle.
 *
 * `RSEMU_INVALID_HANDLE` on failure, which at this point can only mean the
 * allocator refused. Free it with `rsemu_free` once
 * `rsemu_machine_new` has built the machine; the machine does not borrow it.
 */
rsemu_handle rsemu_config_new(void);

/*
 * Says which machine to build.
 *
 * `name` is a catalog name when `source` is NULL -- `rsemu_catalog_name` lists
 * them -- and otherwise the name diagnostics should use for the description,
 * the way a filename appears in `file:line:col`.
 *
 * `source` is the text of a `.machine` description, as `(pointer, length)`
 * and not NUL-terminated. It is copied, so the caller may free it as soon as
 * this returns. rsemu never opens a file on the caller's behalf: an embedder
 * that wants `include` resolution or a search path owns that policy, and a C
 * ABI that took paths would have to invent an encoding rule for them.
 *
 * Safety:
 *
 * `name` must be a NUL-terminated UTF-8 string, and `source` must be readable
 * for `source_len` bytes or NULL with a zero length.
 */
rsemu_status rsemu_config_machine(
    rsemu_handle config,
    const char *name,
    const uint8_t *source,
    size_t source_len
);

/*
 * Binds `len` bytes to the media slot `slot`, as `--cart`, `--rom`, `--bios`
 * and `--disk` do.
 *
 * The bytes are copied, so the caller may free them as soon as this returns.
 * Binding the same slot twice keeps the second image, the way a repeated
 * command-line option does.
 *
 * Safety:
 *
 * `slot` must be a NUL-terminated UTF-8 string, and `bytes` must be readable
 * for `len` bytes or NULL with a zero length.
 */
rsemu_status rsemu_config_media(
    rsemu_handle config,
    const char *slot,
    const uint8_t *bytes,
    size_t len
);

/*
 * Overrides a `param` in the description, as `rsemu run ... -p ram=8M` does.
 *
 * Both strings are copied. A parameter the description does not declare is
 * reported when the machine is built, not here.
 *
 * Safety:
 *
 * `key` and `value` must be NUL-terminated UTF-8 strings.
 */
rsemu_status rsemu_config_param(
    rsemu_handle config,
    const char *key,
    const char *value
);

/*
 * Builds the machine `config` describes and writes its handle to `out`.
 *
 * The configuration is not consumed: it stays valid, can be adjusted and
 * built again, and must be freed with `rsemu_free` like any other handle.
 * On failure `*out` is set to `RSEMU_INVALID_HANDLE` and the reason is on the
 * **configuration's** handle, so `rsemu_last_error(config, ...)` is where a
 * caller looks -- there is no machine to carry it.
 *
 * A media slot the caller did not bind falls back to whatever rsemu itself
 * ships for it, which is the courtesy `rsemu run` extends: an `apple1` with no
 * `rom` gets rsemu's own monitor, a `pc-at` with no `bios` gets the firmware
 * this repository assembles. The fallback applies only to catalog machines,
 * because it is keyed on the machine's name.
 *
 * Safety:
 *
 * `out` must point at a writable `rsemu_handle`.
 */
rsemu_status rsemu_machine_new(rsemu_handle config, rsemu_handle *out);

/*
 * Frees the object `handle` names, whichever kind it is.
 *
 * `RSEMU_BAD_HANDLE` if it names nothing -- which is what a double free, a
 * free of a handle that was never created, and a free of an uninitialised
 * variable all look like. None of those is undefined behaviour here: the
 * handle is an id, so a wrong one is looked up and not found rather than
 * dereferenced.
 *
 * Freeing a machine another thread is still running is safe. The handle stops
 * resolving immediately; the machine itself is dropped when that call
 * returns.
 */
rsemu_status rsemu_free(rsemu_handle handle);

/*
 * Writes the message explaining `handle`'s last failure.
 *
 * The text `rsemu` would have printed: for a description error that is
 * `file:line:col`, the message and a caret under the offending token. UTF-8,
 * not NUL-terminated; `*out_len` carries the length. Empty when the last call
 * succeeded, and also for the argument errors a status code already explains
 * on its own -- a NULL, a bad handle, an index out of range. There is nothing
 * to add to those that `rsemu_strerror` does not already say.
 *
 * Works on a poisoned machine, and on the configuration whose
 * `rsemu_machine_new` failed.
 *
 * Safety:
 *
 * As `rsemu_build_info`.
 */
rsemu_status rsemu_last_error(rsemu_handle handle, char *out, size_t *out_len);

/*
 * Advances the machine by `nanos` nanoseconds of **virtual** time.
 *
 * Virtual, not wall-clock: this is the unit the whole determinism story is
 * told in, and two runs of the same machine over the same span produce the
 * same `rsemu_machine_state_hash` on any host. How long the call takes is a
 * property of the host; how far the machine moves is not.
 *
 * Additive, so driving a machine in slices and driving it in one call reach
 * the same place. That is what lets an embedder interleave its own work
 * without changing the result.
 */
rsemu_status rsemu_machine_run_ns(rsemu_handle machine, uint64_t nanos);

/*
 * Writes how far the machine has run, in nanoseconds of virtual time.
 *
 * Safety:
 *
 * `out` must point at a writable `uint64_t`.
 */
rsemu_status rsemu_machine_now_ns(rsemu_handle machine, uint64_t *out);

/*
 * Resets the machine. `kind` is one of the `RSEMU_RESET_*` constants.
 */
rsemu_status rsemu_machine_reset(rsemu_handle machine, int32_t kind);

/*
 * Writes how many address spaces the machine has.
 *
 * Safety:
 *
 * `out` must point at a writable `uint32_t`.
 */
rsemu_status rsemu_machine_space_count(rsemu_handle machine, uint32_t *out);

/*
 * Writes the name of address space `index`, as the description spells it.
 *
 * These are the names `rsemu_machine_read` and `rsemu_machine_write` take.
 *
 * Safety:
 *
 * As `rsemu_build_info`.
 */
rsemu_status rsemu_machine_space_name(
    rsemu_handle machine,
    uint32_t index,
    char *out,
    size_t *out_len
);

/*
 * Copies `len` bytes of guest memory at `addr` into `out`.
 *
 * `space` names the address space; NULL means the machine's first one, which
 * is the CPU bus on every machine rsemu ships.
 *
 * The access is a **debug** access: it must not pop a FIFO, clear a status
 * bit or advance a pointer, exactly as a gdb read must not. Reading through
 * this ABI therefore cannot change what the guest goes on to do.
 *
 * The bytes are copied. rsemu does not hand out pointers into guest memory --
 * guest RAM is addressed by byte offset so that it can live in a
 * `SharedArrayBuffer`, and a pointer into it would be invalidated by the next
 * remap or snapshot restore anyway.
 *
 * Safety:
 *
 * `space` must be NULL or a NUL-terminated UTF-8 string, and `out` must be
 * writable for `len` bytes.
 */
rsemu_status rsemu_machine_read(
    rsemu_handle machine,
    const char *space,
    uint64_t addr,
    uint8_t *out,
    size_t len
);

/*
 * Copies `len` bytes from `bytes` into guest memory at `addr`.
 *
 * The mirror of `rsemu_machine_read`, on the same terms: `space` may be
 * NULL for the first space, and the access is a debug access, so writing a
 * device register through this ABI does not run the side effects a guest
 * store would. Use it to plant a program, patch a value or seed RAM before a
 * run -- not to impersonate the guest.
 *
 * Safety:
 *
 * `space` must be NULL or a NUL-terminated UTF-8 string, and `bytes` must be
 * readable for `len` bytes or NULL with a zero length.
 */
rsemu_status rsemu_machine_write(
    rsemu_handle machine,
    const char *space,
    uint64_t addr,
    const uint8_t *bytes,
    size_t len
);

/*
 * Writes a snapshot of the whole machine.
 *
 * Follows the buffer convention: call it once with a zero capacity to learn
 * the size, then again with a buffer that big. The snapshot is
 * self-describing and versioned; `rsemu_machine_load` restores it into a
 * machine built from the same description.
 *
 * Safety:
 *
 * `out_len` must point at a writable `size_t` holding the capacity of `out`,
 * and `out` must be writable for that many bytes.
 */
rsemu_status rsemu_machine_save(
    rsemu_handle machine,
    uint8_t *out,
    size_t *out_len
);

/*
 * Restores a snapshot taken by `rsemu_machine_save`.
 *
 * The machine must have been built from the same description: a snapshot
 * records the shape it was taken from and a mismatch is reported rather than
 * half-applied.
 *
 * Safety:
 *
 * `bytes` must be readable for `len` bytes, or NULL with a zero length.
 */
rsemu_status rsemu_machine_load(
    rsemu_handle machine,
    const uint8_t *bytes,
    size_t len
);

/*
 * Writes the machine's deterministic state hash.
 *
 * Two runs that took the same path produce the same value, on any host and
 * under any thread count in deterministic mode. This is the regression gate
 * an embedded rsemu is worth having for: run, hash, compare.
 *
 * Safety:
 *
 * `out` must point at a writable `uint64_t`.
 */
rsemu_status rsemu_machine_state_hash(rsemu_handle machine, uint64_t *out);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* RSEMU_H */
