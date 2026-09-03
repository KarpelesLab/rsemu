#!/usr/bin/env python3
"""Derive the set of builds that actually covers this crate's `cfg` gates.

Cargo features are additive, so `--all-features` compiles every gate that is a
conjunction of features — but `--all-features` also turns on `std`, and `std`
is what the `no_std` rule exists to keep out of the emulation core. The only
build that can catch a `std`-only construct behind a feature is
`--no-default-features --features <that feature>`, which is why CI sweeps the
features one at a time.

A one-at-a-time sweep, though, never compiles code gated on a *pair* of
features. `src/cpu/riscv/engine.rs` is `#[cfg(all(feature = "cpu-riscv-lift",
feature = "jit"))]`: neither feature alone builds it, so the only build that
ever compiled it was `--all-features`, with `std` present. A hand-written list
of extra combinations goes stale the next time somebody adds one — which is
exactly how that file came to be uncompiled by the sweep — so this reads them
out of the tree instead.

What it does:

  * builds the module graph from `mod` declarations, so a `#[cfg(feature = A)]`
    module containing a `#[cfg(feature = B)]` item counts as the conjunction
    {A, B} even though no single `cfg` says so;
  * parses every `cfg`/`cfg_attr` predicate into feature conjunctions,
    distributing over `any(...)`;
  * drops conjunctions feature unification already implies (a build of
    `machine-nes` gets `dev-nes-ppu` for free, so that is not a *pair*);
  * emits the build set: every feature alone, plus one build per surviving
    conjunction.

Not every derived conjunction needs its own build. A conjunction one of whose
members pulls in `std` is already compiled by `--all-features`, and `std` is
present there anyway, so nothing about it is unchecked. The ones that matter
are the conjunctions no member of which reaches `std`: their code is gated
behind a pair, so the only build that ever compiles it is `--all-features` —
with `std` on — and a `println!` in it survives to master. Those are merged
into as few builds as possible (features are additive, so the union of two
`no_std` sets is still `no_std`, and turning every gate on at once cannot hide
a compile error in gated code) and that merged handful is what CI adds.

Subcommands
  features   every feature except `default`, one per line
  combos     every derived conjunction a one-at-a-time sweep misses
  nostd      the subset of those that reach no `std`, before merging
  merged     those, packed into as few builds as cover them — one
             comma-separated feature list per line
  sweep      the whole build set: `features` then `merged`
  explain    combos with the file and line that produced each
  plan       a human summary of the above

Standard library only, and no crates.io tool: the dependency policy has no
room for one (CLAUDE.md).
"""

import json
import os
import re
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


# --------------------------------------------------------------------------
# The manifest's feature graph
# --------------------------------------------------------------------------


def manifest_features():
    """`{feature: set of features it transitively enables, itself included}`."""
    out = subprocess.run(
        ["cargo", "metadata", "--no-deps", "--format-version", "1"],
        cwd=ROOT,
        capture_output=True,
        text=True,
        check=True,
    ).stdout
    raw = json.loads(out)["packages"][0]["features"]

    def closure(f, seen):
        if f in seen:
            return
        seen.add(f)
        for dep in raw.get(f, []):
            # `dep:foo` turns on an optional dependency and `foo/bar` forwards
            # into one — neither names a feature of ours.
            if dep.startswith("dep:") or "/" in dep:
                continue
            closure(dep, seen)

    result = {}
    for f in raw:
        seen = set()
        closure(f, seen)
        result[f] = seen
    return result


# --------------------------------------------------------------------------
# cfg predicates
# --------------------------------------------------------------------------

# A conjunction is a frozenset of feature names that must all be on. A
# predicate becomes a *list* of them: `any(a, b)` is two ways to be true and
# each one is worth compiling.


def _split_top(s):
    """Split a comma-separated argument list at depth 0."""
    parts, depth, cur, i = [], 0, [], 0
    while i < len(s):
        c = s[i]
        if c == '"':
            j = s.find('"', i + 1)
            if j < 0:
                break
            cur.append(s[i : j + 1])
            i = j + 1
            continue
        if c in "([":
            depth += 1
        elif c in ")]":
            depth -= 1
        if c == "," and depth == 0:
            parts.append("".join(cur))
            cur = []
        else:
            cur.append(c)
        i += 1
    if "".join(cur).strip():
        parts.append("".join(cur))
    return [p.strip() for p in parts if p.strip()]


FEATURE_EQ = re.compile(r'^feature\s*=\s*"([^"]+)"$')


def parse_pred(s):
    """Predicate text -> list of conjunctions (frozensets of features).

    Anything that is not a feature test — `test`, `target_os`, `docsrs` — adds
    no requirement. A negation is deliberately ignored rather than tracked: a
    `not(feature = X)` branch is the case the narrow sweep already covers best,
    since it builds with almost everything off.
    """
    s = s.strip()
    m = FEATURE_EQ.match(s)
    if m:
        return [frozenset([m.group(1)])]
    if s.startswith("all(") and s.endswith(")"):
        combos = [frozenset()]
        for arg in _split_top(s[4:-1]):
            sub = parse_pred(arg)
            combos = [a | b for a in combos for b in sub]
        return combos
    if s.startswith("any(") and s.endswith(")"):
        out = []
        for arg in _split_top(s[4:-1]):
            out.extend(parse_pred(arg))
        return out or [frozenset()]
    return [frozenset()]


def balanced(text, start):
    """Text inside the parens opening at `start`, and the index past them."""
    depth, i = 0, start
    while i < len(text):
        c = text[i]
        if c == '"':
            j = text.find('"', i + 1)
            if j < 0:
                break
            i = j + 1
            continue
        if c == "(":
            depth += 1
        elif c == ")":
            depth -= 1
            if depth == 0:
                return text[start + 1 : i], i + 1
        i += 1
    return "", len(text)


CFG_AT = re.compile(r"\bcfg(?:_attr)?\s*\(")


def strip_comments(text):
    """Blank out comments, keeping every byte offset so line numbers hold.

    Module documentation in this crate quotes its own `cfg` attributes — the
    gate on `src/cpu/riscv/engine.rs` is explained in prose directly above it —
    and a sentence about a feature pair must not become a CI job.
    """
    out = list(text)
    i, n = 0, len(text)
    while i < n:
        c = text[i]
        # A raw string, `r"…"` or `r#"…"#`, ends at a quote followed by the
        # same run of hashes and honours no escape in between.
        if c == "r" and (i == 0 or not (text[i - 1].isalnum() or text[i - 1] == "_")):
            j = i + 1
            while j < n and text[j] == "#":
                j += 1
            if j < n and text[j] == '"':
                close = '"' + "#" * (j - i - 1)
                k = text.find(close, j + 1)
                i = n if k < 0 else k + len(close)
                continue
            i += 1
        elif c == '"':
            i += 1
            while i < n and text[i] != '"':
                i += 2 if text[i] == "\\" else 1
            i += 1
        # `'"'` is a character literal and not the start of a string; `'a` is a
        # lifetime and is neither. Two dozen of the former are in this tree, and
        # mistaking one for a quote swallows everything after it.
        elif c == "'":
            if i + 1 < n and text[i + 1] == "\\":
                k = text.find("'", i + 2)
                i = n if k < 0 else k + 1
            elif i + 2 < n and text[i + 2] == "'":
                i += 3
            else:
                i += 1
        elif c == "/" and i + 1 < n and text[i + 1] == "/":
            while i < n and text[i] != "\n":
                out[i] = " "
                i += 1
        elif c == "/" and i + 1 < n and text[i + 1] == "*":
            depth = 1
            out[i] = out[i + 1] = " "
            i += 2
            while i < n and depth:
                if text.startswith("/*", i):
                    depth += 1
                    out[i] = out[i + 1] = " "
                    i += 2
                elif text.startswith("*/", i):
                    depth -= 1
                    out[i] = out[i + 1] = " "
                    i += 2
                else:
                    if text[i] != "\n":
                        out[i] = " "
                    i += 1
        else:
            i += 1
    return "".join(out)


def file_predicates(text):
    """Every cfg predicate in a chunk of source, as (line, conjunction-list)."""
    out = []
    text = strip_comments(text)
    for m in CFG_AT.finditer(text):
        open_paren = text.find("(", m.start())
        if open_paren < 0:
            continue
        inner, _ = balanced(text, open_paren)
        if m.group(0).startswith("cfg_attr"):
            # `cfg_attr(pred, attr, ...)` — only the first argument is a cfg.
            args = _split_top(inner)
            if not args:
                continue
            inner = args[0]
        combos = [c for c in parse_pred(inner) if c]
        if combos:
            out.append((text.count("\n", 0, m.start()) + 1, combos))
    return out


# --------------------------------------------------------------------------
# The module graph, so nesting counts
# --------------------------------------------------------------------------

MOD_DECL = re.compile(
    r"^\s*(?:pub(?:\([^)]*\))?\s+)?mod\s+([A-Za-z_][A-Za-z0-9_]*)\s*;", re.M
)


def module_files():
    """`{path: [conjunction, ...]}` — what must be on for the file to compile.

    Walks `mod x;` declarations from the crate roots, carrying each
    declaration's own `cfg` down into the file it names. A module reachable
    more than one way keeps every route, so nothing is over-constrained.
    """
    roots = [os.path.join(ROOT, "src", "lib.rs")]
    for d in ("tests", "benches", os.path.join("fuzz", "fuzz_targets")):
        p = os.path.join(ROOT, d)
        if not os.path.isdir(p):
            continue
        for name in sorted(os.listdir(p)):
            if name.endswith(".rs"):
                roots.append(os.path.join(p, name))

    ctx = {r: [frozenset()] for r in roots}
    queue = list(roots)
    seen = set()
    while queue:
        path = queue.pop(0)
        if path in seen:
            continue
        seen.add(path)
        try:
            with open(path, encoding="utf-8", errors="replace") as fh:
                text = fh.read()
        except OSError:
            continue
        here = ctx.get(path, [frozenset()])
        for m in MOD_DECL.finditer(text):
            name = m.group(1)
            # The attributes immediately above the declaration.
            attrs = []
            for line in reversed(text[: m.start()].splitlines()):
                st = line.strip()
                if st.startswith("#["):
                    attrs.append(st)
                elif st.startswith("//") or not st:
                    continue
                else:
                    break
            gate = [frozenset()]
            for a in attrs:
                for _, combos in file_predicates(a):
                    gate = [x | y for x in gate for y in combos]
            child_ctx = [a | b for a in here for b in gate]

            base = os.path.dirname(path)
            stem = os.path.basename(path)
            if stem not in ("lib.rs", "mod.rs", "main.rs"):
                base = os.path.join(base, os.path.splitext(stem)[0])
            for cand in (
                os.path.join(base, name + ".rs"),
                os.path.join(base, name, "mod.rs"),
            ):
                if os.path.exists(cand):
                    ctx.setdefault(cand, [])
                    for c in child_ctx:
                        if c not in ctx[cand]:
                            ctx[cand].append(c)
                    queue.append(cand)
                    break

    # Files nothing declares (an inline `mod` block, a `#[path]` attribute)
    # are still scanned, just with no ambient context.
    for dirpath, _, names in os.walk(os.path.join(ROOT, "src")):
        for n in names:
            if n.endswith(".rs"):
                ctx.setdefault(os.path.join(dirpath, n), [frozenset()])
    return ctx


# --------------------------------------------------------------------------
# Putting it together
# --------------------------------------------------------------------------


def derive():
    implied = manifest_features()
    known = set(implied)

    found = {}  # conjunction -> [(relpath, line), ...]
    for path, ambient in module_files().items():
        try:
            with open(path, encoding="utf-8", errors="replace") as fh:
                text = fh.read()
        except OSError:
            continue
        rel = os.path.relpath(path, ROOT)
        for line, combos in file_predicates(text):
            for c in combos:
                for a in ambient:
                    whole = frozenset(x for x in (c | a) if x in known)
                    if len(whole) < 2:
                        continue
                    # Already covered: one member drags in all the others by
                    # feature unification, so the sweep's build of that single
                    # feature compiles this gate.
                    if any(whole <= implied[f] for f in whole):
                        continue
                    found.setdefault(whole, [])
                    if (rel, line) not in found[whole]:
                        found[whole].append((rel, line))
    return implied, found


def merge(combos):
    """Pack conjunctions into as few feature sets as possible.

    Greedy, and deliberately crude: start from the largest conjunction and
    absorb every other one whose features are already present or that keeps the
    set under a soft ceiling. The ceiling exists only so a failure names a
    handful of features rather than fifty — correctness does not need it,
    because turning more features on can never stop a gate from compiling.
    """
    ceiling = 12
    packed = []
    for c in sorted(combos, key=lambda s: (-len(s), sorted(s))):
        for i, p in enumerate(packed):
            if c <= p:
                break
            if len(p | c) <= ceiling:
                packed[i] = p | c
                break
        else:
            packed.append(set(c))
    out = [frozenset(p) for p in packed]
    # The whole point is coverage, so check it rather than trusting the loop.
    for c in combos:
        assert any(c <= p for p in out), "packing lost %s" % sorted(c)
    return out


def classify(implied, found):
    """Split the conjunctions into the `std`-reaching and the `no_std` ones."""
    reaches_std = {f for f in implied if "std" in implied[f]}
    nostd = [c for c in found if not (c & reaches_std)]
    withstd = [c for c in found if c & reaches_std]
    return nostd, withstd


def main():
    cmd = sys.argv[1] if len(sys.argv) > 1 else "sweep"
    implied, found = derive()
    singles = sorted(f for f in implied if f != "default")
    combos = sorted(found, key=lambda c: (len(c), sorted(c)))
    nostd, withstd = classify(implied, found)

    if cmd == "features":
        print("\n".join(singles))
    elif cmd == "combos":
        for c in combos:
            print(",".join(sorted(c)))
    elif cmd == "nostd":
        for c in sorted(nostd, key=lambda s: (len(s), sorted(s))):
            print(",".join(sorted(c)))
    elif cmd == "explain":
        for c in combos:
            where = ", ".join("%s:%d" % (p, l) for p, l in sorted(found[c])[:3])
            more = "" if len(found[c]) <= 3 else " (+%d more)" % (len(found[c]) - 3)
            print(",".join(sorted(c)))
            print("    " + where + more)
    elif cmd == "merged":
        for c in sorted(merge(nostd), key=lambda s: sorted(s)):
            print(",".join(sorted(c)))
    elif cmd == "sweep":
        for f in singles:
            print(f)
        for c in sorted(merge(nostd), key=lambda s: sorted(s)):
            print(",".join(sorted(c)))
    elif cmd == "plan":
        packed = merge(nostd)
        print("features:                     %d" % len(singles))
        print("cfg conjunctions in the tree: %d" % len(combos))
        print("  reaching std (--all-features already compiles them, with std")
        print("  present, so nothing about them is unchecked):        %d" % len(withstd))
        print("  no_std, i.e. only ever compiled with std on today:   %d" % len(nostd))
        print("  merged into builds:                                 %d" % len(packed))
        print("sweep builds: %d single + %d combined" % (len(singles), len(packed)))
        for c in sorted(packed, key=lambda s: sorted(s)):
            print("  " + ",".join(sorted(c)))
    else:
        sys.exit(__doc__)


if __name__ == "__main__":
    main()
