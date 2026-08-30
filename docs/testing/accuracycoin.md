# AccuracyCoin, headlessly

125 NES accuracy tests plus 5 information-only pages on a single NROM cartridge.
MIT, © 2025 Chris Siebert, <https://github.com/100thCoin/AccuracyCoin>.

The ROM is a menu program: you press buttons, it prints `PASS` or `FAIL x` on
screen, and you read the screen. None of that is required. Everything the runner
needs is in work RAM, and this page documents exactly where — because a runner
that silently checks the wrong bytes is worse than no runner at all.

Everything below is from the ROM's own commented source (`AccuracyCoin.asm`,
MIT) and its `README.md`. Both are fetched by `scripts/fetch-testdata.sh`.

## First, what this suite actually tests

Not the CPU. Of its 67 documented sections roughly 22 are CPU-only, 24 PPU, 10
APU, 7 DMA and 3 controller, and the interesting ones are about the *interaction*
between them at exact clock alignments: "NMI Suppression", "NMI Timing", "Sprite
0 Hit behavior", "OAM Corruption", "DMC DMA Bus Conflicts", "DMA + $2007 Read",
"Open Bus". None of those can pass until a CPU, a PPU, an APU, both DMA units
and a cartridge are running together in one realized machine on correct clock
domains.

So it is the **last** phase-3 gate, after
[SingleStepTests and nestest](README.md). The runner is built to report per test
rather than pass/fail as a whole, so it is a progress meter while the machine is
half-built.

The ROM targets an RP2A03G CPU and an RP2C02G PPU. A handful of tests
legitimately behave differently on other revisions.

## The memory map the runner uses

| Address | Name in the ROM | What it is |
| --- | --- | --- |
| `$0035` | `RunningAllTests` | 1 while a run-everything pass is in progress, 0 otherwise. **The completion signal.** |
| `$0037` | `PostAllTestTally` | tests completed so far during a pass |
| `$00EC` | `Debug_EC` | boot progress, stepped `$00` → `$0D` through initialisation |
| `$0400-$04FF` | — | **the results page**: one byte per test |
| `$03FB-$03FF` | — | results for the five "DRAW" pages, which assert nothing |
| `$0500-$05FF` | — | per-test scratch, cleared before each test |
| `$0020-$002F` | `Test_UnOp_*` | operands and expectations for the unofficial-instruction tests |
| `$0050-$006F` | `Test_ZeroPageReserved` | scratch a handful of tests use |

The last three are what the ROM's own on-screen debug menu (Select, after a
test) displays. They are diagnostic only.

### The result byte

The engine's `RunTest` routine calls each test through a `JSR` built in RAM and
stores whatever the test left in `A` at that test's fixed result address. The
byte packs two things:

```
bits 1-0   state:  0 = not run ("TEST")
                   1 = PASS
                   2 = FAIL
                   3 = in progress ("....")
bits 7-2   error code, i.e. byte >> 2
```

That is exactly how the ROM's `DrawTEST` routine reads it back: `AND #$3` for
the state, then `AND #$FC` / `LSR` / `LSR` for the code it prints after `FAIL`.

**The error code is not hexadecimal.** `DrawTEST` writes the code straight into
the nametable as a tile index, and the character tiles run `0`-`9` then `A`-`Z`,
so code 10 prints as `A`, 16 as `G`, 20 as `K`. The README's longer sections
genuinely go past `F` — "Unofficial Instructions" runs `1`-`9`, `A`-`K`. A
base-16 formatter would misreport every code above 15. `error_char()` in the
runner does it the ROM's way.

The README's per-section tables say what each code means; the fetch script pulls
`README.md` down alongside the ROM for exactly that reason.

### Which test owns which byte

[`tests/conformance/accuracycoin_tests.rs`](../../tests/conformance/accuracycoin_tests.rs)
is the full 130-entry table — suite, name, result address — read out of the
ROM's own `TestPages:` table and the `result_*` equates beside it, in menu
order. It is checked by a unit test that runs with no corpus present: 130
entries, 125 real and 5 "DRAW", every address distinct and on a page the ROM
uses.

The five DRAW pages are exactly the ones whose result byte lives on **page 3**
rather than page 4 (`$03FB` `Print magic values`, `$03FC` `CPU RAM`, `$03FD`
`CPU Registers`, `$03FE` `PPU RAM`, `$03FF` `Palette RAM`). That is not a
coincidence — the ROM's own run-all loop compares the result pointer's high byte
against 3 and skips those tests. The runner uses the same rule, so the two can
never disagree.

## Driving the menu with no human

The ROM edge-detects buttons: `ReadController1` keeps `controller` and derives
`controller_New` as the newly-pressed bits, and the menu acts on
`controller_New`. So a button must go from released to held, across at least one
NMI, and be released again before it can be seen a second time.

Button bits are the shift register's output order, which is what the ROM's own
read routine assembles by rotating each `$4016` read into bit 0:

| Bit | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| | A | B | Select | **Start** | Up | Down | Left | Right |

Initialisation ends with `menuCursorYPos = $FF`, which is "the cursor is on the
page number at the top of the menu" — and Start at the top of the menu is the
ROM's "run every test in the ROM" command (`AutomaticallyRunEveryTestInROM`).
**So no navigation is needed at all**: the machine boots straight into the state
where a single Start press runs everything.

The whole sequence is:

1. Run frames with nothing held until initialisation finishes (`$00EC` reaches
   `$0D`).
2. Hold Start for a frame or two, then release it.
3. Wait for `$0035` to become 1 — the ROM has entered the run-everything pass.
4. Wait for `$0035` to return to 0 — the pass is finished. `$0037` counts
   progress meanwhile, so a timeout can say how far it got.
5. Read the 125 result bytes.

Rendering is never needed. The run-all path disables the NMI and rendering
itself while it works, and the runner never looks at a pixel. Nothing in this
sequence depends on frame timing beyond "at least one NMI sees the press", which
is why the machine interface takes whole frames rather than cycles.

### Why not just jump to the routine?

`AutomaticallyRunEveryTestInROM` could be entered by forcing `PC`, which would
skip the boot and the button entirely. The runner does not, for two reasons: the
address is not exported anywhere, so it would have to be found by scanning the
PRG for a byte pattern and would break on the next ROM release; and pressing
Start exercises the controller path, which three of the tests are about. Driving
the ROM the way a person does is both more robust and a wider test.

## What the runner reports

* **`RunStatus::Complete`** — the pass finished. Per-test results are meaningful.
* **`RunStatus::NeverStarted { boot_progress }`** — Start was never picked up.
  `boot_progress` is `$00EC`, so you can see how far initialisation got; a value
  below `$0D` means the machine hung during boot and names roughly where.
* **`RunStatus::TimedOut { completed }`** — the pass began and never ended.
  `completed` is `$0037`, the number of tests that finished, so the next test in
  menu order is the one that hung.

Per test, each of the 125 comes back as `Pass`, `Fail(code)`, `NotRun`, or
`Hung` (the engine marked it in progress and never came back). A partly-built
machine yields a report full of `NotRun` and a handful of passes, which is
exactly the progress meter that makes this suite worth running early.

The `$0500-$05FF` scratch region is captured too. Be careful with it: it is
cleared before *every* test, so after a full pass it holds only the last test's
working values. To use it for diagnosis, run the one failing test on its own.

## Ledgering failures

[`tests/conformance/ledgers/accuracycoin.txt`](../../tests/conformance/ledgers/accuracycoin.txt),
keyed by the low byte of the result address rather than by name — the address is
what the ROM writes and is stable across releases, whereas the display name is
text on a menu. The runner prints both.

Before writing an entry: trace the divergence to a documented hardware-revision
difference. "It fails on a different PPU revision" is a real reason; "it fails
and I do not know why" is a bug, not a ledger entry.
