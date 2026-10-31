# alphard-navi — the GPU and the display pipeline

How the navigation UI gets onto the screen: the PowerVR SGX540 GPU, the
PowerVR services command model, the OpenGL ES / OpenVG userspace, and the
R-Car Display Unit that composes and scans out the result. Companion to
[alphard-navi.md](alphard-navi.md) (board + machine model) and the two KWI docs.

rsemu today models the SGX as `pvr.sgx` — a stand-in that lets the driver
initialise but renders nothing, so the HMI runs on four cores and the screen
stays black. This doc is the map for taking that further: what the hardware is,
what commands cross the kernel boundary, and how the UI is drawn, so the
rendering can eventually be modelled.

Reverse-engineered from the HC59 full-system rootfs (`/vns`, `/usr/lib`,
`/vns/ko`) and the XIP kernel in `LOADING.KWI`. The stack is the stock
Imagination **PowerVR SGX DDK 1.7** (build `1.7@2145535`); nothing about it is
Toyota/Aisin-specific except the HMI on top.

## The two halves

Drawing is split between two pieces of silicon, and they are easy to conflate:

* **PowerVR SGX540** (`0xfce00000`, GIC ID 89) — the GPU. A tile-based
  deferred renderer. It runs OpenGL ES 1.1/2.0 and OpenVG workloads submitted
  from userspace through the PVR services kernel driver. It draws *into
  buffers*; it does not scan out.
* **R-Car Display Unit (DU)** (`0xfff80000`, GIC ID 63) — the display
  controller. It reads finished buffers out of VRAM, composes its hardware
  planes, and drives the panel(s). Kernel driver `rcar_duc`, exposed as
  `/dev/fb0` (front) and `/dev/fb1` (rear) plus an ioctl channel (`SETDISP`,
  `wait_for_vsync`).

A frame is: GPU renders a surface → it is handed to a plane/flip-chain → the DU
composes the planes and scans out at vsync.

## The GPU: PowerVR SGX540

SGX540 is a Series5 USSE core. Register block (the `EUR_CR_*` space) at
`0xfce00000`; the driver reads `EUR_CR_CORE_ID` / `EUR_CR_CORE_REVISION` at init
and refuses to run on a mismatch. Register groups the driver touches include
`EUR_CR_BIF_*` (the MMU/bus-interface: BANK0, CTRL, FAULT, INT_STAT,
MEM_REQ_STAT), `EUR_CR_CLKGATECTL`, and `EUR_CR_EVENT_STATUS`/`_STATUS2` (the
interrupt/event source the LISR reads).

The GPU runs its own firmware, the **EDM microkernel** ("ukernel"): the driver
loads the EDM program into a device memory context at init
(`DevInitSGXPart2KM: Failed to load EDM program` is the failure path) and
thereafter talks to it through shared memory and a kick register. The ukernel
schedules the on-chip pipeline:

* **TA** (Tile Accelerator) — vertex/geometry phase; bins primitives into
  tiles and writes the parameter buffer.
* **3D/ISP+TSP** — pixel phase; rasterises each tile.
* **PDS** (Programmable Data Sequencer) — feeds data and code to the USSE for
  both phases (`PDSVertexCodeData`, `PDSPixelCodeData` in the driver).
* **USSE** — the unified shader ALU that runs the compiled vertex/pixel
  programs (and OpenVG paths).

## Bringup

`/etc/rcS.d/S97sgx_boot.sh` → `/vns/pmng/script/insmod_sgx.sh`:

```
insmod /vns/ko/pvrsrvkm.ko     # PVR services kernel module (331 KiB)
/vns/ko/pvrsrvinit             # uploads the EDM ukernel, creates /dev/pvrsrv
```

No module parameters. `pvrsrvkm` registers the services char device and the
bridge; `pvrsrvinit` (the `SRVINIT` client) runs `DEVINITPART2`, which uploads
the EDM program and the initial PDS/USSE code and brings the core out of reset
(the SGX power domain in `rcar.sysc` must be on first — the platform doc's
"power on SGX error!" path).

## The command model — PVR services bridge

All GPU work crosses the kernel through one ioctl dispatch,
`BridgedDispatchKM`, keyed by a `PVRSRV_BRIDGE_*` id. The SGX-specific ids seen
in `pvrsrvkm.ko` that matter for drawing:

| bridge id | role |
| --- | --- |
| `SGX_GETCLIENTINFO` / `SGX_GETINTERNALDEVINFO` | hand the client the kick register address, ukernel CCB, status/sync memory |
| `SGX_DEVINITPART2` | finish init, load EDM |
| `SGX_DOKICK` | **the render kick** — submit a prepared TA/3D command to the ukernel |
| `SGX_FLUSH_HW_RENDER_TARGET` | fence/flush a render target |
| `SGX_2DQUERYBLTSCOMPLETE` | 2D-blit (HWTRANSFERCONTEXT) completion query |
| `SGX_ADDSHAREDPBDESC` / `FINDSHAREDPBDESC` | shared parameter-buffer (tiler heap) management |
| `SGX_GETMISCINFO` | ukernel status, HWR info |
| `SGX_HWRECOVERY` | hardware-recovery reset after a lockup |

Submission is **circular-command-buffer** based: userspace builds a command in
a CCB shared with the ukernel (the TA/3D control stream, the PDS program
addresses, the USSE code addresses, the parameter-buffer and render-target
handles) and rings the doorbell via `SGX_DOKICK`. The ukernel executes TA then
3D, writes completion into the shared sync objects, and raises
`EUR_CR_EVENT_STATUS`; the kernel LISR wakes the waiters. `HWRECOVERY` plus a
soft reset is the watchdog path if the ukernel stops responding.

### Presenting to the display (the DC swap chain)

The driver also implements a **PVR Display Class** client that bridges rendered
buffers to the DU with vsync-correct flips: `PVRSRVCreateDCSwapChainKM`,
`DestroyDCSwapChain`, and the completion notice `Flip Command Complete Data %u
for display device %u`. A swap chain is a set of buffers in display memory; the
GPU's completion for a frame triggers the DU flip to the next buffer. This is
how GL double-buffering becomes a tear-free scan-out without the CPU polling.

## Userspace graphics stack

Stock IMG DDK libraries in `/usr/lib`:

```
libEGL.so  libIMGegl.so            EGL + the IMG EGL core
libGLES_CM.so                      OpenGL ES 1.1 (fixed-function, "common")
libGLESv2.so  libglslcompiler.so   OpenGL ES 2.0 + the online GLSL-ES compiler
libOpenVG.so  libOpenVGU.so        OpenVG 1.x (2D vector) + the VGU utility lib
libpvr2d.so                        PVR2D 2D/blit API
libsrv_um.so  libsrv_init.so       services usermode + SRVINIT
libpvrEWS_WSEGL.so                 WSEGL plugin: EWS compositor window system
libpvrPVR2D_LINUXFBWSEGL.so        WSEGL plugin: present straight to /dev/fb0
libpvrPVR2D_GRAWSEGL.so            WSEGL plugin: present to the "GRA" surface layer
```

Call path for any GL/VG app: `GLES/VG → EGL (libEGL → libIMGegl) → a WSEGL
plugin (chooses how an EGL window is presented) → PVR2D / libsrv_um → the
bridge → pvrsrvkm → SGX`. The GLES advertised extension set is the usual
Series5 one (PVRTC `GL_IMG_texture_compression_pvrtc`, `GL_IMG_texture_format_
BGRA8888`, ETC1, FBO/`GL_OES_framebuffer_object`, `GL_OES_EGL_image`), plus the
IMG-internal `GL_REL_kick_ta` / `GL_REL_get_meminfo` hooks.

The **WSEGL plugin** is the seam between GL and the screen, and three exist:

* **EWS** (`libpvrEWS_WSEGL`) — a compositing window system. Each EGL window is
  a `CompositorWindowDrawable`; frames reach the screen via `DoPresentFlip`
  (page-flip a flip chain) or `DoPresentBlit` (blit-composite). This is what
  lets several independent GL/VG surfaces (map, menu overlays, status bar) be
  layered into one picture.
* **LINUXFB** (`libpvrPVR2D_LINUXFBWSEGL`) — present a single surface straight
  to `/dev/fb0`, no compositor.
* **GRA** (`libpvrPVR2D_GRAWSEGL`) — present into the vendor "GRA" surface
  layer (see `gra.so` below), so GL output becomes a managed display layer.

## The display pipeline — DU, layers, and VRAM

The R-Car **DU** composes hardware planes and scans out. The vendor layer
manager sits on top:

* **`screen.so`** — the layer/display manager (`S_ScreenDisplay`,
  `S_ScreenLayer`, `displayNo`, `layerNum`, per-layer `buffered` /
  `bufferAutoAllocation`, `targetDisplayNo`). It programs the DU through the
  kernel **DUC** (`DUC_SUCCESS`) and through **AWG** (`awg.so`,
  `AWG_isetlayer`/`setlayer`/`updScreen`). A layer targets a display and has a
  source buffer; `screen.so` owns the plane assignment and update.
* **`librenderingapi.so`** — the higher-level renderer/compositor the HMI
  calls: `Renderer_CreateLayer` / `SetupLayer` / `Composite` / `ShowSurface` /
  `SetVideoOutputMemory` / `SetScale` / `BlackOut`, plus a
  `Rendering_CmdManager` that drives a hardware blend/ROP unit
  (`bruLayOrderInit`, `setBruLayOrderParam`, `setSrcParam`, `getDst` — the
  R-Car **BRU** blend unit). So inter-layer blending and layer ordering are
  hardware-composited, not done on the GPU.
* **`gra.so`** — a surface abstraction (`GRA_createSurface`,
  `GRA_getFrameAddress`, `GRA_createSurfaceFromID`) that hands out display
  surfaces by frame address/id; the GRA WSEGL plugin targets these.

Displays and memory: **dual display** (front + rear), `/dev/fb0` and
`/dev/fb1`, backed by the VRAM planes fixed in the PWI config
([alphard-navi-kwi-boot.md](alphard-navi-kwi-boot.md)):

```
VP_FRONT_VRAM  0x8E800000  0x280000
VP_REAR_VRAM   0x8EA80000  0x280000
```

(0x280000 = 2.5 MiB per plane, above the kernel's `mem=674M`.) The DU ioctls
are `SETDISP` (set the display/plane configuration) and `wait_for_vsync` (the
frame clock userspace blocks on). The panel resolution is set through the DU/fb
and is not hard-coded in these libraries; read it from `/dev/fb0` (`FBIOGET_
VSCREENINFO`) when modelling.

## How the nav UI actually draws

The HMI is layered; the big pieces in `/vns/bin`:

* **`hmictrl.so`** (5.1 MiB) — the HMI controller / scene driver: owns the
  screen state machine, input, and which layers are shown. It drives the
  renderer and the per-domain HMI cores; it does not itself link GL (it works
  through `librenderingapi`/`screen`).
* **`display_hmi_core.so`** — the graphics-heavy HMI core. It is the component
  that links the full GL/VG stack (`libEGL`, `libGLES_CM`, `libGLESv2`,
  `libIMGegl`, `libOpenVG`, `libOpenVGU`, `libglslcompiler`, `libpvr2d`,
  `libpvrEWS_WSEGL`). It manages composited layers via per-plane receive
  handlers (`Hmiapi_{hng,hno,osg,oso}_ReceiveProcLayer`) — i.e. several named
  layers (GL and VG) blended by the compositor/BRU.
* **`mapview.so`** (links `libEGL` + `libGLES_CM`) — the map renderer. **OpenGL
  ES 1.1**, tile-based: it renders map tiles into **FBOs**
  (`ctrlmap_freeTileFBO`, i.e. render-to-texture per tile) and composes them,
  with depth testing (`glDepthFunc`/`glDepthMask`/`glEnable`) and runtime
  view-angle control for the 3D bird's-eye view (`chgViewangle`,
  `chgVerticalViewangle`). Road/polygon geometry + textured tiles; this is the
  heaviest GPU consumer.
* **`font.so`** (2.4 MiB) — text. A **KeiType** OpenType glyph engine
  (`GetGlyphByUnicode`, OpenType layout `fOTLC_*` for coverage/class =
  GSUB/GPOS, outline + bitmap with a glyph cache). Glyphs are rasterised and
  drawn as textured quads through the GL/VG path.
* **`svgr.so`** — an SVG renderer (vector UI assets), and the OpenVG libs — the
  non-map UI (buttons, meters, icons, gauges) is largely **2D vector** drawn
  with OpenVG/SVG so it scales cleanly, composited over/under the map and video
  layers.

Per-frame, end to end:

1. `hmictrl` decides the active screen and layer set and calls
   `librenderingapi`.
2. Each content layer is rendered on the **SGX**: `mapview` draws the map via
   GLES-1.1-to-FBO tiles; `display_hmi_core` draws vector/overlay layers via
   OpenVG + GLES through EGL/EWS. Each `SGX_DOKICK` submits a TA+3D render; the
   ukernel signals completion.
3. Finished surfaces become **display layers** (`screen.so` /
   `Renderer_SetupLayer` / GRA surfaces).
4. The **DU/BRU** composes the planes (map + overlays + any video/camera
   layer) in the configured order and scans out to `/dev/fb0` (and `/dev/fb1`
   for the rear), flipped at vsync via the DC swap chain / `DoPresentFlip`.

Video/camera (reverse camera, DTV) enters as its own DU plane, composited by
the same BRU under the HMI's vector overlays — which is why the GPU is not in
that path.

## What rsemu models, and what full rendering needs

Today `pvr.sgx` answers the init handshake (core id/revision, the power-domain
poll) so the driver loads and the HMI runs, but no TA/3D work is executed — the
composited output is empty, hence the black screen noted in the platform doc.

To actually produce pixels, in rough order of payoff:

1. **The DU plane scan-out** — even with no GPU, modelling the DU reading the
   VRAM planes (`0x8E800000` front) and the `SETDISP`/flip path would surface
   whatever the compositor and any CPU/2D (`PVR2D`/BRU) writes put there
   (splash bitmaps, vector overlays blitted by `DoPresentBlit`).
2. **The PVR2D / transfer (2D blit) path** — `HWTRANSFERCONTEXT` /
   `2DQUERYBLTSCOMPLETE`: many UI composites and the splash are blits, not 3D.
3. **The SGX TA/3D pipeline** — the full job: execute the CCB the client builds
   on `SGX_DOKICK`, run the PDS/USSE programs, write the render target. This is
   what `mapview` and the GL overlays need. DDK 1.7's command stream and
   register semantics are the spec; the EDM ukernel behaviour is the hard part.

## Provenance and open items

* Stack identified from the DDK libraries and `pvrsrvkm.ko` strings; command
  ids and register groups are from `pvrsrvkm.ko`; the HMI decomposition is from
  the `/vns/bin` library linkage and symbol names. DDK build `1.7@2145535`.
* **Inferred, not yet byte-confirmed:** the BRU identification (from
  `Rendering_CmdManager` symbol names), the exact DU plane/format programming,
  the panel resolution (read it from `/dev/fb0` at run time), and the precise
  CCB/command layout DDK 1.7 uses for `SGX_DOKICK` (needs disassembly of
  `libsrv_um`/`libGLESv2` or a captured command stream).
* The SGX register-level behaviour (EDM ukernel protocol, event bits) is the
  main gap for real rendering and is best pinned by tracing `SGX_DOKICK`
  traffic and the shared CCB against the DDK 1.7 headers.
