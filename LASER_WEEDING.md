# Deweeding tool concept

Working notes on adding automated deweeding to solbot. Companion to `PLAN.md`.
Constraint driving everything below: solbot has **~400W total PV power budget**
for drive + compute + tooling combined.

## Growth-stage scenarios

| Stage | Crop state | Guidance available | Mechanical (interrow) | In-row treatment |
|---|---|---|---|---|
| 1 | No plants out yet | None | Narrow blades | Laser/thermal everything green (can't distinguish crop from weed pre-emergence) |
| 2 | Some plants out | Row line not yet reliably computable | Rapeseed interrow guide rows + wide blades + tool_slider | Detect + treat weeds in-row |
| 3 | Row easily detected | Full row line from crop detection | tool_slider on row line, wide blades | Treat weeds in-row |

Stage 1 needs whole-field coverage (~30% of area, roughly interrow + in-row
combined, since blades can't discriminate crop from weed yet). Stages 2/3 only
need in-row coverage (~10% of area) once blades can retake the interrow.

**Conclusion: don't laser stage 1.** It's the worst case for a dwell-time-limited
system (highest weed density, no row structure to exploit) and is exactly what
cheap non-targeted mechanical/thermal methods (harrowing, flame weeding, stale
seedbed) already handle well. Reserve any laser/precision effector for in-row
weeding at stage 2/3, where mechanical blades can't be used without hitting crop.

## Chosen split: tool_slider mechanical (interrow) + dedicated in-row effector

- **Interrow**: stays exactly what it is today — `tool_slider` package,
  vision-guided lateral offset (`tool_slider_controller`) driving wide blades
  on the toolbar's `tool_bar_joint` (prismatic, ±0.10m travel). No laser
  involved here.
- **In-row**: a separate, small, precisely-actuated effector mounted at the
  existing implement mount points (`implement_center/left/right` in
  `description.urdf`, 0.36m apart on the 0.80m tool_bar). Two candidate
  technologies compared below.

## Laser option

### Module selection (fits 400W budget)

Commercial laser-weeding systems (Opt Lasers Blue Reaper 80-320W, laser-electronics.de
BluEX 100W) all need 48V liquid-cooled supplies drawing 200W-1kW+ — incompatible
with this platform outright. Even the passively-cooled Opt Lasers FS-30W (24V,
200W max draw) would burn half the entire power budget on one head.

**Chosen tier**: 5-10W 445nm blue diode engraver modules (Sanwu Lasers,
Laserlands, AliExpress equivalents), 12V DC, ~1.5-2A draw (~18-24W each) —
runs directly off solbot's existing 12V bus. ~€40-60 (5W) / ~€100 (10W).
Passive heatsink + fan cooling, no liquid loop. **Must be adjustable-focus**
(not fixed-focus) to tune the focal point to the toolbar's ~0.4m working
height (`tool_bar_joint` origin z=0.4 in `description.urdf`).

### Throughput math (fixed-aim, no galvo)

Dwell time to kill a cotyledon/2-leaf weed with a 5W diode: ~0.5-2s (using 1s
as working estimate) — much longer than commercial 30-150W galvo systems
(~0.1s/weed) because dwell time trades off against laser power.

A **fixed-aim** beam only hits a weed physically under its ~2-5mm spot, so
continuous forward motion during the dwell window is bounded by:

    max_ground_speed ≈ spot_diameter / dwell_time ≈ 0.003m / 1s ≈ 3mm/s (~11 m/hour)

Far below solbot's nominal nav speed. **A fixed-aim laser cannot fire while
driving at normal speed** — it requires stop-dwell-resume motion. Capacity:
~1 weed/sec/head → 3 heads (one per implement mount) ≈ 3 weeds/sec when
stopped. At ~10-20 weeds/m of row (stage 2/3 in-row density), average
effective forward progress works out to roughly 0.15-0.3 m/s — a real
throughput hit vs. continuous driving, but survivable for a slow cultivating
pass.

**Continuous motion requires a galvo scanner** (decouples aim from vehicle
motion), but a galvo head with useful cycle time needs the higher-power
FS-series (24V, ~200W draw) — which reopens the power-budget problem the
fixed-aim 5-10W tier was chosen to avoid.

## Delta-robot / mechanical effector option

Real precedent: Small Robot Company's "Dick" weeding robot uses **3× igus
drylin delta robots in parallel**, each carrying an electric "zapper"
end-effector that kills weeds without herbicide. PhoenixBot (research
platform) uses a similar delta-arm mechanical weeder.

**igus drylin small delta specs** (off-the-shelf, not custom):
- Workspace: 330mm diameter × 75mm height (larger variant: 660mm × 180mm)
- Payload: 5kg lift / 100N (10kg) press force
- Cycle time: ~60 picks/min (~1/s) rated for pick-and-place
- Price: ~$5,500 (vs $20-25k for traditional industrial delta robots) —
  Small Robot Co pays ~£5k/arm
- Lubrication-free sliding plastic components — explicitly chosen for
  muddy/dusty field conditions (no external lube to foul)
- Power draw: small servo/stepper motion, a fraction of the laser's
  electrical draw for comparable throughput — comfortably fits the 400W budget

## Comparison

| | Galvo laser (30-80W optical) | Delta robot + mechanical/electrical effector |
|---|---|---|
| Unit cost | ~€2-5k DIY-integrated (source + galvo optics) | ~$5,500 turnkey (igus small delta) |
| Power draw | ~200W electrical (half the total budget) | Tens of watts — fits easily |
| Workspace fit | Scan area ~250-600mm² at working distance, needs optical design | 330mm-diameter workspace matches 0.36m implement mount spacing off the shelf |
| Cycle time | 0.1-1s/weed depending on power tier | ~1/s rated (pick-and-place benchmark, not weeding-specific) |
| Contact / crop-damage risk | None — a miss just misses | Mechanical/electrical strike near a miscalibrated position can gouge soil or clip crop; PhoenixBot's ~27mm mean positioning error is coarse relative to seedling spacing |
| Field robustness | Sealed optics needed against dust/mud | Purpose-built lubrication-free design for exactly this environment |
| Maturity | DIY integration project | Field-proven, buyable, running in real cereal fields today |

**Working conclusion**: the delta/mechanical effector fits the 400W power
budget without a fight and is an off-the-shelf, field-proven part, whereas a
galvo laser capable of matching its cycle time reopens the power problem the
fixed-aim laser tier was chosen to avoid. The laser's real advantage (zero
mechanical contact, so a mis-aim just misses instead of risking crop damage)
matters most right next to the crop stem — a **hybrid** (mechanical/delta
effector for weeds with clearance from the crop, laser for weeds touching or
very close to the crop stem) is a reasonable next hypothesis to test, rather
than picking one exclusively.

## Open questions

- What physical end-effector does the delta arm carry — blade/crusher vs.
  electrical "zapper" (Small Robot Co style)? Changes the crop-damage-risk
  analysis materially.
- Is solbot's 400W a continuous (PV-generation-limited) budget or a
  battery-buffered peak budget? Affects whether intermittent high-draw firing
  (laser or delta motors) can exceed 400W briefly between weeds.
- Bench-validate actual dwell time for the chosen 5-10W diode against real
  weed samples before committing to the throughput numbers above.
