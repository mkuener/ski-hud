/* Ski HUD, VARIANT B: 0.96" 128x64 SSD1306 I2C OLED (4 pins VDD GND SCK SDA).
 * Parametric CAD model (JSCAD v2). Units: mm. Separate from site-src/housing/ (variant A, ST7735S colour LCD).
 *
 * OledCAD.build(params) -> { parts:{name:{geom, material, group, label, print, orient, ...}}, groups:{hud,bench,pack}, collisions, presses }
 * OledCAD.printGeom(part) -> the part turned into its print orientation, sitting on Z=0 (STL is Z-up).
 *
 * HUD frame (goggle top centre, used by the goggle-end parts AND the bench fixture):
 *   X = across the face, Y = up, Z = forward toward the lens.
 *   Goggle frame top rail occupies Y [0, railT], Z [-railD/2, railD/2], vent slot at X=0.
 * PACK frame: copied unchanged from variant A (see buildPack).
 * MODULE frame (the OLED board itself): X across (centred), Y = 0 at the pin edge, down to -pcbH;
 *   Z = 0 at the PCB back face, glass on the front (+Z).
 *
 * FDM rules (0.4 mm nozzle, 0.2 mm layers): walls >= 1.2 mm, no feature < 0.6 mm,
 *   sliding fits 0.15 mm per side, loose 0.2-0.35 mm, TPU-to-PETG 0-0.05 mm,
 *   overhangs <= 45 deg or short bridges, no supports anywhere.
 */
(function () {
  const J = (typeof window !== 'undefined' ? window : globalThis).jscadModeling;
  const { cuboid, roundedCuboid, cylinder, cylinderElliptic, roundedCylinder, polygon, sphere } = J.primitives;
  const { union, subtract, intersect } = J.booleans;
  const { translate, rotateX, rotateY, rotateZ, mirrorZ, mirrorX } = J.transforms;
  const { extrudeLinear, extrudeRotate } = J.extrusions;
  const { hull } = J.hulls;
  const { expand } = J.expansions;
  const { vectorText } = J.text;
  const { path2 } = J.geometries;
  const { measureVolume, measureBoundingBox } = J.measurements;
  const D = Math.PI / 180;

  // ---------- OLED module: values from the common mechanical drawing (lcdwiki MC096GX "0.96inch OLED product size").
  // Every one of these is a DATASHEET VALUE: check it on your board with calipers and correct it in the viewer.
  const MODULE = {
    pcbW: 27.3,     // PCB width (along the pin edge)
    pcbH: 27.8,     // PCB height (pin edge to far edge)
    pcbT: 1.2,      // PCB thickness (drawing: 1.2; some boards are 1.0 or 1.6)
    glassW: 26.7,   // glass panel width, centred on the PCB
    glassH: 19.27,  // glass panel height
    glassTop: 4.27, // pin edge of the PCB to the glass edge
    glassT: 1.5,    // glass + tape, PCB front face to glass front face (drawing: 1.5)
    aaW: 21.74,     // active (lit) area width, centred
    aaH: 10.86,     // active area height
    aaTop: 2.10,    // glass edge (pin side) to the active area
    backT: 1.5,     // tallest part on the PCB back (regulator, caps) - estimate
    pin1X: 9.84,    // PCB left edge (seen from the front) to pin 1 (VDD); pitch 2.54
    pinRow: 1.5,    // pin edge of the PCB to the centre of the pin row - estimate
    headerH: 11.3,  // glass front to the pin tips with the header fitted (pins point out the back)
    holeD: 2.0, holeIn: 2.0,   // mounting holes (not used for location: the parts locate on the PCB outline)
    flexX: 6.65, flexW: 14.0,  // flex cable zone on the far edge (from the PCB left edge, width)
  };

  const DEFAULTS = Object.assign({
    // goggles (measure!)
    railT: 11,       // frame top rail height: front/back walls, top surface to their lower edge. MEASURED 11
    ribT: 2,         // ribs between the vent windows are thin bars at the top of the rail, MEASURED 2; open channel below them
    railD: 15,       // rail depth front-to-back, face-side edge to the lens, MEASURED ~15
    ventL: 26.1,     // top-centre vent window length (across the face), MEASURED 26.13. COPOZZ frame: a ladder of windows + ribs
    ventW: 10.2,     // vent window width (front-to-back), MEASURED 10.19 (widest point; the windows have a step)
    ribW: 4,         // rib width between two windows, MEASURED 4 (all ribs the same); the stem's bearing pads sit on them
    stemD: 8,        // stem depth front-to-back: about 0.4 less than the window where the stem passes (centre of the window)
    barZ: -4,        // frame bar shifted toward the face, so its front stays behind the rail front edge (the lens seats there)
    lensGap: 7.5,    // vent centre forward to the inner lens surface (lens sits on the frame's front face: ~railD/2)
    foamD: 16,       // face foam thickness behind the rail (bench clip spine goes behind it; too big is fine, too small is not)
    tilt: 25,        // final pod: screen tilt down from vertical, degrees
    podGap: 2,       // gap between rail underside and pod top
    podZ: -2.5,      // pod shifted toward the face so it clears the lens (the stem stays in the window)
    hood: 3,         // light hood depth in front of the window
    benchTilt: 25,   // bench fixture hinge angle (it is adjustable after printing; this is only the shown pose)
    cableD: 4.6,     // goggle cable outer diameter (4.6 = the 8-core coil of variant A, so the pack boot fits)
    shield: 0.5,     // clear protective sheet in front of the OLED glass (ETH SPH "Cast Sheet 0.5mm transparent"), 0 = none
    ldr: 0,          // 1 = cut the 5 mm light-sensor hole in the bar's front wall (the space inside is always kept free)
    // pack (copied from variant A)
    wall: 1.6, lidTop: 1.8, strapW: 40, strapT: 2.5, plugRoom: 7,
  }, MODULE);
  const TOL = { slide: 0.15, loose: 0.3, tpu: 0.05, pcb: 0.2 };
  const strain = (t, d, L) => +(150 * t * d / (L * L)).toFixed(1);   // cantilever snap strain %, PETG keep <= ~3

  // ---------- helpers ----------
  const C = (min, max) => [(min[0] + max[0]) / 2, (min[1] + max[1]) / 2, (min[2] + max[2]) / 2];
  const S = (min, max) => [max[0] - min[0], max[1] - min[1], max[2] - min[2]];
  function box(min, max) { return cuboid({ center: C(min, max), size: S(min, max) }); }
  function rbox(min, max, r, seg = 12) {
    const s = S(min, max); r = Math.min(r, Math.min(...s) / 2 - 0.05);
    return roundedCuboid({ center: C(min, max), size: s, roundRadius: r, segments: seg });
  }
  const ccw = pts => { let a = 0; for (let i = 0; i < pts.length; i++) { const p = pts[i], q = pts[(i + 1) % pts.length]; a += p[0] * q[1] - q[0] * p[1]; } return a < 0 ? pts.slice().reverse() : pts; };
  // prism from a 2D profile. axis 'x': points are [y,z]; 'y': [z,x]; 'z': [x,y]. Extruded from w0 to w1 along the axis.
  function prism(points, axis, w0, w1) {
    const g = extrudeLinear({ height: w1 - w0 }, polygon({ points: ccw(points) }));
    if (axis === 'z') return translate([0, 0, w0], g);
    if (axis === 'x') return translate([w0, 0, 0], rotateX(90 * D, rotateY(90 * D, g)));
    return translate([0, w0, 0], rotateY(-90 * D, rotateX(-90 * D, g)));
  }
  const cylX = (r, x0, x1, y, z, seg = 32) => translate([(x0 + x1) / 2, y, z], rotateY(90 * D, cylinder({ radius: r, height: x1 - x0, segments: seg })));
  const cylY = (r, y0, y1, x, z, seg = 32) => translate([x, (y0 + y1) / 2, z], rotateX(-90 * D, cylinder({ radius: r, height: y1 - y0, segments: seg })));
  const cylZ = (r, z0, z1, x, y, seg = 24) => translate([x, y, (z0 + z1) / 2], cylinder({ radius: r, height: z1 - z0, segments: seg }));
  const coneX = (r0, r1, x0, x1, y, z) => translate([(x0 + x1) / 2, y, z], rotateY(90 * D, cylinderElliptic({ height: x1 - x0, startRadius: [r0, r0], endRadius: [r1, r1], segments: 32 })));
  const coneY = (r0, r1, y0, y1, x, z) => translate([x, (y0 + y1) / 2, z], rotateX(-90 * D, cylinderElliptic({ height: y1 - y0, startRadius: [r0, r0], endRadius: [r1, r1], segments: 32 })));
  const rectZ = (x0, x1, y0, y1, z) => box([x0, y0, z], [x1, y1, z + 0.01]);

  // Rounded slab along Y: plan (XZ) corners radius rp, bottom edges rb, top edges rt (hull of 8 rounded discs).
  function slab(x0, x1, y0, y1, z0, z1, rp, rb = 0, rt = 0, seg = 24) {
    const disc = (yc, h, rr, cx, cz) => translate([cx, yc, cz], rotateX(-90 * D, rr > 0.01
      ? roundedCylinder({ radius: rp, height: h, roundRadius: rr, segments: seg })
      : cylinder({ radius: rp, height: h, segments: seg })));
    const pcs = [];
    for (const cx of [x0 + rp, x1 - rp]) for (const cz of [z0 + rp, z1 - rp]) {
      pcs.push(rb > 0.01 ? disc(y0 + rb + 0.005, 2 * rb + 0.01, rb, cx, cz) : disc(y0 + 0.005, 0.01, 0, cx, cz));
      pcs.push(rt > 0.01 ? disc(y1 - rt - 0.005, 2 * rt + 0.01, rt, cx, cz) : disc(y1 - 0.005, 0.01, 0, cx, cz));
    }
    return hull(...pcs);
  }
  function text2D(str, cap, stroke) {
    const h = cap * 5 / 7.5;
    const gs = vectorText({ height: h, input: str }).map(s => expand({ delta: stroke / 2, corners: 'round', segments: 8 }, path2.fromPoints({}, s)));
    const g = union(gs), bb = measureBoundingBox(g);
    return translate([-(bb[0][0] + bb[1][0]) / 2, -(bb[0][1] + bb[1][1]) / 2, 0], g);
  }
  function chevron2D(size, ang) {
    const c = Math.cos(ang * D), s = Math.sin(ang * D), k = size / 4;
    const pts = [[1.6, 0], [-1.0, 2.4], [-1.0, 1.3], [0.2, 0], [-1.0, -1.3], [-1.0, -2.4]].map(([x, y]) => [k * (x * c - y * s), k * (x * s + y * c)]);
    return polygon({ points: ccw(pts) });
  }
  function debossY(g2, cx, cz, yTop, depth) { return translate([cx, yTop - depth, cz], rotateX(-90 * D, extrudeLinear({ height: depth + 0.3 }, g2))); }

  // TPU cable boot, sized from the cable radius R (variant A's numbers at R = 2.3).
  function bootDims(R) {
    const rb = R + 0.05, rn = rb + 0.8;
    return { rb, rn, rf: rn + 0.45, fl: 1.2, cs: 0.6, r0: rn + 0.75, r1: rn + 0.15, L: 12, rib: 0.35 };
  }
  function bootGeom(B, w) {
    const pts = [[B.rb, -(w + B.fl)], [B.rf - 0.4, -(w + B.fl)], [B.rf, -(w + B.fl) + 0.4], [B.rf, -w], [B.rn, -w], [B.rn, -B.cs], [B.rn + B.cs, 0]];
    const aS = B.r0 - B.rn - B.cs; pts.push([B.r0, aS]);
    const n = 5, step = (B.L - aS) / n, base = a => B.r0 + (B.r1 - B.r0) * (a - aS) / (B.L - aS);
    for (let i = 0; i < n; i++) { const a0 = aS + i * step, a1 = a0 + step; pts.push([base(a0) - B.rib, a0], [base(a1), a1]); }
    pts.push([B.rb, B.L]);
    return extrudeRotate({ segments: 32 }, polygon({ points: ccw(pts) }));
  }

  // ============================================================
  // OLED module components in the MODULE frame. mode 'wired' = header removed, 4 wires soldered (goggle pod);
  // 'header' = pin header out the back with 4 Dupont jumpers plugged on (bench fixture).
  function oledComps(P, mode) {
    const W = P.pcbW, H = P.pcbH, t = P.pcbT, gF = t + P.glassT;
    const gy1 = -P.glassTop, gy0 = gy1 - P.glassH, ay1 = gy1 - P.aaTop, ay0 = ay1 - P.aaH;
    const fx0 = -W / 2 + P.flexX, fx1 = fx0 + P.flexW;
    const pins = [0, 1, 2, 3].map(i => -W / 2 + P.pin1X + i * 2.54), py = -P.pinRow;
    const c = {
      pcb: { geom: box([-W / 2, -H, 0], [W / 2, 0, t]), kind: 'pcbBlue', label: 'OLED board (blue PCB)' },
      glass: { geom: box([-P.glassW / 2, gy0, t], [P.glassW / 2, gy1, gF]), kind: 'glass', label: 'OLED glass panel' },
      aa: { geom: box([-P.aaW / 2, ay0, gF], [P.aaW / 2, ay1, gF + 0.03]), kind: 'oled', label: 'Active area 21.74 × 10.86', nocheck: true },
      flexF: { geom: box([fx0, -H, t], [fx1, gy0, gF - 0.3]), kind: 'flex', label: 'Panel flex (front bend)' },
      flexB: { geom: box([fx0, -H, -0.3], [fx1, -H + 4, 0]), kind: 'flex', label: 'Panel flex (back)' },
      back: { geom: box([-W / 2 + 4, -H + 4.5, -P.backT], [W / 2 - 4, -3.5, 0]), kind: 'chip', label: 'Parts on the PCB back' },
    };
    if (mode === 'wired') {
      c.solder = { geom: union(box([pins[0] - 0.9, py - 0.8, -1.0], [pins[3] + 0.9, py + 0.8, 0]), box([pins[0] - 0.9, py - 0.8, t], [pins[3] + 0.9, py + 0.8, t + 0.5])), kind: 'steel', label: 'Solder joints (header removed)' };
    } else {
      const cb = P.headerH - gF - 2.5;   // pin length behind the plastic carrier
      c.header = { geom: box([pins[0] - 1.27, py - 1.27, -2.5], [pins[3] + 1.27, py + 1.27, 0]), kind: 'chip', label: 'Pin header carrier' };
      c.dupont = { geom: box([pins[0] - 1.27, py - 1.27, -16.5], [pins[3] + 1.27, py + 1.27, -2.5]), kind: 'dupont', label: '4 Dupont jumpers (female)' };
      c.jumpers = { geom: union(pins.map(x => translate([x, py, -16.5 - 6], cylinder({ radius: 0.65, height: 12, segments: 12 })))), kind: 'cable', label: 'Jumper wires to the ESP32', nocheck: true };
      c.pinsRear = { geom: union(pins.map(x => box([x - 0.32, py - 0.32, -2.5 - cb], [x + 0.32, py + 0.32, -2.5]))), kind: 'steel', label: 'Header pins', nocheck: true };
    }
    return { c, pins, py, gF, ay0, ay1 };
  }
  function mapComps(c, f, pre) { const o = {}; for (const k in c) o[pre + k] = Object.assign({}, c[k], { geom: f(c[k].geom) }); return o; }

  // ============================================================
  // GOGGLE END: screen pod (body + lid), snap stem, frame bar (base + lid + TPU boot)
  function buildHud(P) {
    const comps = {}, rT = P.railT, c = TOL.slide;
    const R = P.cableD / 2, B = bootDims(R), hr = B.rn + TOL.tpu, cs = B.cs;

    // stem arm outer face: as wide as the window allows (max 7.0), the 0.6 mm catches flex in through window and floor slot
    const stemO = Math.max(5.4, Math.min(7.0, P.ventL / 2 - 0.2));
    // never deeper than the window allows, and the bar's slot (shifted by barZ) must stay clear of its wall + the LDR space
    const stemDe = Math.min(P.stemD, P.ventW - 0.4, 2 * (9.4 - 1.3 - Math.abs(P.barZ)));
    // ---- frame bar (outside, on top of the rail) ----
    const imuX0 = 9.0, imuX1 = imuX0 + P.imuL, cx0 = imuX1 + 0.9, cx1 = cx0 + 3.4;
    const BI = { x0: -8.6, x1: cx1 + 2.0, z0: -9.4, z1: 9.4, y0: rT + 1.2, y1: rT + 7.4 };
    const BO = { x0: BI.x0 - 1.6, x1: BI.x1 + 1.6, z0: BI.z0 - 1.6, z1: BI.z1 + 1.6, y0: rT, y1: BI.y1 + 1.2 };
    const bshell = subtract(rbox([BO.x0, BO.y0, BO.z0], [BO.x1, BO.y1, BO.z1], 2), box([BI.x0, BI.y0, BI.z0], [BI.x1, BI.y1, BI.z1]));
    let barBase = intersect(bshell, box([-80, rT - 1, -30], [80, BI.y1, 30]));
    let barLid = intersect(bshell, box([-80, BI.y1, -30], [80, 40, 30]));
    // IMU (ICM-20948 board) on four pads, pressed down by four posts in the lid
    const iy = BI.y0 + 0.8, iw = P.imuW / 2;
    comps.imu = { geom: union(box([imuX0, iy, -iw], [imuX1, iy + 1.6, iw]), box([imuX0 + 0.5, iy + 1.6, -3], [imuX0 + 5, iy + 4.6, 3]), box([imuX1 - 5, iy + 1.6, -3], [imuX1 - 0.5, iy + 4.6, 3]), box([imuX0 + 11, iy + 1.6, -1.5], [imuX0 + 14, iy + 2.5, 1.5])), kind: 'pcbIMU', label: `ICM-20948 IMU ${P.imuL} × ${P.imuW}` };
    const padXZ = [[imuX0 + 2, -iw + 2.4], [imuX0 + 2, iw - 2.4], [imuX1 - 2, -iw + 2.4], [imuX1 - 2, iw - 2.4]].map(([x, z]) => [x, Math.sign(z) * Math.min(Math.abs(z), 6.5)]);
    barBase = union(barBase, ...padXZ.map(([x, z]) => box([x - 1.2, BI.y0 - 0.01, z - 1.2], [x + 1.2, iy, z + 1.2])));
    const imuPosts = padXZ.map(([x, z]) => box([x - 1.0, iy + 1.65, z - 1.0], [x + 1.0, BI.y1 + 0.01, z + 1.0]));
    // stem socket slot in the floor (over the goggle vent)
    barBase = subtract(barBase, box([-stemO - 0.2, rT - 1, -stemDe / 2 - 0.2 - P.barZ], [stemO + 0.2, BI.y0 + 0.1, stemDe / 2 + 0.2 - P.barZ]));
    // cable entry, +X end: the boot neck drops into a U-notch in the base wall, the lid closes it from above
    const cy = (BI.y0 + BI.y1) / 2;
    const csk = coneX(hr, hr + cs + 0.01, BO.x1 - cs, BO.x1 + 0.01, cy, 0);
    barBase = subtract(barBase, cylX(hr, BI.x1 - 0.01, BO.x1 + 0.1, cy, 0), box([BI.x1 - 0.01, cy, -hr], [BO.x1 + 0.1, BI.y1 + 0.1, hr]), hull(csk, translate([0, BI.y1 + 0.2 - cy, 0], csk)));
    comps.cable = { geom: cylX(R, imuX1 + 0.4, BO.x1 + B.L + 2, cy, 0), kind: 'cable', label: `4-wire cable Ø${P.cableD}` };
    // STRAIN RELIEF: cable clamp. Saddle in the base + matching post in the lid, two 0.3 mm ribs bite the jacket,
    // so a pull on the cable never reaches the solder joints on the IMU.
    const cw = R + 1.2, rib = (y0, y1) => intersect(box([cx0, y0, -cw], [cx1, y1, cw]),
      union([cx0 + 0.6, cx1 - 1.2].map(x => subtract(cylX(R + 0.02, x, x + 0.6, cy, 0), cylX(R - 0.3, x - 0.1, x + 0.7, cy, 0)))));
    barBase = union(barBase, subtract(box([cx0, BI.y0 - 0.01, -cw], [cx1, cy, cw]), cylX(R, cx0 - 1, cx1 + 1, cy, 0)), rib(BI.y0, cy));
    const clampPost = union(subtract(box([cx0, cy, -cw], [cx1, BI.y1 + 0.01, cw]), cylX(R, cx0 - 1, cx1 + 1, cy, 0)), rib(cy, BI.y1));
    // lid lip + snap tabs (7 wide, 3.6 long, 0.8 thick, 0.4 bump, 45 deg retention) into blind pockets in the base
    const lipH = 2.0, lipT = 0.8, tL = 3.6, bH = 0.4, tabXs = [2, 27];
    const blip = subtract(box([BI.x0 + c, BI.y1 - lipH, BI.z0 + c], [BI.x1 - c, BI.y1 + 0.01, BI.z1 - c]), box([BI.x0 + c + lipT, BI.y1 - lipH - 1, BI.z0 + c + lipT], [BI.x1 - c - lipT, BI.y1 + 1, BI.z1 - c - lipT]));
    const zl = BI.z1 - c, yb = BI.y1;
    let blip2 = blip; const btabs = [];
    for (const x of tabXs) for (const s of [1, -1]) {
      const zz0 = s > 0 ? zl - lipT : -zl, zz1 = s > 0 ? zl : -zl + lipT;
      blip2 = subtract(blip2, box([x - 4.3, yb - lipH - 1, zz0 - 0.01], [x - 3.5, yb + 0.001, zz1 + 0.01]), box([x + 3.5, yb - lipH - 1, zz0 - 0.01], [x + 4.3, yb + 0.001, zz1 + 0.01]));
      btabs.push(box([x - 3.5, yb - tL, zz0], [x + 3.5, yb + 0.01, zz1]));
    }
    const bb = x => prism([[yb - 3.5, zl], [yb - 2.7, zl + bH], [yb - 2.5, zl + bH], [yb - 2.1, zl]], 'x', x - 3, x + 3);
    const bbs = []; for (const x of tabXs) bbs.push(bb(x), mirrorZ(bb(x)));
    // LIGHT SENSOR (optional 5th wire, auto-brightness): space for a 5 mm LDR (GL5528) behind the front wall at the -X end,
    // facing forward. The space and the notch in the lid lip are always there; the Ø5.3 hole is only cut with ldr = 1.
    const L = { x: -4.5, y: BI.y0 + 2.6, r: 2.6 };
    comps.ldr = { geom: union(cylZ(2.55, BI.z1 - 0.5, P.ldr ? BO.z1 - 0.1 : BI.z1, L.x, L.y), box([L.x - 1.2, L.y - 0.4, BI.z1 - 5], [L.x + 1.2, L.y + 0.4, BI.z1 - 0.5])), kind: P.ldr ? 'ldr' : 'ghost', label: P.ldr ? '5 mm LDR (GL5528), light sensor' : 'Reserved: 5 mm LDR space (ldr = 0, no hole)' };
    if (P.ldr) barBase = subtract(barBase, cylZ(L.r + 0.05 - 0.05, BI.z1 - 0.1, BO.z1 + 0.1, L.x, L.y));
    barLid = union(barLid, blip2, ...btabs, ...bbs, ...imuPosts, clampPost);
    barLid = subtract(barLid, box([L.x - 2.9, BI.y1 - lipH - 1, BI.z1 - 1.5], [L.x + 2.9, BI.y1 + 0.01, BI.z1 + 0.01]));
    barLid = subtract(barLid, cylX(hr, BI.x1 - 0.01, BO.x1 + 0.1, cy, 0), csk, box([BI.x1 - 1.7, BI.y1 - lipH - 1, -B.rf - 0.3], [BI.x1 + 0.01, BI.y1 + 0.01, B.rf + 0.3]));
    const bpk = x => prism([[yb - 3.65, BI.z1 - 0.01], [yb - 3.65, BI.z1 + bH], [yb - 2.35, BI.z1 + bH], [yb - 1.94, BI.z1 - 0.01]], 'x', x - 3.3, x + 3.3);
    for (const x of tabXs) barBase = subtract(barBase, bpk(x), mirrorZ(bpk(x)));
    const rv = 0.4;
    const reveal = subtract(box([BO.x0 - 1, BI.y1 - rv, BO.z0 - 1], [BO.x1 + 1, BI.y1 + rv, BO.z1 + 1]), slab(BO.x0 + rv, BO.x1 - rv, BI.y1 - rv - 0.1, BI.y1 + rv + 0.1, BO.z0 + rv, BO.z1 - rv, 2 - rv));
    barBase = subtract(barBase, reveal, box([BO.x0 - 0.1, BI.y1 - 1.2, -4], [BO.x0 + 0.8, BI.y1 + 0.01, 4]));
    barLid = subtract(barLid, reveal, debossY(text2D('HUD B', 2.6, 0.5), 12, 0, BO.y1, 0.3), debossY(chevron2D(5, -90), 31, 0, BO.y1, 0.3));
    let barBoot = translate([BO.x1, cy, 0], rotateY(90 * D, bootGeom(B, 1.6)));
    barBoot = subtract(barBoot, box([BI.x1 - 3, BI.y1, -8], [BI.x1, BI.y1 + 8, 8]), box([BI.x1 - 3, BI.y0 - 8, -8], [BI.x1, BI.y0, 8]));
    // shift the whole bar toward the face by barZ (its floor slot was cut at -barZ, so it lands over the window)
    const BZ = g => translate([0, 0, P.barZ], g);
    barBase = BZ(barBase); barLid = BZ(barLid); barBoot = BZ(barBoot);
    for (const k of ['imu', 'cable', 'ldr']) comps[k].geom = BZ(comps[k].geom);

    // ---- screen pod (inside the goggle, hangs under the rail) ----
    // pod local: x across, y = 0 at the pod top, down; z forward (back face z=0)
    const W = P.pcbW, H = P.pcbH, cl = TOL.pcb, tabGap = 0.35;
    const PI = { x0: -(W / 2 + tabGap + 0.7 + c), x1: W / 2 + tabGap + 0.7 + c, y1: -1.2, z0: 1.2 };
    PI.y0 = PI.y1 - (H + 2 * cl);
    const yT = PI.y1 - cl;                 // PCB pin edge
    const zp0 = PI.z0 + 2.2;               // PCB back face (2.2 mm room behind for parts + wires)
    const zp1 = zp0 + P.pcbT, zg = zp1 + P.glassT, zf = zg + 0.2;   // glass front, front plate inner face
    PI.z1 = zf;
    const shD = P.shield > 0.05 ? P.shield + 0.1 : 0, zw = zf + shD;   // clear sheet pocket in the front plate's inner face
    const PO = { x0: PI.x0 - 1.2, x1: PI.x1 + 1.2, y0: PI.y0 - 1.2, y1: 0, z0: 0, z1: zw + 1.2 };
    const zs = PI.z0;                      // lid / body split
    const M = oledComps(P, 'wired'), toPod = g => translate([0, yT, zp0], g);
    const podComps = mapComps(M.c, toPod, '');
    // wire bundle (4 × ~1.1 mm silicone wire, flat) from the pads up through the top wall
    podComps.wires = { geom: box([-2.4, yT - P.pinRow - 0.6, zp0 - 2.1], [2.4, PO.y1 + 0.5, zp0 - 1.05]), kind: 'cable', label: '4 wires to the bar (VDD GND SCK SDA)' };
    const pshell = slab(PO.x0, PO.x1, PO.y0, PO.y1, PO.z0, PO.z1, 1.2, 0.4, 0.8);
    let body = subtract(intersect(pshell, box([-40, -50, zs], [40, 10, 30])), box([PI.x0, PI.y0, PI.z0 - 1], [PI.x1, PI.y1, PI.z1]));
    let lid = intersect(pshell, box([-40, -50, -5], [40, 10, zs]));
    // window over the active area (+0.5 margin), 45 deg chamfer through the whole front plate
    const wx = P.aaW / 2 + 0.5, wy1 = yT + M.ay1 + 0.5, wy0 = yT + M.ay0 - 0.5, ch = PO.z1 - zw;
    body = subtract(body, hull(rectZ(-wx, wx, wy0, wy1, zw - 0.02), rectZ(-wx - ch - 0.01, wx + ch + 0.01, wy0 - ch - 0.01, wy1 + ch + 0.01, PO.z1 + 0.01)));
    // clear sheet: lies in a pocket (0.2 mm clearance round it, 0.1 mm deeper than the sheet) between the plate and the glass,
    // 2 mm bigger than the window all round, kept clear of the corner pads. Keeps fog and snow off the panel.
    let sheet = null;
    if (shD) {
      const sx = wx + 2.0, sy1 = Math.min(wy1 + 2.0, yT - 3.95), sy0 = wy0 - 2.0;
      body = subtract(body, box([-sx - 0.2, sy0 - 0.2, zf - 0.01], [sx + 0.2, sy1 + 0.2, zw]));
      sheet = { geom: box([-sx, sy0, zw - P.shield], [sx, sy1, zw]), kind: 'sheet', label: `Clear sheet ${P.shield} mm, cut ${(2 * sx).toFixed(1)} × ${(sy1 - sy0).toFixed(1)}`, size: [+(2 * sx).toFixed(1), +(sy1 - sy0).toFixed(1)] };
    }
    // corner pads: the PCB front rests on them (clear of the glass and the flex), front plate behind them
    const padX = [W / 2 - 3.5, W / 2 - 0.3], padY = [[yT - 3.5, yT - 0.3], [yT - H + 0.3, yT - H + 3.5]];
    const corner = (z0, z1) => { const o = []; for (const s of [1, -1]) for (const [ya, yb2] of padY) o.push(box([s > 0 ? padX[0] : -padX[1], ya, z0], [s > 0 ? padX[1] : -padX[0], yb2, z1])); return o; };
    // body pads: bottom ones stand on the bottom wall, top ones hang from the top wall with a 45 deg underside
    // (the body prints bottom face down, so nothing starts in mid-air)
    for (const s of [1, -1]) {
      const x0 = s > 0 ? padX[0] : -padX[1], x1 = s > 0 ? padX[1] : -padX[0];
      body = union(body, box([x0, PI.y0 - 0.01, zp1], [x1, padY[1][1], zf + 0.01]),
        prism([[PI.y1 + 0.01, zp1], [yT - 1.8, zp1], [yT - 1.8 - (zf - zp1), zf + 0.01], [PI.y1 + 0.01, zf + 0.01]], 'x', x0, x1));
    }
    // light hood: straight cheeks, brow with a 45 deg underside (prints without support), chin flared 45 deg so
    // it never cuts the rays going down to the lens but stops the eye from seeing the panel from below
    const hz = PO.z1 + P.hood, ox = wx + ch + 0.3, oy0 = wy0 - ch - 0.3, hw = 1.6;   // 0.3 step off the window chamfer (no coincident faces)
    let hoodG = null;
    if (P.hood > 0.05) {
      const inner = hull(rectZ(-ox, ox, oy0, PO.y1 - 1.2, PO.z1 - 0.02), rectZ(-ox, ox, oy0 - P.hood, PO.y1 - 1.2, hz));
      const outer = hull(rectZ(-ox - hw, ox + hw, oy0 - hw, PO.y1, PO.z1 - 0.02), rectZ(-ox - hw, ox + hw, oy0 - P.hood - hw, PO.y1, hz));
      const brow = intersect(outer, prism([[PO.y1 - 1.2, hz], [PO.y1 - 1.2 - P.hood, PO.z1 - 0.02], [PO.y1 - 1.2, PO.z1 - 0.02]], 'x', -ox - hw, ox + hw));
      // gusset under the chin: its underside runs back down to the front face at 45 deg
      const cy0 = oy0 - hw, chinG = prism([[cy0 + 0.01, PO.z1 - 0.02], [cy0 - P.hood, hz], [cy0 - 2 * P.hood, PO.z1 - 0.02]], 'x', -ox - hw, ox + hw);
      hoodG = union(subtract(outer, inner), brow, chinG);
      body = union(body, hoodG);
    }
    // lid: lip ring inside the walls (stays below the PCB back), snap tabs on the sides, corner posts press the PCB
    const tz1 = zf - 0.3, pbH = 0.45, tY0 = (PO.y0 + PO.y1) / 2 - 4, tY1 = tY0 + 8;
    const lipTop = zp0 - 0.45, xl = PI.x1 - c;   // below the panel flex on the PCB back
    let plip = subtract(box([PI.x0 + c, PI.y0 + c, zs - 0.01], [PI.x1 - c, PI.y1 - c, lipTop]), box([PI.x0 + c + 0.85, PI.y0 + c + 0.85, zs - 1], [PI.x1 - c - 0.85, PI.y1 - c - 0.85, lipTop + 1]));
    plip = subtract(plip, box([PI.x0 - 1, tY0 - 0.8, zs - 1], [PI.x0 + c + 0.86, tY1 + 0.8, lipTop + 1]), box([PI.x1 - c - 0.86, tY0 - 0.8, zs - 1], [PI.x1 + 1, tY1 + 0.8, lipTop + 1]),
      box([-5.2, PI.y1 - 2, zs - 1], [5.2, PI.y1 + 1, lipTop + 1]));   // wire / solder-joint gap at the top
    const ptab = s => box([s > 0 ? xl - 0.7 : -xl, tY0, zs - 0.01], [s > 0 ? xl : -xl + 0.7, tY1, tz1]);
    const pb = () => prism([[tz1 - 0.1, xl], [tz1 - 0.8, xl + pbH], [tz1 - 1.3, xl + pbH], [tz1 - 1.3, xl]], 'y', tY0 + 0.5, tY1 - 0.5);
    lid = union(lid, plip, ptab(1), ptab(-1), pb(), mirrorX(pb()), ...corner(zs - 0.01, zp0));
    // release windows for the tab bumps (push a pin in to take the lid off)
    body = subtract(body, box([PI.x1 - 0.1, tY0, tz1 - 1.5], [PO.x1 + 0.1, tY1, tz1 + 0.15]), box([PO.x0 - 0.1, tY0, tz1 - 1.5], [PI.x0 + 0.1, tY1, tz1 + 0.15]));
    // wire hole through the top wall, behind the PCB
    body = subtract(body, box([-3.0, PI.y1 - 0.1, zs - 0.01], [3.0, PO.y1 + 0.1, zp0]));

    // place: tilt about the pod's top-back edge, hang under the rail; the wire hole sits under the vent
    const zOff = (zs + zp0) / 2;
    const T = g => translate([0, -P.podGap, -zOff + P.podZ], rotateX(P.tilt * D, g));
    const Tinv = g => rotateX(-P.tilt * D, translate([0, P.podGap, zOff - P.podZ], g));
    if (sheet) podComps.sheet = sheet;
    body = T(body); lid = T(lid);
    Object.assign(comps, mapComps(podComps, T, 'oled_'));
    // stem: two 1.4 mm spring arms from the pod top up through the vent and the bar floor; 0.4 mm catches click
    // over the floor. Wire channel between the arms. Bearing pads either side of the window rise through the open channel
    // and push up under the 2 mm ribs, so pod + bar clamp the ribs;
    // so pod + bar clamp the rail and knocks go into the frame, while the vent stays open for air.
    const sTop = BI.y0 + 1.6, yr = -P.podGap + 0.3;
    const aO = stemO, aI = aO - 1.4, pw = Math.min(3.0, P.ribW - 0.6), sd = stemDe / 2;
    const arm = s => box([s > 0 ? aI : -aO, -P.podGap - 2.5, -sd], [s > 0 ? aO : -aI, sTop, sd]);
    const wedge = s => prism([[s * aO, sTop], [s * aO, BI.y0 + 0.15], [s * (aO + 0.6), BI.y0 + 0.15], [s * (aO + 0.6), BI.y0 + 0.6]], 'z', -sd, sd);
    const fillet = s => prism([[s * aI, yr - 0.01], [s * aI, yr + 1.0], [s * (aI - 1.0), yr - 0.01]], 'z', -sd, sd);
    const pad = s => box([s > 0 ? P.ventL / 2 + 0.3 : -P.ventL / 2 - 0.3 - pw, -P.podGap - 2.5, -sd], [s > 0 ? P.ventL / 2 + 0.3 + pw : -P.ventL / 2 - 0.3, rT - P.ribT, sd]);
    let stem = union(arm(1), arm(-1), wedge(1), wedge(-1), fillet(1), fillet(-1), box([-aO, -P.podGap - 2.5, -sd], [aO, yr, sd]), pad(1), pad(-1));
    // locators: two rigid blocks that fill the window's length (0.25 mm clearance at each end), so the pod can't slide
    // sideways in a long window. They stop 0.5 mm below the rail top and never reach the bar.
    const lx1 = P.ventL / 2 - 0.25, lx0 = Math.max(aO + 1.4, lx1 - 2.0), lz = Math.min(sd, 2.5);
    const locate = lx1 - lx0 >= 0.8;
    if (locate) for (const s2 of [1, -1]) stem = union(stem, box([s2 > 0 ? lx0 : -lx1, -P.podGap - 2.5, -lz], [s2 > 0 ? lx1 : -lx0, rT - 0.5, lz]));
    stem = subtract(stem, box([-2.6, -P.podGap - 6, -sd - 1], [2.6, yr + 0.01, sd + 1]));
    // keep the stem out of the cavity and out of the lid's slab
    stem = subtract(stem, T(box([PI.x0, PI.y0, PI.z0 - 0.01], [PI.x1, PI.y1, PI.z1])), T(box([PO.x0 - 1, PO.y0 - 1, -6], [PO.x1 + 1, PO.y1 + 0.2, zs])));
    body = union(body, stem);
    const wins = []; for (let i = -4; i <= 4; i++) { const x = i * (P.ventL + P.ribW); wins.push(box([x - P.ventL / 2, -1, -P.ventW / 2], [x + P.ventL / 2, rT + 1, P.ventW / 2])); }
    const rail = subtract(box([-45, 0, -P.railD / 2], [45, rT, P.railD / 2]), ...wins, box([-46, -1, -P.ventW / 2], [46, rT - P.ribT, P.ventW / 2]));
    const armL = sTop - yr;
    const reach = +measureBoundingBox(body)[1][2].toFixed(1);
    // 4-wire route for the viewer (cable -> IMU pads -> stem channel -> OLED pads)
    const toW = (x, y, z) => { const t = P.tilt * D; return [x, y * Math.cos(t) - z * Math.sin(t) - P.podGap, y * Math.sin(t) + z * Math.cos(t) - zOff + P.podZ]; };
    const route = [[cx0 - 0.4, cy, P.barZ], [imuX1 - 1.5, iy + 3.2, 6.0 + P.barZ], [imuX0 + 3, iy + 3.4, 6.0 + P.barZ], [3.5, BI.y0 + 1.4, 0.6], [0, BI.y0 - 0.6, 0], [0, rT / 2, 0], [0, -P.podGap + 0.5, 0],
      toW(0, PI.y1 + 0.2, zp0 - 1.0), toW(0, yT - P.pinRow, zp0 - 1.0)];

    return {
      parts: {
        pod_body: { geom: body, material: 'PETG', label: 'Screen pod body + stem + hood', print: `bottom face down, stem up (leans ${P.tilt}°), 4 mm brim, no supports`, orient: g => rotateX(90 * D, Tinv(g)), layer: 0.16, infill: '100 % (thin walls)', note: 'Bottom wall prints first, top wall bridges 30 mm' },
        pod_lid: { geom: lid, material: 'PETG', label: 'Screen pod lid', print: 'back face down, tabs up, no supports', orient: g => Tinv(g), layer: 0.16, infill: '100 %' },
        bar_base: { geom: barBase, material: 'PETG', label: 'Frame bar base', print: 'floor down, no supports', orient: g => rotateX(90 * D, g), layer: 0.2, infill: '20 %' },
        bar_lid: { geom: barLid, material: 'PETG', label: 'Frame bar lid', print: 'top face down (text on the bed), no supports', orient: g => rotateX(-90 * D, g), layer: 0.2, infill: '20 %' },
        bar_boot: { geom: barBoot, material: 'TPU', label: 'Cable boot, bar (TPU 95A)', print: 'flange end down, 3 mm brim', orient: g => rotateY(-90 * D, g), layer: 0.2, infill: '100 %' },
      },
      comps, rail, glandOut: [BO.x1 + B.L, cy, P.barZ], route,
      presses: [['bar_base', 'c:cable', 'cable clamp ribs bite 0.3 mm into the jacket'], ['bar_lid', 'c:cable', 'cable clamp ribs bite 0.3 mm into the jacket']],
      dims: { bar: [BO.x1 - BO.x0, BO.y1 - BO.y0, BO.z1 - BO.z0], pod: [PO.x1 - PO.x0, PO.y1 - PO.y0, PO.z1 - PO.z0 + P.hood] },
      explode: { bar_lid: [0, 12, 0], bar_boot: [14, 0, 0], pod_body: [0, -6, 8], pod_lid: [0, -6, -10] },
      checks: {
        strain: { bar: strain(lipT, bH - c, tL), pod: strain(0.7, pbH - c, tz1 - 0.8 - zs), stem: strain(1.4, 0.4, armL) },
        armL: +armL.toFixed(1), reach, barFront: +(P.railD / 2 - (BO.z1 + P.barZ)).toFixed(1), barBack: +((BO.z0 + P.barZ) + P.railD / 2).toFixed(1), stemW: +(2 * stemO).toFixed(1), stemD: +stemDe.toFixed(1), locate, padW: +Math.min(3.0, P.ribW - 0.6).toFixed(1), lensClear: +(P.lensGap - reach).toFixed(1),
        window: [+(2 * wx).toFixed(2), +(wy1 - wy0).toFixed(2)], sheet: sheet ? sheet.size : null, ldr: !!P.ldr,
      },
    };
  }

  // ============================================================
  // BENCH FIXTURE: a spring clip that slides onto the goggle's top edge from behind (over the face foam),
  // plus a cradle that holds the OLED with its pin header and Dupont jumpers, on a friction hinge
  // (a 20 mm piece of 1.75 mm filament is the hinge pin). No vent stem, no goggle measurements beyond railT/foamD.
  function buildBench(P) {
    const comps = {}, rT = P.railT, jt = 2.0;
    const W = P.pcbW, H = P.pcbH, gF = P.pcbT + P.glassT;
    // ---- cradle, local frame: hinge pin axis = X axis through the origin; module face +Z ----
    const top = -3.0, zpb = 4.1;                 // module flex edge 3 mm below the pin, PCB back 4.1 in front of it
    const xi = W / 2 + 0.15, xo = xi + 1.6;      // rail inner / outer face
    const zB0 = zpb - 1.3, zB1 = zpb - 0.1, zF0 = zpb + gF + 0.1, zF1 = zF0 + 1.0;   // back lip, front lip
    const ybot = top - H;
    let cradle = [];
    for (const s of [1, -1]) {
      const X = (a, b) => s > 0 ? [a, b] : [-b, -a];
      const [w0, w1] = X(xi, xo), [b0, b1] = X(W / 2 - 0.55, xi + 0.01), [f0, f1] = X(P.glassW / 2 - 1.0, xi + 0.01);
      cradle.push(box([w0, ybot - 1.7, zB0], [w1, top - 0.5, zF1]));                 // side wall
      cradle.push(box([b0, ybot - 1.7, zB0], [b1, top - 0.5, zB1]));                 // back lip (0.55 over the PCB back)
      cradle.push(box([f0, ybot - 1.7, zF0], [f1, top - 0.5, zF1]));                 // front lip (1.0 over the glass edge)
      // ear: ring round the pin, joined to the rail top
      const ear = hull(cylX(2.8, w0, w1, 0, 0), box([w0, top - 1.5, zB0], [w1, top - 0.5, zF1]));
      cradle.push(subtract(ear, cylX(0.95, w0 - 1, w1 + 1, 0, 0, 20)));
    }
    cradle.push(box([-xo, ybot - 1.7, zB0], [xo, ybot - 0.1, zF1]));                 // bottom bar = stop under the pin edge
    cradle = union(cradle);
    cradle = subtract(cradle, cylX(0.95, -xo - 1, xo + 1, 0, 0, 20));
    // module placed upside down (pins at the bottom), so the Dupont plugs point back-down, clear of the frame
    const Mb = oledComps(P, 'header');
    const toCr = g => translate([0, top - H, zpb], rotateZ(180 * D, g));
    const crComps = mapComps(Mb.c, toCr, '');
    crComps.pin = { geom: cylX(0.875, -xo - 1.0, xo + 1.0, 0, 0, 16), kind: 'cable', label: 'Hinge pin: 1.75 mm filament, 34 mm' };

    // ---- clip (HUD frame) ----
    let zS = -P.railD / 2 - P.foamD;             // spine inner face: just behind the face foam
    const yH = -jt - 3.0, zH = P.railD / 2 - 8.5; // hinge axis
    const tw = xi + 0.1;                          // tongue half width: 0.1 mm interference with each ear = friction
    const zTop1 = P.railD / 2 - 1.0, nw = 6.0, ck = 3.0;   // top jaw end (1 mm short of the lens seat), narrow half width, cheek width
    // grip bump on the solid front lip of the rail, in front of the vent window (not over the opening)
    const zBump = Math.min(zTop1 - 1.2, Math.max(P.ventW / 2 + 1.2, (P.ventW / 2 + P.railD / 2) / 2));
    const zFull = zH - 2.6 - 5.6;                 // cheeks start here (45 deg gusset under them)
    const zA = zFull - (tw - nw);                 // the narrow jaw forks into two 45 deg arms here
    zS = Math.min(zS, zA + jt - 1.0);             // thin foam: spine sits a bit further back so the jaw has room to fork
    const topJaw = box([-nw, rT, zS - jt], [nw, rT + jt, zTop1]);
    const spine = box([-nw, -jt, zS - jt], [nw, rT + jt, zS]);
    const armP = s => prism([[zA, s * (nw - ck)], [zA, s * nw], [zFull, s * tw], [zH + 2.6, s * tw], [zH + 2.6, s * (tw - ck)], [zFull, s * (tw - ck)]], 'y', -jt, 0);
    const botJaw = union(box([-nw, -jt, zS - jt], [nw, 0, zA + 0.01]), armP(1), armP(-1));
    // two cheeks round the pin (their outer faces rub on the cradle ears), each with a 45 deg gusset below
    const cheek = s => { const a = s > 0 ? tw - ck : -tw, b = s > 0 ? tw : -tw + ck;
      return union(hull(cylX(2.6, a, b, yH, zH), box([a, yH, zH - 2.6], [b, -jt + 0.01, zH + 2.6])),
        prism([[-jt + 0.01, zFull], [yH - 2.6, zH - 2.6], [-jt + 0.01, zH - 2.6]], 'x', a, b)); };
    const tongue = union(cheek(1), cheek(-1)), gusset = tongue;
    const bump = cylX(0.7, -nw, nw, rT + 0.2, zBump, 24);               // 0.5 mm grip bump under the top jaw
    let clip = union(topJaw, spine, botJaw, tongue, gusset, bump);
    // lead-in on the top jaw tip, pin hole
    clip = subtract(clip, prism([[rT - 0.01, zTop1 - 1.2], [rT - 0.01, zTop1 + 0.01], [rT + 1.2, zTop1 + 0.01]], 'x', -nw - 1, nw + 1), cylX(0.95, -tw - 1, tw + 1, yH, zH, 20));
    clip = subtract(clip, debossY(text2D('B', 4, 0.6), 0, (zS + zTop1) / 2, rT + jt, 0.3));

    const TB = g => translate([0, yH, zH], rotateX(P.benchTilt * D, g));
    const TBinv = g => rotateX(-P.benchTilt * D, translate([0, -yH, -zH], g));
    Object.assign(comps, mapComps(crComps, TB, 'oled_'));
    const foam = box([-45, -3, zS], [45, rT, -P.railD / 2]);
    const wins = []; for (let i = -4; i <= 4; i++) { const x = i * (P.ventL + P.ribW); wins.push(box([x - P.ventL / 2, -1, -P.ventW / 2], [x + P.ventL / 2, rT + 1, P.ventW / 2])); }
    const rail = subtract(box([-45, 0, -P.railD / 2], [45, rT, P.railD / 2]), ...wins, box([-46, -1, -P.ventW / 2], [46, rT - P.ribT, P.ventW / 2]));
    const reach = +measureBoundingBox(TB(cradle))[1][2].toFixed(1);
    return {
      parts: {
        bench_clip: { geom: clip, material: 'PETG', label: 'Bench clip (slides onto the goggle top edge)', print: 'spine face down, jaws standing up, no supports', orient: g => g, layer: 0.2, infill: '100 % (walls only)' },
        bench_cradle: { geom: TB(cradle), material: 'PETG', label: 'Bench OLED cradle (friction hinge)', print: 'front (glass side) down, ears up, no supports', orient: g => rotateX(180 * D, TBinv(g)), layer: 0.2, infill: '100 % (walls only)' },
      },
      comps, rail, foam,
      presses: [['bench_clip', 'bench_cradle', 'hinge: tongue is 0.1 mm wider than the gap between the ears (friction)']],
      ctxPress: { gap: rT, bump: 0.5 },
      explode: { bench_cradle: [0, -10, 10], bench_clip: [0, 8, -8] },
      checks: { reach, lensClear: +(P.lensGap - reach).toFixed(1), drop: +(-(measureBoundingBox(TB(cradle))[0][1])).toFixed(1) },
    };
  }

  // ============================================================
  // STRAP PACK: copied from variant A (site-src/housing/cad.js, 2026-11-10) with the cable made a parameter.
  // ESP32 Lite + 103450 LiPo; the internal 8-pin connector pair becomes a 4-pin one (only 4 wires to the goggle).
  function buildPack(P) {
    const w = P.wall, c = TOL.slide, BOOT = bootDims(P.cableD / 2), bootHoleR = BOOT.rn + TOL.tpu;
    const IN = { x0: -30, x1: 30 + P.plugRoom, z0: -22, z1: 22, y0: w, y1: w + 18.8 };
    const OUT = { x0: IN.x0 - w, x1: IN.x1 + w, z0: IN.z0 - w, z1: IN.z1 + w, y0: 0, y1: IN.y1 + P.lidTop };
    const ys = w + 10.4 + 1.8, pcbTop = ys + 1.6, lidIn = IN.y1;
    const R = { p: 3.5, b: 2.0, t: 3.0 }, Ri = { p: R.p - w, b: R.b - w, t: 1.2 };
    const comps = {};
    comps.battery = { geom: box([-23.2, w, -20.2], [26.8, w + 10.4, 13.8]), kind: 'battery', label: '103450 LiPo' };
    comps.foam = { geom: box([-20.5, w + 10.4, -19.8], [26.5, ys, 4.8]), kind: 'foam', label: 'foam tape + Kapton' };
    const bx0 = -21.2, bx1 = 29.8, bz0 = -20.2, bz1 = 5.2, zc = (bz0 + bz1) / 2;
    comps.pcb = { geom: box([bx0, ys, bz0], [bx1, pcbTop, bz1]), kind: 'pcb', label: 'ESP32 Lite' };
    comps.usb = { geom: box([23.3, pcbTop, zc + 3 - 4.47], [30.6, pcbTop + 3.2, zc + 3 + 4.47]), kind: 'steel', label: 'USB-C' };
    comps.jst = { geom: box([24.6, pcbTop, zc - 10.5], [29.9, pcbTop + 4.6, zc - 4.5]), kind: 'cream', label: 'battery socket' };
    const plugX1 = 29.9 + 4.5, wireX1 = Math.min(plugX1 + 2.0, IN.x1 - 0.2);
    comps.batplug = { geom: box([29.9, pcbTop + 0.3, zc - 10.2], [plugX1, pcbTop + 4.3, zc - 5.0]), kind: 'cream', label: 'battery plug' };
    comps.batwires = { geom: box([plugX1, w + 1.0, zc - 9.6], [wireX1, pcbTop + 3.6, zc - 5.6]), kind: 'cable', label: 'battery wires' };
    comps.chip = { geom: box([-14, pcbTop, zc - 5], [-7, pcbTop + 0.9, zc + 2]), kind: 'chip', label: 'ESP32 chip' };
    comps.ch340 = { geom: box([10, pcbTop, zc + 1.5], [19, pcbTop + 1.6, zc + 6.5]), kind: 'chip', label: 'CH340C' };
    comps.connector = { geom: box([-5, 3.0, 16.6], [3, 16.0, 20.0]), kind: 'cream', label: '4-pin JST 1.25 pair' };
    comps.splices = { geom: box([-20, 4.6, 15.6], [-8.5, 11.4, 20.3]), kind: 'shrink', label: '4 splices' };
    const gl = { y: 6.4, z: 16.4, r: P.cableD / 2 };
    comps.coil = { geom: cylX(gl.r, OUT.x0 - BOOT.L - 1.5, -24.5, gl.y, gl.z), kind: 'cable', label: 'cable tail' };
    const sw = { x: 19.4, z: 15.4 }, plTop = lidIn;
    comps.switch = { geom: union(box([sw.x - 6, plTop - 4.3, sw.z - 6], [sw.x + 6, plTop - 0.8, sw.z + 6]), cylY(3.3, plTop - 0.8, plTop, sw.x, sw.z)), kind: 'chip', label: '12×12×4.3 switch' };
    const outer = slab(OUT.x0, OUT.x1, OUT.y0, OUT.y1, OUT.z0, OUT.z1, R.p, R.b, R.t);
    const cavity = slab(IN.x0, IN.x1, IN.y0, IN.y1, IN.z0, IN.z1, Ri.p, Ri.b, Ri.t);
    const shell = subtract(outer, cavity);
    let base = intersect(shell, box([-60, -5, -40], [60, ys, 40]));
    let lid = intersect(shell, box([-60, ys, -40], [60, 40, 40]));
    const lipH = 3.5, lipT = 1.0, ledge = 1.4;
    const lipIn = (y0, y1) => slab(IN.x0 + c + lipT, IN.x1 - c - lipT, y0, y1, IN.z0 + c + lipT, IN.z1 - c - lipT, Math.max(0.3, Ri.p - c - lipT));
    let lipBand = subtract(slab(IN.x0 + c, IN.x1 - c, ys - lipH, ys + 0.01, IN.z0 + c, IN.z1 - c, Ri.p - c), lipIn(ys - lipH - 1, ys + 1));
    const ledgeRing = subtract(slab(IN.x0 - 0.01, IN.x1 + 0.01, ys, ys + ledge, IN.z0 - 0.01, IN.z1 + 0.01, Ri.p), lipIn(ys - 1, ys + ledge + 1));
    const tabL = 5.0, zl = IN.z1 - c, bumpH = 0.5, tabs = [];
    for (const x of [-16, 16]) for (const s of [1, -1]) {
      const zz0 = s > 0 ? zl - lipT : -zl, zz1 = s > 0 ? zl : -zl + lipT;
      lipBand = subtract(lipBand, box([x - 4.8, ys - lipH - 1, zz0 - 0.01], [x - 4.0, ys + 0.001, zz1 + 0.01]), box([x + 4.0, ys - lipH - 1, zz0 - 0.01], [x + 4.8, ys + 0.001, zz1 + 0.01]));
      tabs.push(box([x - 4.0, ys - tabL, zz0], [x + 4.0, ys + 0.01, zz1]));
    }
    const bumpP = x => prism([[ys - 5.0, zl], [ys - 4.0, zl + bumpH], [ys - 3.8, zl + bumpH], [ys - 3.3, zl]], 'x', x - 3.5, x + 3.5);
    const bumps = []; for (const x of [-16, 16]) bumps.push(bumpP(x), mirrorZ(bumpP(x)));
    const posts = [[-19.4, bz0 + 1.6], [-19.4, bz1 - 1.8], [27.4, bz1 - 1.8]].map(([x, z]) => box([x - 1.2, pcbTop + 0.05, z - 1.2], [x + 1.2, lidIn + 0.01, z + 1.2]));
    const pk = { i: 6.15, o: 7.6, top: lidIn + 0.01, bot: plTop - 3.8, f: 0.8, k: 0.35 }, recessBot = lidIn - 0.4;
    let pocket = subtract(box([sw.x - pk.o, pk.bot, sw.z - pk.o], [sw.x + pk.o, pk.top, sw.z + pk.o]), box([sw.x - pk.i, pk.bot - 1, sw.z - pk.i], [sw.x + pk.i, pk.top + 1, sw.z + pk.i]));
    const fBot = plTop - 4.3 - 1.0, fTop = recessBot - 0.5;
    for (const s of [1, -1]) {
      const xi = sw.x + s * pk.i, xo = sw.x + s * pk.o, x0 = Math.min(xi, xo), x1 = Math.max(xi, xo);
      const xf = xi + s * pk.f, xf0 = Math.min(xi, xf), xf1 = Math.max(xi, xf), xc0 = Math.min(xf, xo), xc1 = Math.max(xf, xo);
      pocket = subtract(pocket, box([x0 - 0.01, pk.bot - 0.01, sw.z - 2.5], [x1 + 0.01, fTop, sw.z - 2.0]), box([x0 - 0.01, pk.bot - 0.01, sw.z + 2.0], [x1 + 0.01, fTop, sw.z + 2.5]),
        box([s > 0 ? xc0 : xc0 - 0.01, pk.bot - 0.01, sw.z - 2.01], [s > 0 ? xc1 + 0.01 : xc1, fTop, sw.z + 2.01]));
      const catchP = prism([[xi, fBot], [xi - s * pk.k, fBot + 0.5], [xi - s * pk.k, plTop - 4.3], [xi, plTop - 4.3]], 'z', sw.z - 2.0, sw.z + 2.0);
      pocket = union(pocket, box([xf0, fBot, sw.z - 2.0], [xf1, pk.bot + 0.01, sw.z + 2.0]), catchP);
    }
    lid = union(lid, lipBand, ...tabs, ledgeRing, ...bumps, ...posts, intersect(pocket, box([IN.x0, 0, IN.z0], [IN.x1, 40, IN.z1])));
    if (wireX1 > IN.x1 - c - lipT - 0.3) lid = subtract(lid, box([IN.x1 - c - lipT - 0.3, ys - lipH - 1, zc - 10.0], [IN.x1 + 0.01, ys + ledge + 0.01, zc - 5.2]));
    lid = subtract(lid, cylY(6.3, lidIn - 1, OUT.y1 + 1, sw.x, sw.z), coneY(6.3, 6.61, OUT.y1 - 0.3, OUT.y1 + 0.01, sw.x, sw.z), cylY(8.5, recessBot, lidIn + 1.0, sw.x, sw.z));
    const uzc = zc + 3, uyc = pcbTop + 1.6, TW = 13.2, TH = 7.0;
    const tunnel = { z0: uzc - TW / 2, z1: uzc + TW / 2, y0: uyc - TH / 2, y1: uyc + TH / 2 };
    const tunnelCut = union(box([30.7, tunnel.y0, tunnel.z0], [OUT.x1 + 1, tunnel.y1, tunnel.z1]),
      hull(box([OUT.x1 - 0.6, tunnel.y0, tunnel.z0], [OUT.x1 - 0.59, tunnel.y1, tunnel.z1]), box([OUT.x1 + 0.01, tunnel.y0 - 0.61, tunnel.z0 - 0.61], [OUT.x1 + 0.02, tunnel.y1 + 0.61, tunnel.z1 + 0.61])));
    const tWall = (za, zb) => box([31.0, ys + 0.3, za], [IN.x1 + 0.01, lidIn + 0.01, zb]);
    lid = union(lid, tWall(tunnel.z0 - 1.2, tunnel.z0), tWall(tunnel.z1, tunnel.z1 + 1.2));
    lid = subtract(lid, tunnelCut); base = subtract(base, tunnelCut);
    const pocketP = x => prism([[ys - 5.15, IN.z1 - 0.01], [ys - 5.15, IN.z1 + 0.5], [ys - 3.65, IN.z1 + 0.5], [ys - 3.14, IN.z1 - 0.01]], 'x', x - 3.8, x + 3.8);
    for (const x of [-16, 16]) base = subtract(base, pocketP(x), mirrorZ(pocketP(x)));
    for (const x of [-18, 18]) base = subtract(base, box([x - 4.7, 4.4, OUT.z1 - 0.6], [x + 4.7, 6.0, OUT.z1 + 1]), box([x - 4.7, 4.4, OUT.z0 - 1], [x + 4.7, 6.0, OUT.z0 + 0.6]));
    base = union(base,
      box([-24.6, w - 0.01, -19.5], [-23.5, w + 5, 12]), box([27.1, w - 0.01, -19], [28.1, w + 5, 12]),
      box([-20, w - 0.01, 14.15], [24, w + 5, 14.9]),
      box([-15, w - 0.01, IN.z0 - 0.01], [-5, w + 5, -20.5]), box([5, w - 0.01, IN.z0 - 0.01], [15, w + 5, -20.5]));
    base = union(base, subtract(box([IN.x0 + BOOT.fl + 0.3, w - 0.01, gl.z - 4.2], [IN.x0 + 5.5, gl.y, gl.z + 4.2]), cylX(BOOT.rb, IN.x0, IN.x0 + 7, gl.y, gl.z)));
    base = subtract(base, cylX(bootHoleR, OUT.x0 - 1, IN.x0 + 0.01, gl.y, gl.z), coneX(bootHoleR + BOOT.cs + 0.01, bootHoleR, OUT.x0 - 0.01, OUT.x0 + BOOT.cs, gl.y, gl.z));
    const an = { y: 4.6, z: uzc };
    base = union(base, cylX(2.4, IN.x1 - 2.6, IN.x1 + 0.01, an.y, an.z), box([IN.x1 - 2.6, w - 0.01, an.z - 2.4], [IN.x1 + 0.01, an.y, an.z + 2.4]));
    base = subtract(base, cylX(1.85, IN.x1 - 1.8, IN.x1 + 0.01, an.y, an.z), cylX(1.1, IN.x1 - 0.1, OUT.x1 + 0.1, an.y, an.z));
    const weeps = [-26, 9, 30].filter(x => x < IN.x1 - 3);
    for (const x of weeps) base = subtract(base, cylZ(0.75, IN.z1 - 0.1, OUT.z1 + 0.1, x, 4.0));
    const rv = 0.4;
    const reveal = subtract(box([OUT.x0 - 1, ys - rv, OUT.z0 - 1], [OUT.x1 + 1, ys + rv, OUT.z1 + 1]), slab(OUT.x0 + rv, OUT.x1 - rv, ys - rv - 0.1, ys + rv + 0.1, OUT.z0 + rv, OUT.z1 - rv, R.p - rv));
    base = subtract(base, reveal, box([OUT.x0 - 0.1, ys - 1.4, -5], [OUT.x0 + 0.8, ys + 0.01, 3]), box([OUT.x1 - 0.8, ys - 1.4, 9], [OUT.x1 + 0.1, ys + 0.01, 17]));
    lid = subtract(lid, reveal, debossY(text2D('SKI HUD B', 3.6, 0.6), -2, -12, OUT.y1, 0.4), debossY(chevron2D(6, 180), -26, 0, OUT.y1, 0.4));
    const pz = OUT.z1 + 1.5, px = 24;
    let plate = rbox([-px, -1.4 - P.strapT, -pz], [px, 0, pz], 1.8);
    plate = subtract(plate, box([-px - 1, -P.strapT - 0.1, -P.strapW / 2 - 0.3], [px + 1, 0.1, P.strapW / 2 + 0.3]));
    for (let x = -px + 4; x <= px - 4; x += 6) plate = union(plate, box([x - 0.6, -P.strapT - 0.11, -P.strapW / 2 - 0.3], [x + 0.6, -P.strapT + 0.5, P.strapW / 2 + 0.3]));
    const hookArm = (x) => union(rbox([x - 4.5, -1.6, OUT.z1 + 0.1], [x + 4.5, 6.2, pz], 0.6),
      prism([[6.0, OUT.z1 + 0.1], [5.0, OUT.z1 - 0.35], [4.6, OUT.z1 - 0.35], [4.6, OUT.z1 + 0.1]], 'x', x - 4.5, x + 4.5));
    for (const x of [-18, 18]) plate = union(plate, hookArm(x), mirrorZ(hookArm(x)));
    const portPlug = union(
      box([OUT.x1 - 4.0, tunnel.y0 + 0.2, tunnel.z0 + 0.2], [OUT.x1, tunnel.y1 - 0.2, tunnel.z1 - 0.2]),
      box([OUT.x1 - 3.4, tunnel.y0, tunnel.z0], [OUT.x1 - 2.8, tunnel.y1, tunnel.z1]),
      box([OUT.x1 - 1.8, tunnel.y0, tunnel.z0], [OUT.x1 - 1.2, tunnel.y1, tunnel.z1]),
      rbox([OUT.x1, tunnel.y0 - 0.8, tunnel.z0 - 0.8], [OUT.x1 + 1.2, tunnel.y1 + 0.8, tunnel.z1 + 0.8], 0.5),
      rbox([OUT.x1 + 0.4, an.y - 1.6, an.z - 1.6], [OUT.x1 + 1.2, tunnel.y0 - 0.3, an.z + 1.6], 0.35),
      cylX(1.0, IN.x1 - 0.05, OUT.x1 + 0.45, an.y, an.z),
      coneX(0.8, 1.45, IN.x1 - 1.45, IN.x1 - 0.05, an.y, an.z));
    const cap = subtract(union(
      translate([sw.x, OUT.y1 + 0.2, sw.z], rotateX(-90 * D, roundedCylinder({ radius: 6.0, height: 3.0, roundRadius: 1.2, segments: 40 }))),
      cylY(6.0, lidIn + 1.0, OUT.y1, sw.x, sw.z), cylY(8.2, lidIn, lidIn + 1.0, sw.x, sw.z)),
      translate([sw.x, OUT.y1 + 1.7 + 6 - 0.5, sw.z], sphere({ radius: 6, segments: 32 })));
    const boot = translate([OUT.x0, gl.y, gl.z], rotateY(-90 * D, bootGeom(BOOT, w)));
    const rx = g => rotateX(90 * D, g), rxn = g => rotateX(-90 * D, g);
    return {
      parts: {
        pack_base: { geom: base, material: 'PETG', label: 'Pack base', print: 'head side down, open side up, no supports', orient: rx, layer: 0.2, infill: '20 %' },
        pack_lid: { geom: lid, material: 'PETG', label: 'Pack lid', print: 'top face down (text on the bed), no supports', orient: rxn, layer: 0.2, infill: '20 %' },
        pack_plate: { geom: plate, material: 'PETG', label: 'Strap clamp plate', print: 'head face down, hooks up, no supports', orient: rx, layer: 0.2, infill: '30 %' },
        button_cap: { geom: cap, material: 'TPU', label: 'Button cap (TPU 95A)', print: 'flange down, dome up', orient: rx, layer: 0.2, infill: '100 %' },
        port_plug: { geom: portPlug, material: 'TPU', label: 'USB-C plug + tether (TPU 95A)', print: 'outer face down, flat', orient: g => rotateY(90 * D, g), layer: 0.2, infill: '100 %' },
        pack_boot: { geom: boot, material: 'TPU', label: 'Cable boot, pack (TPU 95A)', print: 'flange end down, 3 mm brim', orient: g => rotateY(90 * D, g), layer: 0.2, infill: '100 %' },
      },
      comps, presses: [],
      dims: { L: OUT.x1 - OUT.x0, W: OUT.z1 - OUT.z0, T: OUT.y1 - OUT.y0 + 1.4 + P.strapT },
      glandOut: [OUT.x0 - BOOT.L, gl.y, gl.z],
      explode: { pack_lid: [0, 14, 0], button_cap: [0, 24, 0], pack_plate: [0, -12, 0], port_plug: [14, 0, 0], pack_boot: [-16, 0, 0] },
      checks: { usbRecess: +(OUT.x1 - 30.6).toFixed(2), tunnel: [TW, TH], plugClear: +(IN.x1 - plugX1).toFixed(1) },
    };
  }

  // ---------- checks ----------
  function bbOverlap(a, b) { return a[0][0] < b[1][0] && a[1][0] > b[0][0] && a[0][1] < b[1][1] && a[1][1] > b[0][1] && a[0][2] < b[1][2] && a[1][2] > b[0][2]; }
  // every printed part and component against every other one; known press fits are reported separately
  function collisions(items, presses) {
    const out = [], pr = [];
    const isPress = (a, b) => presses.find(p => (p[0] === a && p[1] === b) || (p[0] === b && p[1] === a));
    const bbs = items.map(i => measureBoundingBox(i.geom));
    for (let i = 0; i < items.length; i++) for (let j = i + 1; j < items.length; j++) {
      if (!bbOverlap(bbs[i], bbs[j])) continue;
      const v = measureVolume(intersect(items[i].geom, items[j].geom));
      if (v <= 0.05) continue;
      const p = isPress(items[i].name, items[j].name), rec = { a: items[i].name, b: items[j].name, v: +v.toFixed(2) };
      if (p) pr.push(Object.assign(rec, { why: p[2] })); else out.push(rec);
    }
    return { out, pr };
  }
  function printGeom(part) {
    const g = part.orient ? part.orient(part.geom) : part.geom, bb = measureBoundingBox(g);
    return translate([-(bb[0][0] + bb[1][0]) / 2, -(bb[0][1] + bb[1][1]) / 2, -bb[0][2]], g);
  }
  // rough print time on a typical bed-slinger, 0.2 mm, small parts are mostly walls
  const printMin = (vol, mat) => Math.round(4 + vol / (mat === 'TPU' ? 45 : 120));

  function build(params) {
    const P = Object.assign({}, DEFAULTS, params || {});
    P.imuL = P.imuL || 25.4; P.imuW = P.imuW || 17.8;
    const hud = buildHud(P), bench = buildBench(P), pack = buildPack(P);
    const groups = { hud, bench, pack }, parts = {};
    for (const g in groups) for (const k in groups[g].parts) {
      const p = groups[g].parts[k]; p.group = g; parts[k] = p;
      p.volume = measureVolume(p.geom);
      const bb = measureBoundingBox(p.geom); p.size = [0, 1, 2].map(i => +(bb[1][i] - bb[0][i]).toFixed(1));
      const pb = measureBoundingBox(printGeom(p)); p.printSize = [0, 1, 2].map(i => +(pb[1][i] - pb[0][i]).toFixed(1));
      p.minutes = printMin(p.volume, p.material);
    }
    const list = (o, pre) => Object.keys(o).filter(k => !o[k].nocheck).map(k => ({ name: pre + k, geom: o[k].geom }));
    const res = {};
    res.hud = collisions(list(hud.parts, '').concat(list(hud.comps, 'c:'), [{ name: 'ctx:rail', geom: hud.rail }]), hud.presses);
    res.bench = collisions(list(bench.parts, '').concat(list(bench.comps, 'c:')), bench.presses);
    res.pack = collisions(list(pack.parts, '').concat(list(pack.comps, 'c:')), pack.presses);
    const coll = [].concat(res.hud.out, res.bench.out, res.pack.out), presses = [].concat(res.hud.pr, res.bench.pr, res.pack.pr);
    return { P, parts, hud, bench, pack, collisions: coll, presses };
  }

  const api = { build, DEFAULTS, MODULE, TOL, printGeom, toPolygons: g => J.geometries.geom3.toPolygons(g) };
  (typeof window !== 'undefined' ? window : globalThis).OledCAD = api;
})();
