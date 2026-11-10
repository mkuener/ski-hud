/* Ski HUD housings, Variant A: 0.96in 160x80 ST7735S colour LCD. Parametric CAD model (JSCAD v2). Units: mm.
 * (Variant B, the SSD1306 OLED version, lives in ../housing-oled/ and is a separate model.)
 *
 * One source of truth for the 3D viewer and the STL export.
 * SkiCAD.build(params) -> { parts:{name:{geom, frame, material, orient, ...}}, comps:{...}, context:{...} }
 * SkiCAD.printGeom(part) -> the part rotated into its print orientation, sitting on Z=0 (STL is Z-up).
 *
 * PACK frame (strap pack, lying on its head side):
 *   X = length along the strap, front (toward goggle, coil exit) = -X, rear (USB-C) = +X
 *   Y = thickness, head side Y=0, outward +Y
 *   Z = height on the head; +Z points DOWN when worn (weep holes are on the +Z wall)
 * HUD frame (goggle top centre):
 *   X = across the face, Y = up, Z = forward toward the lens
 *   Goggle frame top rail occupies Y [0, railT], Z [-railD/2, railD/2]. It is a ladder of vent windows
 *   (winL x winH) between ribs (ribW); a window is centred at X=0 and the HUD clamps the two ribs at its ends. The magnetic lens
 *   sits on the rail's front face (Z = railD/2); the face foam is cavityD behind the lens.
 *
 * FDM rules used throughout (0.4 mm nozzle, 0.2 mm layers):
 *   walls 1.2-1.8 mm (3-4 perimeters), no feature thinner than 0.6 mm, text strokes 0.5-0.6 mm,
 *   sliding fits 0.15 mm per side, loose fits 0.3 mm, TPU-to-PETG 0-0.05 mm (TPU squeezes),
 *   overhangs <= 45 deg or bridges, every bed face starts on a round or chamfer (no elephant-foot fit problems).
 */
(function () {
  const J = window.jscadModeling;
  const { cuboid, roundedCuboid, cylinder, cylinderElliptic, roundedCylinder, polygon, sphere } = J.primitives;
  const { union, subtract, intersect } = J.booleans;
  const { translate, rotateX, rotateY, mirrorZ, mirrorX } = J.transforms;
  const { extrudeLinear, extrudeRotate } = J.extrusions;
  const { hull } = J.hulls;
  const { expand } = J.expansions;
  const { vectorText } = J.text;
  const { path2 } = J.geometries;
  const { measureVolume, measureBoundingBox } = J.measurements;
  const D = Math.PI / 180;

  const DEFAULTS = {
    wall: 1.6,        // pack wall (4 perimeters at 0.4 mm)
    lidTop: 1.8,      // pack lid top plate
    strapW: 40,       // goggle strap width
    strapT: 2.5,      // goggle strap thickness
    railT: 6,         // goggle frame top rail thickness at the centre rib
    railD: 18,        // rail depth front-to-back (context only; the lens sits on its front face)
    ribW: 4,          // MEASURED: rib width (X) between vent windows (the ribs at the centre window's ends are clamped)
    winL: 26.13,      // MEASURED: centre vent window length (X)
    winH: 10.19,      // MEASURED: centre vent window height = front-to-back opening (Z)
    cavityD: 20,      // lens inner face to face foam, at the top centre
    eyeH: 35,         // eye (pupil) height below the underside of the rail
    tilt: 20,         // screen tilt down from vertical, degrees
    podGap: 2,        // gap between rail underside and pod top
    plugRoom: 7,      // room past the board's battery socket for the plug and its bent wires (PH ~6, XH ~8)
    hood: 1.5,        // screen hood (brow + cheeks) depth in front of the pod
  };
  const TOL = { slide: 0.15, loose: 0.3, tpu: 0.05 };
  const RPLAN = 110;   // assumed plan radius of the frame front / lens at the top (not measured yet)
  // cantilever snap strain, %: 1.5 * thickness * deflection / length^2 (PETG: keep <= ~3 %, yields ~4-5 %)
  const strain = (t, d, L) => +(150 * t * d / (L * L)).toFixed(1);

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

  // Debossed single-stroke text / marks on a +Y face. 2D +y maps to -Z (up when worn / away from the lens).
  function text2D(str, cap, stroke) {
    const h = cap * 5 / 7.5;   // vectorText height 5 -> ~7.5 mm cap height
    const gs = vectorText({ height: h, input: str }).map(s => expand({ delta: stroke / 2, corners: 'round', segments: 8 }, path2.fromPoints({}, s)));
    const g = union(gs), bb = measureBoundingBox(g);
    return translate([-(bb[0][0] + bb[1][0]) / 2, -(bb[0][1] + bb[1][1]) / 2, 0], g);
  }
  function chevron2D(size, ang) {   // V pointing along 2D angle ang (deg)
    const c = Math.cos(ang * D), s = Math.sin(ang * D), k = size / 4;
    const pts = [[1.6, 0], [-1.0, 2.4], [-1.0, 1.3], [0.2, 0], [-1.0, -1.3], [-1.0, -2.4]].map(([x, y]) => [k * (x * c - y * s), k * (x * s + y * c)]);
    return polygon({ points: ccw(pts) });
  }
  function debossY(g2, cx, cz, yTop, depth) {
    return translate([cx, yTop - depth, cz], rotateX(-90 * D, extrudeLinear({ height: depth + 0.3 }, g2)));
  }

  // TPU coil boot: inner flange behind the wall, neck through it, 45 deg shoulder into a countersink,
  // then a ribbed taper that spreads the bend over 12 mm. Profile (r, a): a = distance out from the wall face.
  const BOOT = { rb: 2.35, rn: 3.15, rf: 3.6, fl: 1.2, cs: 0.6, r0: 3.9, r1: 3.3, L: 12, rib: 0.35 };
  function bootGeom(w) {
    const B = BOOT, pts = [[B.rb, -(w + B.fl)], [B.rf - 0.4, -(w + B.fl)], [B.rf, -(w + B.fl) + 0.4], [B.rf, -w], [B.rn, -w], [B.rn, -B.cs], [B.rn + B.cs, 0]];
    const aS = B.r0 - B.rn - B.cs; pts.push([B.r0, aS]);
    const n = 5, step = (B.L - aS) / n, base = a => B.r0 + (B.r1 - B.r0) * (a - aS) / (B.L - aS);
    for (let i = 0; i < n; i++) { const a0 = aS + i * step, a1 = a0 + step; pts.push([base(a0) - B.rib, a0], [base(a1), a1]); }
    pts.push([B.rb, B.L]);
    return extrudeRotate({ segments: 32 }, polygon({ points: ccw(pts) }));   // axis = +Z
  }
  // wall cut that takes the boot: hole + 45 deg countersink on the outside (TPU fit +0.05)
  const bootHoleR = BOOT.rn + TOL.tpu;

  // ============================================================
  function buildPack(P) {
    const w = P.wall, c = TOL.slide;
    const IN = { x0: -30, x1: 30 + P.plugRoom, z0: -22, z1: 22, y0: w, y1: w + 18.8 };  // battery 10.4 + gap 1.8 + PCB 1.6 + 5.0 above
    const OUT = { x0: IN.x0 - w, x1: IN.x1 + w, z0: IN.z0 - w, z1: IN.z1 + w, y0: 0, y1: IN.y1 + P.lidTop };
    const ys = w + 10.4 + 1.8;              // split plane = PCB underside (13.8 with defaults)
    const pcbTop = ys + 1.6;
    const lidIn = IN.y1;                     // lid inner face
    // outer form: plan corners R3.5, head-side edges R2, outer top edges R3; cavity concentric -> uniform 1.6 wall
    const R = { p: 3.5, b: 2.0, t: 3.0 }, Ri = { p: R.p - w, b: R.b - w, t: 1.2 };

    // ---- components (PACK frame) ----
    const comps = {};
    comps.battery = { geom: box([-23.2, w, -20.2], [26.8, w + 10.4, 13.8]), kind: 'battery', label: '103450 LiPo' };
    comps.foam = { geom: box([-20.5, w + 10.4, -19.8], [26.5, ys, 4.8]), kind: 'foam', label: 'foam tape + Kapton' };
    const bx0 = -21.2, bx1 = 29.8, bz0 = -20.2, bz1 = 5.2;   // ESP32 Lite outline
    const zc = (bz0 + bz1) / 2;                               // board centre line
    comps.pcb = { geom: box([bx0, ys, bz0], [bx1, pcbTop, bz1]), kind: 'pcb', label: 'ESP32 Lite' };
    comps.usb = { geom: box([23.3, pcbTop, zc + 3 - 4.47], [30.6, pcbTop + 3.2, zc + 3 + 4.47]), kind: 'steel', label: 'USB-C' };
    comps.jst = { geom: box([24.6, pcbTop, zc - 10.5], [29.9, pcbTop + 4.6, zc - 4.5]), kind: 'cream', label: 'battery socket' };
    // Battery plug in the socket (PH 2.0 assumed, ~4.5 mm proud of the socket), wires bent down past the board edge.
    const plugX1 = 29.9 + 4.5, wireX1 = Math.min(plugX1 + 2.0, IN.x1 - 0.2);
    comps.batplug = { geom: box([29.9, pcbTop + 0.3, zc - 10.2], [plugX1, pcbTop + 4.3, zc - 5.0]), kind: 'cream', label: 'battery plug' };
    comps.batwires = { geom: box([plugX1, w + 1.0, zc - 9.6], [wireX1, pcbTop + 3.6, zc - 5.6]), kind: 'cable', label: 'battery wires' };
    comps.chip = { geom: box([-14, pcbTop, zc - 5], [-7, pcbTop + 0.9, zc + 2]), kind: 'chip', label: 'ESP32 chip' };
    comps.ch340 = { geom: box([10, pcbTop, zc + 1.5], [19, pcbTop + 1.6, zc + 6.5]), kind: 'chip', label: 'CH340C' };
    comps.connector = { geom: box([-8, 3.0, 16.6], [5, 16.0, 20.0]), kind: 'cream', label: '8-pin JST 1.25 pair' };
    comps.splices = { geom: box([-24, 4.6, 15.6], [-8.5, 11.4, 20.3]), kind: 'shrink', label: '8 splices' };
    const gl = { y: 6.4, z: 16.4, r: 2.3 };
    comps.coil = { geom: cylX(gl.r, OUT.x0 - BOOT.L - 1.5, -24.5, gl.y, gl.z), kind: 'cable', label: 'coil tail' };
    // low-profile 12x12x4.3 tact switch, hanging from the lid in a snap pocket, plunger up against the cap flange
    const sw = { x: 19.4, z: 15.4 };
    const plTop = lidIn;
    comps.switch = { geom: union(box([sw.x - 6, plTop - 4.3, sw.z - 6], [sw.x + 6, plTop - 0.8, sw.z + 6]), cylY(3.3, plTop - 0.8, plTop, sw.x, sw.z)), kind: 'chip', label: '12×12×4.3 switch' };

    // ---- shell ----
    const outer = slab(OUT.x0, OUT.x1, OUT.y0, OUT.y1, OUT.z0, OUT.z1, R.p, R.b, R.t);
    const cavity = slab(IN.x0, IN.x1, IN.y0, IN.y1, IN.z0, IN.z1, Ri.p, Ri.b, Ri.t);
    const shell = subtract(outer, cavity);
    let base = intersect(shell, box([-60, -5, -40], [60, ys, 40]));
    let lid = intersect(shell, box([-60, ys, -40], [60, 40, 40]));

    // ---- lid: labyrinth lip all round (3.5 mm into the base, 0.15 mm sliding gap) + ledge to the lid walls ----
    const lipH = 3.5, lipT = 1.0, ledge = 1.4;
    const lipIn = (y0, y1) => slab(IN.x0 + c + lipT, IN.x1 - c - lipT, y0, y1, IN.z0 + c + lipT, IN.z1 - c - lipT, Math.max(0.3, Ri.p - c - lipT));
    let lipBand = subtract(slab(IN.x0 + c, IN.x1 - c, ys - lipH, ys + 0.01, IN.z0 + c, IN.z1 - c, Ri.p - c), lipIn(ys - lipH - 1, ys + 1));
    const ledgeRing = subtract(slab(IN.x0 - 0.01, IN.x1 + 0.01, ys, ys + ledge, IN.z0 - 0.01, IN.z1 + 0.01, Ri.p), lipIn(ys - 1, ys + ledge + 1));
    // snap tabs: 8 wide, 5 long, 1.0 thick, cut free by 0.8 mm relief slots; 0.5 mm bump (0.35 engages the wall)
    // 27 deg lead-in, 45 deg retention face: firm, but it can be pried off without breaking anything.
    const tabL = 5.0, zl = IN.z1 - c, bumpH = 0.5;
    const tabs = [];
    for (const x of [-16, 16]) for (const s of [1, -1]) {
      const zz0 = s > 0 ? zl - lipT : -zl, zz1 = s > 0 ? zl : -zl + lipT;
      lipBand = subtract(lipBand, box([x - 4.8, ys - lipH - 1, zz0 - 0.01], [x - 4.0, ys + 0.001, zz1 + 0.01]), box([x + 4.0, ys - lipH - 1, zz0 - 0.01], [x + 4.8, ys + 0.001, zz1 + 0.01]));
      tabs.push(box([x - 4.0, ys - tabL, zz0], [x + 4.0, ys + 0.01, zz1]));
    }
    const bumpP = x => prism([[ys - 5.0, zl], [ys - 4.0, zl + bumpH], [ys - 3.8, zl + bumpH], [ys - 3.3, zl]], 'x', x - 3.5, x + 3.5);
    const bumps = [];
    for (const x of [-16, 16]) bumps.push(bumpP(x), mirrorZ(bumpP(x)));
    // board hold-down posts (avoid the header holes, JST and USB), 0.05 mm above the board
    const posts = [[-19.4, bz0 + 1.6], [-19.4, bz1 - 1.8], [27.4, bz1 - 1.8]].map(([x, z]) => box([x - 1.2, pcbTop + 0.05, z - 1.2], [x + 1.2, lidIn + 0.01, z + 1.2]));
    // switch pocket: a frame round the switch body, two snap fingers catch under it so a press can't push it out.
    // Frame (1.45 walls) hangs from the lid by its corners, outside the cap recess; fingers 0.8 thick, 4.1 free, 0.35 catch.
    const pk = { i: 6.15, o: 7.6, top: lidIn + 0.01, bot: plTop - 3.8, f: 0.8, k: 0.35 }, recessBot = lidIn - 0.4;
    let pocket = subtract(box([sw.x - pk.o, pk.bot, sw.z - pk.o], [sw.x + pk.o, pk.top, sw.z + pk.o]), box([sw.x - pk.i, pk.bot - 1, sw.z - pk.i], [sw.x + pk.i, pk.top + 1, sw.z + pk.i]));
    const fBot = plTop - 4.3 - 1.0, fTop = recessBot - 0.5;
    for (const s of [1, -1]) {
      const xi = sw.x + s * pk.i, xo = sw.x + s * pk.o, x0 = Math.min(xi, xo), x1 = Math.max(xi, xo);
      const xf = xi + s * pk.f, xf0 = Math.min(xi, xf), xf1 = Math.max(xi, xf), xc0 = Math.min(xf, xo), xc1 = Math.max(xf, xo);
      // slits either side + thin the wall behind the finger
      pocket = subtract(pocket, box([x0 - 0.01, pk.bot - 0.01, sw.z - 2.5], [x1 + 0.01, fTop, sw.z - 2.0]), box([x0 - 0.01, pk.bot - 0.01, sw.z + 2.0], [x1 + 0.01, fTop, sw.z + 2.5]),
        box([s > 0 ? xc0 : xc0 - 0.01, pk.bot - 0.01, sw.z - 2.01], [s > 0 ? xc1 + 0.01 : xc1, fTop, sw.z + 2.01]));
      const catchP = prism([[xi, fBot], [xi - s * pk.k, fBot + 0.5], [xi - s * pk.k, plTop - 4.3], [xi, plTop - 4.3]], 'z', sw.z - 2.0, sw.z + 2.0);
      pocket = union(pocket, box([xf0, fBot, sw.z - 2.0], [xf1, pk.bot + 0.01, sw.z + 2.0]), catchP);
    }
    lid = union(lid, lipBand, ...tabs, ledgeRing, ...bumps, ...posts, intersect(pocket, box([IN.x0, 0, IN.z0], [IN.x1, 40, IN.z1])));
    // battery wires pass the rear lip: notch it there (only if the plug room is short enough for them to meet)
    if (wireX1 > IN.x1 - c - lipT - 0.3) lid = subtract(lid, box([IN.x1 - c - lipT - 0.3, ys - lipH - 1, zc - 10.0], [IN.x1 + 0.01, ys + ledge + 0.01, zc - 5.2]));
    // button: hole + 0.3 chamfer outside, 1.4 mm recess for the cap flange (0.4 below the lid face = press travel)
    lid = subtract(lid, cylY(6.3, lidIn - 1, OUT.y1 + 1, sw.x, sw.z), coneY(6.3, 6.61, OUT.y1 - 0.3, OUT.y1 + 0.01, sw.x, sw.z), cylY(8.5, recessBot, lidIn + 1.0, sw.x, sw.z));
    // USB-C tunnel through the rear wall (the battery plug needs room past the board edge), side walls hang from the lid
    const uzc = zc + 3, uyc = pcbTop + 1.6, TW = 13.2, TH = 7.0;
    const tunnel = { z0: uzc - TW / 2, z1: uzc + TW / 2, y0: uyc - TH / 2, y1: uyc + TH / 2 };
    const tunnelCut = union(box([30.7, tunnel.y0, tunnel.z0], [OUT.x1 + 1, tunnel.y1, tunnel.z1]),
      hull(box([OUT.x1 - 0.6, tunnel.y0, tunnel.z0], [OUT.x1 - 0.59, tunnel.y1, tunnel.z1]), box([OUT.x1 + 0.01, tunnel.y0 - 0.61, tunnel.z0 - 0.61], [OUT.x1 + 0.02, tunnel.y1 + 0.61, tunnel.z1 + 0.61])));
    const tWall = (za, zb) => box([31.0, ys + 0.3, za], [IN.x1 + 0.01, lidIn + 0.01, zb]);
    lid = union(lid, tWall(tunnel.z0 - 1.2, tunnel.z0), tWall(tunnel.z1, tunnel.z1 + 1.2));
    lid = subtract(lid, tunnelCut);
    base = subtract(base, tunnelCut);

    // ---- base features ----
    // blind snap pockets (0.5 deep, 1.1 mm skin left, 45 deg roof) - no holes through the wall
    const pocketP = x => prism([[ys - 5.15, IN.z1 - 0.01], [ys - 5.15, IN.z1 + 0.5], [ys - 3.65, IN.z1 + 0.5], [ys - 3.14, IN.z1 - 0.01]], 'x', x - 3.8, x + 3.8);
    for (const x of [-16, 16]) base = subtract(base, pocketP(x), mirrorZ(pocketP(x)));
    // grooves for the strap-plate hooks (outer long faces, near the bottom)
    for (const x of [-18, 18]) base = subtract(base, box([x - 4.7, 4.4, OUT.z1 - 0.6], [x + 4.7, 6.0, OUT.z1 + 1]), box([x - 4.7, 4.4, OUT.z0 - 1], [x + 4.7, 6.0, OUT.z0 + 0.6]));
    // battery cradle: ribs on all four sides, 0.3 mm off the pouch (swelling room), 5 mm tall
    base = union(base,
      box([-24.6, w - 0.01, -19.5], [-23.5, w + 5, 12]), box([27.1, w - 0.01, -19], [28.1, w + 5, 12]),
      box([-20, w - 0.01, 14.15], [24, w + 5, 14.9]),
      box([-15, w - 0.01, IN.z0 - 0.01], [-5, w + 5, -20.5]), box([5, w - 0.01, IN.z0 - 0.01], [15, w + 5, -20.5]));
    // coil: boot hole + countersink through the front wall, inside cradle (glue) behind the boot flange
    base = union(base, subtract(box([IN.x0 + BOOT.fl + 0.3, w - 0.01, gl.z - 4.2], [IN.x0 + 5.5, gl.y, gl.z + 4.2]), cylX(BOOT.rb, IN.x0, IN.x0 + 7, gl.y, gl.z)));
    base = subtract(base, cylX(bootHoleR, OUT.x0 - 1, IN.x0 + 0.01, gl.y, gl.z), coneX(bootHoleR + BOOT.cs + 0.01, bootHoleR, OUT.x0 - 0.01, OUT.x0 + BOOT.cs, gl.y, gl.z));
    // port-plug tether anchor: 2.2 mm hole through the rear wall into a closed cup (no path into the pack)
    const an = { y: 4.6, z: uzc };
    base = union(base, cylX(2.4, IN.x1 - 2.6, IN.x1 + 0.01, an.y, an.z), box([IN.x1 - 2.6, w - 0.01, an.z - 2.4], [IN.x1 + 0.01, an.y, an.z + 2.4]));
    base = subtract(base, cylX(1.85, IN.x1 - 1.8, IN.x1 + 0.01, an.y, an.z), cylX(1.1, IN.x1 - 0.1, OUT.x1 + 0.1, an.y, an.z));
    // weep holes, 1.5 mm, in the wall that faces down when worn: meltwater and condensate drain, pressure equalises
    const weeps = [-26, 9, 30].filter(x => x < IN.x1 - 3);
    for (const x of weeps) base = subtract(base, cylZ(0.75, IN.z1 - 0.1, OUT.z1 + 0.1, x, 4.0));

    // ---- shared finishing: 0.4 x 0.4 shadow-line reveal at the seam on both halves, pry slots at both ends ----
    const rv = 0.4;
    const reveal = subtract(box([OUT.x0 - 1, ys - rv, OUT.z0 - 1], [OUT.x1 + 1, ys + rv, OUT.z1 + 1]), slab(OUT.x0 + rv, OUT.x1 - rv, ys - rv - 0.1, ys + rv + 0.1, OUT.z0 + rv, OUT.z1 - rv, R.p - rv));
    base = subtract(base, reveal, box([OUT.x0 - 0.1, ys - 1.4, -5], [OUT.x0 + 0.8, ys + 0.01, 3]), box([OUT.x1 - 0.8, ys - 1.4, 9], [OUT.x1 + 0.1, ys + 0.01, 17]));
    lid = subtract(lid, reveal);
    // debossed marks on the lid (0.4 deep, 0.6 strokes): name + chevron pointing to the coil / goggle end
    lid = subtract(lid, debossY(text2D('SKI HUD', 3.6, 0.6), -2, -12, OUT.y1, 0.4), debossY(chevron2D(6, 180), -24, 0, OUT.y1, 0.4));

    // ---- strap clamp plate (snaps under the base, strap sandwiched in its channel) ----
    // The plate is at least 1.4 mm wider than the strap channel on each side; with a strap wider than the
    // pack the hook arms lean in from outside the strap edge up to the grooves.
    const zch = P.strapW / 2 + 0.3, zb = Math.max(OUT.z1 + 0.1, zch + 0.2), pz = Math.max(OUT.z1 + 1.5, zb + 1.4), px = 24;
    let plate = rbox([-px, -1.4 - P.strapT, -pz], [px, 0, pz], 1.8);
    plate = subtract(plate, box([-px - 1, -P.strapT - 0.1, -zch], [px + 1, 0.1, zch]));
    for (let x = -px + 4; x <= px - 4; x += 6) plate = union(plate, box([x - 0.6, -P.strapT - 0.11, -zch], [x + 0.6, -P.strapT + 0.5, zch]));
    // hook arms 1.4 thick, 6.2 long, 0.45 mm catch (groove is 0.6 deep)
    const hookArm = (x) => union(hull(rbox([x - 4.5, -1.6, zb], [x + 4.5, -0.6, zb + 1.4], 0.4), rbox([x - 4.5, 5.2, OUT.z1 + 0.1], [x + 4.5, 6.2, OUT.z1 + 1.5], 0.4)),
      prism([[6.0, OUT.z1 + 0.1], [5.0, OUT.z1 - 0.35], [4.6, OUT.z1 - 0.35], [4.6, OUT.z1 + 0.1]], 'x', x - 4.5, x + 4.5));
    for (const x of [-18, 18]) plate = union(plate, hookArm(x), mirrorZ(hookArm(x)));

    // ---- TPU port plug: body with two line-to-line sealing ribs, flange over the chamfered mouth,
    //      tether strip down to a push-in mushroom anchor. Flat outer face = prints face down.
    const portPlug = union(
      box([OUT.x1 - 4.0, tunnel.y0 + 0.2, tunnel.z0 + 0.2], [OUT.x1, tunnel.y1 - 0.2, tunnel.z1 - 0.2]),
      box([OUT.x1 - 3.4, tunnel.y0, tunnel.z0], [OUT.x1 - 2.8, tunnel.y1, tunnel.z1]),
      box([OUT.x1 - 1.8, tunnel.y0, tunnel.z0], [OUT.x1 - 1.2, tunnel.y1, tunnel.z1]),
      rbox([OUT.x1, tunnel.y0 - 0.8, tunnel.z0 - 0.8], [OUT.x1 + 1.2, tunnel.y1 + 0.8, tunnel.z1 + 0.8], 0.5),
      rbox([OUT.x1 + 0.4, an.y - 1.6, an.z - 1.6], [OUT.x1 + 1.2, tunnel.y0 - 0.3, an.z + 1.6], 0.35),
      cylX(1.0, IN.x1 - 0.05, OUT.x1 + 0.45, an.y, an.z),
      coneX(0.8, 1.45, IN.x1 - 1.45, IN.x1 - 0.05, an.y, an.z));

    // ---- TPU button cap: floats in the recess; the switch spring presses its flange up against the lid = seal ----
    const cap = subtract(union(
      translate([sw.x, OUT.y1 + 0.2, sw.z], rotateX(-90 * D, roundedCylinder({ radius: 6.0, height: 3.0, roundRadius: 1.2, segments: 40 }))),
      cylY(6.0, lidIn + 1.0, OUT.y1, sw.x, sw.z),
      cylY(8.2, lidIn, lidIn + 1.0, sw.x, sw.z)),
      translate([sw.x, OUT.y1 + 1.7 + 6 - 0.5, sw.z], sphere({ radius: 6, segments: 32 })));   // 0.5 mm thumb dish

    // ---- TPU coil boot (front wall) ----
    const boot = translate([OUT.x0, gl.y, gl.z], rotateY(-90 * D, bootGeom(w)));

    const rx = g => rotateX(90 * D, g), rxn = g => rotateX(-90 * D, g);
    return {
      parts: {
        pack_base: { geom: base, material: 'PETG', frame: 'pack', label: 'Pack base', print: 'head side down, open side up, no supports', orient: rx },
        pack_lid: { geom: lid, material: 'PETG', frame: 'pack', label: 'Pack lid', print: 'top face down (logo on the bed), no supports', orient: rxn },
        pack_plate: { geom: plate, material: 'PETG', frame: 'pack', label: 'Strap clamp plate', print: 'head face down, hooks up', orient: rx },
        button_cap: { geom: cap, material: 'TPU', frame: 'pack', label: 'Button cap (TPU 95A)', print: 'flange down, dome up', orient: rx },
        port_plug: { geom: portPlug, material: 'TPU', frame: 'pack', label: 'USB-C plug + tether (TPU 95A)', print: 'outer face down, flat', orient: g => rotateY(90 * D, g) },
        pack_boot: { geom: boot, material: 'TPU', frame: 'pack', label: 'Coil boot, pack (TPU 95A)', print: 'flange end down, 3 mm brim', orient: g => rotateY(90 * D, g) },
      },
      comps, dims: { L: OUT.x1 - OUT.x0, W: OUT.z1 - OUT.z0, T: OUT.y1 - OUT.y0 + 1.4 + P.strapT, Lboot: OUT.x1 + 1.2 - (OUT.x0 - BOOT.L) },
      glandOut: [OUT.x0 - BOOT.L, gl.y, gl.z], ys,
      explode: { pack_lid: [0, 14, 0], button_cap: [0, 24, 0], pack_plate: [0, -12, 0], port_plug: [14, 0, 0], pack_boot: [-16, 0, 0] },
      checks: {
        usbRecess: +(OUT.x1 - 30.6).toFixed(2), tunnel: [TW, TH], plugRoom: P.plugRoom,
        plugClear: +(IN.x1 - plugX1).toFixed(1), weeps: weeps.length,
        strain: { lid: strain(lipT, bumpH - c, tabL), plate: strain(1.4, 0.45, 6.2), sw: strain(pk.f, pk.k, fTop - (fBot + 0.3)) },
      },
    };
  }

  // ============================================================
  function buildHud(P) {
    const comps = {}, rT = P.railT, c = TOL.slide;
    // ---- window clamp geometry (world frame) ----
    // The measured goggle has a vent WINDOW at the top centre (26.13 x 10.19 mm) with 4 mm ribs at its ends.
    // Two spring arms rise through that window, 0.3 mm inside its ends. Two pads under the end ribs push up,
    // the bar's floor sits on top of the same ribs, and the catches hook outward over the floor: both ribs
    // are clamped. The arms flex inward, into the open window. Wires run up between the arms.
    const wl = P.winL / 2, armT = 1.4, xo = wl - 0.3, xi = xo - armT;
    const armD = Math.min(6.0, Math.max(2.4, P.winH - 0.6));      // arm depth (Z), 0.3 clear of the window's front/back
    const xb = xo + 0.2;                                            // slot outer edge: catches overlap the floor by 0.4
    const padX1 = Math.min(16.8, wl + Math.max(1.0, P.ribW - 0.8)); // bearing pads reach under the end ribs
    const hx = 0, hr2 = 1.8;                                        // wire hole in the bar floor, between the arms
    const zL = P.railD / 2, zF = zL - P.cavityD;                    // lens inner face, face foam
    const sag = x => x * x / (2 * RPLAN);                           // frame and lens curve back away from the centre
    // ---- frame bar (outside, on top of the rail): centred on the window, IMU above the catches ----
    const hbx = Math.max(13.6, wl + 0.6);
    const BI = { x0: -hbx, x1: hbx + 1.4, z0: -9.4, z1: 9.4, y0: rT + 1.2, y1: rT + 8.4 };
    const BO = { x0: BI.x0 - 1.6, x1: BI.x1 + 1.6, z0: BI.z0 - 1.6, z1: BI.z1 + 1.6, y0: rT, y1: BI.y1 + 1.2 };
    const bshell = subtract(rbox([BO.x0, BO.y0, BO.z0], [BO.x1, BO.y1, BO.z1], 2), box([BI.x0, BI.y0, BI.z0], [BI.x1, BI.y1, BI.z1]));
    let barBase = intersect(bshell, box([-60, rT - 1, -30], [60, BI.y1, 30]));
    let barLid = intersect(bshell, box([-60, BI.y1, -30], [60, 40, 30]));
    // IMU (ICM-20948 board) centred, on four 1.8 mm pads so it clears the catches, pressed down by four lid posts
    const imuX0 = -12.85, imuX1 = imuX0 + 25.7, iy = BI.y0 + 1.8;
    comps.imu = { geom: union(box([imuX0, iy, -8.85], [imuX1, iy + 1.6, 8.85]), box([imuX0 + 0.5, iy + 1.6, -3], [imuX0 + 5, iy + 4.6, 3]), box([imuX1 - 5, iy + 1.6, -3], [imuX1 - 0.5, iy + 4.6, 3]), box([imuX0 + 11, iy + 1.6, -1.5], [imuX0 + 14, iy + 2.5, 1.5])), kind: 'pcbIMU', label: 'ICM-20948 IMU' };
    // pads stay clear of the arm slots: inboard of them for wide windows, outboard of the catch tips for narrow ones
    const padPX = xi - 2.0 >= 6 ? Math.min(10.85, xi - 2.0) : Math.min(11.85, xo + 0.6 + 0.3 + 1.2);
    const padXZ = [[-padPX, -6.5], [-padPX, 6.5], [padPX, -6.5], [padPX, 6.5]];
    barBase = union(barBase, ...padXZ.map(([x, z]) => box([x - 1.2, BI.y0 - 0.01, z - 1.2], [x + 1.2, iy, z + 1.2])));
    const imuPosts = padXZ.map(([x, z]) => box([x - 1.0, iy + 1.65, z - 1.0], [x + 1.0, BI.y1 + 0.01, z + 1.0]));
    // bar front sits 0.5 mm behind the lens's seat face, even at its ends where the frame curves back;
    // floor features are cut at their world Z (local = world - dzB)
    const dzB = Math.min(0, zL - 0.5 - sag(Math.max(-BO.x0, BO.x1)) - BO.z1);
    // two arm slots, 0.6 mm inward room for the arms to flex; the floor outside them rests on the end ribs
    for (const s of [1, -1]) {
      const a = s * (xi - 0.6), b = s * xb;
      barBase = subtract(barBase, box([Math.min(a, b), rT - 1, -armD / 2 - 0.3 - dzB], [Math.max(a, b), BI.y0 + 0.1, armD / 2 + 0.3 - dzB]));
    }
    // wire hole down into the centre window (seal with a dab of silicone after wiring)
    barBase = subtract(barBase, cylY(hr2, rT - 1, BI.y0 + 0.1, hx, -dzB));
    // cable entry, +X end: the boot neck drops into a U-notch in the base wall, the lid closes it from above
    const hr = bootHoleR, cs = BOOT.cs, cy = BI.y1 - hr;
    const csk = coneX(hr, hr + cs + 0.01, BO.x1 - cs, BO.x1 + 0.01, cy, 0);
    barBase = subtract(barBase, cylX(hr, BI.x1 - 0.01, BO.x1 + 0.1, cy, 0), box([BI.x1 - 0.01, cy, -hr], [BO.x1 + 0.1, BI.y1 + 0.1, hr]), hull(csk, translate([0, BI.y1 + 0.2 - cy, 0], csk)));
    comps.cable = { geom: cylX(2.3, imuX1 + 0.4, BO.x1 + BOOT.L + 2, cy, 0), kind: 'cable', label: 'coil tail' };
    // lid lip + snap tabs (7 wide, 3.6 long, 0.8 thick, 0.4 bump, 45 deg retention) into blind pockets in the base
    const lipH = 2.0, lipT = 0.8, tL = 3.6, bH = 0.4;
    const blip = subtract(box([BI.x0 + c, BI.y1 - lipH, BI.z0 + c], [BI.x1 - c, BI.y1 + 0.01, BI.z1 - c]), box([BI.x0 + c + lipT, BI.y1 - lipH - 1, BI.z0 + c + lipT], [BI.x1 - c - lipT, BI.y1 + 1, BI.z1 - c - lipT]));
    const zl = BI.z1 - c, yb = BI.y1;
    let blip2 = blip; const btabs = [];
    const tabX = [-(hbx - 5), hbx - 5];
    for (const x of tabX) for (const s of [1, -1]) {
      const zz0 = s > 0 ? zl - lipT : -zl, zz1 = s > 0 ? zl : -zl + lipT;
      blip2 = subtract(blip2, box([x - 4.3, yb - lipH - 1, zz0 - 0.01], [x - 3.5, yb + 0.001, zz1 + 0.01]), box([x + 3.5, yb - lipH - 1, zz0 - 0.01], [x + 4.3, yb + 0.001, zz1 + 0.01]));
      btabs.push(box([x - 3.5, yb - tL, zz0], [x + 3.5, yb + 0.01, zz1]));
    }
    const bb = x => prism([[yb - 3.5, zl], [yb - 2.7, zl + bH], [yb - 2.5, zl + bH], [yb - 2.1, zl]], 'x', x - 3, x + 3);
    const bbs = []; for (const x of tabX) bbs.push(bb(x), mirrorZ(bb(x)));
    barLid = union(barLid, blip2, ...btabs, ...bbs, ...imuPosts);
    barLid = subtract(barLid, cylX(hr, BI.x1 - 0.01, BO.x1 + 0.1, cy, 0), csk, box([BI.x1 - 1.7, BI.y1 - lipH - 1, -BOOT.rf - 0.3], [BI.x1 + 0.01, BI.y1 + 0.01, BOOT.rf + 0.3]));
    const bpk = x => prism([[yb - 3.65, BI.z1 - 0.01], [yb - 3.65, BI.z1 + bH], [yb - 2.35, BI.z1 + bH], [yb - 1.94, BI.z1 - 0.01]], 'x', x - 3.3, x + 3.3);
    for (const x of tabX) barBase = subtract(barBase, bpk(x), mirrorZ(bpk(x)));
    // finishing: seam reveal, pry slot at the -X end, debossed name + chevron pointing at the lens
    const rv = 0.4;
    const reveal = subtract(box([BO.x0 - 1, BI.y1 - rv, BO.z0 - 1], [BO.x1 + 1, BI.y1 + rv, BO.z1 + 1]), slab(BO.x0 + rv, BO.x1 - rv, BI.y1 - rv - 0.1, BI.y1 + rv + 0.1, BO.z0 + rv, BO.z1 - rv, 2 - rv));
    barBase = subtract(barBase, reveal, box([BO.x0 - 0.1, BI.y1 - 1.2, -4], [BO.x0 + 0.8, BI.y1 + 0.01, 4]));
    barLid = subtract(barLid, reveal, debossY(text2D('SKI HUD', 2.6, 0.5), -3.5, 0, BO.y1, 0.3), debossY(chevron2D(5, -90), 9.5, 0, BO.y1, 0.3));
    // TPU boot: same profile as the pack's, flange trimmed flat top and bottom to fit the 6.2 mm cavity
    let barBoot = translate([BO.x1, cy, 0], rotateY(90 * D, bootGeom(1.6)));
    barBoot = subtract(barBoot, box([BI.x1 - 3, BI.y1, -6], [BI.x1, BI.y1 + 6, 6]), box([BI.x1 - 3, BI.y0 - 6, -6], [BI.x1, BI.y0, 6]));
    const mvB = g => translate([0, 0, dzB], g);
    barBase = mvB(barBase); barLid = mvB(barLid); barBoot = mvB(barBoot);
    comps.imu.geom = mvB(comps.imu.geom); comps.cable.geom = mvB(comps.cable.geom);

    // ---- screen pod (inside the goggle, hangs under the rail) ----
    // local: x across, y down from the top edge (y<=0.8), z forward (back face z=0, front z=8.8)
    const PO = { x0: -17.3, x1: 17.3, y0: -28.2, y1: 0.8, z0: 0, z1: 8.8 };
    const PI = { x0: -15.9, x1: 15.9, y0: -26.8, y1: -0.6, z0: 1.2, z1: 7.0 };
    const zs = 2.5; // split between back and front
    // R1.5 corners, but only R0.5 on the bottom edges: the back half prints standing on that face
    const pshell = subtract(slab(PO.x0, PO.x1, PO.y0, PO.y1, PO.z0, PO.z1, 1.5, 0.5, 1.4), box([PI.x0, PI.y0, PI.z0], [PI.x1, PI.y1, PI.z1]));
    let podF = intersect(pshell, box([-30, -40, zs], [30, 10, 20]));
    let podB = intersect(pshell, box([-30, -40, -5], [30, 10, zs]));
    // screen window: 1 mm border over the glass edge, 45 deg bezel chamfer outside
    podF = subtract(podF, box([-12.4, -19.6, PI.z1 - 1], [10.4, -7.6, PO.z1 + 1]),
      hull(box([-12.4, -19.6, PO.z1 - 1.0], [10.4, -7.6, PO.z1 - 0.99]), box([-13.6, -20.8, PO.z1 + 0.2], [11.6, -6.4, PO.z1 + 0.21])));
    // hood: brow flush with the pod top + cheeks tapering to nothing at the window's lower edge.
    // Keeps light from the vent and the sides off the screen and limits spill onto the lens edges.
    if (P.hood > 0.05) {
      const hz = PO.z1 + P.hood;
      const cheek = (x0, x1) => prism([[PO.y1, PO.z1 - 0.6], [PO.y1, hz], [-20.8, PO.z1 - 0.6]], 'x', x0, x1);
      podF = union(podF, box([-15.4, -0.4, PO.z1 - 1.5], [13.4, PO.y1, hz]), cheek(-15.4, -14.2), cheek(12.2, 13.4));
    }
    // back lip + side snap tabs (0.7 thick, 0.45 bump), release windows in the front part.
    // The lip stands on the back plate and is tied to the walls by a ledge, except where the tabs are:
    // there the lip section is slit free and flexes with its tab from the back plate (about 4.6 mm).
    const lipInner = box([PI.x0 + c + 0.85, PI.y0 + c + 0.85, PI.z0 - 1], [PI.x1 - c - 0.85, PI.y1 - c - 0.85, zs + 3]);
    const plip = subtract(union(box([PI.x0 + c, PI.y0 + c, PI.z0 - 0.01], [PI.x1 - c, PI.y1 - c, zs + 1.9]), box([PI.x0 - 0.01, PI.y0 - 0.01, PI.z0 - 0.01], [PI.x1 + 0.01, PI.y1 + 0.01, zs])), lipInner);
    const xl = PI.x1 - c, tz1 = 6.8, pbH = 0.45;
    const ptab = s => box([s > 0 ? xl - 0.7 : -xl, -17.6, zs - 0.01], [s > 0 ? xl : -xl + 0.7, -9.6, tz1]);
    const plip2 = subtract(plip, box([PI.x0 - 1, -18.4, PI.z0 - 1], [PI.x1 + 1, -17.6, zs + 3]), box([PI.x0 - 1, -9.6, PI.z0 - 1], [PI.x1 + 1, -8.8, zs + 3]),
      box([PI.x1 - c - 0.01, -17.6, PI.z0 - 1], [PI.x1 + 1, -9.6, zs + 0.01]), box([PI.x0 - 1, -17.6, PI.z0 - 1], [PI.x0 + c + 0.01, -9.6, zs + 0.01]));
    const pb = () => prism([[tz1 - 0.1, xl], [tz1 - 0.8, xl + pbH], [tz1 - 1.3, xl + pbH], [tz1 - 1.3, xl]], 'y', -17.1, -10.1);
    podB = union(podB, plip2, ptab(1), ptab(-1), pb(), mirrorX(pb()));
    podF = subtract(podF, box([PI.x1 - 0.1, -17.6, tz1 - 1.5], [PO.x1 + 0.1, -9.6, tz1 + 0.15]), box([PO.x0 - 0.1, -17.6, tz1 - 1.5], [PI.x0 + 0.1, -9.6, tz1 + 0.15]));
    // press ribs on the back plate (clear of the header row)
    podB = union(podB, ...[[-13, -3.6], [13, -3.6], [-13, -24], [13, -24]].map(([x, y]) => box([x - 1, y - 1, PI.z0 - 0.01], [x + 1, y + 1, 2.55])));
    // wire opening in the top wall (back part), under the bar's wire hole in the -X window
    podB = subtract(podB, box([hx - 2.2, PI.y1 - 0.1, 0.2], [hx + 2.2, PO.y1 + 0.1, 2.3]));
    // screen module (local)
    const mod = union(box([-14.85, -25.75, 2.6], [14.85, -1.65, 4.2]), box([-13.4, -20.35, 4.2], [11.4, -6.85, 6.9]), box([-9.5, -4.0, 1.4], [9.5, -1.65, 2.6]));
    // place: tilt about the top-back edge, hang under the rail; pod sits 2.2 mm behind the window centre so the
    // clamp stands on the pod (and the hood gets room in front)
    const T = g => translate([0, -P.podGap - 0.8, -2.2], rotateX(P.tilt * D, g));
    const Tinv = g => rotateX(-P.tilt * D, translate([0, P.podGap + 0.8, 2.2], g));
    podF = T(podF); podB = T(podB);
    comps.screen = { geom: T(mod), kind: 'pcbBlue', label: '0.96″ screen' };
    // window clamp: two legs, each a bearing pad under an end rib (top bears on the rib's underside) with a
    // 1.4 mm arm rising through the window 0.3 mm inside its end; 0.6 mm catches point outward over the bar
    // floor. Inner root fillets. The window between the legs stays open (wires, air).
    const sTop = BI.y0 + 1.6, yc = -P.podGap - 2.5;
    const zA = [-armD / 2, armD / 2];
    const arm = s => box([s > 0 ? xi : -xo, yc, zA[0]], [s > 0 ? xo : -xi, sTop, zA[1]]);
    const wedge = s => prism([[s * xo, BI.y0 + 0.15], [s * (xo + 0.6), BI.y0 + 0.15], [s * (xo + 0.6), BI.y0 + 0.6], [s * xo, sTop]], 'z', zA[0], zA[1]);
    const fillet = s => prism([[s * xi, -0.01], [s * xi, 1.0], [s * (xi - 1.0), -0.01]], 'z', zA[0], zA[1]);
    const pad = s => box([s > 0 ? xi - 1.0 : -padX1, yc, zA[0]], [s > 0 ? padX1 : -(xi - 1.0), 0, zA[1]]);
    let stem = union(arm(1), arm(-1), wedge(1), wedge(-1), fillet(1), fillet(-1), pad(1), pad(-1));
    stem = subtract(stem, T(box([PI.x0, PI.y0, PI.z0], [PI.x1, PI.y1, PI.z1])), T(box([PO.x0 - 1, PO.y0 - 1, zs], [PO.x1 + 1, PO.y1 + 0.2, PO.z1 + 6])));
    podB = union(podB, stem);

    // context: goggle frame top rail as a ladder of vent windows, a window centred at X=0 (not printed)
    let rail = box([-45, 0, -P.railD / 2], [45, rT, P.railD / 2]);
    for (let k = -4; k <= 4; k++) {
      const a = Math.max(-44, k * (P.winL + P.ribW) - wl), b = Math.min(44, a + P.winL);
      if (b - a > 0.5) rail = subtract(rail, box([a, -1, -P.winH / 2], [b, rT + 1, P.winH / 2]));
    }
    const armL = sTop - 0;
    // reflection check: where must the screen face so its image in the lens reaches the eye?
    // Lens taken as a flat mirror at zL; eye 12 mm behind the face foam (assumption), eyeH below the rail.
    const scr = T(translate([-1, -13.6, 6.9], cuboid({ size: [0.1, 0.1, 0.1] }))), sc = C(...measureBoundingBox(scr));
    const ye = -P.eyeH, zeM = 2 * zL - (zF - 12);
    const tiltSuggest = Math.atan2(sc[1] - ye, zeM - sc[2]) / D;

    return {
      parts: {
        bar_base: { geom: barBase, material: 'PETG', frame: 'hud', label: 'Frame bar base', print: 'floor down, no supports', orient: g => rotateX(90 * D, g) },
        bar_lid: { geom: barLid, material: 'PETG', frame: 'hud', label: 'Frame bar lid', print: 'top face down', orient: g => rotateX(-90 * D, g) },
        bar_boot: { geom: barBoot, material: 'TPU', frame: 'hud', label: 'Coil boot, bar (TPU 95A)', print: 'flange end down, 3 mm brim', orient: g => rotateY(-90 * D, g) },
        pod_front: { geom: podF, material: 'PETG', frame: 'hud', label: 'Screen pod front', print: 'top face down (hood brow on the bed), no supports', orient: g => rotateX(-90 * D, Tinv(g)) },
        pod_back: { geom: podB, material: 'PETG', frame: 'hud', label: 'Screen pod back + clamp', print: `bottom face down, clamp arms up (leans ${P.tilt}°), 5 mm brim, no supports`, orient: g => rotateX(90 * D, Tinv(g)) },
      },
      comps, rail, glandOut: [BO.x1 + BOOT.L, cy, dzB],
      dims: { bar: [BO.x1 - BO.x0, BO.y1 - BO.y0, BO.z1 - BO.z0], pod: [PO.x1 - PO.x0, PO.y1 - PO.y0, PO.z1 - PO.z0 + P.hood] },
      explode: { bar_lid: [0, 12, 0], bar_boot: [14, 0, 0], pod_front: [0, -6, 14], pod_back: [0, -6, 0] },
      checks: {
        strain: { bar: strain(lipT, bH - c, tL), pod: strain(0.7, pbH - c, tz1 - 0.8 - PI.z0), stem: strain(armT, 0.4, armL) },
        armL: +armL.toFixed(1),
        winNeedL: +(2 * (hr2 + 0.5 + armT + 0.9)).toFixed(1), winNeedH: +(armD + 0.6).toFixed(1), armD: +armD.toFixed(1),
        zL, zF, barFront: +(BO.z1 + dzB + sag(Math.max(-BO.x0, BO.x1))).toFixed(1), lensSag: +sag(15.4).toFixed(1), tiltSuggest: +tiltSuggest.toFixed(0),
      },
    };
  }

  // ---------- checks ----------
  function bbOverlap(a, b) { return a[0][0] < b[1][0] && a[1][0] > b[0][0] && a[0][1] < b[1][1] && a[1][1] > b[0][1] && a[0][2] < b[1][2] && a[1][2] > b[0][2]; }
  function collisions(items) {
    const out = [];
    const bbs = items.map(i => measureBoundingBox(i.geom));
    for (let i = 0; i < items.length; i++) for (let j = i + 1; j < items.length; j++) {
      if (!bbOverlap(bbs[i], bbs[j])) continue;
      const v = measureVolume(intersect(items[i].geom, items[j].geom));
      if (v > 0.05) out.push({ a: items[i].name, b: items[j].name, v: +v.toFixed(2) });
    }
    return out;
  }

  // Close the mesh for printing. CSG leaves T-junctions (a vertex of one face sitting on the edge of its
  // neighbour) and hairline slivers (< 0.02 mm) where curved cuts meet. Weld vertices closer than 0.02 mm
  // (far below print resolution), drop collapsed faces, split every edge that has no partner at the mesh
  // vertices lying on it, and fan those polygons from their centre. Repeats until nothing changes.
  const WELD = 0.02;
  function weld(polys) {
    const verts = [], grid = new Map(), cell = v => v.map(x => Math.floor(x / WELD));
    const find = v => {
      const c = cell(v);
      for (let dx = -1; dx <= 1; dx++) for (let dy = -1; dy <= 1; dy++) for (let dz = -1; dz <= 1; dz++) {
        const L = grid.get((c[0] + dx) + ',' + (c[1] + dy) + ',' + (c[2] + dz)); if (!L) continue;
        for (const i of L) { const u = verts[i]; if (Math.hypot(u[0] - v[0], u[1] - v[1], u[2] - v[2]) < WELD) return i; }
      }
      verts.push(v); const g = c.join(','); if (!grid.has(g)) grid.set(g, []); grid.get(g).push(verts.length - 1); return verts.length - 1;
    };
    const out = [];
    for (const p of polys) {
      const ids = []; for (const v of p) { const i = find(v); if (ids[ids.length - 1] !== i) ids.push(i); }
      while (ids.length > 1 && ids[0] === ids[ids.length - 1]) ids.pop();
      if (ids.length >= 3 && new Set(ids).size === ids.length) out.push(ids.map(i => verts[i]));
    }
    return out;
  }
  function closeMesh(polys) {
    const tag = v => v.map(x => Math.round(x * 1e5)).join(',');
    polys = weld(polys);
    for (let pass = 0; pass < 6; pass++) {
      const cnt = new Map(), ek = (a, b) => { const p = tag(a), q = tag(b); return p < q ? p + '|' + q : q + '|' + p; };
      for (const p of polys) p.forEach((v, i) => { const k = ek(v, p[(i + 1) % p.length]); cnt.set(k, (cnt.get(k) || 0) + 1); });
      const loose = new Map();   // vertices on open edges
      for (const p of polys) p.forEach((v, i) => { const w = p[(i + 1) % p.length]; if (cnt.get(ek(v, w)) === 1) { loose.set(tag(v), v); loose.set(tag(w), w); } });
      if (!loose.size) break;
      const cand = [...loose.values()];
      polys = polys.flatMap(p => {
        const out = []; let split = false;
        p.forEach((a, i) => {
          out.push(a); const b = p[(i + 1) % p.length];
          if (cnt.get(ek(a, b)) !== 1) return;
          const d = [b[0] - a[0], b[1] - a[1], b[2] - a[2]], L2 = d[0] * d[0] + d[1] * d[1] + d[2] * d[2];
          const on = [];
          for (const v of cand) {
            const t = ((v[0] - a[0]) * d[0] + (v[1] - a[1]) * d[1] + (v[2] - a[2]) * d[2]) / L2;
            if (t <= 1e-6 || t >= 1 - 1e-6) continue;
            const e = [a[0] + t * d[0] - v[0], a[1] + t * d[1] - v[1], a[2] + t * d[2] - v[2]];
            if (e[0] * e[0] + e[1] * e[1] + e[2] * e[2] < WELD * WELD / 4) on.push([t, v]);
          }
          if (on.length) { split = true; on.sort((x, y) => x[0] - y[0]).forEach(([, v]) => out.push(v)); }
        });
        if (!split) return [p];
        const c = [0, 1, 2].map(k => out.reduce((s, v) => s + v[k], 0) / out.length);
        return out.map((v, i) => [c, v, out[(i + 1) % out.length]]);
      });
      polys = weld(polys);
    }
    // whatever is still open is a pinhole: walk each open loop and cap it
    const half = new Map(), vtx = new Map();
    for (const p of polys) p.forEach((v, i) => { const w = p[(i + 1) % p.length], a = tag(v), b = tag(w); vtx.set(a, v); vtx.set(b, w); half.set(a + '>' + b, [a, b]); });
    const next = new Map();
    for (const [k, [a, b]] of half) if (!half.has(b + '>' + a)) next.set(b, a);   // cap runs the other way
    const used = new Set();
    for (const s of next.keys()) {
      if (used.has(s)) continue;
      const loop = []; let cur = s;
      while (cur !== undefined && !used.has(cur) && loop.length < 64) { used.add(cur); loop.push(vtx.get(cur)); cur = next.get(cur); }
      if (cur === s && loop.length >= 3) {
        const c = [0, 1, 2].map(k => loop.reduce((t, v) => t + v[k], 0) / loop.length);
        loop.forEach((v, i) => polys.push([c, v, loop[(i + 1) % loop.length]]));
      }
    }
    return polys;
  }

  function printGeom(part) {
    const g = part.orient ? part.orient(part.geom) : part.geom, bb = measureBoundingBox(g);
    const g2 = J.modifiers.generalize({ snap: true, triangulate: true }, translate([-(bb[0][0] + bb[1][0]) / 2, -(bb[0][1] + bb[1][1]) / 2, -bb[0][2]], g));
    const polys = closeMesh(J.geometries.geom3.toPolygons(g2).map(p => p.vertices.map(v => [v[0], v[1], v[2]])));
    return J.geometries.geom3.create(polys.map(vs => J.geometries.poly3.create(vs)));
  }

  function build(params) {
    const P = Object.assign({}, DEFAULTS, params || {});
    const pack = buildPack(P), hud = buildHud(P);
    const parts = Object.assign({}, pack.parts, hud.parts);
    for (const k in parts) {
      parts[k].volume = measureVolume(parts[k].geom);
      const bb = measureBoundingBox(parts[k].geom); parts[k].size = [0, 1, 2].map(i => +(bb[1][i] - bb[0][i]).toFixed(1));
    }
    // how far the pod (with hood) reaches toward the lens, measured from the vent centre
    const pbF = measureBoundingBox(hud.parts.pod_front.geom), pbB = measureBoundingBox(hud.parts.pod_back.geom);
    hud.checks.podReach = +Math.max(pbF[1][2], pbB[1][2]).toFixed(1);       // front-most point (incl. hood)
    hud.checks.podBack = +Math.min(pbF[0][2], pbB[0][2]).toFixed(1);        // rear-most point
    hud.checks.podBottom = +Math.min(pbF[0][1], pbB[0][1]).toFixed(1);      // lowest point (Y below the rail)
    hud.checks.lensClear = +(hud.checks.zL - hud.checks.lensSag - hud.checks.podReach).toFixed(1);   // lens curves back at the hood corners
    hud.checks.eyeClear = +(hud.checks.podBottom + P.eyeH).toFixed(1);       // pod bottom above the eye line
    const list = (o, pre) => Object.keys(o).map(k => ({ name: pre + k, geom: o[k].geom }));
    const packItems = list(pack.parts, '').concat(list(pack.comps, 'c:'));
    const hudItems = list(hud.parts, '').concat(list(hud.comps, 'c:'), [{ name: 'ctx:rail', geom: hud.rail }]);
    const coll = collisions(packItems).concat(collisions(hudItems));
    return { P, parts, pack, hud, collisions: coll };
  }

  window.SkiCAD = { build, DEFAULTS, TOL, printGeom, toPolygons: g => J.geometries.geom3.toPolygons(g) };
})();
