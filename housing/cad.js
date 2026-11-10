/* Ski HUD housings: parametric CAD model (JSCAD v2). Units: mm.
 *
 * One source of truth for the 3D viewer now and for STL/3MF export later.
 * SkiCAD.build(params) -> { parts:{name:{geom, frame, material, ...}}, comps:{...}, context:{...} }
 *
 * PACK frame (strap pack, lying on its head side):
 *   X = length along the strap, front (toward goggle, coil exit) = -X, rear (USB-C) = +X
 *   Y = thickness, head side Y=0, outward +Y
 *   Z = height on the head
 * HUD frame (goggle top centre):
 *   X = across the face, Y = up, Z = forward toward the lens
 *   Goggle frame top rail occupies Y [0, railT], Z [-railD/2, railD/2], vent slot at X=0.
 */
(function () {
  const J = window.jscadModeling;
  const { cuboid, roundedCuboid, cylinder, cylinderElliptic, roundedCylinder, polygon } = J.primitives;
  const { union, subtract, intersect } = J.booleans;
  const { translate, rotateX, rotateY, mirrorZ, mirrorX } = J.transforms;
  const { extrudeLinear } = J.extrusions;
  const { measureVolume, measureBoundingBox } = J.measurements;
  const D = Math.PI / 180;

  const DEFAULTS = {
    wall: 1.6,        // pack wall (4 perimeters at 0.4 mm)
    lidTop: 1.8,      // pack lid top plate
    strapW: 40,       // goggle strap width
    strapT: 2.5,      // goggle strap thickness
    railT: 6,         // goggle frame top rail thickness at the vent
    railD: 18,        // rail depth front-to-back
    ventL: 16,        // vent slot length (X)
    ventW: 5,         // vent slot width (Z)
    tilt: 20,         // screen tilt down from vertical, degrees
    podGap: 2,        // gap between rail underside and pod top
  };

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
  const coneX = (r0, r1, x0, x1, y, z) => translate([(x0 + x1) / 2, y, z], rotateY(90 * D, cylinderElliptic({ height: x1 - x0, startRadius: [r0, r0], endRadius: [r1, r1], segments: 32 })));

  // ============================================================
  function buildPack(P) {
    const w = P.wall;
    // inner cavity and split height
    const IN = { x0: -30, x1: 30, z0: -22, z1: 22, y0: w, y1: w + 18.8 };  // battery 10.4 + gap 1.8 + PCB 1.6 + 5.0 above
    const OUT = { x0: IN.x0 - w, x1: IN.x1 + w, z0: IN.z0 - w, z1: IN.z1 + w, y0: 0, y1: IN.y1 + P.lidTop };
    const ys = w + 10.4 + 1.8;              // split plane = PCB underside (13.8 with defaults)
    const pcbTop = ys + 1.6;

    // ---- components (PACK frame) ----
    const comps = {};
    comps.battery = { geom: box([-23.2, w, -20.2], [26.8, w + 10.4, 13.8]), kind: 'battery', label: '103450 LiPo' };
    comps.foam = { geom: box([-20.5, w + 10.4, -19.8], [26.5, ys, 4.8]), kind: 'foam', label: 'foam tape + Kapton' };
    const bx0 = -21.2, bx1 = 29.8, bz0 = -20.2, bz1 = 5.2;   // ESP32 Lite outline (kept clear of the lid lip on the -Z side)
    const zc = (bz0 + bz1) / 2;                               // board centre line
    comps.pcb = { geom: box([bx0, ys, bz0], [bx1, pcbTop, bz1]), kind: 'pcb', label: 'ESP32 Lite' };
    comps.usb = { geom: box([23.3, pcbTop, zc + 3 - 4.47], [30.6, pcbTop + 3.2, zc + 3 + 4.47]), kind: 'steel', label: 'USB-C' };
    comps.jst = { geom: box([24.6, pcbTop, zc - 10.5], [29.9, pcbTop + 4.6, zc - 4.5]), kind: 'cream', label: 'battery socket (leads soldered)' };
    comps.chip = { geom: box([-14, pcbTop, zc - 5], [-7, pcbTop + 0.9, zc + 2]), kind: 'chip', label: 'ESP32 chip' };
    comps.ch340 = { geom: box([10, pcbTop, zc + 1.5], [19, pcbTop + 1.6, zc + 6.5]), kind: 'chip', label: 'CH340C' };
    comps.connector = { geom: box([-8, 3.0, 16.6], [5, 16.0, 20.0]), kind: 'cream', label: '8-pin JST 1.25 pair' };
    comps.splices = { geom: box([-24, 4.6, 15.6], [-8.5, 11.4, 20.3]), kind: 'shrink', label: '8 splices' };
    const gl = { y: 7.5, z: 17.8, r: 2.3 };
    comps.coil = { geom: cylX(gl.r, -44, -24.5, gl.y, gl.z), kind: 'cable', label: 'coil tail' };
    // low-profile 12x12x4.3 tact switch, hanging from the lid, plunger up
    const sw = { x: 19.4, z: 15.4 };
    const lidIn = IN.y1;                     // lid inner face
    const plTop = lidIn - 0.5;               // plunger top (TPU cap nub reaches here)
    comps.switch = { geom: union(box([sw.x - 6, plTop - 4.3, sw.z - 6], [sw.x + 6, plTop - 0.8, sw.z + 6]), cylY(3.3, plTop - 0.8, plTop, sw.x, sw.z)), kind: 'chip', label: '12×12×4.3 switch' };

    // ---- shell ----
    const shell = subtract(rbox([OUT.x0, OUT.y0, OUT.z0], [OUT.x1, OUT.y1, OUT.z1], 2.4), box([IN.x0, IN.y0, IN.z0], [IN.x1, IN.y1, IN.z1]));
    let base = intersect(shell, box([-60, -5, -40], [60, ys, 40]));
    let lid = intersect(shell, box([-60, ys, -40], [60, 40, 40]));

    // ---- lid lip (slides inside the base walls) + snap bumps on the long sides ----
    // Three sides only (front + both long sides): the board's USB end sits right against the rear wall.
    // A 1.4 mm ledge above the split bridges the lip to the lid walls.
    const lipH = 3.5, lipT = 1.0, c = 0.15, ledge = 1.4;
    const ring = (y0, y1, x0) => subtract(box([x0, y0, IN.z0 - 0.01], [IN.x1 - 1.6, y1, IN.z1 + 0.01]),
      box([IN.x0 + c + lipT, y0 - 1, IN.z0 + c + lipT], [IN.x1 + 5, y1 + 1, IN.z1 - c - lipT]));
    const lipBand = intersect(ring(ys - lipH, ys + 0.01, IN.x0 + c), box([IN.x0 + c, -5, IN.z0 + c], [IN.x1, 40, IN.z1 - c]));
    // Snap tabs: 8 mm wide, 5 mm long (lip is 1.0 thick), cut free of the lip by 0.8 mm relief slots,
    // 0.3 mm bump near the tip -> about 1.8 % bending strain, fine for PETG.
    const tabL = 5.0, zl = IN.z1 - c; // lip outer face
    let lipT2 = lipBand;
    const tabs = [];
    for (const x of [-16, 16]) for (const s of [1, -1]) {
      const zz0 = s > 0 ? zl - lipT : -zl, zz1 = s > 0 ? zl : -zl + lipT;
      lipT2 = subtract(lipT2, box([x - 4.8, ys - lipH - 1, zz0 - 0.01], [x - 4.0, ys + 0.001, zz1 + 0.01]), box([x + 4.0, ys - lipH - 1, zz0 - 0.01], [x + 4.8, ys + 0.001, zz1 + 0.01]));
      tabs.push(box([x - 4.0, ys - tabL, zz0], [x + 4.0, ys + 0.01, zz1]));
    }
    const lip = union(lipT2, ...tabs, ring(ys, ys + ledge, IN.x0 - 0.01));
    const bumpP = (x) => prism([[ys - 4.9, zl], [ys - 4.1, zl + 0.3], [ys - 3.5, zl + 0.3], [ys - 3.5, zl]], 'x', x - 3.5, x + 3.5);
    const bumps = [];
    for (const x of [-16, 16]) { bumps.push(bumpP(x)); bumps.push(mirrorZ(bumpP(x))); }
    // board hold-down posts (avoid the header holes, JST and USB)
    const posts = [[-19.4, bz0 + 1.6], [-19.4, bz1 - 1.8], [27.4, bz1 - 1.8]].map(([x, z]) => box([x - 1.2, pcbTop + 0.05, z - 1.2], [x + 1.2, lidIn + 0.01, z + 1.2]));
    // switch pocket walls
    const pocket = subtract(box([sw.x - 7.2, plTop - 3.8, sw.z - 7.2], [sw.x + 7.2, lidIn + 0.01, sw.z + 7.2]),
      box([sw.x - 6.15, plTop - 5, sw.z - 6.15], [sw.x + 6.15, lidIn + 1, sw.z + 6.15]));
    lid = union(lid, lip, ...bumps, ...posts, intersect(pocket, box([IN.x0, 0, IN.z0], [IN.x1, 40, IN.z1])));
    // button: hole through the top + recess for the TPU cap flange
    lid = subtract(lid, cylY(6.3, lidIn - 1, OUT.y1 + 1, sw.x, sw.z), cylY(8.5, lidIn - 1, lidIn + 1.0, sw.x, sw.z));
    // USB-C notch (open at the bottom edge so the lid drops over the receptacle) + funnel for plug overmolds
    const uz0 = zc + 3 - 4.47 - 0.45, uz1 = zc + 3 + 4.47 + 0.45, uy1 = pcbTop + 3.2 + 0.4;
    lid = subtract(lid, box([IN.x1 - 1, ys - 0.1, uz0], [OUT.x1 + 1, uy1, uz1]));
    lid = subtract(lid, box([OUT.x1 - 0.7, ys - 0.1, uz0 - 1.4], [OUT.x1 + 1, uy1 + 1.4, uz1 + 1.4]));

    // ---- base features ----
    // snap windows for the lid bumps
    for (const x of [-16, 16]) for (const s of [1, -1]) {
      const z0 = s > 0 ? IN.z1 - 0.1 : OUT.z0 - 0.1, z1 = s > 0 ? OUT.z1 + 0.1 : IN.z0 + 0.1;
      base = subtract(base, box([x - 4.0, ys - 5.1, z0], [x + 4.0, ys - 3.25, z1]));
    }
    // grooves for the strap-plate hooks (outer long faces, near the bottom)
    for (const x of [-18, 18]) {
      base = subtract(base, box([x - 4.7, 4.4, OUT.z1 - 0.6], [x + 4.7, 6.0, OUT.z1 + 1]));
      base = subtract(base, box([x - 4.7, 4.4, OUT.z0 - 1], [x + 4.7, 6.0, OUT.z0 + 0.6]));
    }
    // battery locating ribs (front stop, side wall)
    base = union(base, box([-24.6, w - 0.01, -19.5], [-23.5, w + 5, 12]), box([-20, w - 0.01, 14.15], [24, w + 5, 14.9]));
    // coil: hole through the front wall, outside strain-relief cone, inside cable cradle
    base = union(base, subtract(coneX(3.3, 4.6, OUT.x0 - 6, OUT.x0 + 0.01, gl.y, gl.z), cylX(gl.r + 0.15, OUT.x0 - 7, OUT.x0 + 1, gl.y, gl.z)));
    base = subtract(base, cylX(gl.r + 0.15, OUT.x0 - 8, IN.x0 + 1, gl.y, gl.z));
    base = union(base, subtract(box([IN.x0 - 0.01, w - 0.01, gl.z - 4.2], [IN.x0 + 5, gl.y, gl.z + 4.2]), cylX(gl.r + 0.15, IN.x0 - 1, IN.x0 + 6, gl.y, gl.z)));

    // ---- strap clamp plate (snaps under the base, strap sandwiched in its channel) ----
    const pz = OUT.z1 + 1.5, px = 24;
    let plate = rbox([-px, -1.4 - P.strapT, -pz], [px, 0, pz], 1.2);
    plate = subtract(plate, box([-px - 1, -P.strapT - 0.1, -P.strapW / 2 - 0.3], [px + 1, 0.1, P.strapW / 2 + 0.3]));
    for (let x = -px + 4; x <= px - 4; x += 6) plate = union(plate, box([x - 0.6, -P.strapT - 0.11, -P.strapW / 2 - 0.3], [x + 0.6, -P.strapT + 0.5, P.strapW / 2 + 0.3]));
    // hook arms 1.4 thick, 6.2 long, 0.5 mm catch -> about 3 % strain
    const hookArm = (x) => union(box([x - 4.5, -0.01, OUT.z1 + 0.1], [x + 4.5, 6.2, pz]),
      prism([[6.0, OUT.z1 + 0.1], [5.0, OUT.z1 - 0.45], [4.6, OUT.z1 - 0.45], [4.6, OUT.z1 + 0.1]], 'x', x - 4.5, x + 4.5));
    for (const x of [-18, 18]) { plate = union(plate, hookArm(x), mirrorZ(hookArm(x))); }

    // ---- TPU button cap ----
    const cap = union(
      translate([sw.x, OUT.y1 + 0.2, sw.z], rotateX(-90 * D, roundedCylinder({ radius: 6.0, height: 3.0, roundRadius: 1.2, segments: 40 }))),
      cylY(6.0, lidIn + 1.0, OUT.y1, sw.x, sw.z),
      cylY(8.2, lidIn, lidIn + 1.0, sw.x, sw.z),
      cylY(2.2, plTop, lidIn + 0.01, sw.x, sw.z));

    return {
      parts: {
        pack_base: { geom: base, material: 'PETG', frame: 'pack', label: 'Pack base', print: 'open side up, no supports' },
        pack_lid: { geom: lid, material: 'PETG', frame: 'pack', label: 'Pack lid', print: 'top face down, no supports' },
        pack_plate: { geom: plate, material: 'PETG', frame: 'pack', label: 'Strap clamp plate', print: 'flat side down' },
        button_cap: { geom: cap, material: 'TPU', frame: 'pack', label: 'Button cap (TPU 95A)', print: 'flange down' },
      },
      comps, dims: { L: OUT.x1 - OUT.x0, W: OUT.z1 - OUT.z0, T: OUT.y1 - OUT.y0 + 1.4 + P.strapT },
      glandOut: [OUT.x0 - 6, gl.y, gl.z], ys, explode: { pack_lid: [0, 14, 0], button_cap: [0, 24, 0], pack_plate: [0, -12, 0] },
      checks: { buttonGap: +(plTop - plTop).toFixed(2), usbRecess: +(OUT.x1 - 30.6).toFixed(2) },
    };
  }

  // ============================================================
  function buildHud(P) {
    const comps = {}, rT = P.railT;
    // ---- frame bar (outside, on top of the rail) ----
    const BI = { x0: -8.6, x1: 36.0, z0: -9.4, z1: 9.4, y0: rT + 1.2, y1: rT + 7.4 };
    const BO = { x0: BI.x0 - 1.6, x1: BI.x1 + 1.6, z0: BI.z0 - 1.6, z1: BI.z1 + 1.6, y0: rT, y1: BI.y1 + 1.2 };
    const bshell = subtract(rbox([BO.x0, BO.y0, BO.z0], [BO.x1, BO.y1, BO.z1], 2), box([BI.x0, BI.y0, BI.z0], [BI.x1, BI.y1, BI.z1]));
    let barBase = intersect(bshell, box([-60, rT - 1, -30], [60, BI.y1, 30]));
    let barLid = intersect(bshell, box([-60, BI.y1, -30], [60, 40, 30]));
    // IMU (ICM-20948 board) on four pads
    const imuX0 = 9.0, imuX1 = imuX0 + 25.7, iy = BI.y0 + 0.8;
    comps.imu = { geom: union(box([imuX0, iy, -8.85], [imuX1, iy + 1.6, 8.85]), box([imuX0 + 0.5, iy + 1.6, -3], [imuX0 + 5, iy + 4.6, 3]), box([imuX1 - 5, iy + 1.6, -3], [imuX1 - 0.5, iy + 4.6, 3]), box([imuX0 + 11, iy + 1.6, -1.5], [imuX0 + 14, iy + 2.5, 1.5])), kind: 'pcbIMU', label: 'ICM-20948 IMU' };
    barBase = union(barBase, ...[[imuX0 + 2, -6.5], [imuX0 + 2, 6.5], [imuX1 - 2, -6.5], [imuX1 - 2, 6.5]].map(([x, z]) => box([x - 1.2, BI.y0 - 0.01, z - 1.2], [x + 1.2, iy, z + 1.2])));
    // stem socket slot in the floor (over the goggle vent)
    barBase = subtract(barBase, box([-7.2, rT - 1, -1.9], [7.2, BI.y0 + 0.1, 1.9]));
    // cable entry, +X end: hole + outside strain-relief cone
    const cy = rT + 4.7;
    barBase = union(barBase, subtract(coneX(4.4, 3.3, BO.x1 - 0.01, BO.x1 + 5, cy, 0), cylX(2.45, BO.x1 - 1, BO.x1 + 6, cy, 0)));
    barBase = subtract(barBase, cylX(2.45, BI.x1 - 1, BO.x1 + 6, cy, 0));
    comps.cable = { geom: cylX(2.3, imuX1 + 0.4, BO.x1 + 12, cy, 0), kind: 'cable', label: 'coil tail' };
    // lid lip + snap bumps / windows
    const c = 0.15, lipH = 2.0, lipT = 0.8;
    const blip = subtract(box([BI.x0 + c, BI.y1 - lipH, BI.z0 + c], [BI.x1 - c, BI.y1 + 0.01, BI.z1 - c]), box([BI.x0 + c + lipT, BI.y1 - lipH - 1, BI.z0 + c + lipT], [BI.x1 - c - lipT, BI.y1 + 1, BI.z1 - c - lipT]));
    // snap tabs 7 mm wide, 3.6 mm long, 0.8 thick, 0.25 bump -> about 3.5 % strain
    const zl = BI.z1 - c, yb = BI.y1, tL = 3.6;
    let blip2 = blip; const btabs = [];
    for (const x of [2, 26]) for (const s of [1, -1]) {
      const zz0 = s > 0 ? zl - lipT : -zl, zz1 = s > 0 ? zl : -zl + lipT;
      blip2 = subtract(blip2, box([x - 4.3, yb - lipH - 1, zz0 - 0.01], [x - 3.5, yb + 0.001, zz1 + 0.01]), box([x + 3.5, yb - lipH - 1, zz0 - 0.01], [x + 4.3, yb + 0.001, zz1 + 0.01]));
      btabs.push(box([x - 3.5, yb - tL, zz0], [x + 3.5, yb + 0.01, zz1]));
    }
    const bb = (x) => prism([[yb - 3.5, zl], [yb - 2.9, zl + 0.25], [yb - 2.4, zl + 0.25], [yb - 2.4, zl]], 'x', x - 3, x + 3);
    const bbs = []; for (const x of [2, 26]) { bbs.push(bb(x), mirrorZ(bb(x))); }
    barLid = subtract(union(barLid, blip2, ...btabs, ...bbs), cylX(2.7, BI.x1 - 2.5, BO.x1 + 1, cy, 0));
    for (const x of [2, 26]) {
      barBase = subtract(barBase, box([x - 3.4, yb - 3.65, BI.z1 - 0.1], [x + 3.4, yb - 2.25, BO.z1 + 0.1]), box([x - 3.4, yb - 3.65, BO.z0 - 0.1], [x + 3.4, yb - 2.25, BI.z0 + 0.1]));
    }

    // ---- screen pod (inside the goggle, hangs under the rail) ----
    // local: x across, y down from the top edge (y<=0.8), z forward (back face z=0, front z=8.8)
    const PO = { x0: -17.3, x1: 17.3, y0: -28.2, y1: 0.8, z0: 0, z1: 8.8 };
    const PI = { x0: -15.9, x1: 15.9, y0: -26.8, y1: -0.6, z0: 1.2, z1: 7.0 };
    const zs = 2.5; // split between back and front
    const pshell = subtract(rbox([PO.x0, PO.y0, PO.z0], [PO.x1, PO.y1, PO.z1], 1.5), box([PI.x0, PI.y0, PI.z0], [PI.x1, PI.y1, PI.z1]));
    let podF = intersect(pshell, box([-30, -40, zs], [30, 10, 20]));
    let podB = intersect(pshell, box([-30, -40, -5], [30, 10, zs]));
    // screen window with outward chamfer
    podF = subtract(podF, box([-12.4, -19.6, PI.z1 - 1], [10.4, -7.6, PO.z1 + 1]), box([-13.4, -20.6, PO.z1 - 0.8], [11.4, -6.6, PO.z1 + 1]));
    // back lip + bumps, windows in the front part
    const plip = subtract(box([PI.x0 + c, PI.y0 + c, zs - 0.01], [PI.x1 - c, PI.y1 - c, zs + 1.9]), box([PI.x0 + c + 0.85, PI.y0 + c + 0.85, zs - 1], [PI.x1 - c - 0.85, PI.y1 - c - 0.85, zs + 3]));
    // side snap tabs: 0.7 thick, reach 4.3 mm forward past the screen's PCB edge, 0.3 bump -> about 2.5 % strain
    const xl = PI.x1 - c, tz1 = 6.8;
    const ptab = s => box([s > 0 ? xl - 0.7 : -xl, -17.6, zs - 0.01], [s > 0 ? xl : -xl + 0.7, -9.6, tz1]);
    let plip2 = subtract(plip, box([PI.x0, -18.4, zs - 1], [PI.x1, -17.6, zs + 3]), box([PI.x0, -9.6, zs - 1], [PI.x1, -8.8, zs + 3]));
    const pb = () => prism([[tz1 - 0.1, xl], [tz1 - 0.8, xl + 0.3], [tz1 - 1.3, xl + 0.3], [tz1 - 1.3, xl]], 'y', -17.1, -10.1);
    podB = union(podB, plip2, ptab(1), ptab(-1), pb(), mirrorX(pb()));
    podF = subtract(podF, box([PI.x1 - 0.1, -17.6, tz1 - 1.5], [PO.x1 + 0.1, -9.6, tz1 + 0.15]), box([PO.x0 - 0.1, -17.6, tz1 - 1.5], [PI.x0 + 0.1, -9.6, tz1 + 0.15]));
    // press ribs on the back plate (clear of the header row)
    podB = union(podB, ...[[-13, -3.6], [13, -3.6], [-13, -24], [13, -24]].map(([x, y]) => box([x - 1, y - 1, PI.z0 - 0.01], [x + 1, y + 1, 2.55])));
    // wire opening in the top wall (back part)
    podB = subtract(podB, box([-2.6, PI.y1 - 0.1, 0.2], [2.6, PO.y1 + 0.1, 2.0]));
    // screen module (local)
    const mod = union(box([-14.85, -25.75, 2.6], [14.85, -1.65, 4.2]), box([-13.4, -20.35, 4.2], [11.4, -6.85, 6.9]), box([-9.5, -4.0, 1.4], [9.5, -1.65, 2.6]));
    // place: tilt about the top-back edge, then hang under the rail
    const T = g => translate([0, -P.podGap - 0.75, -0.5], rotateX(P.tilt * D, g));
    podF = T(podF); podB = T(podB);
    comps.screen = { geom: T(mod), kind: 'pcbBlue', label: '0.96″ screen' };
    // stem: two 1.4 mm spring arms from the pod top up through the vent and the bar floor,
    // 0.4 mm catches click over the floor (about 1 % strain); the gap between the arms is the wire channel
    const sTop = BI.y0 + 1.6;
    const arm = s => box([s > 0 ? 5.6 : -7.0, -P.podGap - 2.5, -1.7], [s > 0 ? 7.0 : -5.6, sTop, 1.7]);
    const wedge = s => prism([[s * 7.0, sTop], [s * 7.0, BI.y0 + 0.15], [s * 7.6, BI.y0 + 0.15], [s * 7.6, BI.y0 + 0.6]], 'z', -1.7, 1.7);
    let stem = union(arm(1), arm(-1), wedge(1), wedge(-1), box([-7.0, -P.podGap - 2.5, -1.7], [7.0, -P.podGap + 0.3, 1.7]));
    stem = subtract(stem, box([-2.6, -P.podGap - 6, -1.0], [2.6, -P.podGap + 1, 1.0]));
    stem = subtract(stem, T(box([PI.x0, PI.y0, PI.z0], [PI.x1, PI.y1, PI.z1])), podF);
    podB = union(podB, stem);

    // context: goggle frame rail with the vent (not printed)
    const rail = subtract(box([-45, 0, -P.railD / 2], [45, rT, P.railD / 2]), box([-P.ventL / 2, -1, -P.ventW / 2], [P.ventL / 2, rT + 1, P.ventW / 2]));

    return {
      parts: {
        bar_base: { geom: barBase, material: 'PETG', frame: 'hud', label: 'Frame bar base', print: 'floor down' },
        bar_lid: { geom: barLid, material: 'PETG', frame: 'hud', label: 'Frame bar lid', print: 'top face down' },
        pod_front: { geom: podF, material: 'PETG', frame: 'hud', label: 'Screen pod front', print: 'window face down' },
        pod_back: { geom: podB, material: 'PETG', frame: 'hud', label: 'Screen pod back + stem', print: 'back face down, stem up' },
      },
      comps, rail, glandOut: [BO.x1 + 5, cy, 0],
      dims: { bar: [BO.x1 - BO.x0, BO.y1 - BO.y0, BO.z1 - BO.z0], pod: [PO.x1 - PO.x0, PO.y1 - PO.y0, PO.z1 - PO.z0] },
      explode: { bar_lid: [0, 12, 0], pod_front: [0, -6, 14], pod_back: [0, -6, 0] },
    };
  }

  // ---------- checks ----------
  function bbOverlap(a, b) { return a[0][0] < b[1][0] && a[1][0] > b[0][0] && a[0][1] < b[1][1] && a[1][1] > b[0][1] && a[0][2] < b[1][2] && a[1][2] > b[0][2]; }
  function collisions(items) {
    const out = [];
    const bbs = items.map(i => measureBoundingBox(i.geom));
    for (let i = 0; i < items.length; i++) for (let j = i + 1; j < items.length; j++) {
      if (items[i].skip && items[i].skip.includes(items[j].name)) continue;
      if (items[j].skip && items[j].skip.includes(items[i].name)) continue;
      if (!bbOverlap(bbs[i], bbs[j])) continue;
      const v = measureVolume(intersect(items[i].geom, items[j].geom));
      if (v > 0.05) out.push({ a: items[i].name, b: items[j].name, v: +v.toFixed(2) });
    }
    return out;
  }

  function build(params) {
    const P = Object.assign({}, DEFAULTS, params || {});
    const pack = buildPack(P), hud = buildHud(P);
    const parts = Object.assign({}, pack.parts, hud.parts);
    for (const k in parts) {
      parts[k].volume = measureVolume(parts[k].geom);
      const bb = measureBoundingBox(parts[k].geom); parts[k].size = [0, 1, 2].map(i => +(bb[1][i] - bb[0][i]).toFixed(1));
    }
    const list = (o, pre) => Object.keys(o).map(k => ({ name: pre + k, geom: o[k].geom }));
    const packItems = list(pack.parts, '').concat(list(pack.comps, 'c:'));
    // the coil intentionally runs through the gland; the cap nub touches the plunger
    const hudItems = list(hud.parts, '').concat(list(hud.comps, 'c:'), [{ name: 'ctx:rail', geom: hud.rail }]);
    const coll = collisions(packItems).concat(collisions(hudItems));
    return { P, parts, pack, hud, collisions: coll };
  }

  window.SkiCAD = { build, DEFAULTS, toPolygons: g => J.geometries.geom3.toPolygons(g) };
})();
