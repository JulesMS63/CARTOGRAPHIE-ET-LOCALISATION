// Tests du noyau de localisation (Jour 2) : node loc_test.js lidar_web.html
const fs = require('fs'), assert = require('assert');
const html = fs.readFileSync(process.argv[2], 'utf8');
const core = html.split('/* CORE-START */')[1].split('/* CORE-END */')[0];
const A = new Function(core + `; return {benchDataset, BENCH_POSES, prepareLocalisation, localise, locAvailable, LOC_TYPES, poseError, offsetPose, randomOffset, makeRng, summarize, isSuccess, distanceTransform};`)();
let n = 0; const ok = (c, m) => { assert(c, m); n++; };
const thr = { pos: 0.05, ang: 2 };

// --- générateur pseudo-aléatoire reproductible
const r1 = A.makeRng(5), r2 = A.makeRng(5), r3 = A.makeRng(6);
const s1 = [r1(), r1(), r1()], s2 = [r2(), r2(), r2()], s3 = [r3(), r3(), r3()];
ok(JSON.stringify(s1) === JSON.stringify(s2), 'même graine -> même suite');
ok(JSON.stringify(s1) !== JSON.stringify(s3), 'graines différentes -> suites différentes');
ok(s1.every(v => v >= 0 && v < 1), 'valeurs dans [0,1[');

// --- erreurs de pose
let e = A.poseError({ x: 1, y: 1, th: 179 }, { x: 1, y: 1, th: -179 });
ok(Math.abs(e.ang - 2) < 1e-9 && e.pos === 0, 'écart angulaire avec rebouclage ±180°');
e = A.poseError({ x: 3, y: 4, th: 0 }, { x: 0, y: 0, th: 0 }); ok(Math.abs(e.pos - 5) < 1e-12, 'erreur de position euclidienne');
ok(A.poseError({ x: NaN, y: 0, th: 0 }, { x: 0, y: 0, th: 0 }).pos === Infinity, 'NaN -> échec (Infinity)');
const o = A.offsetPose({ x: 1, y: 2, th: 170 }, 0.1, -0.2, 20); ok(Math.abs(o.th + 170) < 1e-9 && Math.abs(o.x - 1.1) < 1e-12, 'pose décalée, angle rebouclé');
const rr = A.makeRng(9);
for (let i = 0; i < 20; i++) { const f = A.randomOffset(0.4, 12, rr); ok(Math.abs(Math.hypot(f.dx, f.dy) - 0.4) < 1e-9 && Math.abs(f.dth) === 12, 'décalage de norme imposée'); }

// --- seuils et statistiques
ok(A.isSuccess({ pos: 0.05, ang: 2 }, thr) && !A.isSuccess({ pos: 0.051, ang: 0 }, thr) && !A.isSuccess({ pos: 0, ang: 2.1 }, thr), 'seuils inclusifs, position ET angle');
const S = A.summarize([{ pos: 0.01, ang: 0.5, ms: 2 }, { pos: 0.2, ang: 0.5, ms: 4 }, { pos: 0.02, ang: 5, ms: 6 }, { pos: 0.03, ang: 1, ms: 8 }], thr);
ok(S.n === 4 && S.success === 2 && S.rate === 0.5 && S.meanMs === 5 && S.maxMs === 8, 'résumé : taux, temps moyen/max');
ok(Math.abs(S.medPos - 0.025) < 1e-12 && S.medPosOk === 0.02, 'médianes (toutes / réussies)');
ok(isNaN(A.summarize([], thr).rate), 'lot vide -> NaN, pas de division par zéro');

// --- champ de distance exact (comparaison avec le calcul naïf)
{
  const nx = 23, ny = 17, mask = new Uint8Array(nx * ny); const rg = A.makeRng(11);
  for (let i = 0; i < 9; i++) mask[Math.floor(rg() * nx * ny)] = 1;
  const d = A.distanceTransform(mask, nx, ny, 0.1);
  const pts = []; for (let i = 0; i < mask.length; i++) if (mask[i]) pts.push([i % nx, Math.floor(i / nx)]);
  let worst = 0;
  for (let j = 0; j < ny; j++) for (let i = 0; i < nx; i++) { const ref = 0.1 * Math.min(...pts.map(p => Math.hypot(p[0] - i, p[1] - j))); worst = Math.max(worst, Math.abs(d[j * nx + i] - ref)); }
  ok(worst < 1e-6, 'transformée en distance exacte (écart max ' + worst.toExponential(1) + ')');
}

// --- banc simulé : déterminisme et vérité terrain
const D1 = A.benchDataset(7), D2 = A.benchDataset(7), D3 = A.benchDataset(8);
ok(D1.length === A.BENCH_POSES.length && D1.length === 8, '8 poses connues');
ok(JSON.stringify(D1[0].scan.ranges) === JSON.stringify(D2[0].scan.ranges), 'banc reproductible (même graine)');
ok(JSON.stringify(D1[0].scan.ranges) !== JSON.stringify(D3[0].scan.ranges), 'graine différente -> bruit différent');
ok(D1.every((d, i) => d.truth.x === A.BENCH_POSES[i][0] && d.truth.th === A.BENCH_POSES[i][2]), 'vérité = poses connues');

// --- localisation : tous les types et toutes les méthodes, petit écart
const idx = 2, test = D1[idx], refs = D1.filter((_, i) => i !== idx).map(d => ({ name: d.name, scan: d.scan, pose: d.truth }));
const L = A.prepareLocalisation(refs, { scan: test.scan, truth: test.truth }, { res: 0.05 });
ok(L.refScan.name !== test.name, 'le scan testé n\'est jamais dans sa propre référence');
ok(refs.length === 7 && L.refs === 7, 'carte construite avec les 7 autres scans');
for (const t of Object.keys(A.LOC_TYPES)) ok(A.locAvailable(L, t), 'référence disponible : ' + t);
ok(!A.locAvailable(A.prepareLocalisation([], { scan: test.scan, truth: test.truth }), 'grid'), 'sans autre scan : pas de grille');
let count = 0;
for (const t of Object.keys(A.LOC_TYPES)) for (const [m] of A.LOC_TYPES[t].methods) {
  const rows = [], rg = A.makeRng(21);
  for (let k = 0; k < 6; k++) {
    const f = A.randomOffset(0.1, 3, rg), init = A.offsetPose(test.truth, f.dx, f.dy, f.dth);
    const r = A.localise(L, t, m, init, { seed: 50 + k });
    ok(isFinite(r.ms) && r.ms >= 0 && r.pose && isFinite(r.pose.x), t + '/' + m + ' renvoie une pose et un temps');
    rows.push({ ...A.poseError(r.pose, test.truth), ms: r.ms });
  }
  const s = A.summarize(rows, thr); count++;
  ok(s.rate === 1, t + '/' + m + ' réussit à 0,1 m / 3° (taux ' + s.rate + ', pos méd ' + s.medPos.toFixed(3) + ')');
}
ok(count === 14, '14 méthodes testées');

// --- départ parfait : ne doit pas dégrader la pose
for (const [t, m] of [['grid', 'gd'], ['vec', 'icp'], ['scan', 'icp'], ['scan', 'icpl']]) {
  const r = A.localise(L, t, m, test.truth); const er = A.poseError(r.pose, test.truth);
  ok(er.pos < 0.03 && er.ang < 1.5, t + '/' + m + ' reste sur la vérité (' + er.pos.toFixed(3) + ' m)');
}
// --- reproductibilité du filtre à particules
const init = A.offsetPose(test.truth, 0.3, -0.2, 8);
const a = A.localise(L, 'grid', 'mc', init, { seed: 4 }), b = A.localise(L, 'grid', 'mc', init, { seed: 4 }), c = A.localise(L, 'grid', 'mc', init, { seed: 5 });
ok(a.pose.x === b.pose.x && a.pose.th === b.pose.th, 'filtre à particules : même graine -> même résultat');
ok(c.pose.x !== a.pose.x || c.pose.th !== a.pose.th, 'filtre à particules : autre graine -> autre tirage');
// --- limite connue de la force brute : hors fenêtre, elle échoue ; le filtre à particules est plus tolérant
const far = A.offsetPose(test.truth, 0.9, 0, 0), win = { xy: 0.5, th: 15 };
const eb = A.poseError(A.localise(L, 'grid', 'brute', far, { win }).pose, test.truth);
ok(eb.pos > 0.3, 'force brute : vérité hors fenêtre -> échec (erreur ' + eb.pos.toFixed(2) + ' m)');
const em = A.poseError(A.localise(L, 'grid', 'mc', far, { win, seed: 3 }).pose, test.truth);
ok(em.pos < 0.1, 'filtre à particules : récupère un écart de 0,9 m (erreur ' + em.pos.toFixed(3) + ' m)');
try { A.localise(L, 'grid', 'xyz', init); ok(false, 'méthode inconnue'); } catch (x) { ok(/inconnue/.test(x.message), 'méthode inconnue -> erreur explicite'); }
console.log('loc_test : ' + n + ' vérifications OK');
