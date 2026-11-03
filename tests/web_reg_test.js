// Tests du recalage automatique (retrouver la pose d'un scan à partir d'un scan de référence) : node reg_test.js lidar_web.html
const fs = require('fs'), assert = require('assert');
const html = fs.readFileSync(process.argv[2], 'utf8');
const core = html.split('/* CORE-START */')[1].split('/* CORE-END */')[0];
const A = new Function(core + `; return {simulateScan, benchDataset, makeRefFromScans, registerScan, poseError, offsetPose, randomOffset, makeRng, densify, validPts, polarBins, visConflicts, trackerNew, trackerStep, composePose, invPose, SIM_WALLS, wrapDeg};`)();
let n = 0; const ok = (c, m) => { assert(c, m); n++; };
const data = A.benchDataset(7, { noise: 0.01, outliers: 0.03 });
const refOf = (idx) => A.makeRefFromScans(idx.map(i => ({ scan: data[i].scan, pose: data[i].truth })));

// --- densification : les points sont reliés, les trous trop grands ne le sont pas
const d = A.densify(Float64Array.from([0, 0, 0.2, 0, 2, 0]), 0.25, 0.05);
ok(d.length / 2 > 3 && d[d.length - 2] === 2 && d.every(v => isFinite(v)), 'densification : points ajoutés entre voisins proches');
ok(Math.max(...Array.from({ length: d.length / 2 }, (_, i) => d[2 * i]).filter(x => x <= 0.2)) <= 0.2 + 1e-12, 'densification : pas de pont au-delà de 25 cm');
ok(A.validPts(Float64Array.from([NaN, 0, 1, 1, 0.001, 0.001])).length === 2, 'points non finis ou collés au capteur retirés');

// --- la référence est réutilisable pour un ou plusieurs scans
const r1 = refOf([0]), rAll = refOf([0, 1, 2, 3, 4, 5, 6]);
ok(r1.n === 1 && rAll.n === 7 && rAll.P.length > r1.P.length, 'référence fusionnée (7 scans)');
ok(A.makeRefFromScans([]) === null, 'référence vide -> null');

// --- avec une position approximative, la bonne pose est retrouvée (1 m / 30°, bruit 1 cm, 3 % de parasites)
const rng = A.makeRng(5); let okCount = 0, total = 0;
for (let t = 1; t < data.length; t++) {
  const off = A.randomOffset(1, 30, rng), g = A.offsetPose(data[t].truth, off.dx, off.dy, off.dth);
  const res = A.registerScan(refOf([0]), data[t].scan, { center: g, radius: 1.3, thRange: 39 });
  const e = A.poseError(res.pose, data[t].truth); total++; if (e.pos <= 0.05 && e.ang <= 2) okCount++;
  ok(res.verdict !== 'echec' && res.inlierFrac > 0.3 && isFinite(res.ms), 'scan ' + t + ' : verdict et recouvrement renseignés');
}
ok(okCount === total, 'recalage « autour de la position saisie » : ' + okCount + '/' + total + ' retrouvés à 5 cm et 2°');
// --- plusieurs références
{
  const t = 3, g = A.offsetPose(data[t].truth, 0.6, -0.4, 15);
  const res = A.registerScan(refOf([0, 1, 2, 4, 5, 6, 7]), data[t].scan, { center: g, radius: 1, thRange: 30 });
  const e = A.poseError(res.pose, data[t].truth); ok(e.pos < 0.03 && e.ang < 1.5 && res.verdict === 'bonne', 'avec 7 scans de référence : ' + e.pos.toFixed(3) + ' m');
}
// --- recherche large : la vraie position est toujours parmi les propositions, et un résultat faux n'est jamais « fiable »
for (const [r, t] of [[0, 4], [1, 5], [5, 4], [4, 2], [7, 4], [6, 5]]) {
  const res = A.registerScan(refOf([r]), data[t].scan, { center: { x: data[r].truth.x, y: data[r].truth.y, th: 0 }, radius: 6, thRange: 180, limits: { ambig: 0.6, altInl: 0.6 } });
  const errs = res.candidates.map(c => A.poseError(c.pose, data[t].truth)), hit = errs.findIndex(e => e.pos <= 0.05 && e.ang <= 2);
  ok(hit >= 0 && hit <= 1, 'recherche large ' + r + '->' + t + ' : bonne pose parmi les 2 premières (rang ' + hit + ')');
  ok(hit === 0 || res.verdict !== 'bonne', 'recherche large ' + r + '->' + t + ' : résultat faux jamais annoncé fiable');
}
// --- pas de recouvrement : pas de faux positif « fiable »
{
  const far = { scan: data[0].scan, pose: { x: 40, y: 40, th: 0 } }, ref = A.makeRefFromScans([far]);
  const res = A.registerScan(ref, data[3].scan, { center: { x: 2, y: 2, th: 0 }, radius: 1.5, thRange: 45 });
  ok(res.verdict === 'echec' && res.ok === false, 'aucun recoupement -> échec (' + res.verdict + ')');
  const tiny = { ...data[0].scan, ranges: data[0].scan.ranges.slice(0, 10), angles: data[0].scan.angles.slice(0, 10) };
  ok(A.registerScan(refOf([1]), tiny, { center: data[0].truth }).verdict === 'echec', 'scan trop pauvre -> échec explicite');
}
// --- reproductible
const a = A.registerScan(refOf([0]), data[2].scan, { center: A.offsetPose(data[2].truth, 0.5, 0.3, 10), radius: 1, thRange: 30 });
const b = A.registerScan(refOf([0]), data[2].scan, { center: A.offsetPose(data[2].truth, 0.5, 0.3, 10), radius: 1, thRange: 30 });
ok(a.pose.x === b.pose.x && a.pose.th === b.pose.th, 'résultat identique à entrée identique');
// --- en chaîne (robot qui avance et scanne) : chaque scan est recalé sur le précédent, sans dérive notable
{
  const rg = A.makeRng(11); let pose = { x: 1, y: 1, th: 0 }; const truth = [];
  for (let i = 0; i < 14; i++) {
    truth.push({ ...pose });
    let nx = pose.x + 0.6 * Math.cos(pose.th * Math.PI / 180), ny = pose.y + 0.6 * Math.sin(pose.th * Math.PI / 180), th = pose.th;
    if (nx < 0.6 || nx > 5.4 || ny < 0.6 || ny > 3.4) { th += 35; nx = pose.x; ny = pose.y; } else th += 12 * (rg() - 0.3);
    pose = { x: nx, y: ny, th };
  }
  const scans = truth.map((p, i) => A.simulateScan('s' + i, p, rg, { noise: 0.01, outliers: 0.03 }));
  const est = [truth[0]]; let allGood = true;
  for (let i = 1; i < scans.length; i++) {
    const res = A.registerScan(A.makeRefFromScans([{ scan: scans[i - 1], pose: est[i - 1] }]), scans[i], { center: est[i - 1], radius: 1.5, thRange: 45 });
    est.push(res.pose); allGood = allGood && res.verdict === 'bonne';
  }
  const e = A.poseError(est[est.length - 1], truth[truth.length - 1]);
  ok(allGood, 'chaîne de 14 scans : tous les liens « fiables »');
  ok(e.pos < 0.05 && e.ang < 2, 'chaîne de 14 scans : erreur finale ' + e.pos.toFixed(3) + ' m / ' + e.ang.toFixed(1) + '°');
  // un scan hors de la fenêtre de recherche n'est jamais annoncé « fiable » à tort
  const far = A.registerScan(A.makeRefFromScans([{ scan: scans[0], pose: truth[0] }]), scans[6], { center: truth[0], radius: 0.5, thRange: 10 });
  ok(!far.pose || A.poseError(far.pose, truth[6]).pos <= 0.05 || far.verdict !== 'bonne', 'hors fenêtre : pas de faux « fiable »');
  ok(!far.pose || (Math.hypot(far.pose.x - truth[0].x, far.pose.y - truth[0].y) <= 0.5 * 1.15 + 1e-9 && Math.abs(A.wrapDeg(far.pose.th - truth[0].th)) <= 11.5 + 1e-9), 'la pose trouvée ne sort jamais de la fenêtre demandée');
}

// --- suivi par carte locale (N derniers scans) : obstacle qui bouge, pas de 0,8 m, plusieurs graines
function trajectory(seed, steps, step) {
  const rg = A.makeRng(seed); let p = { x: 1, y: 1, th: 0 }; const T = [];
  for (let i = 0; i < steps; i++) {
    T.push({ ...p });
    let nx = p.x + step * Math.cos(p.th * Math.PI / 180), ny = p.y + step * Math.sin(p.th * Math.PI / 180), th = p.th;
    if (nx < 0.6 || nx > 5.4 || ny < 0.6 || ny > 3.4) { th += 35; nx = p.x; ny = p.y; } else th += 12 * (rg() - 0.3);
    p = { x: nx, y: ny, th };
  }
  return T;
}
function dynScans(T, seed, noise, outl) {                           // une boîte de 0,5 m posée ailleurs à chaque scan (jamais sur le capteur)
  const rg = A.makeRng(seed + 100), h = 0.25;
  return T.map((p, i) => {
    let cx, cy; do { cx = 1 + 4 * rg(); cy = 0.8 + 2.4 * rg(); } while (Math.hypot(cx - p.x, cy - p.y) < 0.7);
    const walls = A.SIM_WALLS.concat([[cx - h, cy - h, cx + h, cy - h], [cx + h, cy - h, cx + h, cy + h], [cx + h, cy + h, cx - h, cy + h], [cx - h, cy + h, cx - h, cy - h]]);
    return A.simulateScan('d' + i, p, rg, { noise, outliers: outl, walls });
  });
}
function track(T, scans, N) {
  const tr = A.trackerNew({ N }); let bad = 0;
  scans.forEach((s, i) => { const r = A.trackerStep(tr, s, i === 0 ? T[0] : null); if (r.verdict !== 'bonne' && r.verdict !== 'base') bad++; });
  const e = tr.items.map((it, i) => A.poseError(it.pose, T[i]).pos);
  return { rms: Math.sqrt(e.reduce((a, b) => a + b * b, 0) / e.length), max: Math.max(...e), bad };
}
{
  const res = {};
  for (const seed of [11, 12, 13]) {
    const T = trajectory(seed, 30, 0.5), sc = dynScans(T, seed, 0.01, 0.03);
    res[seed] = { n1: track(T, sc, 1), n5: track(T, sc, 5) };
    ok(res[seed].n5.rms < 0.03 && res[seed].n5.max < 0.06, 'carte locale N=5, obstacle mobile, graine ' + seed + ' : erreur ' + (100 * res[seed].n5.rms).toFixed(1) + ' cm (max ' + (100 * res[seed].n5.max).toFixed(1) + ' cm)');
  }
  const mean = k => [11, 12, 13].reduce((a, s) => a + res[s][k].rms, 0) / 3;
  ok(mean('n5') <= mean('n1') * 1.05, 'la carte locale (N=5) est au moins aussi précise que le scan précédent seul');
  // pas de 0,8 m, bruit 2 cm, 6 % de parasites : le scan précédent seul décroche, la carte locale tient
  const T = trajectory(12, 40, 0.8), sc = dynScans(T, 12, 0.02, 0.06), a = track(T, sc, 1), b = track(T, sc, 5);
  ok(b.rms < 0.05 && b.max < 0.10, 'pas de 0,8 m + obstacle mobile : carte locale ' + (100 * b.rms).toFixed(1) + ' cm, scan précédent seul ' + (100 * a.rms).toFixed(0) + ' cm');
  ok(b.rms <= a.rms, 'la carte locale ne fait jamais pire que le scan précédent seul');
}
// --- départage de deux positions aussi bonnes par le mouvement prévu
{
  const T = trajectory(12, 30, 0.5), sc = dynScans(T, 12, 0.01, 0.03), est = [T[0]];
  const tr = A.trackerNew({ N: 3 });
  sc.forEach((s, i) => A.trackerStep(tr, s, i === 0 ? T[0] : null));
  const e = tr.items.map((it, i) => A.poseError(it.pose, T[i]).pos);
  ok(Math.max(...e) < 0.06, 'N=3, graine 12 (deux positions presque aussi bonnes au lien 23) : le mouvement prévu départage, erreur max ' + (100 * Math.max(...e)).toFixed(1) + ' cm');
}
console.log('reg_test : ' + n + ' vérifications OK');
