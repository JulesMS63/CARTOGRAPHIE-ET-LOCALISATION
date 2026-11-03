// Tests du recalage automatique (retrouver la pose d'un scan à partir d'un scan de référence) : node reg_test.js lidar_web.html
const fs = require('fs'), assert = require('assert');
const html = fs.readFileSync(process.argv[2], 'utf8');
const core = html.split('/* CORE-START */')[1].split('/* CORE-END */')[0];
const A = new Function(core + `; return {simulateScan, benchDataset, makeRefFromScans, registerScan, poseError, offsetPose, randomOffset, makeRng, densify, validPts, polarBins, visConflicts};`)();
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
  const ef = A.poseError(far.pose, truth[6]);
  ok(ef.pos <= 0.05 || far.verdict !== 'bonne', 'hors fenêtre : pas de faux « fiable »');
}
console.log('reg_test : ' + n + ' vérifications OK');
