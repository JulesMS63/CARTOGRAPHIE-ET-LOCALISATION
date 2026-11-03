// Tests de la localisation sur une grille importée : node web_grid_test.js lidar_web.html
const fs = require('fs'), assert = require('assert');
const html = fs.readFileSync(process.argv[2], 'utf8');
const core = html.split('/* CORE-START */')[1].split('/* CORE-END */')[0];
const A = new Function(core + '; return {benchDataset, gridForScans, gridToCSV, parseGridCsv, makeRefFromGrid, locateOnGrid, poseError, gridMeta, gridFileName, metaFromGridName, parseVecTxt, makeRefFromSegments, VectorMap, extractSegments, localPoints, segToWorld, vectorMapToTxt};')();
let n = 0; const ok = (c, m) => { assert(c, m); n++; };

// --- lecture d'une grille : rangée du haut en premier, -1 inconnu / 0 libre / ≥1 occupé
const g0 = A.parseGridCsv('# commentaire\n-1,0,1\n0,0,1\n1,1,1\n');
ok(g0.nx === 3 && g0.ny === 3 && g0.occ === 5 && g0.free === 3, 'grille 3×3 lue : 5 occupés, 3 libres');
ok(g0.cls[0 * 3 + 0] === 1 && g0.cls[2 * 3 + 0] === -1, 'la dernière rangée du fichier est le bas (y = 0), la première le haut');
ok(A.parseGridCsv('en-tête,texte\n0,0,100\n0,100,0\n-1,0,0\n').occ === 2, 'une ligne d’en-tête est ignorée ; 100 = occupé (format ROS)');
let err = ''; try { A.parseGridCsv('a,b\nc,d\n'); } catch (e) { err = e.message; } ok(/grille/.test(err), 'fichier sans grille : message clair');
err = ''; try { A.parseGridCsv('0,1,0\n0,1\n1,1,1\n'); } catch (e) { err = e.message; } ok(/même longueur/.test(err), 'lignes de longueurs différentes : refusé');

// --- nom de fichier d'export : l'origine et le carreau s'y retrouvent
const data = A.benchDataset(7, { noise: 0.01, outliers: 0.03 });
const maps = data.slice(0, 4).map(m => Object.assign({}, m.scan, { pose: m.truth }));
const g = A.gridForScans(maps, 0.05, 0.5); maps.forEach(s => g.update(s, s.pose));
const name = A.gridFileName(g), meta = A.metaFromGridName(name);
ok(Math.abs(meta.res_m - 0.05) < 1e-9 && Math.abs(meta.x0 - g.xmin) < 0.006 && Math.abs(meta.y0 - g.ymin) < 0.006, 'nom de fichier ' + name + ' : carreau et origine relus');
ok(A.metaFromGridName('grille.csv') === null && A.gridMeta(g).nx === g.nx, 'nom sans origine : rien d’inventé ; grille.json complet');

// --- localiser des scans qui n'ont PAS servi à construire la grille
const P = A.parseGridCsv(A.gridToCSV(g)), ref = A.makeRefFromGrid(P, meta.res_m, g.xmin, g.ymin);
ok(ref && ref.fromGrid && ref.field && ref.P.length > 100, 'référence construite depuis la grille');
let okAll = 0, okNear = 0, tot = 0;
for (const t of data.slice(4)) {
  tot++;
  const r = A.locateOnGrid(ref, t.scan), e = r.pose ? A.poseError(r.pose, t.truth) : { pos: 9, ang: 9 };
  ok(r.verdict !== 'echec' && e.pos < 0.05 && e.ang < 2, t.name + ' : toute la grille, x/y à ' + (100 * e.pos).toFixed(1) + ' cm, θ à ' + e.ang.toFixed(1) + '°');
  okAll++;
  const r2 = A.locateOnGrid(ref, t.scan, { near: { x: t.truth.x + 0.9, y: t.truth.y - 0.7, radius: 1.5 } }), e2 = r2.pose ? A.poseError(r2.pose, t.truth) : { pos: 9, ang: 9 };
  ok(e2.pos < 0.05 && e2.ang < 2 && r2.verdict === 'bonne', t.name + ' : près d’une position approximative, x/y à ' + (100 * e2.pos).toFixed(1) + ' cm');
  okNear++;
}
// --- carte vectorielle : fichier de segments, puis localisation
const vm = new A.VectorMap({ angTolDeg: 10, distTol: 0.06, gapTol: 0.15 });
data.slice(0, 4).forEach(m => vm.addSegments(A.extractSegments(A.localPoints(m.scan), {}).map(sg => A.segToWorld(sg, m.truth)))); vm.prune(1);
const segs = A.parseVecTxt(A.vectorMapToTxt(vm.segments));
ok(segs.length === vm.segments.length && segs.length >= 8, 'carte vectorielle relue : ' + segs.length + ' segments');
let e3 = ''; try { A.parseVecTxt('# rien\n1,2\n'); } catch (e) { e3 = e.message; } ok(/segments/.test(e3), 'fichier sans segments : message clair');
const vref = A.makeRefFromSegments(segs);
ok(vref && vref.fromVec && vref.N.length === vref.P.length, 'référence construite depuis les segments');
for (const t of data.slice(4)) {
  const r = A.locateOnGrid(vref, t.scan), e = r.pose ? A.poseError(r.pose, t.truth) : { pos: 9, ang: 9 };
  ok(r.verdict !== 'echec' && e.pos < 0.05 && e.ang < 2, t.name + ' : carte vectorielle, partout, x/y à ' + (100 * e.pos).toFixed(1) + ' cm, θ à ' + e.ang.toFixed(1) + '°');
  const r2 = A.locateOnGrid(vref, t.scan, { near: { x: t.truth.x - 0.8, y: t.truth.y + 0.6, radius: 1.5 } }), e2 = r2.pose ? A.poseError(r2.pose, t.truth) : { pos: 9, ang: 9 };
  ok(e2.pos < 0.05 && e2.ang < 2, t.name + ' : carte vectorielle, près d’une position, x/y à ' + (100 * e2.pos).toFixed(1) + ' cm');
}
ok(A.makeRefFromSegments([{ x1: 0, y1: 0, x2: 0.1, y2: 0 }]) === null, 'trop peu de murs : pas de référence');
// --- une grille vide ou presque est refusée proprement
ok(A.makeRefFromGrid({ nx: 10, ny: 10, cls: new Int8Array(100) }, 0.05, 0, 0) === null, 'grille sans mur : pas de référence');
console.log('grid_test : ' + n + ' vérifications OK');
