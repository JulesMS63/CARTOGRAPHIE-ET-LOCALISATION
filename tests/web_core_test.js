// Chaîne complète sur les 4 scans simulés (data_synth) : segments, carte vectorielle, grille d'occupation.
// Usage : node tests/web_core_test.js lidar_web.html data_synth
const fs = require('fs'), assert = require('assert');
const html = fs.readFileSync(process.argv[2], 'utf8');
const core = html.split('/* CORE-START */')[1].split('/* CORE-END */')[0];
const api = new Function(core + `; return {makeScan, localPoints, worldPoints, parseScanText, extractSegments, VectorMap, segToWorld,
  gridForScans, Seg, cutBy, intersection, angleBetween, areClose, mergeSeg, LidarParser, vectorMapToTxt, scanToCSV, makeZipParts, D2R};`)();
const dir = process.argv[3] || 'data_synth';
const poses = JSON.parse(fs.readFileSync(dir + '/poses_true.json'));
const scans = [];
for (const name of Object.keys(poses)) {
  const s = api.parseScanText(fs.readFileSync(`${dir}/${name}.csv`, 'utf8'), name)[0];
  s.pose = { x: poses[name][0], y: poses[name][1], th: poses[name][2] };
  scans.push(s);
}
const out = { perScan: {}, vec: null, grid: null };
const vm = new api.VectorMap();
for (const s of scans) {
  const segs = api.extractSegments(api.localPoints(s));
  out.perScan[s.name] = { n: s.ranges.length, nseg: segs.length };
  vm.addSegments(segs.map(sg => api.segToWorld(sg, s.pose)));
}
vm.prune(1);
out.vec = vm.segments.map(s => [s.x1, s.y1, s.x2, s.y2].map(v => +v.toFixed(4)));
const g = api.gridForScans(scans, 0.05);
for (const s of scans) g.update(s, s.pose);
const c = g.classify();
out.grid = { nx: g.nx, ny: g.ny, occ: c.filter(v => v === 1).length, free: c.filter(v => v === 0).length };
assert.strictEqual(out.vec.length, 11, 'carte vectorielle : 11 segments');
assert.deepStrictEqual([out.grid.nx, out.grid.ny], [142, 102], 'grille 142 x 102 cellules');
assert.strictEqual(out.grid.occ, 453, 'cellules occupées'); assert.strictEqual(out.grid.free, 8993, 'cellules libres');
assert(Object.values(out.perScan).every(s => s.nseg >= 10 && s.nseg <= 20), 'segments par scan plausibles');
console.log('web_core_test : OK (' + out.vec.length + ' segments, ' + out.grid.occ + ' cellules occupées, ' + out.grid.free + ' libres)');
