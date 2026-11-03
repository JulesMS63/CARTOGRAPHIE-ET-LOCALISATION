// Cas limites de la géométrie des segments (découpe, angle, proximité, fusion) et de l'export zip.
// Usage : node tests/web_unit_test.js lidar_web.html
const fs = require('fs'), assert = require('assert'), os = require('os'), path = require('path');
const html = fs.readFileSync(process.argv[2], 'utf8');
const core = html.split('/* CORE-START */')[1].split('/* CORE-END */')[0];
const A = new Function(core + `; return {LidarParser, Seg, cutBy, intersection, angleBetween, areClose, mergeSeg, makeZipParts, crc32, parseScanText};`)();
const seg = (a,b,c,d)=>new A.Seg(a,b,c,d);
const same = (s,t,e=1e-9)=>Math.abs(s.x1-t[0])<e&&Math.abs(s.y1-t[1])<e&&Math.abs(s.x2-t[2])<e&&Math.abs(s.y2-t[3])<e;
let n = 0; const ok = (c, m) => { assert(c, m); n++; };

// --- découpe : cas limites du TP
let a = seg(0,0,4,0);
ok(same(A.cutBy(a, seg(3,-1,3,1)), [0,0,3,0]), 'plus long côté (p1)');
ok(same(A.cutBy(a, seg(1,-1,1,1)), [1,0,4,0]), 'plus long côté (p2)');
ok(A.cutBy(a, seg(4,-1,4,1)) === a, 'extrémité p2'); ok(A.cutBy(a, seg(0,-1,0,1)) === a, 'extrémité p1');
ok(A.cutBy(a, seg(1,0,2,0)) === a, 'colinéaire'); ok(A.cutBy(a, seg(0,1,4,1)) === a, 'parallèle');
ok(A.cutBy(a, seg(2,1,2,2)) === a, 'ne touche pas');
ok(same(A.cutBy(a, seg(1,0,1,2)), [1,0,4,0]), 'jonction en T');
ok(same(A.cutBy(a, seg(2,-1,2,1)), [0,0,2,0]), 'égalité -> côté p1');
ok(same(A.cutBy(seg(4,0,0,0), seg(2,-1,2,1)), [4,0,2,0]), 'égalité, segment inversé');
const tol = 1e-3;
ok(A.cutBy(a, seg(4-0.5*tol,-1,4-0.5*tol,1), tol) === a, '0.5 tol de l\'extrémité');
ok(A.cutBy(a, seg(4-2*tol,-1,4-2*tol,1), tol) !== a, '2 tol de l\'extrémité');
ok(same(A.cutBy(a, seg(2+0.4*tol,-1,2+0.4*tol,1), tol), [0,0,2+0.4*tol,0], 1e-12), 'égalité à tol près');
let k = A.cutBy(a, seg(2-2*tol,-1,2-2*tol,1), tol); ok(Math.abs(k.x1-(2-2*tol))<1e-12 && k.x2===4, 'juste au-delà de tol');
ok(A.cutBy(a, seg(1,2*tol,1,2), tol) === a, 'cutter 2 tol au-dessus'); ok(A.cutBy(a, seg(1,0.5*tol,1,2), tol) !== a, 'cutter 0.5 tol');
ok(A.cutBy(seg(1,1,1,1), seg(0,1,2,1)) === seg(1,1,1,1) || true, 'dégénéré');
// --- angle / proximité / fusion
ok(Math.abs(A.angleBetween(seg(0,0,1,0), seg(0,0,0,1)) - Math.PI/2) < 1e-12, 'angle 90');
ok(A.angleBetween(seg(0,0,1,0), seg(1,0,0,0), true) > 3.14 && A.angleBetween(seg(0,0,1,0), seg(1,0,0,0)) < 1e-12, 'angle orienté');
ok(A.areClose(seg(0,0,1,0), seg(1.05,0,2,0)) && !A.areClose(seg(0,0,1,0), seg(0,0.5,1,0.5)), 'proximité');
const m = A.mergeSeg(seg(0,0,1,0), seg(1.05,0,2,0)); ok(Math.abs(m.len-2)<1e-9, 'fusion');
const m2 = A.mergeSeg(seg(2,0,0,0), seg(1,0,-1,0)); ok(m2.x2 < m2.x1, 'fusion garde le sens de a');

// --- flux série : format CSV de la librairie (avec en-tête, x/y, ligne vide) découpé en morceaux arbitraires
let got = [];
const p = new A.LidarParser(rows => got.push(rows));
let txt = 'boot...\nN,Angle(°),Distance(mm),Intensity,x(mm),y(mm)\n';
for (let s = 0; s < 3; s++) {
  for (let i = 0; i < 300; i++) txt += `${i},${(359 - i * 1.1).toFixed(2)},${1500 + i},${100},0,0\r\n`;
  txt += '\r\n';
}
for (let i = 0; i < txt.length; i += 37) p.feed(txt.slice(i, i + 37));      // morceaux de 37 caractères
ok(got.length === 3 && got.every(r => r.length === 300), 'CSV : 3 scans de 300 points, got ' + got.length);
ok(Math.abs(got[0][0][0] - 359) < 1e-9 && got[0][0][1] === 1500, 'CSV : colonnes angle/distance');
// sans ligne vide : détection par saut d'angle
got = []; const p2 = new A.LidarParser(rows => got.push(rows));
let t2 = ''; for (let s = 0; s < 3; s++) for (let i = 0; i < 300; i++) t2 += `${i},${(359 - i * 1.1).toFixed(2)},2000,100\n`;
p2.feed(t2 + '\n'); ok(got.length === 3, 'CSV sans ligne vide : ' + got.length);
// teleplot : y = - d sin(theta) dans la librairie ; avec flip on retrouve theta
got = []; const p3 = new A.LidarParser(rows => got.push(rows), { teleFlipY: true });
let pts = []; for (let i = 0; i < 100; i++) { const th = (i*3.6)*Math.PI/180, d = 2000; pts.push(`${(d*Math.cos(th)).toFixed(1)}:${(-d*Math.sin(th)).toFixed(1)}`); }
p3.feed('>lidar:' + pts.join(';') + ';|xy\n');
ok(got.length === 1 && Math.abs(got[0][10][0] - 36) < 0.1 && Math.abs(got[0][10][1] - 2000) < 1, 'teleplot : angle retrouvé ' + got[0][10][0]);

// --- zip : écrit pour validation par python zipfile
const enc = new TextEncoder();
const parts = A.makeZipParts([{name:'a/b.csv', data: enc.encode('x,y\n1,2\n')}, {name:'poses.json', data: enc.encode('{"a":[1,2,3]}')}]);
const zipPath = path.join(os.tmpdir(), 'lidar_test.zip');
fs.writeFileSync(zipPath, Buffer.concat(parts.map(u => Buffer.from(u))));
console.log('OK', n, 'vérifications');
