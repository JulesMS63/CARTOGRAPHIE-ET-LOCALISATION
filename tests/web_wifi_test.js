// Tests de la liaison Wi-Fi (adresse et lecture des messages) : node web_wifi_test.js lidar_web.html
const fs = require('fs'), assert = require('assert');
const html = fs.readFileSync(process.argv[2], 'utf8');
const core = html.split('/* CORE-START */')[1].split('/* CORE-END */')[0];
const A = new Function(core + '; return {wsUrlOf, parseWsMessage, LidarParser};')();
let n = 0; const ok = (c, m) => { assert(c, m); n++; };

// --- adresse saisie -> url
ok(A.wsUrlOf('192.168.4.1') === 'ws://192.168.4.1:81/', 'adresse seule : port 81 du sketch');
ok(A.wsUrlOf(' 192.168.4.1:81 ') === 'ws://192.168.4.1:81/', 'espaces ignorés');
ok(A.wsUrlOf('ws://192.168.4.1:81/') === 'ws://192.168.4.1:81/', 'url ws:// complète');
ok(A.wsUrlOf('http://192.168.4.1/') === 'ws://192.168.4.1:81/', 'adresse de la page web du sketch (port 80) -> websocket 81');
ok(A.wsUrlOf('robot.local') === 'ws://robot.local:81/', 'nom de machine');
ok(A.wsUrlOf('10.0.0.7:8081') === 'ws://10.0.0.7:8081/', 'autre port');
ok(A.wsUrlOf('') === null && A.wsUrlOf('bad host!') === null && A.wsUrlOf('1.2.3.4:99999') === null && A.wsUrlOf(null) === null, 'adresses inutilisables refusées');

// --- message JSON du sketch actuel : {"a":..,"d":..}, un point sur deux
const pts = []; for (let i = 0; i < 100; i++) pts.push(`{"a":${(i * 3.6).toFixed(1)},"d":${1000 + i}}`);
let m = A.parseWsMessage('[' + pts.join(',') + ']');
ok(m.kind === 'json' && m.rows.length === 100, 'JSON du sketch : 100 points lus');
ok(Math.abs(m.rows[1][0] - 3.6) < 1e-9 && m.rows[1][1] === 1001 && m.rows[1][2] === 255, 'angle (°), distance (mm) ; intensité 255 quand absente');
m = A.parseWsMessage('[' + pts.join(',') + ',]');
ok(m.kind === 'json' && m.rows.length === 100, 'virgule en trop avant ] (bug du sketch) : lu quand même');
m = A.parseWsMessage('[{"a":1,"d":500,"i":77},{"a":2,"d":0},{"a":3,"d":-4},{"a":"x","d":5},{"d":9},{"a":361,"d":800}]');
ok(m.rows.length === 2 && m.rows[0][2] === 77 && Math.abs(m.rows[1][0] - 1) < 1e-9, 'distance nulle, négative, angle illisible : ignorés ; 361° ramené à 1° ; intensité lue');
ok(A.parseWsMessage('{"points":[{"angle":10,"distance":900,"intensity":5}]}').rows[0][1] === 900, 'objet {"points":[...]} et noms longs');
ok(A.parseWsMessage('[[10,900,5],[20,950]]').rows.length === 2, 'tableaux [angle, distance, intensité]');

// --- autres messages
ok(A.parseWsMessage('[{"a":1,"d":').kind === 'invalide', 'message tronqué : signalé, pas d’exception');
ok(A.parseWsMessage('').kind === 'vide' && A.parseWsMessage('   \n').kind === 'vide', 'message vide');
ok(A.parseWsMessage('0,10.5,1200,255,5,5\n').kind === 'texte', 'CSV : confié à LidarParser');

// --- un message CSV passe bien par LidarParser comme sur l'USB
let got = []; const P = new A.LidarParser(r => got.push(r));
const csv = []; for (let i = 0; i < 80; i++) csv.push(`${i},${(i * 4.5).toFixed(1)},${1000 + i},200,0,0`);
P.feed(csv.join('\n') + '\n\n');
ok(got.length === 1 && got[0].length === 80, 'tour CSV reçu par message : un scan de 80 points');
console.log(n + ' vérifications Wi-Fi OK');
