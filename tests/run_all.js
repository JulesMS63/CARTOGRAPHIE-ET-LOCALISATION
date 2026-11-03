// Lance tous les tests de la page : node tests/run_all.js
const { spawnSync } = require('child_process'), path = require('path');
const root = path.join(__dirname, '..');
let failed = 0;
for (const t of ['web_core_test.js', 'web_unit_test.js', 'web_loc_test.js', 'web_reg_test.js', 'web_grid_test.js', 'web_wifi_test.js']) {
  const r = spawnSync(process.execPath, [path.join(__dirname, t), path.join(root, 'lidar_web.html'), path.join(root, 'data_synth')], { encoding: 'utf8' });
  const last = (r.stdout || '').trim().split('\n').pop();
  console.log((r.status === 0 ? 'OK     ' : 'ÉCHEC  ') + t.padEnd(20) + ' ' + (r.status === 0 ? last : (r.stderr || r.stdout).split('\n').slice(0, 6).join('\n')));
  if (r.status !== 0) failed++;
}
process.exit(failed ? 1 : 0);
