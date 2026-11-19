// Native JS runner (Node) para tests/fisica-utilidades.test.js
// Stub de `window`/`document`, carga app.js real y ejecuta la suite ESM.
import fs from 'fs';

globalThis.window = globalThis;
globalThis.document = globalThis.document || {};

eval(fs.readFileSync('app.js', 'utf8'));

globalThis.fisica = globalThis.window.appNamespace.fisica;

const { summary } = await import('./test-runner.js');
await import('./fisica-utilidades.test.js');
await import('./fisica-propulsion.test.js');

const s = summary();
console.log(`\nResultados: PASSED=${s.passes} FAILED=${s.failures}`);
s.details.forEach((d) => {
  console.log((d.ok ? '✓' : '✗') + ' ' + d.name + (d.ok ? '' : ' — ' + d.error));
});
process.exit(s.failures === 0 ? 0 : 1);
