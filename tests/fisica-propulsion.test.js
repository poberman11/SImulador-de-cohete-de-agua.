import { describe, it, expect } from './test-runner.js';

const integrarPropulsion = window.appNamespace.integrarPropulsion;
const { litrosAMetrosCubicos } = window.appNamespace.fisica;

const PARAMS = {
  anguloGrados: 85,
  mProyectil: 0.5,
  volAguaInicial: litrosAMetrosCubicos(0.5), // 0,0005 m³
  psiInicial: 60,
  volTanque: litrosAMetrosCubicos(2), // 0,002 m³
  areaBoquilla: 1e-4,
  cd: 0.9,
};

const ESTADO_INICIAL = {
  x: 0,
  y: 0,
  vx: 0,
  vy: 0,
  masa: 0.5 + 1000 * litrosAMetrosCubicos(0.5), // 1,0 kg
  volAgua: litrosAMetrosCubicos(0.5),
};

describe('fisica.propulsion', () => {
  it('devuelve puntos y estadoFinal con velocidad de salida', () => {
    const res = integrarPropulsion(ESTADO_INICIAL, PARAMS, 0.001);
    expect(res.puntos.length).toBeGreaterThan(1);
    expect(typeof res.estadoFinal.vx).toBe('number');
    expect(typeof res.estadoFinal.vy).toBe('number');
    expect(typeof res.estadoFinal.presion).toBe('number');
  });

  it('puntos tienen t creciendo desde 0 con paso ≈ 0,001 s', () => {
    const res = integrarPropulsion(ESTADO_INICIAL, PARAMS, 0.001);
    expect(res.puntos[0].t).toBe(0);
    let previo = res.puntos[0].t;
    for (let i = 1; i < res.puntos.length; i += 1) {
      const actual = res.puntos[i].t;
      expect(actual).toBeGreaterThan(previo);
      previo = actual;
    }
  });

  it('masa decrece monotonía y nunca cae por debajo del proyectil', () => {
    const res = integrarPropulsion(ESTADO_INICIAL, PARAMS, 0.001);
    let previo = res.puntos[0].masa;
    for (let i = 1; i < res.puntos.length; i += 1) {
      if (res.puntos[i].masa > previo + 1e-12) {
        throw new Error(`masa aumentó en el paso ${i}`);
      }
      previo = res.puntos[i].masa;
    }
    expect(res.estadoFinal.masa).toBeLessThan(ESTADO_INICIAL.masa);
    expect(res.estadoFinal.masa).toBeGreaterThan(0.499999);
  });

  it('volAgua llega a 0 y nunca es negativo', () => {
    const res = integrarPropulsion(ESTADO_INICIAL, PARAMS, 0.001);
    expect(res.estadoFinal.volAgua).toBe(0);
    for (const p of res.puntos) {
      if (p.volAgua < 0) {
        throw new Error(`volAgua negativo: ${p.volAgua}`);
      }
    }
    // El último punto debe estar vacío (o el paso anterior quedó vaciado).
    const ultimo = res.puntos[res.puntos.length - 1];
    expect(ultimo.volAgua).toBe(0);
  });

  it('presión no negativa en todos los puntos', () => {
    const res = integrarPropulsion(ESTADO_INICIAL, PARAMS, 0.001);
    for (const p of res.puntos) {
      expect(p.presion).toBeGreaterThan(-1e-12);
    }
  });

  it('cohete sale hacia arriba (ángulo 85°)', () => {
    const res = integrarPropulsion(ESTADO_INICIAL, PARAMS, 0.001);
    expect(res.estadoFinal.vy).toBeGreaterThan(0);
    expect(res.estadoFinal.vx).toBeGreaterThan(0);
    expect(res.estadoFinal.vy).toBeGreaterThan(res.estadoFinal.vx);
  });
});
