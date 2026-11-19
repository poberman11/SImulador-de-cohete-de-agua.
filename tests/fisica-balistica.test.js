import { describe, it, expect } from './test-runner.js';

const integrarBalistica = window.appNamespace.integrarBalistica;

const ESTADO_INICIAL = {
  x: 0,
  y: 2,
  vx: 20,
  vy: 30,
  masa: 0.5,
};

describe('fisica.balistica', () => {
  it('devuelve puntos e impacto con la forma esperada', () => {
    const res = integrarBalistica(ESTADO_INICIAL, 0.01);
    expect(Array.isArray(res.puntos)).toBe(true);
    expect(res.puntos.length).toBeGreaterThan(1);
    expect(typeof res.impacto.t).toBe('number');
    expect(typeof res.impacto.x).toBe('number');
    expect(typeof res.impacto.y).toBe('number');
    for (const p of res.puntos) {
      expect(typeof p.t).toBe('number');
      expect(typeof p.x).toBe('number');
      expect(typeof p.y).toBe('number');
      expect(typeof p.vx).toBe('number');
      expect(typeof p.vy).toBe('number');
    }
  });

  it('t crece desde 0 con paso ≈ 0,01 s', () => {
    const res = integrarBalistica(ESTADO_INICIAL, 0.01);
    expect(res.puntos[0].t).toBe(0);
    let previo = res.puntos[0].t;
    for (let i = 1; i < res.puntos.length; i += 1) {
      const actual = res.puntos[i].t;
      expect(actual).toBeGreaterThan(previo);
      expect(actual - previo).toBeCloseTo(0.01, 6);
      previo = actual;
    }
  });

  it('detecta impacto: el último punto crudo tiene y <= 0', () => {
    const res = integrarBalistica(ESTADO_INICIAL, 0.01);
    const ultimo = res.puntos[res.puntos.length - 1];
    expect(ultimo.y).toBeLessThanOrEqual(0);
    const anterior = res.puntos[res.puntos.length - 2];
    expect(anterior.y).toBeGreaterThan(0);
  });

  it('el impacto tiene y = 0 y x de aterrizaje > 0', () => {
    const res = integrarBalistica(ESTADO_INICIAL, 0.01);
    expect(res.impacto.y).toBe(0);
    expect(res.impacto.x).toBeGreaterThan(0);
    expect(res.impacto.x).toBeGreaterThan(ESTADO_INICIAL.x);
  });

  it('el tiempo de vuelo es > 0 y el impacto ocurre dentro del último paso', () => {
    const res = integrarBalistica(ESTADO_INICIAL, 0.01);
    expect(res.impacto.t).toBeGreaterThan(0);
    const ultimo = res.puntos[res.puntos.length - 1].t;
    const penultimo = res.puntos[res.puntos.length - 2].t;
    expect(res.impacto.t).toBeGreaterThanOrEqual(penultimo);
    expect(res.impacto.t).toBeLessThanOrEqual(ultimo);
  });

  it('proyectil lanzado verticalmente desde y = 1 m aterriza en x ≈ 0', () => {
    const res = integrarBalistica({ x: 0, y: 1, vx: 0, vy: 25, masa: 0.5 }, 0.01);
    expect(res.impacto.t).toBeGreaterThan(0);
    expect(Math.abs(res.impacto.x)).toBeLessThan(0.5);
  });

  it('si el estado inicial ya está en el suelo, el impacto es t = 0', () => {
    const res = integrarBalistica({ x: 5, y: 0, vx: 10, vy: 0, masa: 0.5 }, 0.01);
    expect(res.impacto.t).toBe(0);
    expect(res.impacto.x).toBe(5);
    expect(res.impacto.y).toBe(0);
  });
});
