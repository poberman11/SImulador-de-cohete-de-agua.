import { describe, it, expect } from './test-runner.js';

const simularLanzamiento = window.appNamespace.simularLanzamiento;

const VALID = { litrosProyectil: 0.5, litrosAgua: 1.0, psi: 60, anguloGrados: 45 };

describe('simulador', () => {
  it('calcula v0, tiempo de vuelo y posición final para entrada válida', () => {
    const res = simularLanzamiento(VALID);
    expect(res.despega).toBe(true);
    expect(res.v0).toBeGreaterThan(0);
    expect(res.tiempoVuelo).toBeGreaterThan(0);
    expect(res.posicionFinal.y).toBeCloseTo(0, 2);
    expect(res.puntos.length).toBeGreaterThan(10);
  });

  it('rechaza entrada que no despega', () => {
    const res = simularLanzamiento({ litrosProyectil: 10, litrosAgua: 0.1, psi: 5, anguloGrados: 45 });
    expect(res.despega).toBe(false);
    expect(typeof res.mensaje).toBe('string');
    expect(res.mensaje.length).toBeGreaterThan(0);
  });

  it('rechaza litrosProyectil <= 0', () => {
    for (const lp of [0, -1, -0.5]) {
      const res = simularLanzamiento({ ...VALID, litrosProyectil: lp });
      expect(res.despega).toBe(false);
    }
  });

  it('rechaza litrosAgua fuera de (0, 2]', () => {
    for (const la of [0, -0.5, 2.1, 10]) {
      const res = simularLanzamiento({ ...VALID, litrosAgua: la });
      expect(res.despega).toBe(false);
    }
  });

  it('rechaza psi <= 0', () => {
    for (const p of [0, -10]) {
      const res = simularLanzamiento({ ...VALID, psi: p });
      expect(res.despega).toBe(false);
    }
  });

  it('rechaza anguloGrados fuera de [0, 90]', () => {
    for (const a of [-1, 90.5, 180]) {
      const res = simularLanzamiento({ ...VALID, anguloGrados: a });
      expect(res.despega).toBe(false);
    }
  });

  it('puntos tienen forma {t,x,y,vx,vy} y t creciente desde 0', () => {
    const res = simularLanzamiento(VALID);
    expect(res.puntos[0].t).toBe(0);
    let previo = res.puntos[0].t;
    for (const p of res.puntos) {
      expect(typeof p.t).toBe('number');
      expect(typeof p.x).toBe('number');
      expect(typeof p.y).toBe('number');
      expect(typeof p.vx).toBe('number');
      expect(typeof p.vy).toBe('number');
      expect(p.t).toBeGreaterThanOrEqual(previo);
      previo = p.t;
    }
  });

  it('tiempoVuelo es el instante de impacto (interpolado), dentro del último paso de puntos', () => {
    const res = simularLanzamiento(VALID);
    const tUltimo = res.puntos[res.puntos.length - 1].t;
    expect(res.tiempoVuelo).toBeGreaterThan(0);
    // impacto.t es el cruce interpolado de y = 0, por tanto <= al último paso crudo
    expect(res.tiempoVuelo).toBeLessThanOrEqual(tUltimo);
    // y difiere como máximo un paso balístico (0,01 s) del último paso crudo
    expect(tUltimo - res.tiempoVuelo).toBeLessThanOrEqual(0.011);
  });

  it('alturaMaxima es el mayor y de los puntos y es > 0', () => {
    const res = simularLanzamiento(VALID);
    let maxY = 0;
    for (const p of res.puntos) {
      if (p.y > maxY) maxY = p.y;
    }
    expect(res.alturaMaxima).toBeCloseTo(maxY, 6);
    expect(res.alturaMaxima).toBeGreaterThan(0);
  });

  // --- Casos borde (Task 8) ---

  it('agua = 0 → despega=false con mensaje claro, sin lanzar', () => {
    const res = simularLanzamiento({ ...VALID, litrosAgua: 0 });
    expect(res.despega).toBe(false);
    expect(typeof res.mensaje).toBe('string');
    expect(res.mensaje.length).toBeGreaterThan(0);
    expect(res.mensaje.toLowerCase().includes('agua')).toBe(true);
  });

  it('agua negativa → despega=false con mensaje claro', () => {
    const res = simularLanzamiento({ ...VALID, litrosAgua: -0.5 });
    expect(res.despega).toBe(false);
    expect(typeof res.mensaje).toBe('string');
    expect(res.mensaje.length).toBeGreaterThan(0);
  });

  it('psi = 0 → despega=false con mensaje claro, sin lanzar', () => {
    const res = simularLanzamiento({ ...VALID, psi: 0 });
    expect(res.despega).toBe(false);
    expect(typeof res.mensaje).toBe('string');
    expect(res.mensaje.length).toBeGreaterThan(0);
    expect(res.mensaje.toLowerCase().includes('psi')).toBe(true);
  });

  it('tanque más allá de capacidad (3 L > 2 L) → error de validación', () => {
    const res = simularLanzamiento({ ...VALID, litrosAgua: 3 });
    expect(res.despega).toBe(false);
    expect(typeof res.mensaje).toBe('string');
    expect(res.mensaje.length).toBeGreaterThan(0);
    expect(res.mensaje.includes('2 litros')).toBe(true);
  });

  it('empuje insuficiente → "no despega", sin trazado de trayectoria', () => {
    const res = simularLanzamiento({ litrosProyectil: 10, litrosAgua: 0.1, psi: 5, anguloGrados: 45 });
    expect(res.despega).toBe(false);
    expect(typeof res.mensaje).toBe('string');
    expect(res.mensaje.toLowerCase().includes('no despega')).toBe(true);
    expect(Array.isArray(res.puntos)).toBe(true);
    expect(res.puntos.length).toBe(0);
  });

  it('cuando despega=true, ningún valor de salida es NaN ni infinito', () => {
    const res = simularLanzamiento(VALID);
    expect(res.despega).toBe(true);
    const finito = (n) => typeof n === 'number' && isFinite(n);
    expect(finito(res.v0)).toBe(true);
    expect(finito(res.tiempoVuelo)).toBe(true);
    expect(finito(res.posicionFinal.x)).toBe(true);
    expect(finito(res.posicionFinal.y)).toBe(true);
    expect(finito(res.alturaMaxima)).toBe(true);
    for (const p of res.puntos) {
      expect(finito(p.t)).toBe(true);
      expect(finito(p.x)).toBe(true);
      expect(finito(p.y)).toBe(true);
      expect(finito(p.vx)).toBe(true);
      expect(finito(p.vy)).toBe(true);
    }
  });

  it('lanzamiento vertical (90°) → x permanece ≈ 0 en todo el recorrido', () => {
    const res = simularLanzamiento({ ...VALID, anguloGrados: 90 });
    expect(res.despega).toBe(true);
    for (const p of res.puntos) {
      expect(Math.abs(p.x)).toBeLessThan(1e-6);
    }
    expect(Math.abs(res.posicionFinal.x)).toBeLessThan(1e-6);
  });

  it('anguloGrados = 0° despega=true con salidas finitas', () => {
    const res = simularLanzamiento({ ...VALID, anguloGrados: 0 });
    expect(res.despega).toBe(true);
    const finito = (n) => typeof n === 'number' && isFinite(n);
    expect(finito(res.v0)).toBe(true);
    expect(finito(res.tiempoVuelo)).toBe(true);
    expect(finito(res.posicionFinal.x)).toBe(true);
    expect(finito(res.posicionFinal.y)).toBe(true);
    expect(finito(res.alturaMaxima)).toBe(true);
    expect(res.tiempoVuelo).toBeGreaterThan(0);
  });
});
