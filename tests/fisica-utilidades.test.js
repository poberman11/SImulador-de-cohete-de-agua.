import { describe, it, expect } from './test-runner.js';

describe('fisica.utilidades', () => {
  it('convierte litros a metros cubicos', () => {
    expect(fisica.litrosAMetrosCubicos(1)).toBeCloseTo(0.001, 6);
    expect(fisica.litrosAMetrosCubicos(2.5)).toBeCloseTo(0.0025, 6);
    expect(fisica.litrosAMetrosCubicos(0)).toBe(0);
  });

  it('convierte psi a pascales', () => {
    expect(fisica.psiAPascales(1)).toBeCloseTo(6894.76, 2);
    expect(fisica.psiAPascales(100)).toBeCloseTo(689476, 2);
  });

  it('ley de Boyle: presión cae al aumentar volumen de aire', () => {
    const p = fisica.presionManometrica(100, 0.001, 0.0015, 0.002);
    expect(p).toBeGreaterThan(0);
    expect(p).toBeLessThan(fisica.psiAPascales(100));
  });

  it('velocidad jet desde diferencia de presión', () => {
    const deltaP = fisica.psiAPascales(10);
    const v = fisica.velocidadJet(deltaP);
    expect(v).toBeGreaterThan(0);
    expect(v).toBeCloseTo(Math.sqrt(2 * deltaP / fisica.RHO_AGUA), 6);
  });

  it('empuje devuelve fuerza y caudal masico positivos con agua', () => {
    const r = fisica.empuje(20, 0.5, 0.002, 1e-4, 0.9);
    expect(r.fuerza).toBeGreaterThan(0);
    expect(r.caudalMasico).toBeGreaterThan(0);
  });

  it('empuje es cero si no hay agua', () => {
    const vacio = fisica.empuje(20, 0, 0.002, 1e-4, 0.9);
    expect(vacio.fuerza).toBe(0);
    expect(vacio.caudalMasico).toBe(0);
  });

  it('arrastre es un vector opuesto a la velocidad', () => {
    const r = fisica.arrastre({ x: 10, y: 0 }, 0.01, 0.47, 1.225);
    expect(r.fx).toBeLessThan(0);
    expect(r.fy).toBe(0);

    const cero = fisica.arrastre({ x: 0, y: 0 }, 0.01, 0.47, 1.225);
    expect(cero.fx).toBe(0);
    expect(cero.fy).toBe(0);
  });
});
