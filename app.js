// --- Constantes físicas (SI) ---
const LITRO_A_M3 = 0.001;       // 1 litro = 0.001 m³
const RHO_AGUA = 1000;          // kg/m³
const GRAVEDAD = 9.81;          // m/s²
const PSI_A_PASCAL = 6894.76;   // 1 PSI = 6894.76 Pa
const ATMOSFERA_PASCAL = 101325;// 1 atm = 101325 Pa
const RHO_AIRE = 1.225;         // kg/m³ (aire a 15 °C, nivel del mar)
const AREA_BOQUILLA = 1e-4;     // m² (área de boquilla de referencia)
const CD = 0.9;                 // coeficiente de descarga del chorro
const CD_AIRE = 0.47;           // coeficiente de arrastre (esfera)
const AREA_FRONTAL = 0.01;      // m² (área frontal de referencia)

/** Convierte litros a metros cúbicos. */
function litrosAMetrosCubicos(litros) {
  return litros * LITRO_A_M3;
}

/** Convierte PSI a pascales. */
function psiAPascales(psi) {
  return psi * PSI_A_PASCAL;
}

/**
 * Presión manométrica del aire del tanque por la ley de Boyle.
 * P1_abs * V1 = P2_abs * V2  =>  P2_manométrico = P2_abs - P_atm.
 */
function presionManometrica(psiInicial, volAireInicial, volAireActual, volTanque) {
  const v = Math.min(Math.max(volAireActual, 1e-9), volTanque);
  const p1Abs = psiAPascales(psiInicial) + ATMOSFERA_PASCAL;
  const p2Abs = p1Abs * (volAireInicial / v);
  return Math.max(0, p2Abs - ATMOSFERA_PASCAL);
}

/** Velocidad del chorro: v = sqrt(2 * ΔP / ρ_agua). */
function velocidadJet(deltaPascales) {
  return Math.sqrt(Math.max(0, 2 * deltaPascales / RHO_AGUA));
}

/**
 * Empuje del chorro de agua.
 * Devuelve { fuerza, caudalMasico }.
 */
function empuje(psi, volAgua, volTanque, areaBoquilla, cd) {
  if (volAgua <= 0) {
    return { fuerza: 0, caudalMasico: 0 };
  }
  const deltaP = psiAPascales(psi);
  const v = velocidadJet(deltaP);
  const caudalMasico = cd * RHO_AGUA * areaBoquilla * v;
  const fuerza = caudalMasico * v;
  return { fuerza, caudalMasico };
}

/**
 * Fuerza de arrastre aerodinámico, vector opuesto a la velocidad.
 * Devuelve { fx, fy }.
 */
function arrastre(velocidad, areaFrontal, cdAir, rhoAire) {
  const vx = velocidad.x;
  const vy = velocidad.y;
  const v = Math.hypot(vx, vy);
  if (v === 0) {
    return { fx: 0, fy: 0 };
  }
  const fd = 0.5 * rhoAire * v * v * cdAir * areaFrontal;
  return {
    fx: -fd * (vx / v),
    fy: -fd * (vy / v),
  };
}

window.appNamespace = {
  fisica: {
    litrosAMetrosCubicos,
    psiAPascales,
    presionManometrica,
    velocidadJet,
    empuje,
    arrastre,
  },
};

// Alias de conveniencia para consumir directamente `fisica` en tests.
window.fisica = window.appNamespace.fisica;
