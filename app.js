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

// Paso fijo de integración para la fase de propulsión.
const DT_PROPULSION = 0.001; // s
const MAX_STEPS_PROPULSION = 200000; // freno de seguridad contra bucles infinitos

/**
 * Integración RK4 (4.º orden) de la fase de propulsión del cohete de agua.
 *
 * Integra mientras quede agua (volAgua > 0) y la presión manométrica sea
 * positiva. El estado evoluciona bajo:
 *   - empuje del chorro a lo largo de û = [cos θ, sin θ]
 *   - peso: -m·g·ĵ
 *   - arrastre aerodinámico opuesto a v
 *   - masa variable: m = mProyectil + RHO_AGUA · volAgua
 *   - volAgua se consume por el caudal másico del chorro
 *   - presión cae por la ley de Boyle a medida que el aire se expande
 *
 * Devuelve { puntos, estadoFinal }. estadoFinal incluye la velocidad de
 * salida del lanzador (v₀ para la fase balística).
 */
function integrarPropulsion(estadoInicial, params, dt = DT_PROPULSION) {
  const {
    anguloGrados,
    mProyectil,
    volAguaInicial,
    psiInicial,
    volTanque,
    areaBoquilla,
    cd,
  } = params;
  const theta = (anguloGrados * Math.PI) / 180;
  const cosT = Math.cos(theta);
  const sinT = Math.sin(theta);
  const volAireInicial = volTanque - volAguaInicial;

  function presionPascales(volAgua) {
    return presionManometrica(psiInicial, volAireInicial, volTanque - volAgua, volTanque);
  }

  function derivada(estado) {
    const { vx, vy, masa, volAgua } = estado;
    const pPa = presionPascales(volAgua);
    // empuje espera PSI (los convierte a Pascales internamente).
    const psi = pPa / PSI_A_PASCAL;
    const { fuerza, caudalMasico } = empuje(psi, volAgua, volTanque, areaBoquilla, cd);
    const arr = arrastre({ x: vx, y: vy }, AREA_FRONTAL, CD_AIRE, RHO_AIRE);
    return {
      x: vx,
      y: vy,
      vx: (fuerza * cosT + arr.fx) / masa,
      vy: (fuerza * sinT - masa * GRAVEDAD + arr.fy) / masa,
      masa: -caudalMasico,
      volAgua: -caudalMasico / RHO_AGUA,
      presion: pPa,
    };
  }

  function intermedio(estado, k, h) {
    return {
      x: estado.x + h * k.x,
      y: estado.y + h * k.y,
      vx: estado.vx + h * k.vx,
      vy: estado.vy + h * k.vy,
      masa: estado.masa + h * k.masa,
      volAgua: estado.volAgua + h * k.volAgua,
    };
  }

  const puntos = [];
  let estado = { ...estadoInicial };
  let t = 0;
  let pasosRestantes = MAX_STEPS_PROPULSION;

  const k0 = derivada(estado);
  puntos.push({
    t,
    x: estado.x,
    y: estado.y,
    vx: estado.vx,
    vy: estado.vy,
    masa: estado.masa,
    volAgua: estado.volAgua,
    presion: k0.presion,
  });

  while (estado.volAgua > 0 && presionPascales(estado.volAgua) > 0 && pasosRestantes-- > 0) {
    const k1 = derivada(estado);
    const k2 = derivada(intermedio(estado, k1, dt / 2));
    const k3 = derivada(intermedio(estado, k2, dt / 2));
    const k4 = derivada(intermedio(estado, k3, dt));

    estado = {
      x: estado.x + (dt / 6) * (k1.x + 2 * k2.x + 2 * k3.x + k4.x),
      y: estado.y + (dt / 6) * (k1.y + 2 * k2.y + 2 * k3.y + k4.y),
      vx: estado.vx + (dt / 6) * (k1.vx + 2 * k2.vx + 2 * k3.vx + k4.vx),
      vy: estado.vy + (dt / 6) * (k1.vy + 2 * k2.vy + 2 * k3.vy + k4.vy),
      masa: estado.masa + (dt / 6) * (k1.masa + 2 * k2.masa + 2 * k3.masa + k4.masa),
      volAgua: estado.volAgua + (dt / 6) * (k1.volAgua + 2 * k2.volAgua + 2 * k3.volAgua + k4.volAgua),
    };

    if (estado.volAgua < 0) {
      estado.volAgua = 0;
    }
    if (estado.masa < mProyectil) {
      estado.masa = mProyectil;
    }

    t += dt;
    const p = presionPascales(estado.volAgua);
    puntos.push({
      t,
      x: estado.x,
      y: estado.y,
      vx: estado.vx,
      vy: estado.vy,
      masa: estado.masa,
      volAgua: estado.volAgua,
      presion: p,
    });
  }

  const ultimo = puntos[puntos.length - 1];
  const estadoFinal = {
    x: ultimo.x,
    y: ultimo.y,
    vx: ultimo.vx,
    vy: ultimo.vy,
    masa: ultimo.masa,
    volAgua: ultimo.volAgua,
    presion: ultimo.presion,
  };
  return { puntos, estadoFinal };
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
  integrarPropulsion,
};

// Alias de conveniencia para consumir directamente `fisica` en tests.
window.fisica = window.appNamespace.fisica;
