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
const V_TANQUE = 0.002;         // m³ capacidad del tanque (2 litros)

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

// Paso fijo de integración para la fase balística.
const DT_BALISTICA = 0.01; // s
const MAX_STEPS_BALISTICA = 200000; // freno de seguridad contra bucles infinitos

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
    t: ultimo.t, // instante de salida del lanzador (útil para el orquestador)
  };
  return { puntos, estadoFinal };
}

/**
 * Integración RK4 (4.º orden) de la fase balística del proyectil.
 *
 * El proyectil se mueve bajo gravedad y arrastre aerodinámico con masa
 * constante (mProyectil, sin agua):
 *   a = (-m·g·ĵ - arrastre(v)) / m
 *
 * Integra con paso fijo dt hasta que y <= 0 (impacto con el suelo). El
 * punto de impacto se interpola linealmente entre el último punto con
 * y > 0 y el primero con y <= 0.
 *
 * Devuelve { puntos, impacto } donde:
 *   puntos: Array<{t, x, y, vx, vy}> con el estado en cada paso RK4.
 *   impacto: {t, x, y} con y = 0 y t, x interpolados del cruce de suelo.
 */
function integrarBalistica(estadoInicial, dt = DT_BALISTICA) {
  const m = estadoInicial.masa;
  if (!(m > 0)) {
    // Defensa: masa no positiva no puede integrarse (división por cero).
    return {
      puntos: [{
        t: 0,
        x: estadoInicial.x,
        y: estadoInicial.y,
        vx: estadoInicial.vx,
        vy: estadoInicial.vy,
      }],
      impacto: { t: 0, x: estadoInicial.x, y: estadoInicial.y },
    };
  }

  function derivada(estado) {
    const { vx, vy } = estado;
    const arr = arrastre({ x: vx, y: vy }, AREA_FRONTAL, CD_AIRE, RHO_AIRE);
    return {
      x: vx,
      y: vy,
      vx: arr.fx / m,
      vy: -GRAVEDAD + arr.fy / m,
    };
  }

  function intermedio(estado, k, h) {
    return {
      x: estado.x + h * k.x,
      y: estado.y + h * k.y,
      vx: estado.vx + h * k.vx,
      vy: estado.vy + h * k.vy,
    };
  }

  function rk4Step(estado) {
    const k1 = derivada(estado);
    const k2 = derivada(intermedio(estado, k1, dt / 2));
    const k3 = derivada(intermedio(estado, k2, dt / 2));
    const k4 = derivada(intermedio(estado, k3, dt));
    return {
      x: estado.x + (dt / 6) * (k1.x + 2 * k2.x + 2 * k3.x + k4.x),
      y: estado.y + (dt / 6) * (k1.y + 2 * k2.y + 2 * k3.y + k4.y),
      vx: estado.vx + (dt / 6) * (k1.vx + 2 * k2.vx + 2 * k3.vx + k4.vx),
      vy: estado.vy + (dt / 6) * (k1.vy + 2 * k2.vy + 2 * k3.vy + k4.vy),
    };
  }

  const puntos = [];
  let estado = { ...estadoInicial };
  let t = 0;
  let pasosRestantes = MAX_STEPS_BALISTICA;

  puntos.push({
    t,
    x: estado.x,
    y: estado.y,
    vx: estado.vx,
    vy: estado.vy,
  });

  while (estado.y > 0 && pasosRestantes-- > 0) {
    estado = rk4Step(estado);
    t += dt;
    puntos.push({
      t,
      x: estado.x,
      y: estado.y,
      vx: estado.vx,
      vy: estado.vy,
    });
  }

  // Impacto interpolado linealmente entre el último punto con y > 0 y el
  // primero con y <= 0.
  let impacto;
  if (puntos.length >= 2) {
    const previo = puntos[puntos.length - 2]; // y > 0
    const actual = puntos[puntos.length - 1]; // y <= 0
    const frac = previo.y / (previo.y - actual.y); // fracción del paso en que y cruza 0
    impacto = {
      t: previo.t + frac * (actual.t - previo.t),
      x: previo.x + frac * (actual.x - previo.x),
      y: 0,
    };
  } else {
    // El estado inicial ya está en el suelo (o por debajo).
    impacto = { t: 0, x: estadoInicial.x, y: 0 };
  }

  return { puntos, impacto };
}

/**
 * Orquesta la simulación completa: fase 1 (propulsión) → fase 2 (balística).
 *
 * Entradas (unidades de usuario):
 *   litrosProyectil: masa del proyectil vacío, numéricamente en kg
 *                    (equivalente agua: 1 L ≈ 1 kg).
 *   litrosAgua:      volumen de agua en el tanque, en litros (0 < v <= 2 L).
 *   psi:             presión inicial del aire, manométrica, en PSI (> 0).
 *   anguloGrados:    ángulo de lanzamiento en grados [0, 90].
 *
 * Salida: { v0, tiempoVuelo, posicionFinal: {x, y}, alturaMaxima,
 *           puntos: Array<{t, x, y, vx, vy}>, despega, mensaje? }
 *
 * Muestreo de puntos: se fusionan los puntos de ambas fases tal cual los
 * produce el integrador RK4 (paso 0,001 s en propulsión, 0,01 s en
 * balística), desplazando el tiempo de la fase balística por la duración
 * de la propulsión. No se interpola entre pasos: se documenta esta
 * elección (muestreo por paso de integración en lugar de una rejilla cada
 * 0,05 s) para preservar los estados exactos de RK4, incluido v₀ en la
 * salida del lanzador, sin error de muestreo intermedio.
 */
function simularLanzamiento({ litrosProyectil, litrosAgua, psi, anguloGrados }) {
  // --- Validación de entradas ---
  if (!(litrosProyectil > 0)) {
    return {
      despega: false,
      mensaje: 'litrosProyectil debe ser mayor que 0.',
      v0: 0,
      tiempoVuelo: 0,
      posicionFinal: { x: 0, y: 0 },
      alturaMaxima: 0,
      puntos: [],
    };
  }
  const litrosTanque = V_TANQUE / LITRO_A_M3; // 2 L
  if (!(litrosAgua > 0) || litrosAgua > litrosTanque) {
    return {
      despega: false,
      mensaje: `litrosAgua debe ser mayor que 0 y menor o igual a ${litrosTanque} litros (capacidad del tanque).`,
      v0: 0,
      tiempoVuelo: 0,
      posicionFinal: { x: 0, y: 0 },
      alturaMaxima: 0,
      puntos: [],
    };
  }
  if (!(psi > 0)) {
    return {
      despega: false,
      mensaje: 'psi debe ser mayor que 0.',
      v0: 0,
      tiempoVuelo: 0,
      posicionFinal: { x: 0, y: 0 },
      alturaMaxima: 0,
      puntos: [],
    };
  }
  if (!(anguloGrados >= 0 && anguloGrados <= 90)) {
    return {
      despega: false,
      mensaje: 'anguloGrados debe estar entre 0 y 90 grados.',
      v0: 0,
      tiempoVuelo: 0,
      posicionFinal: { x: 0, y: 0 },
      alturaMaxima: 0,
      puntos: [],
    };
  }

  const mProyectil = litrosProyectil; // kg
  const volAgua = litrosAgua * LITRO_A_M3;

  // --- Comprobación de despegue: empuje inicial debe superar el peso ---
  const { fuerza: empujeInicial } = empuje(psi, volAgua, V_TANQUE, AREA_BOQUILLA, CD);
  const peso = mProyectil * GRAVEDAD;
  if (empujeInicial <= peso) {
    return {
      despega: false,
      mensaje: `No despega: empuje inicial (${empujeInicial.toFixed(2)} N) <= peso del proyectil (${peso.toFixed(2)} N).`,
      v0: 0,
      tiempoVuelo: 0,
      posicionFinal: { x: 0, y: 0 },
      alturaMaxima: 0,
      puntos: [],
    };
  }

  // --- Fase 1: propulsión (estado en la salida del lanzador) ---
  const estadoInicialProp = {
    x: 0,
    y: 0,
    vx: 0,
    vy: 0,
    masa: mProyectil + RHO_AGUA * volAgua,
    volAgua,
  };
  const paramsProp = {
    anguloGrados,
    mProyectil,
    volAguaInicial: volAgua,
    psiInicial: psi,
    volTanque: V_TANQUE,
    areaBoquilla: AREA_BOQUILLA,
    cd: CD,
  };
  const { puntos: puntosProp, estadoFinal } = integrarPropulsion(estadoInicialProp, paramsProp);
  const tProp = estadoFinal.t;
  const v0 = Math.hypot(estadoFinal.vx, estadoFinal.vy);

  // --- Fase 2: balística desde la salida del lanzador hasta el impacto ---
  const estadoInicialBal = {
    x: estadoFinal.x,
    y: estadoFinal.y,
    vx: estadoFinal.vx,
    vy: estadoFinal.vy,
    masa: estadoFinal.masa,
  };
  const { puntos: puntosBal, impacto } = integrarBalistica(estadoInicialBal);

  // --- Fusión de puntos: tiempo de balística desplazado por tProp ---
  const puntos = [];
  for (const p of puntosProp) {
    puntos.push({ t: p.t, x: p.x, y: p.y, vx: p.vx, vy: p.vy });
  }
  for (const p of puntosBal) {
    if (tProp + p.t === tProp) {
      continue;
    }
    puntos.push({ t: tProp + p.t, x: p.x, y: p.y, vx: p.vx, vy: p.vy });
  }

  // --- Altura máxima sobre todos los puntos de ambas fases ---
  let alturaMaxima = 0;
  for (const p of puntos) {
    if (p.y > alturaMaxima) {
      alturaMaxima = p.y;
    }
  }

  const resultado = {
    v0,
    tiempoVuelo: tProp + impacto.t,
    posicionFinal: { x: impacto.x, y: impacto.y },
    alturaMaxima,
    puntos,
    despega: true,
  };

  // Guardia final: ningún valor no finito llega al renderizador.
  const finito = (n) => typeof n === 'number' && isFinite(n);
  if (
    !finito(resultado.v0) ||
    !finito(resultado.tiempoVuelo) ||
    !finito(resultado.posicionFinal.x) ||
    !finito(resultado.posicionFinal.y) ||
    !finito(resultado.alturaMaxima)
  ) {
    return {
      despega: false,
      mensaje: 'Error interno: la simulación produjo valores no válidos.',
    };
  }

  return resultado;
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
  integrarBalistica,
  simularLanzamiento,
};

// Alias de conveniencia para consumir directamente `fisica` en tests.
window.fisica = window.appNamespace.fisica;

// --- UI: formulario de lanzamiento (Task 6) ---
(function inicializarFormularioLanzamiento() {
  if (typeof document === 'undefined' || typeof document.getElementById !== 'function') {
    return;
  }
  const form = document.getElementById('form-lanzamiento');
  if (!form) {
    return;
  }

  const inputs = {
    litrosProyectil: document.getElementById('litros-proyectil'),
    litrosAgua: document.getElementById('litros-agua'),
    psi: document.getElementById('psi'),
    anguloGrados: document.getElementById('angulo'),
  };
  const errores = {
    litrosProyectil: document.getElementById('error-litros-proyectil'),
    litrosAgua: document.getElementById('error-litros-agua'),
    psi: document.getElementById('error-psi'),
    anguloGrados: document.getElementById('error-angulo'),
  };
  const boton = form.querySelector('button[type="submit"]');
  const resultado = document.getElementById('resultado');
  const litrosTanque = V_TANQUE / LITRO_A_M3; // 2 L

  function limpiarErrores() {
    for (const clave of Object.keys(errores)) {
      if (errores[clave]) errores[clave].textContent = '';
    }
    if (resultado) {
      resultado.textContent = '';
    }
  }

  function mostrarError(clave, mensaje) {
    if (errores[clave]) errores[clave].textContent = mensaje;
  }

  function validar() {
    limpiarErrores();

    const litrosProyectil = Number(inputs.litrosProyectil.value);
    const litrosAgua = Number(inputs.litrosAgua.value);
    const psi = Number(inputs.psi.value);
    const anguloGrados = Number(inputs.anguloGrados.value);

    let valido = true;

    if (!(litrosProyectil > 0)) {
      mostrarError('litrosProyectil', 'Debe ser mayor que 0.');
      valido = false;
    }
    if (!(litrosAgua > 0) || litrosAgua > litrosTanque) {
      mostrarError('litrosAgua', `Debe ser mayor que 0 y menor o igual a ${litrosTanque} L.`);
      valido = false;
    }
    if (!(psi > 0)) {
      mostrarError('psi', 'Debe ser mayor que 0.');
      valido = false;
    }
    if (!(anguloGrados >= 0 && anguloGrados <= 90)) {
      mostrarError('anguloGrados', 'Debe estar entre 0 y 90 grados.');
      valido = false;
    }

    return valido
      ? { litrosProyectil, litrosAgua, psi, anguloGrados }
      : null;
  }

  form.addEventListener('submit', (evento) => {
    evento.preventDefault();
    const datos = validar();
    if (!datos) {
      return;
    }
    if (boton) {
      boton.disabled = true;
    }
    try {
      window.lastSimulation = simularLanzamiento(datos);
      if (resultado) {
        if (window.lastSimulation.despega) {
          resultado.textContent =
            'Lanzamiento OK — v0: ' +
            window.lastSimulation.v0.toFixed(2) +
            ' m/s, alcance: ' +
            window.lastSimulation.posicionFinal.x.toFixed(2) +
            ' m.';
        } else {
          resultado.textContent = window.lastSimulation.mensaje;
        }
      }
      console.log('window.lastSimulation:', window.lastSimulation);
      if (typeof window.appNamespace.renderizarVisualizacion === 'function') {
        window.appNamespace.renderizarVisualizacion(window.lastSimulation);
      }
    } catch (error) {
      if (resultado) {
        resultado.textContent = 'Error al simular: ' + error.message;
      }
      console.error(error);
    } finally {
      if (boton) {
        boton.disabled = false;
      }
    }
  });
})();

// --- Visualización: trayectoria (canvas), resumen y tabla de puntos (Task 7) ---
(function inicializarVisualizacion() {
  if (
    typeof document === 'undefined' ||
    typeof document.getElementById !== 'function'
  ) {
    return;
  }

  const canvas = document.getElementById('trayectoria');
  const resumen = document.getElementById('resumen');
  const tabla = document.getElementById('puntos');
  const envoltorio = document.getElementById('envoltorio-puntos');
  const tablaBody = tabla && tabla.querySelector('tbody');
  const estadoVacio = document.getElementById('estado-vacio');
  if (!canvas || !resumen || !tablaBody || !estadoVacio || !envoltorio) {
    return;
  }

  const ctx = canvas.getContext('2d');
  const COLOR_ACENTO = '#1a3c5e';
  const COLOR_EJES = '#666';
  const MARGEN = { izquierda: 46, derecha: 20, arriba: 18, abajo: 42 };

  function escapeHtml(texto) {
    const elemento = document.createElement('div');
    elemento.textContent = texto;
    return elemento.innerHTML;
  }

  function ajustarCanvas() {
    const dpr = Math.max(1, window.devicePixelRatio || 1);
    const rect = canvas.getBoundingClientRect();
    const ancho = Math.max(1, Math.round(rect.width * dpr));
    const alto = Math.max(1, Math.round(rect.height * dpr));
    canvas.width = ancho;
    canvas.height = alto;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  }

  function limpiarLienzo() {
    ctx.save();
    ctx.setTransform(1, 0, 0, 1, 0, 0);
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    ctx.restore();
  }

  function escalasPlot(puntos, anchoCss, altoCss) {
    let xMax = 0;
    let yMax = 0;
    for (const p of puntos) {
      if (p.x > xMax) xMax = p.x;
      if (p.y > yMax) yMax = p.y;
    }
    if (!(xMax > 0)) xMax = 1;
    if (!(yMax > 0)) yMax = 0.5;
    return {
      ancho: anchoCss,
      alto: altoCss,
      anchoPlot: anchoCss - MARGEN.izquierda - MARGEN.derecha,
      altoPlot: altoCss - MARGEN.arriba - MARGEN.abajo,
      xMax: xMax * 1.08,
      yMax: yMax * 1.10,
    };
  }

  function pxX(plot, x) {
    return MARGEN.izquierda + (x / plot.xMax) * plot.anchoPlot;
  }

  function pxY(plot, altoCss, y) {
    return altoCss - MARGEN.abajo - (y / plot.yMax) * plot.altoPlot;
  }

  function dibujarEjes(plot, altoCss) {
    if (!(plot.xMax > 0) || !(plot.yMax > 0)) {
      return;
    }
    const x0 = pxX(plot, 0);
    const y0 = pxY(plot, altoCss, 0);

    ctx.strokeStyle = COLOR_EJES;
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(x0, y0);
    ctx.lineTo(pxX(plot, plot.xMax), y0); // eje X
    ctx.moveTo(x0, y0);
    ctx.lineTo(x0, pxY(plot, altoCss, plot.yMax)); // eje Y
    ctx.stroke();

    const ticks = 5;
    ctx.fillStyle = '#333';
    ctx.font = '10px sans-serif';

    // Marcas y etiquetas del eje X.
    for (let i = 0; i <= ticks; i += 1) {
      const x = (plot.xMax / ticks) * i;
      const xPx = pxX(plot, x);
      ctx.beginPath();
      ctx.moveTo(xPx, y0);
      ctx.lineTo(xPx, y0 + 4);
      ctx.stroke();
      ctx.textAlign = 'center';
      ctx.textBaseline = 'top';
      ctx.fillText(x.toFixed(0), xPx, y0 + 6);
    }
    ctx.textAlign = 'center';
    ctx.fillText('X (m)', pxX(plot, plot.xMax), y0 + 22);

    // Marcas y etiquetas del eje Y.
    for (let i = 0; i <= ticks; i += 1) {
      const y = (plot.yMax / ticks) * i;
      const yPx = pxY(plot, altoCss, y);
      ctx.beginPath();
      ctx.moveTo(x0, yPx);
      ctx.lineTo(x0 - 4, yPx);
      ctx.stroke();
      ctx.textAlign = 'right';
      ctx.textBaseline = 'middle';
      ctx.fillText(y.toFixed(0), x0 - 8, yPx);
    }
    ctx.textAlign = 'right';
    ctx.fillText('Y (m)', x0 - 8, MARGEN.arriba / 2);

    // Origen (lanzador).
    ctx.fillStyle = '#111';
    ctx.beginPath();
    ctx.arc(x0, y0, 3, 0, 2 * Math.PI);
    ctx.fill();
    ctx.fillStyle = '#333';
    ctx.font = '10px sans-serif';
    ctx.textAlign = 'right';
    ctx.textBaseline = 'bottom';
    ctx.fillText('Lanzador', x0 - 8, y0 - 8);
  }

  function dibujarTrayectoria(puntos, plot, altoCss) {
    if (!puntos || puntos.length === 0) {
      return;
    }
    ctx.strokeStyle = COLOR_ACENTO;
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    let primero = true;
    for (const p of puntos) {
      const xPx = pxX(plot, p.x);
      const yPx = pxY(plot, altoCss, p.y);
      if (primero) {
        ctx.moveTo(xPx, yPx);
        primero = false;
      } else {
        ctx.lineTo(xPx, yPx);
      }
    }
    ctx.stroke();

    const ultimo = puntos[puntos.length - 1];
    ctx.fillStyle = COLOR_ACENTO;
    ctx.beginPath();
    ctx.arc(pxX(plot, ultimo.x), pxY(plot, altoCss, ultimo.y), 3, 0, 2 * Math.PI);
    ctx.fill();
  }

  function renderizar(sim) {
    ajustarCanvas();
    const dpr = window.devicePixelRatio || 1;
    const anchoCss = canvas.width / dpr;
    const altoCss = canvas.height / dpr;

    const datosValidos =
      sim &&
      sim.despega === true &&
      Array.isArray(sim.puntos) &&
      sim.puntos.length > 0 &&
      typeof sim.v0 === 'number' &&
      isFinite(sim.v0) &&
      typeof sim.tiempoVuelo === 'number' &&
      isFinite(sim.tiempoVuelo) &&
      sim.posicionFinal &&
      typeof sim.posicionFinal.x === 'number' &&
      isFinite(sim.posicionFinal.x) &&
      typeof sim.posicionFinal.y === 'number' &&
      isFinite(sim.posicionFinal.y) &&
      typeof sim.alturaMaxima === 'number' &&
      isFinite(sim.alturaMaxima);

    if (!datosValidos) {
      estadoVacio.hidden = true;
      canvas.hidden = true;
      tabla.hidden = true;
      envoltorio.hidden = true;
      limpiarLienzo();
      resumen.hidden = false;
      let mensajeHtml = '';
      if (sim && sim.mensaje) {
        mensajeHtml = '<p class="mensaje">' + escapeHtml(sim.mensaje) + '</p>';
      } else if (sim && sim.despega === true) {
        // despega: true pero valores no finitos: defensa contra NaN en el render.
        mensajeHtml =
          '<p class="mensaje">Error: la simulación produjo valores no válidos.</p>';
      }
      resumen.innerHTML = mensajeHtml;
      return;
    }

    estadoVacio.hidden = true;
    canvas.hidden = false;
    tabla.hidden = false;
    envoltorio.hidden = false;
    resumen.hidden = false;
    limpiarLienzo();

    const plot = escalasPlot(sim.puntos, anchoCss, altoCss);
    dibujarEjes(plot, altoCss);
    dibujarTrayectoria(sim.puntos, plot, altoCss);

    resumen.innerHTML =
      '<div class="fila"><span class="etiqueta">v₀</span><span class="valor">' +
      sim.v0.toFixed(2) +
      ' m/s</span></div>' +
      '<div class="fila"><span class="etiqueta">Tiempo de vuelo</span><span class="valor">' +
      sim.tiempoVuelo.toFixed(2) +
      ' s</span></div>' +
      '<div class="fila"><span class="etiqueta">Posición final</span><span class="valor">(' +
      sim.posicionFinal.x.toFixed(2) +
      ' m, ' +
      sim.posicionFinal.y.toFixed(2) +
      ' m)</span></div>' +
      '<div class="fila"><span class="etiqueta">Altura máxima</span><span class="valor">' +
      sim.alturaMaxima.toFixed(2) +
      ' m</span></div>';

    tablaBody.innerHTML = '';
    const fragmento = document.createDocumentFragment();
    for (const p of sim.puntos) {
      const fila = document.createElement('tr');
      fila.innerHTML =
        '<td>' +
        p.t.toFixed(3) +
        '</td><td>' +
        p.x.toFixed(3) +
        '</td><td>' +
        p.y.toFixed(3) +
        '</td><td>' +
        p.vx.toFixed(3) +
        '</td><td>' +
        p.vy.toFixed(3) +
        '</td>';
      fragmento.appendChild(fila);
    }
    tablaBody.appendChild(fragmento);
  }

  function redibujar() {
    const sim = window.lastSimulation;
    if (sim && sim.despega) {
      renderizar(sim);
    }
  }

  window.addEventListener('resize', redibujar);

  window.appNamespace = window.appNamespace || {};
  window.appNamespace.renderizarVisualizacion = renderizar;

  // Renderiza si ya hay una simulación previa (por ejemplo, tras recargar).
  if (window.lastSimulation) {
    renderizar(window.lastSimulation);
  }
})();
