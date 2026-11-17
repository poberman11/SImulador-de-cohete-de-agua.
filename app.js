const appNamespace = {};

window.appNamespace = {
  fisica: {
    // --- Constantes físicas (SI) ---
    GRAVEDAD: 9.81,          // m/s²
    RHO_AGUA: 1000,          // kg/m³
    LITRO_A_M3: 0.001,       // 1 litro = 0.001 m³
    PSI_A_PASCAL: 6894.76,   // 1 PSI = 6894.76 Pa
    ATMOSFERA_PASCAL: 101325,// 1 atm = 101325 Pa
    RHO_AIRE: 1.225,         // kg/m³ (aire a 15 °C, nivel del mar)

    /** Convierte litros a metros cúbicos. */
    litrosAMetrosCubicos(litros) {
      return litros * this.LITRO_A_M3;
    },

    /** Convierte PSI a pascales. */
    psiAPascales(psi) {
      return psi * this.PSI_A_PASCAL;
    },

    /**
     * Presión manométrica del aire del tanque por la ley de Boyle.
     * P1_abs * V1 = P2_abs * V2  =>  P2_manométrico = P2_abs - P_atm.
     */
    presionManometrica(psiInicial, volAireInicial, volAireActual, volTanque) {
      const v = Math.min(Math.max(volAireActual, 1e-9), volTanque);
      const p1Abs = this.psiAPascales(psiInicial) + this.ATMOSFERA_PASCAL;
      const p2Abs = p1Abs * (volAireInicial / v);
      return Math.max(0, p2Abs - this.ATMOSFERA_PASCAL);
    },

    /** Velocidad del chorro: v = sqrt(2 * ΔP / ρ_agua). */
    velocidadJet(deltaPascales) {
      return Math.sqrt(2 * deltaPascales / this.RHO_AGUA);
    },

    /**
     * Empuje del chorro de agua.
     * Devuelve { fuerza, caudalMasico }.
     */
    empuje(psi, volAgua, volTanque, areaBoquilla, cd) {
      if (volAgua <= 0) {
        return { fuerza: 0, caudalMasico: 0 };
      }
      const deltaP = this.psiAPascales(psi);
      const v = this.velocidadJet(deltaP);
      const caudalMasico = cd * this.RHO_AGUA * areaBoquilla * v;
      const fuerza = caudalMasico * v;
      return { fuerza, caudalMasico };
    },

    /**
     * Fuerza de arrastre aerodinámico, vector opuesto a la velocidad.
     * Devuelve { fx, fy }.
     */
    arrastre(velocidad, areaFrontal, cdAir, rhoAire) {
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
    },
  },
};

// Alias de conveniencia para consumir directamente `fisica` en tests.
window.fisica = window.appNamespace.fisica;
