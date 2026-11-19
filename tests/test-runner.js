// Minimal test runner: describe / it / expect.
// Works as ESM in the browser and via `await import()` in Node.
const results = { passes: 0, failures: 0, details: [] };

export function describe(name, fn) {
  fn();
}

export function it(name, fn) {
  try {
    fn();
    results.passes += 1;
    results.details.push({ name, ok: true });
    console.log(`✓ ${name}`);
  } catch (e) {
    results.failures += 1;
    results.details.push({ name, ok: false, error: e.message });
    console.error(`✗ ${name} — ${e.message}`);
  }
}

export function expect(actual) {
  return {
    toBe(expected) {
      if (actual !== expected) {
        throw new Error(`expected ${actual} to be ${expected}`);
      }
    },
    toBeCloseTo(expected, decimals) {
      const tol = Math.pow(10, -decimals);
      if (Math.abs(actual - expected) > tol) {
        throw new Error(`expected ${actual} to be close to ${expected} (${decimals} decimals)`);
      }
    },
    toBeGreaterThan(expected) {
      if (!(actual > expected)) {
        throw new Error(`expected ${actual} > ${expected}`);
      }
    },
    toBeGreaterThanOrEqual(expected) {
      if (!(actual >= expected)) {
        throw new Error(`expected ${actual} >= ${expected}`);
      }
    },
    toBeLessThan(expected) {
      if (!(actual < expected)) {
        throw new Error(`expected ${actual} < ${expected}`);
      }
    },
    toBeLessThanOrEqual(expected) {
      if (!(actual <= expected)) {
        throw new Error(`expected ${actual} <= ${expected}`);
      }
    },
  };
}

export function summary() {
  return { ...results };
}
