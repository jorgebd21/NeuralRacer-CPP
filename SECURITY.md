# Security Policy

## Supported Versions

Security updates and critical memory safety patches are provided for the following versions:

| Version | Supported | Notes |
| :---: | :---: | :--- |
| **v2.6.x** | :white_check_mark: | Current active production release. |
| **v2.5.x** | :white_check_mark: | Critical security and memory fixes only. |
| < v2.5.0 | :x: | End of Life (EOL). Please upgrade to the latest release. |

---

## Scope of Security Considerations

As a high-performance C++17 simulation engine incorporating native SIMD instructions and file I/O, security considerations focus primarily on:

1. **Untrusted JSON Deserialization:** Parsing arbitrary genome brains (`data/best.json`) or procedural track topologies (`data/tracks/*.json`). Input parsing must never cause buffer overflows, memory corruption, or infinite loops.
2. **HPC Memory Safety:** Contiguous ring buffers (`FixedRingBuffer`), 1D spatial hash arenas (`FlatSpatialGrid`), and AVX2 vector registers must enforce strict boundary checks and avoid undefined behavior.
3. **Concurrency & Thread Safety:** OpenMP parallel sections must prevent data races, lock contention, or memory corruption.

---

## Reporting a Vulnerability

If you discover a potential security vulnerability or memory hazard in **NeuralRacer-CPP**:

1. **Do not open a public issue.**
2. Report the vulnerability privately via **GitHub Security Advisory** on the repository or email the maintainer directly at `jorgebaezadiaz@gmail.com` with the subject tag `[SECURITY: NeuralRacer-CPP]`.
3. Provide a clear description including:
   * Engine version and operating system.
   * Minimal reproducible test case (e.g., malformed JSON payload or specific compiler flags).
   * Potential impact or attack vector.

### Response Commitment

* **Initial Acknowledgment:** Within 48 hours of report submission.
* **Triage & Assessment:** Within 5 business days.
* **Fix & Release Advisory:** Coordinated public disclosure and patch release once verified.
