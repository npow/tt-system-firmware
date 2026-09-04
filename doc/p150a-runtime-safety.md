# P150A runtime safety contract

This branch addresses failures that turn workload completion, cleanup, stalled
peripherals, or power control into loss of ARC/PCIe management. These are software
invariants, conditional on the hardware's documented register and DMA behavior.
Passing native simulation does not establish a physical instantaneous power bound
or prove the absence of all hardware and software bugs.

| Invariant | Enforcement | Regression evidence |
| --- | --- | --- |
| Every ARC command needs an explicit admission policy; unknown commands are rejected before dispatch. | `REGISTER_MESSAGE` compile assertion and central `command_allowed` gate, including builtins. | Exact registered surface, unknown/content-dependent commands, rejection with no register effects and subsequent queue responsiveness. |
| A runtime power request cannot gate MRISC PHY, Tensix, or L2CPU without a quiesce handshake. | `apply_power_settings` validates the entire request before I/O; no runtime quiesce handshake exists, so disable requests fail. AICLK idle remains allowed. | Every defined flag combination at validity counts 4 and 15; rejected requests preserve power state and register call counts. This regression fails on the preceding implementation. |
| Failed voltage/clock changes cannot silently authorize a later increase. | DVFS lock, direction-sensitive busy/FMAX rollback, voltage acknowledgement, bounded PLL rollback/reference-clock fallback. | Failed GO_BUSY and host-FMAX requests followed by another target calculation; regulator failure tests. |
| Reset selection is single-writer and reset teardown excludes hardware handlers. | Atomic first-reset latch and central reset-pending gate. | A second reset cannot replace the first; only queue-local protocol messages remain admitted. |
| A timeout cannot allow DMA to overwrite an expired stack object or make an old response satisfy a new transfer. | Persistent buffers and quarantine of engines whose timed-out transfers cannot be aborted. | Source/production-build review; native tests do not emulate the physical DMA engines. |
| Board power protection cannot intentionally reset or gate management to enforce its limit. | Firmware startup policy, strict board arbiter, stale-sample clamp, bounded upward AICLK slew; no management-domain shutdown in that response. | Default startup, early DMC policy, strict arbitration, stale/recovered and saturated samples. Physical transient behavior still needs measurement. |
| Repeated PGOOD faults do not permanently remove eligibility for recovery after power stabilizes. | Fault reporting is separated from reset-release eligibility. | PGOOD repeated-drop/stable-rise tests. |

The matching KMD retains valid MRISC/Tensix/L2CPU power bits on all Blackhole power
requests, including last-close aggregation. Its host-only test checks every
16-bit flag value at all 16 flag-validity counts (1,048,576 combinations), including
preservation of AICLK, unknown bits, and idempotence. Firmware independently rejects
unsafe raw requests, so driver cooperation is not the sole enforcement point.

## Residual boundaries

- Current UMD/Luwen can map BAR/TLB controls directly, and on-device kernels can
  generate NoC traffic. An ioctl allowlist does not authenticate those paths.
- A software polling deadline cannot bound an individual synchronous PCIe MMIO
  load if the endpoint/fabric cannot return a completion. AER/DPC provides error
  handling after a failure; it is not proof that platform firmware cannot reboot.
- No source-aware NoC firewall and bounded error responder was found in the
  audited source. Complete isolation of arbitrary broken or hostile device code
  would need that facility and restricted host mappings/typed management APIs.
- The power controller is reactive and telemetry is sampled. A 300 W policy does
  not imply that physical power never exceeds 300 W between samples.

Do not test destructive commands or deliberately stall ARC on the production card
to validate these invariants. Use simulation and source/binary inspection first.
Hardware acceptance uses the exact persistent bundle and KMD: startup policy,
safe telemetry, sustained 120-core/GDDR/L2CPU load with a requested 1350 MHz ceiling,
cleanup, and real Servo workloads. Record actual clocks/power, heartbeat, boot ID,
firmware faults and PCIe errors. A clean load phase with failed cleanup is a failed
acceptance run.
