^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_utils_diagnostics
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* perf(autoware_utils_diagnostics): drop per-cycle shrink_to_fit in clear() (`#106 <https://github.com/autowarefoundation/autoware_utils/issues/106>`_)
  clear() called values.shrink_to_fit() right after values.clear(), releasing
  the underlying buffer every cycle and forcing the next add_key_value() push_backs
  to reallocate on hot localization callbacks. Drop the shrink_to_fit() call so the
  buffer capacity is retained across cycles; .clear() and the level/message resets
  are unchanged, so published messages are output-identical.
  Add a pub/sub characterization test that pins the observable behavior across
  clear() cycles (value contents/order, level, message, dedup), proving the removal
  is behavior-preserving.
* feat: make DiagnosticsInterface's node argument as template type (`#102 <https://github.com/autowarefoundation/autoware_utils/issues/102>`_)
  * templatize DiagnosticsInterface
  * copilot review
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Koichi Imai, Yutaka Kondo

1.7.2 (2026-05-01)
------------------
* fix: to be consistent version in all package.xml(s)
* Contributors: github-actions

1.7.0 (2026-03-12)
------------------

1.6.0 (2026-02-20)
------------------

1.5.0 (2025-12-30)
------------------
* feat: add `timeout_diagnostics` (`#84 <https://github.com/autowarefoundation/autoware_utils/issues/84>`_)
  * add: timeout_diagnostics (see below)
  * This changes are copied from `autoware_control_command_gate`
  - https://github.com/autowarefoundation/autoware_universe/tree/01ff80101dfc78a0288f063e975e6e16ac042bf7/control/autoware_control_command_gate
  * We are going to use `timeout_diagnostics` via `autoware_utils` not `autoware_universe` in near future for adaptation to new ADAPI (>= 1.9.0)
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Junya Sasaki

1.4.2 (2025-05-21)
------------------

1.4.1 (2025-05-15)
------------------

1.4.0 (2025-04-22)
------------------

1.3.0 (2025-03-21)
------------------
* unify version
* update changelog
* feat(autoware_utils_diagnostics): split package (`#45 <https://github.com/autowarefoundation/autoware_utils/issues/45>`_)
* Contributors: Takagi, Isamu, Yutaka Kondo

* feat(autoware_utils_diagnostics): split package (`#45 <https://github.com/autowarefoundation/autoware_utils/issues/45>`_)
* Contributors: Takagi, Isamu

1.2.0 (2025-02-26)
------------------

1.1.0 (2025-01-27)
------------------

1.0.0 (2024-05-02)
------------------
