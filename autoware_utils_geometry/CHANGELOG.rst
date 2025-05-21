^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_utils_geometry
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.4.2 (2025-05-21)
------------------
* fix: include tf2/convert.hpp instead of tf2/convert.h (`#67 <https://github.com/autowarefoundation/autoware_utils/issues/67>`_)
* Contributors: Takagi, Isamu

1.4.1 (2025-05-15)
------------------
* fix: include `tf2/utils.hpp` instead of `tf2/utils.h` (`#65 <https://github.com/autowarefoundation/autoware_utils/issues/65>`_)
  * include tf2/utils.hpp
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Yutaka Kondo

1.4.0 (2025-04-22)
------------------
* chore: sync files (`#61 <https://github.com/autowarefoundation/autoware_utils/issues/61>`_)
  * chore: sync files
  * style(pre-commit): autofix
  ---------
  Co-authored-by: github-actions <github-actions@github.com>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* feat(autoware_utils_geometry): resolve jazzy maybe-uninitialized (`#63 <https://github.com/autowarefoundation/autoware_utils/issues/63>`_)
  feat(autoware_geometry_utils): resolve maybe-uninitizlied error in jazzy
* ci(pre-commit): quarterly autoupdate (`#60 <https://github.com/autowarefoundation/autoware_utils/issues/60>`_)
  * ci(pre-commit): quarterly autoupdate
  updates:
  - [github.com/igorshubovych/markdownlint-cli: v0.43.0 → v0.44.0](https://github.com/igorshubovych/markdownlint-cli/compare/v0.43.0...v0.44.0)
  - [github.com/adrienverge/yamllint: v1.35.1 → v1.37.0](https://github.com/adrienverge/yamllint/compare/v1.35.1...v1.37.0)
  - [github.com/scop/pre-commit-shfmt: v3.10.0-2 → v3.11.0-1](https://github.com/scop/pre-commit-shfmt/compare/v3.10.0-2...v3.11.0-1)
  - [github.com/pycqa/isort: 5.13.2 → 6.0.1](https://github.com/pycqa/isort/compare/5.13.2...6.0.1)
  - [github.com/psf/black: 24.10.0 → 25.1.0](https://github.com/psf/black/compare/24.10.0...25.1.0)
  - [github.com/pre-commit/mirrors-clang-format: v19.1.6 → v20.1.0](https://github.com/pre-commit/mirrors-clang-format/compare/v19.1.6...v20.1.0)
  - [github.com/cpplint/cpplint: 2.0.0 → 2.0.1](https://github.com/cpplint/cpplint/compare/2.0.0...2.0.1)
  - [github.com/python-jsonschema/check-jsonschema: 0.30.0 → 0.32.1](https://github.com/python-jsonschema/check-jsonschema/compare/0.30.0...0.32.1)
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(autoware_utils_geometry): fix procedure to check if point is on edge (`#56 <https://github.com/autowarefoundation/autoware_utils/issues/56>`_)
  * fix procedure to check if point is on edge
  * add test cases
  ---------
* fix: convex_full header not found (`#62 <https://github.com/autowarefoundation/autoware_utils/issues/62>`_)
* test(autoware_utils): port unit tests from autoware.universe (`#28 <https://github.com/autowarefoundation/autoware_utils/issues/28>`_)
* fix: boost convex_hull for newer boost versions (`#58 <https://github.com/autowarefoundation/autoware_utils/issues/58>`_)
  * Fix boost convex_hull for newer boost versions
  * Specify the distros for searchability
  * Use boost version for explicity
  ---------
* Contributors: Mitsuhiro Sakamoto, Takagi, Isamu, Tim Clephas, awf-autoware-bot[bot], pre-commit-ci[bot], storrrrrrrrm

1.3.0 (2025-03-21)
------------------
* unify version
* update changelog
* feat(autoware_utils_geometry): split package (`#48 <https://github.com/autowarefoundation/autoware_utils/issues/48>`_)
  * feat(autoware_utils_geometry): split package
  * compatibility header
  * rename namespace
  * fix namespace
  * fix path
  * fix include path
  ---------
* feat(autoware_utils_pcl): split package (`#29 <https://github.com/autowarefoundation/autoware_utils/issues/29>`_)
  * feat(autoware_utils_pcl): split package
  * add maintainer
  * fix build error
  * add compatibility
  * rename package
  * update readme
  ---------
* Contributors: Takagi, Isamu, Yutaka Kondo

* feat(autoware_utils_geometry): split package (`#48 <https://github.com/autowarefoundation/autoware_utils/issues/48>`_)
  * feat(autoware_utils_geometry): split package
  * compatibility header
  * rename namespace
  * fix namespace
  * fix path
  * fix include path
  ---------
* feat(autoware_utils_pcl): split package (`#29 <https://github.com/autowarefoundation/autoware_utils/issues/29>`_)
  * feat(autoware_utils_pcl): split package
  * add maintainer
  * fix build error
  * add compatibility
  * rename package
  * update readme
  ---------
* Contributors: Takagi, Isamu

1.2.0 (2025-02-26)
------------------

1.1.0 (2025-01-27)
------------------

1.0.0 (2024-05-02)
------------------
