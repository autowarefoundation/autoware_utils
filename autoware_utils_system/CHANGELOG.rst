^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_utils_system
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.8.0 (2026-06-09)
------------------
* test(autoware_utils_system): cover LRUCache get/clear/size/empty/capacity and LRU reorder (`#104 <https://github.com/autowarefoundation/autoware_utils/issues/104>`_)
  The existing parameterized test only exercised put() and contains(). Add
  gtest cases that assert concrete values and branches for the previously
  untested API: get() hit/miss, the splice-to-front recency refresh on get(),
  put() overwriting an existing key while refreshing recency, clear(), and the
  size()/empty()/capacity() invariants (size never exceeds capacity).
  Tests only; no production-code change.
* Contributors: Yutaka Kondo

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
* chore: replace INSTANTIATE_TEST_CASE_P with INSTANTIATE_TEST_SUITE_P (`#73 <https://github.com/autowarefoundation/autoware_utils/issues/73>`_)
  replace INSTANTIATE_TEST_SUITE_P with INSTANTIATE_TEST_CASE_P
* Contributors: Daisuke Nishimatsu

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
* feat(autoware_utils_system): split package (`#38 <https://github.com/autowarefoundation/autoware_utils/issues/38>`_)
  * feat(autoware_utils_system): split package
  * update for compatibility
  ---------
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* Contributors: Takagi, Isamu, Yutaka Kondo

* feat(autoware_utils_system): split package (`#38 <https://github.com/autowarefoundation/autoware_utils/issues/38>`_)
  * feat(autoware_utils_system): split package
  * update for compatibility
  ---------
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* Contributors: Takagi, Isamu

1.2.0 (2025-02-26)
------------------

1.1.0 (2025-01-27)
------------------

1.0.0 (2024-05-02)
------------------
