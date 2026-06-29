^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_utils_rclcpp
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* feat(autoware_utils_rclcpp): templatize NodeT for `get_or_declare_parameter` (`#112 <https://github.com/autowarefoundation/autoware_utils/issues/112>`_)
  * templatize NodeT for get_or_declare_parameter
  * style(pre-commit): autofix
  * delete unnecessary comments
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Koichi Imai

1.8.0 (2026-06-10)
------------------
* test(autoware_utils_rclcpp): cover check_qos depth>1 throw for Latest/Newest (`#107 <https://github.com/autowarefoundation/autoware_utils/issues/107>`_)
  The QoS-validation branch InterProcessPollingSubscriber's check_qos() was
  uncovered: existing tests only construct subscribers with depth==1 (or the
  All policy with QoS{10}, whose check_qos is a no-op). The throw path for the
  Latest and Newest policies and the All-deep-qos no-throw path were dark.
  Add gtest cases that assert:
  - Latest/Newest construction with rclcpp::QoS{10} throws std::invalid_argument.
  - Latest/Newest construction with rclcpp::QoS{1} (depth==1) does not throw.
  - All construction with rclcpp::QoS{10} does not throw (check_qos ignores depth).
  Type aliases keep the subscriber template comma out of the gtest macro
  argument lists. Tests only; no production-code change.
* Contributors: Yutaka Kondo

1.7.2 (2026-05-01)
------------------
* fix: to be consistent version in all package.xml(s)
* Contributors: github-actions

1.7.0 (2026-03-12)
------------------

1.6.0 (2026-02-20)
------------------
* feat(autoware_utils_rclcpp): add function to get the timestamp of the latest message (`#91 <https://github.com/autowarefoundation/autoware_utils/issues/91>`_)
  * feat(polling_subscriber): add getter of latest_timestamp.
  * fix(rclcpp): fixed dead code.
  * add explanation of PollingSubscriber in README.md
  * update explanation of PollingSubscriber in README.md
  * add testcase to check initial value of PollingSubscriber
  * style(pre-commit): autofix
  * use std::optional instead of the magic number.
  * updated README.md for the previous commit
  * chore(autoware_utils_rclcpp): add precondition of latest_timestamp() method
  * style(pre-commit): autofix
  * change latest_timestamp() to return std::nullopt if there is no valid data received
  * style(pre-commit): autofix
  * chore(polling_subscriber): rename method name
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Takayuki AKAMINE

1.5.0 (2025-12-30)
------------------

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
* feat(autoware_utils_rclcpp): split package (`#40 <https://github.com/autowarefoundation/autoware_utils/issues/40>`_)
  * feat(autoware_utils_rclcpp): split package
  * update test
  * update readme
  ---------
* Contributors: Takagi, Isamu, Yutaka Kondo

* feat(autoware_utils_rclcpp): split package (`#40 <https://github.com/autowarefoundation/autoware_utils/issues/40>`_)
  * feat(autoware_utils_rclcpp): split package
  * update test
  * update readme
  ---------
* Contributors: Takagi, Isamu

1.2.0 (2025-02-26)
------------------

1.1.0 (2025-01-27)
------------------

1.0.0 (2024-05-02)
------------------
