^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_utils_rclcpp
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

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
