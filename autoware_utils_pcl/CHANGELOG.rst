^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_utils_pcl
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.8.0 (2026-06-09)
------------------
* test(autoware_utils_pcl): cover transform_point_cloud_from_ros_msg with hand-computed assertions (`#105 <https://github.com/autowarefoundation/autoware_utils/issues/105>`_)
  The templated transform_point_cloud_from_ros_msg<Scalar> in pcl_conversion.hpp
  had zero test coverage (the existing test/cases/transform.cpp only exercises
  transform_pointcloud from transforms.hpp).
  Add a new gtest file test/cases/pcl_conversion.cpp that builds
  sensor_msgs::msg::PointCloud2 messages by hand (three tightly-packed FLOAT32
  x/y/z fields) and asserts hand-computed output coordinates with Scalar=float:
  - identity transform on an unorganized (height==1) cloud: output size/width/
  height/is_dense and each point copied exactly;
  - a known 90deg-about-Z rotation plus translation on an unorganized cloud,
  with per-point expected coordinates computed by hand;
  - an organized 2x2 cloud (height>1) exercising the row/col nested loop;
  - header.frame_id propagation;
  - a sizeof(pcl::PointXYZ)==16 stride sanity check.
  The file is picked up by the existing GLOB_RECURSE in CMakeLists.txt. Add
  sensor_msgs to package.xml (kept sorted) for the PointCloud2 dependency.
  No production code changes.
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

1.4.2 (2025-05-21)
------------------

1.4.1 (2025-05-15)
------------------

1.4.0 (2025-04-22)
------------------
* test(autoware_utils): port unit tests from autoware.universe (`#28 <https://github.com/autowarefoundation/autoware_utils/issues/28>`_)
* feat: remove managed transform buffer (`#41 <https://github.com/autowarefoundation/autoware_utils/issues/41>`_)
* Contributors: Amadeusz Szymko, storrrrrrrrm

1.3.0 (2025-03-21)
------------------
* unify version
* update changelog
* feat(autoware_utils_tf): rename package (`#46 <https://github.com/autowarefoundation/autoware_utils/issues/46>`_)
* feat(autoware_utils_pcl): split package (`#29 <https://github.com/autowarefoundation/autoware_utils/issues/29>`_)
  * feat(autoware_utils_pcl): split package
  * add maintainer
  * fix build error
  * add compatibility
  * rename package
  * update readme
  ---------
* Contributors: Takagi, Isamu, Yutaka Kondo

* feat(autoware_utils_tf): rename package (`#46 <https://github.com/autowarefoundation/autoware_utils/issues/46>`_)
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
