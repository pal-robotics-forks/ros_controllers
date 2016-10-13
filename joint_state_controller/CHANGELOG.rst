^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package joint_state_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

0.2.4 (2016-03-07)
------------------
* Removed added temporary files
* Fixed catkin issues
* Contributors: Hilario Tome, hilariotome

0.2.3 (2015-10-08)
------------------
* Address -Wunused-parameter warnings
* Contributors: Adolfo Rodriguez Tsouroukdissian

0.2.2 (2015-07-01)
------------------
* Add extra joints support:
  Allow to optionally specify a set of extra joints for state publishing that
  are not contained in the JointStateInterface associated to the controller.
  The state of these joints can be specified via ROS parameters, and remains
  constant over time.
* Add test suite
* Migrate to package format2
* Contributors: Adolfo Rodriguez Tsouroukdissian

0.2.1 (2015-03-19)
------------------

0.2.0 (2015-03-12)
------------------

0.1.0 (2014-10-29)
------------------
* First release of PAL's hydro backport
