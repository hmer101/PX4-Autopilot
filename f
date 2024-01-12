* [32mmain                                                                       [m 64f28c4c07 ekf2: delete unused gps error norm field
  [31mremotes/origin/Differentialdrive_guidance                                  [m 347c6255e7 TEMPORARY COMMIT, DONT USE THIS VERSION
  [31mremotes/origin/FWPosCtrlSeparateVtolTransition                             [m ffa37adae3 FWPosCtrl: Rearrange VTOL transition code into eparate functions.
  [31mremotes/origin/HEAD                                                        [m -> origin/main
  [31mremotes/origin/RC_loss_params                                              [m fbda20e2a9 RC Loss failsafe param docs
  [31mremotes/origin/TECS_airspeedless_stall_prevention                          [m 19cc8d7fd6 [TECS]: WIP Add Stall prevention function into TECS for testing. TODO remove testing code from airspeed selector to forse airspeedless in FW mode. TODO test stall prevention better
  [31mremotes/origin/acceleration-stick-brake-factor                             [m fef8c5e42e StickAccelerationXY: make brake filter configurable
  [31mremotes/origin/acquire-gimbal-control-v2-protocol                          [m 310194185a mission block: acquire gimbal control also for gimbal v2 protocol
  [31mremotes/origin/add_attitude_reference_model                                [m 1b83d720b2 FixedWingPositionControl: Use reference model for all auto modes except when in vtol transition
  [31mremotes/origin/add_mission_interface                                       [m ebe6d57fb8 [Mission]: Update the mission mode to use the planned mission interface library
  [31mremotes/origin/add_rtl_vtol_safe_point_approach                            [m 28c2b756de [RTL] Add VTOL landing approach for safe points the same way as defined for the home position. Perform this VTOL landing approach if optional safe point loiter position are defined
  [31mremotes/origin/attitude-generation                                         [m 7c8dc3d229 TEMP
  [31mremotes/origin/battery_estimates_on_level_flight                           [m 1573d52ec7 flight phase estimation: use tecs height rate reference to check for level flight
  [31mremotes/origin/battery_params_update                                       [m 9b0873f528 battery: allow battery internal resistance and capacity to change dynamically
  [31mremotes/origin/beacon_fusion                                               [m 0e1e1afcf9 Correct dates in the license headers
  [31mremotes/origin/beta                                                        [m 3ad2c641da README.md spelling mistake corrected
  [31mremotes/origin/build_and_deploy_mix                                        [m bc62fa53fb ci: build every target on a single step
  [31mremotes/origin/clear_approaches_on_startup                                 [m 38923584c2 mavlink_mission: clear rally points with approaches on startup
  [31mremotes/origin/debug-symbols-in-px4                                        [m c86a0a9199 cmake: Add debug symbols to the .px4 file
  [31mremotes/origin/dev/h7-socketcan                                            [m 8e01fe661a fixup! Update submodule nuttx
  [31mremotes/origin/dev/socketcan-nuttx-10.3.0                                  [m 05a2a32150 fixup! Update submodules nuttx, apps
  [31mremotes/origin/disable-flight-time-battery-reaction-with-parameter-upstream[m f8e1708ebf failsafe: COM_LOW_BAT_ACT also configures time based low battery action
  [31mremotes/origin/docker-multiarch                                            [m 2eb2b2a17f github: test file changed action
  [31mremotes/origin/double_storage_rally_geofence                               [m 9f81ec3164 mission_base: fix validity on abort landing
  [31mremotes/origin/ekf2-terrain-state-wip                                      [m b6d08fecb3 flow testing
  [31mremotes/origin/ethernet_default_ip                                         [m 412c8cbb3c boards: change default IP from 192.168.0.3 to 10.41.10.2
  [31mremotes/origin/external_modes_ros2_and_multi_conf_ekf                      [m 7531adab3e ekf2-ev: filter q_error for frame correction
  [31mremotes/origin/external_modes_ros2_integration_tests                       [m 3c8b6b999e ci: add external navigation integration tests
  [31mremotes/origin/external_modes_standalone                                   [m 6fef20988e updated warning (#22367)
  [31mremotes/origin/failure-detector-status                                     [m 42221483c4 vehicle_status: Remove duplicate failure detector status
  [31mremotes/origin/feat_fast_descend                                           [m 7c7b5dd465 TECS: allow for fast descend up to maximum airspeed. Use pitch control loop to control max airspeed while giving minimal throttle.
  [31mremotes/origin/flow-use-autopilot-gyro                                     [m e9670be63d HACK: force the EKF to use its own gyro for flow compensation
  [31mremotes/origin/fmu-v6x_SLIP_CM4_testing                                    [m 930459319e Revert "Change target tcp server to Linux Laptop"
  [31mremotes/origin/github_actions                                              [m d72d6ea28e WIP: in tree Dockerfile and github actions push to github registry
  [31mremotes/origin/google-style                                                [m 437148f8f3 Makefile: target to test clang-format on the entire codebase
  [31mremotes/origin/google-style-showcase                                       [m 25ed6b67fb Showcase clang-format google style
  [31mremotes/origin/hamishwillee-patch-1                                        [m 7bccbfecb4 4003_qavr5 : inch in title breaks docs rendering
  [31mremotes/origin/hamishwillee-patch-2                                        [m d74af6d1f8 Delete simpleapp CMakeLists
  [31mremotes/origin/hamishwillee-patch-3                                        [m 2b8db472ff geofence_params.c
  [31mremotes/origin/hinwil-flight-test                                          [m 7e49147bcf Fix feedforward acceleration setpoints for fixedwing offboard position
  [31mremotes/origin/hitl-with-actuators-hack                                    [m 9c616fcf63 HACK to enable actuators in HITL
  [31mremotes/origin/ina238_calibration_upstream                                 [m 66670c71f8 more refactoring
  [31mremotes/origin/jake/navigator-flighttask-refactor                          [m 4fdf69df27 moved Navigator to work_queue. Start refactoring navigator
  [31mremotes/origin/jake/refactor-precland-into-flighttask                      [m 1b226ed241 change land setpoint
  [31mremotes/origin/lp55231                                                     [m bbbaeffd74 Added demo for LP55231
  [31mremotes/origin/maetugr/battery                                             [m 28d63af4dd battery: remove unused _first_parameter_update
  [31mremotes/origin/maetugr/cmake                                               [m 55f0ab8f63 ubuntu: reuse REQUIREMENTS_FILE variable
  [31mremotes/origin/maetugr/limited-airmode                                     [m 3fa95d58cd TEMP2
  [31mremotes/origin/maetugr/mavlink-manual-control-aux                          [m 2668ecaafb Support MAVLink extension MANUAL_CONTROL.aux
  [31mremotes/origin/maetugr/small-findings                                      [m 6cb610ded2 generate_msg_docs: ommit line with no effect
  [31mremotes/origin/main                                                        [m 64f28c4c07 ekf2: delete unused gps error norm field
  [31mremotes/origin/matrix-improvements                                         [m 8fae3801ae matrix: Slice templated on const and non-const matrix cases
  [31mremotes/origin/mc-simulation-control-allocation-default                    [m e13b56f806 Switch simulated solo to control allocation
  [31mremotes/origin/minor-battery-refactor                                      [m e5e58ba290 Battery: minor refactor
  [31mremotes/origin/mpc-velocity-parameter                                      [m 5635af70be Drop opinionated MPC_VEL_MANUAL airframe defaults
  [31mremotes/origin/new_uart_driver_api                                         [m fbfb93ab26 Cleaned up the API and added SBUS support
  [31mremotes/origin/nuttx_master_10.3+_s32k3                                    [m 64e638a6b4 nxp_mr-canhubk3 Add PROBEs for Debugging
  [31mremotes/origin/nxp-dev                                                     [m 5e5a1e1b1f make demo work with pca.
  [31mremotes/origin/omnicopter-testing                                          [m 767cb3dc79 FailureDetector: check if ESCs have current
  [31mremotes/origin/param_add_file                                              [m 4a23597d8a Fix up output
  [31mremotes/origin/pid-class                                                   [m 8c3a76b5ad TEMP
  [31mremotes/origin/pixhawk_v3_internal_mags                                    [m b544b5c446 px4/fmu-v3: correctly identify internal mags (ist8310+hmc5883) on some boards
  [31mremotes/origin/potaito/refactor-precland-into-flighttask                   [m 9551a31a94 add fallback landing
  [31mremotes/origin/potaito/remove-waypoint-alt-below-home-check                [m de56f64076 navigator: remove checkIfBelowHomeAltitude
  [31mremotes/origin/potaito/track-orientation-in-precland                       [m e381da34e5 Update src/modules/landing_target_estimator/landing_target_estimator_params.c
  [31mremotes/origin/pr-DNM-param-pi-testing                                     [m 971fb70d1a DNM px4_fmu-v5x Using Probes
  [31mremotes/origin/pr-List_reverse                                             [m 9012d1aa0e containers/List: fix default add (O(1)) and create new addFront() addBack() methods
  [31mremotes/origin/pr-accel_cal_sphere                                         [m 9955cb4cc8 update accelerometer calibration to use Levenberg Marquardt least squares fit
  [31mremotes/origin/pr-add-airspeed-mode                                        [m 9624fbad1b Add new Airspeed flight mode (only fixed-wing)
  [31mremotes/origin/pr-add-landinggear-plane                                    [m 39c1b52f79 Add wheels to plane model
  [31mremotes/origin/pr-add-zmo-vtol-airframe-master                             [m 7f2f40ac47 add ZMO VTOL config
  [31mremotes/origin/pr-adis16477_debug                                          [m fd014acd47 WIP: adis16477 debug
  [31mremotes/origin/pr-advanced-plane                                           [m fcc0c79c01 Add airframe
  [31mremotes/origin/pr-airspeed_selector_param                                  [m 890ae36e78 airspeed_selector: new ASPD_SEL_EN to control enabling/disabling module
  [31mremotes/origin/pr-alphafilter_improvements                                 [m b68ab63c6a mathlib: AlphaFilter add additional checks and updates methods
  [31mremotes/origin/pr-angular_accel_backward_finite_difference                 [m 779229a41c sensors/vehicle_angular_velocity: use 3 coefficient backward finite difference for angular acceleration (on FIFO data)
  [31mremotes/origin/pr-ark_flow_ekf2                                            [m 423c43ad8e boards: ARK CAN Flow enable ekf2
  [31mremotes/origin/pr-ark_gps_ekf2                                             [m 65e6938779 board: ARK CAN RTK GPS enable ekf2
  [31mremotes/origin/pr-at-command                                               [m 1280a7f92d mavlink: early return in parser
  [31mremotes/origin/pr-autotune-sine-sweep                                      [m addaaf0eb4 atune: paremeterize intial RLS variance
  [31mremotes/origin/pr-battery-reset-filter-vtol-transition-main                [m 1e223b21eb battery: reset current filter when transitioning to FW
  [31mremotes/origin/pr-bbsram_check_point_vehicle_status                        [m 2a8c83356a WIP: BBSRAM check point vehicle_status_s
  [31mremotes/origin/pr-bidirectional-dshot                                      [m 44600804bc Merge remote-tracking branch 'px4/main' into pr-bidirectional-dshot
  [31mremotes/origin/pr-bitcraze_crazyflie_v2.1                                  [m 4e0dab2b29 crazyflie v2.1
  [31mremotes/origin/pr-bmi_accels                                               [m 293e459b9e drivers/imu/bosch: run accels at max rate rather than IMU_GYRO_RATEMAX
  [31mremotes/origin/pr-board_default_i2c_bus_speed                              [m be6cf24c79 move I2C bus clock defaults to board px4_i2c_buses
  [31mremotes/origin/pr-board_sync                                               [m d8148c8e3b boards: sync
  [31mremotes/origin/pr-board_testing                                            [m 63448faf2d [DO NOT MERGE] update NuttX to pr-nuttx_sd_testing
  [31mremotes/origin/pr-boards_f7_spi_dma_threshold                              [m 5f45e46237 [DO NOT MERGE] boards: stm32f7/h7 disable SPI DMA threshold
  [31mremotes/origin/pr-boards_loops_per_msec_review                             [m 39cb0bc79f boards: px4_fmu-v4pro update CONFIG_BOARD_LOOPSPERMSEC
  [31mremotes/origin/pr-boards_matek_h743_clock_tree_adjust                      [m 7004ce102d boards: matek h743-slim clock tree adjust
  [31mremotes/origin/pr-boards_mro_h7_lower_clock                                [m 7d9281d934 boards: mRo H7 lower CPU 480 MHz -> 400 MHz to reduce temperature
  [31mremotes/origin/pr-boards_priority_inheritance_fix_holders                  [m 3837b45a49 boards: NuttX set CONFIG_SEM_PREALLOCHOLDERS=32
  [31mremotes/origin/pr-boards_priority_inheritance_update                       [m 2aa1d4dd6f NuttX disable priority inheritance and adjust parameter save scheduling
  [31mremotes/origin/pr-boards_qiotek_h743                                       [m ff51389f47 WIP: qiotek h743 board support
  [31mremotes/origin/pr-boards_rcc_reset_flags                                   [m 6bef2bdeb1 boards: px4_fmu-v5 print RCC reset flags on boot
  [31mremotes/origin/pr-boards_stm32h7_dma                                       [m 77ac41f123 boards: stm32h7 enable SPI and serial DMA everywhere
  [31mremotes/origin/pr-boards_stm32h7_telem_flow_control_pulldowns              [m 7197cb9eae boards: cubeorange add CTS/RTS pulldowns
  [31mremotes/origin/pr-bosch_imu_fifo                                           [m f0de50c20d WIP: drivers/imu/bosch output data changes
  [31mremotes/origin/pr-bp-fat                                                   [m 03a32aa2ce PX4 NuttX with Fat wrap fix Backport
  [31mremotes/origin/pr-broadcom_locking                                         [m a37a589f15 WIP: afbrs50 try SPI_LOCK
  [31mremotes/origin/pr-ca_misc                                                  [m a88c6434b6 control_allocator: add parameter based ActuatorEffectivenessCustom
  [31mremotes/origin/pr-ca_vtol_standard_unified_test                            [m 45c8b64810 hacks to make unified effectiveness matrix work
  [31mremotes/origin/pr-camera-capture-logging                                   [m 70ed3feaf2 Add camera capture topic advertising to constructor
  [31mremotes/origin/pr-camera-trigger                                           [m 21b4052d82 Log camera topics
  [31mremotes/origin/pr-can_msg                                                  [m b491356d3f UAVCAN: publish CAN frames to uORB (ORB_ID(can_frame_out))
  [31mremotes/origin/pr-castacks_fully_actuated_px4                              [m 5c09254a83 Gazebo model for the fully-actuated hexarotor added
  [31mremotes/origin/pr-cleanup-fw-state                                         [m ec3c3c0030 Add velocity control as a fw state F F
  [31mremotes/origin/pr-cleanup-pitchbody                                        [m 0d6416cb8a encapsulate attitude setpoints from position control
  [31mremotes/origin/pr-cli_manual_control                                       [m e788fe45b6 systemcmds/manual_control: command line stick input for testing
  [31mremotes/origin/pr-clipping_protection                                      [m 7adab8f65e multicopter limit max throttle during clipping
  [31mremotes/origin/pr-cmake_check_submodules                                   [m a0a0a3e983 cmake: git check submodules updates
  [31mremotes/origin/pr-cmake_colour                                             [m 5911871869 Revert "cmake: Limit color output to terminals"
  [31mremotes/origin/pr-cmake_ide_improvements                                   [m 268d29a3a9 cmake IDE improvements
  [31mremotes/origin/pr-commander_auto_posctl                                    [m 849f4531a3 commander: automatically initialize to assist mode if using mavlink manual control
  [31mremotes/origin/pr-commander_consolidate_arming                             [m 0d30e192dc WIP: commander arming simple
  [31mremotes/origin/pr-commander_function_name_standardization                  [m db0f4ecc48 Disable multicopter attitude autotuner for fmu-v5 test build.
  [31mremotes/origin/pr-commander_gimbal_timeout                                 [m e389d6ffbb commander: cleanup mavlink system present and validity checks
  [31mremotes/origin/pr-commander_home_publish_continuous                        [m ffce9885ca commander: periodically republish home position if valid
  [31mremotes/origin/pr-commander_manual_control_cleanup                         [m 63b52f1058 commander: move manual control updates to separate methods
  [31mremotes/origin/pr-commander_mode_reject_print                              [m e635fccd25 commander: centralize main_state strings and use for rejection message
  [31mremotes/origin/pr-commander_wq                                             [m e7d1f07105 commander: move to WQ with callback scheduling (vehicle_command, action_request, etc)
  [31mremotes/origin/pr-containers                                               [m 2057dd3381 github: fix workspace path
  [31mremotes/origin/pr-control_allocator_custom_params                          [m 7e224d6bb5 [RFC] control_allocator: custom configuration parameters
  [31mremotes/origin/pr-crazyflie_fixes                                          [m 53cdb802b1 bitcraze crazyflie fixes
  [31mremotes/origin/pr-ctrl_zero_h7_review                                      [m da5c61bf46 WIP: ctrl zero h7 pixracer pro sync
  [31mremotes/origin/pr-debug-tailsitter                                         [m 18e79c9649 Tailsitter add yaw controls
  [31mremotes/origin/pr-default_logging_cleanup                                  [m a891179a96 cleanup logger default profile
  [31mremotes/origin/pr-developer-tooling                                        [m 5822830450 Add a simple commandline tool This commandline tool will start to capture more and more aspects of the build and deploy process and become a very similar tool compared to ADB
  [31mremotes/origin/pr-device_tree_experiment                                   [m 1f39b42525 WIP: device tree MVP
  [31mremotes/origin/pr-disarm_switch_safety                                     [m 0c4d7350c5 commander: add COM_DISARM_SAFE to optionally disable disarm safety protections
  [31mremotes/origin/pr-distributed-lockstep                                     [m 8c7e64fb74 Run ignition gazebo as distributed lockstep
  [31mremotes/origin/pr-dma-poll-removed                                         [m a7fb14e4db Update Bootloaders with no DMP poll
  [31mremotes/origin/pr-dob-rate-control                                         [m d6201bb137 ADD TODO
  [31mremotes/origin/pr-drivers_imu_int16_negate                                 [m 5a5ef74606 drivers/imu: use math::negate on int16 data
  [31mremotes/origin/pr-drivers_mag_probe_retry                                  [m 230e7bdbc6 WIP
  [31mremotes/origin/pr-drivers_rc_ghst                                          [m 8a4d93cca3 drivers/rc/ghst_rc: create new standalone ghst_rc driver
  [31mremotes/origin/pr-drivers_rc_input_split                                   [m ad7b74847e split drivers/rc_input per protocol
  [31mremotes/origin/pr-dshot_min                                                [m f9bf6fb276 drivers/dshot: allow min throttle 0
  [31mremotes/origin/pr-dsm_bind_always_dsmx_11                                  [m 0608a7fcc4 WIP: dsm bind default to DSMX 11 ms
  [31mremotes/origin/pr-edf-hv                                                   [m 3c19efe9ea mc rate control: add gyroscopic torque compensation loop
  [31mremotes/origin/pr-ekf2-ev-reset                                            [m 0fe712ba1a ekf2_ev: do not reset learned bias
  [31mremotes/origin/pr-ekf2-events                                              [m d9086ce2a0 ekf2: migrate uorb events to events interface
  [31mremotes/origin/pr-ekf2-gnss-yaw-rtk-fixed                                  [m cce7ee2c6a ekf2-test-gnss-yaw: add unit test for fix type requirement
  [31mremotes/origin/pr-ekf2-gpos-uncertainty                                    [m 5465a2184e ekf2: set uncertainty of global origin to 0 when fusing GNSS data
  [31mremotes/origin/pr-ekf2-gravity-observation                                 [m 4a111d679d ekf2: gravity fusion updates
  [31mremotes/origin/pr-ekf2-terrain-state                                       [m be15d3d898 WIP: migrate range height control to use terrain state
  [31mremotes/origin/pr-ekf2_accel_clipping_gps_reset                            [m a5e89d6fe1 ekf2: on accel clipping immediately reset to GNSS vel/pos (if data is good)
  [31mremotes/origin/pr-ekf2_external_vision_control_refactor                    [m 12ba97f4ed EV HGT bias est predict central
  [31mremotes/origin/pr-ekf2_fake_position_fusion_add_height                     [m 076da2262c WIP: ekf2: fake position fusion include height if no hgt mode enabled
  [31mremotes/origin/pr-ekf2_filtered_innovation                                 [m 32d6081826 WIP: ekf2 filtered innovation hacks
  [31mremotes/origin/pr-ekf2_generic_state_inhibit                               [m ea8354ec66 ekf2: generic state inhibit mechanism
  [31mremotes/origin/pr-ekf2_gps_all_delayed                                     [m 17d45401fa [WIP] ekf2: GPS always on delayed time horizon
  [31mremotes/origin/pr-ekf2_gps_fusion_timeout_checks                           [m db51bfc9e8 ekf2: handle GPS vel/pos timeouts individually
  [31mremotes/origin/pr-ekf2_gps_pos_drift_warn                                  [m a863fde9f1 ekf2: GPS checks apply pos deriv limits to low pass, not raw value
  [31mremotes/origin/pr-ekf2_heading_overhaul_and_magless_fixed_heading_init     [m 34a911fa18 ekf2: increase yaw estimator heading fusion rate
  [31mremotes/origin/pr-ekf2_height_all                                          [m b3f39eafcc WIP: ekf2 multi height hacks
  [31mremotes/origin/pr-ekf2_imu_cal_updated                                     [m 0ca0e954fb ekf2: handle IMU calibration changes on delayed time horizon
  [31mremotes/origin/pr-ekf2_info_warning_restore                                [m 5ac792eed6 [RFC] ekf2: rethink info/warning/errors
  [31mremotes/origin/pr-ekf2_innovation_seq_monitoring                           [m b418980491 [WIP] ekf2: innovation sequence monitoring and estimator aid src status consistency
  [31mremotes/origin/pr-ekf2_limit_fake_fusion                                   [m b85beb17e1 ekf2: improve attitude estimation without horizontal aiding
  [31mremotes/origin/pr-ekf2_mag_3d_hagl_reset_when_observable                   [m 1b1004daf2 ekf2: mag 3d control HAGL reset wait until observable
  [31mremotes/origin/pr-ekf2_mag_bias_learning_req_cleanup                       [m b2d3b6cf4b Update src/modules/ekf2/EKF2.cpp
  [31mremotes/origin/pr-ekf2_mag_field_ne_disturbed                              [m be453c098f ekf2: mag field disturbed only care about NE for heading
  [31mremotes/origin/pr-ekf2_merge_estimator_interface                           [m f907c4a0a1 ekf: merge backend classes EstimatorInterface + Ekf
  [31mremotes/origin/pr-ekf2_mocap                                               [m 9a86f520e8 hacks
  [31mremotes/origin/pr-ekf2_never_moved_baro_init                               [m 49f0e189ec ekf2: continuing updating baro offset if never moved
  [31mremotes/origin/pr-ekf2_odometry_ENU_support                                [m 45d31893e7 ekf2: add simple odometry ENU frame support
  [31mremotes/origin/pr-ekf2_optical_flow_compensation                           [m cd20b0a6eb ekf2: adjust flow gyro compensation
  [31mremotes/origin/pr-ekf2_optical_flow_control_minor                          [m 695c9a2ff4 cont'd
  [31mremotes/origin/pr-ekf2_optical_flow_log_gyro_bias                          [m 064ea0d465 ekf2: optical flow control always use provided flow gyro (with bias applied)
  [31mremotes/origin/pr-ekf2_output_predictor_init_cleanup                       [m 0290a619e1 Merge remote-tracking branch 'px4/main' into pr-ekf2_output_predictor_init_cleanup
  [31mremotes/origin/pr-ekf2_output_predictor_misc                               [m 877a82328c WIP
  [31mremotes/origin/pr-ekf2_perf_counters                                       [m 92cc067f18 WIP: ekf2 EKF backend perf counters
  [31mremotes/origin/pr-ekf2_preflight_fail_flags                                [m f29f7bb1e3 move estimator_status preflight failure flags to estimator_status_flags
  [31mremotes/origin/pr-ekf2_preflight_innovation_simple                         [m d311060195 ekf2: push preflight filtered innovation checks into backend
  [31mremotes/origin/pr-ekf2_quat_expq                                           [m 35bac7c6ce WIP: ekf2 quaternion expq
  [31mremotes/origin/pr-ekf2_remove_const_references                             [m a3d2acad96 ekf2: remove const references
  [31mremotes/origin/pr-ekf2_selector_respect_sensor_prio                        [m f496997060 ekf2: selector prefer higher priority IMU
  [31mremotes/origin/pr-ekf2_sensor_bias_consider_states                         [m f111988880 ekf2: simplify logic for inhibited consider states and covariance matrix fixes
  [31mremotes/origin/pr-ekf2_yaw_estimator_req_sacc                              [m 52272f2f0d ekf2: yaw estimator expand GPS velocity requirements
  [31mremotes/origin/pr-ekf_init_cleanup_and_height_source_generic               [m e4e9dc62ed [WIP]: ekf2 only require baro or mag if enabled and always update all available height sources
  [31mremotes/origin/pr-ekf_init_no_baro                                         [m c09850294d WIP: ekf2 allow initialising without baro depending on configuration
  [31mremotes/origin/pr-ekf_yaw_reset_vel                                        [m 5c649e78b6 [WIP] multicopter adjust all velocity/acceleration states and setpoints on yaw reset
  [31mremotes/origin/pr-enable-quad-tailsitter-diff-thrust-main                  [m f624b963e2 Tailsitter: only support single matrix and mc rate controller, and CA only
  [31mremotes/origin/pr-estimator_aid_src_status_contd                           [m 9d7fb5e6bc ekf2: estimator aid source status (mag heading, mag 3d)
  [31mremotes/origin/pr-estimator_aid_src_status_minimal_ev                      [m f3be32e02f ekf2: add estimator aid source status (EV pos, EV vel)
  [31mremotes/origin/pr-extendend_hw_ver_rev_format-changes                      [m 0c3e896101 boards: update manifest.c to follow the new hw_ver_rev format
  [31mremotes/origin/pr-failure_detector_rate_ctrl                               [m 6c5af97868 rate controller failure detector at takeoff
  [31mremotes/origin/pr-fake-ev                                                  [m 0835c432e3 ekf2: select estimator instance manually using RC channel
  [31mremotes/origin/pr-feedforward                                              [m 8904f6f487 Fix heightrate feedforard for fixdwings
  [31mremotes/origin/pr-feedforward-heightrate                                   [m dea7cb13bf FW pos controller
  [31mremotes/origin/pr-fix-board-orientation                                    [m af522a592f Fix board orientation
  [31mremotes/origin/pr-fix-hil                                                  [m 9eb1e286c0 Add helper script for running HITL
  [31mremotes/origin/pr-fix-hitl-battery                                         [m 7cb224185d mavlink: use 4s for HITL
  [31mremotes/origin/pr-fix-landed-state                                         [m c01f0d7471 Fix case handling
  [31mremotes/origin/pr-float_conversion                                         [m f9973e0464 WIP: enable -Wfloat-conversion everywhere
  [31mremotes/origin/pr-fmu1062                                                  [m fe62de353e nxp_fmurt1062-v2:Remove fixed wing
  [31mremotes/origin/pr-fmuv6xrt-bidirectional-dshot                             [m 0816d09811 Update NuttX
  [31mremotes/origin/pr-force-bt-manual                                          [m da5041b43a backtransition when taking over manual control
  [31mremotes/origin/pr-fw-attitude-control                                      [m ae0a35847a WIP add attitude setpoints
  [31mremotes/origin/pr-fw-autotrim                                              [m affd78f360 TEMP: auto-trim
  [31mremotes/origin/pr-fw-l1-local-coordinates                                  [m 979e3a24a3 Use local path setpoints for fixedwing position control
  [31mremotes/origin/pr-fw-local-pos-control                                     [m 910810a768 Add local path setpoint Pass setpoints to npfg
  [31mremotes/origin/pr-fw-loiter-fixes-master                                   [m a8529672f1 mission block: use dist_xy instead of dist for horizontal pos error check
  [31mremotes/origin/pr-fw-offboard-attitude-setpoints                           [m 1b514be1e5 Add yaw time constant
  [31mremotes/origin/pr-fw-pos-ctrl-cleanup                                      [m 8f827ff6f3 fw pos ctrl: some incremental cleanup of the class - make less ambiguous variable names - fix some incorrect comments - add units to variable descriptions - start documenting methods
  [31mremotes/origin/pr-fw-pos-param-checks-master                               [m 3f7ff429ce FW Pos Control params: improve description of FW_AIRSPD_TRIM
  [31mremotes/origin/pr-fw-tecs-modes-master                                     [m 1e8047838f TECS: fix publishing of tecs_status
  [31mremotes/origin/pr-fw-velocity-control                                      [m 16f0bb1c38 Take velocity magnitude as airspeed setpoint for offboard Rebase fix Do not use altitude setpoint for velocity control
  [31mremotes/origin/pr-fw_dead_reckon_rtl                                       [m e7b7ccf68f auto mission fly home before complete loss of global position estimate
  [31mremotes/origin/pr-fw_pos_ctrl_refactor                                     [m 49f998f02a fixed comment
  [31mremotes/origin/pr-gazebo-airsim-plugin                                     [m 6369b42d71 Add SITL Target for typhoon_h480 with airsim
  [31mremotes/origin/pr-gazebo-battery-status-plugin                             [m ba8eb47451 Handle battery status messages Handle simulator mavlink messages Fix
  [31mremotes/origin/pr-gazebo-motor-update                                      [m 263ddcdabc new motor namespace.
  [31mremotes/origin/pr-geofence_buffer_zone                                     [m 70109e91dc WIP
  [31mremotes/origin/pr-geofence_prearm_check                                    [m b7b03c1b5e addressed review comments: added a geofence method which tells if vehicle is closer to the fence than the minimum distance (hardcoded)
  [31mremotes/origin/pr-gimbal_fix_test                                          [m 672051c34a gimbal: add paranoid quaternion checks and initialized values
  [31mremotes/origin/pr-github_actions_timeout                                   [m 7ccfc91dfc github actions add timeout-minutes
  [31mremotes/origin/pr-gps-base                                                 [m e339297388 Basic gps base station app
  [31mremotes/origin/pr-gps-valid-again-message-main                             [m 346945b501 Commander: add functionality for RTL due to GNSS loss
  [31mremotes/origin/pr-gps_device_id_timestamp_sample                           [m 0d828f3651 gps: add device_id
  [31mremotes/origin/pr-gps_heading_fusion_separate                              [m 9b2b5386b0 WIP: sensor_gps_relative
  [31mremotes/origin/pr-gps_set_clock                                            [m f706e3541d drivers/gps: set system clock more aggressively
  [31mremotes/origin/pr-gps_ubx_mon_span                                         [m 3772fff721 drivers/gps support ublox basic spectrum analyzer (UBX-MON-SPAN)
  [31mremotes/origin/pr-group-mission-items                                      [m 150018abbb Add mission checksum to group messages
  [31mremotes/origin/pr-gyro_slope_filter                                        [m a4dd162fb3 sensors/vehicle_angular_velocity: use linear regression to determine angular acceleration (aka slope filter)
  [31mremotes/origin/pr-gz-angle-scaling                                         [m c24cf6979a Fix gz servo angle scale to match joint angles
  [31mremotes/origin/pr-gz-crazyflie                                             [m 7dc4f3122f 3d meshfiles
  [31mremotes/origin/pr-gz-loong                                                 [m 50cac20187 Add foxtech loong Remove airspeed sensors WIP Fix
  [31mremotes/origin/pr-gz-vtol                                                  [m ec15530e4f Added fixed to VTOL for gz (#21097)
  [31mremotes/origin/pr-h7-adc-clock                                             [m cf648a3852 stm32h7:adc Dynamically set clock prescaler & BOOST
  [31mremotes/origin/pr-hinwil-testing                                           [m 0bdd780bdd Set feedforward heightrate setpoint in offboard mode
  [31mremotes/origin/pr-hitl-run-script                                          [m d1fbc2540c Run jsbsim
  [31mremotes/origin/pr-hmc5883_z_fix                                            [m ae8fdf4c04 drivers/magnetometer/hmc5883: fix Z direction
  [31mremotes/origin/pr-hmc5883_z_fix_backport                                   [m 507d025535 drivers/magnetometer/hmc5883: fix Z direction
  [31mremotes/origin/pr-home-attitude                                            [m 156b68b252 commander/mavlink: use home attitude, not only yaw
  [31mremotes/origin/pr-hrt_min_interval_reduce                                  [m a19e1f780e reduce HRT_INTERVAL_MIN to 10 us
  [31mremotes/origin/pr-i2c_debug                                                [m 87e9fd5c71 drivers/device/nuttx/I2C: add transfer failure details
  [31mremotes/origin/pr-i2cspibusiterator_simple                                 [m b4ac18e050 px4_platform_common: I2CBusIterator/SPIBusIterator return bus integer directly
  [31mremotes/origin/pr-icm20602_quickstart                                      [m 75b789acda WIP: icm20602 skip reset if already configured (quick start)
  [31mremotes/origin/pr-icm42688p_sensor_config                                  [m 6dcf4c21cc wrong
  [31mremotes/origin/pr-improve_survey_resume                                    [m ac2c4097d0 navigator: improve survey resume
  [31mremotes/origin/pr-imu_icm4xxxx_cleanup                                     [m fdf118d28c drivers/imu/invensense: icm42xxx minor cleanup and consistency changes
  [31mremotes/origin/pr-imu_quickstart                                           [m fbe06dde22 Update src/drivers/imu/invensense/icm20602/InvenSense_ICM20602_registers.hpp
  [31mremotes/origin/pr-imxrt1170                                                [m cb5e85fd8b F N
  [31mremotes/origin/pr-imxrt1170-imxrt1062                                      [m ad00e02c3c Intial Commit NXP FMURT1170
  [31mremotes/origin/pr-imxrt1170-imxrt1062-testing                              [m c7ed811555 DNM Boade Bring up state
  [31mremotes/origin/pr-ina231                                                   [m dd58bd6df2 new INA231 power_monitor driver
  [31mremotes/origin/pr-input-capture                                            [m 5e64e9e08e Remove ppk capture from input capture
  [31mremotes/origin/pr-invensense_imu_drdy_compare_exchange                     [m de650dc83c drivers/imu: data ready scheduling only schedule cycle if thread is ready
  [31mremotes/origin/pr-io-check-binary                                          [m d5d09bab84 IO: Simplify CRC check logic
  [31mremotes/origin/pr-iridium_fixes                                            [m 16ea3b2e33 telemetry: enable iridium
  [31mremotes/origin/pr-ist8310_probe_retry                                      [m ddbc75bb56 [DO NOT MERGE] ist8310 retry probe and enable debug
  [31mremotes/origin/pr-ist8310_self_test                                        [m 2b90749462 WIP: ist8310 self test
  [31mremotes/origin/pr-jammy                                                    [m eb9660d744 Install microDDS build requirements ubuntu.sh --microdds
  [31mremotes/origin/pr-jenkins                                                  [m 2a575c0e22 jenkins ci: listener topic list update
  [31mremotes/origin/pr-jenkins_hardfault_log_test                               [m ae37600422 Jenkins: hardware add basic hardfault test
  [31mremotes/origin/pr-jenkins_hardware                                         [m 27f17d4ad0 WIP
  [31mremotes/origin/pr-jenkins_listener_try2                                    [m 2a575c0e22 jenkins ci: listener topic list update
  [31mremotes/origin/pr-jenkins_ostest_reboot                                    [m fb53cbade7 Jenkins: hardware reboot after ostest
  [31mremotes/origin/pr-jenkins_param_testing                                    [m ced277bf0e [DO NOT MERGE] parameter testing
  [31mremotes/origin/pr-land-target-refactor                                     [m 148678d27a LTE: advertise published topic early to be logged
  [31mremotes/origin/pr-lib_drivers_timestamp_sample_monotonic                   [m 7f99cf5f49 lib/drivers/{accelerometer,gyroscope}: ensure sane monotonically increasing timestamp_sample
  [31mremotes/origin/pr-littlefs_new                                             [m 7667b3a782 boards: use littlefs for parameter storage
  [31mremotes/origin/pr-log-camera-capture                                       [m 8cc39096cb load_mon: NuttX cpuload use system times for calculation
  [31mremotes/origin/pr-logger_limit_buffer_size                                 [m 897d8e293e logger: automatically limit buffer size to largest available free chunk (NuttX only)
  [31mremotes/origin/pr-logger_orb_id                                            [m 08c06de595 logger: use ORB_ID enum as msg_id
  [31mremotes/origin/pr-logger_priority_boost_control                            [m 9766fcfb6f [WIP] logger priority boost command line
  [31mremotes/origin/pr-logger_start_delay                                       [m b940f68b4c logger: new optional start delay time (SDLOG_DELAY_S)
  [31mremotes/origin/pr-logger_subscription_init_late                            [m 14baf5aadf logger: reduce multi-EKF logging to first 4 instances
  [31mremotes/origin/pr-logger_watchdog_purge                                    [m 223f0ef8b4 logger: remove watchdog priority boost
  [31mremotes/origin/pr-lsm303d_l3gd20_cleanup                                   [m d26d6ca7d0 boards: px4_fmu-v2 strip out v2m and v3 detection/support
  [31mremotes/origin/pr-ltest_refactor                                           [m 3c848f6cbc fix clang-tidy fail
  [31mremotes/origin/pr-ltest_refactor_cleanup                                   [m 6c7800faab cleanup landing_target_estimator
  [31mremotes/origin/pr-mag_calib_ukf_multi                                      [m d602cf0fd1 WIP: save cal if disarmed, C -> C++, cleanup
  [31mremotes/origin/pr-matek_h743                                               [m f5e84c35f5 WIP
  [31mremotes/origin/pr-matek_h743_cleanup_unify                                 [m f09929006a boards/matek/h743: consolidate and fix h743 variants
  [31mremotes/origin/pr-matrix_print_lower_triangular_only                       [m 5cc7cff035 Matrix: adjust printing for symmetric P (lower triangular only)
  [31mremotes/origin/pr-matrix_vector_inline                                     [m 61778511ea lib/matrix: inline common Vector3f operators
  [31mremotes/origin/pr-mavlink_DISTANCE_SENSOR                                  [m 46dba854b7 mavlink: DISTANCE_SENSOR populate all fields
  [31mremotes/origin/pr-mavlink_battery_status                                   [m 9e5e9f1b7c mavlink: streams/BATTERY_STATUS fix voltage extension fields
  [31mremotes/origin/pr-mavlink_default_interface                                [m 33b5437eee mavlink: remove default device and consolidate serial vs UDP init handling
  [31mremotes/origin/pr-mavlink_devid                                            [m 1c0d7fc45c Add DeviceBusType_MAVLINK
  [31mremotes/origin/pr-mavlink_iridium_minor                                    [m f2972a186e mavlink: Iridium mode collect special handling
  [31mremotes/origin/pr-mavlink_lazy                                             [m 4ae8cf295a mavlink: receiver lazily allocate handlers on first use
  [31mremotes/origin/pr-mavlink_opt                                              [m 0e320ad4a1 mavlink: increase optimization to ${MAX_CUSTOM_OPT_LEVEL}
  [31mremotes/origin/pr-mavlink_rate_mult_limit                                  [m 8d95c70003 mavlink: rate multiplier and scheduling optimizations
  [31mremotes/origin/pr-mavlink_receiver_cmd_case                                [m f56640f072 mavlink: receiver handle_message_command_both() use case statement
  [31mremotes/origin/pr-mavlink_signing                                          [m 142589f1de MAVLink app: Add signing support.
  [31mremotes/origin/pr-mc_ground_contact_handling                               [m c9e3f9714c multicopter land detector move ground contact intent to mc_pos_control
  [31mremotes/origin/pr-mc_hover_thrust_stabilized                               [m 0a90621635 multicopter hover thrust estimator use vehicle_thrust_setpoint (work in stabilized mode)
  [31mremotes/origin/pr-mc_pos_control_jerk_max_enforce                          [m d92b7b2f8c mc_pos_control: enforce MPC_JERK_MAX in PositionControl
  [31mremotes/origin/pr-mc_pos_ctrl_filters                                      [m 1cc6732ec7 mc_pos_control: add velocity filter and remove controllib usage
  [31mremotes/origin/pr-mc_rate_control_minor_simplifiations                     [m 7f12d1b2b3 mc_rate_control: minor cleanup
  [31mremotes/origin/pr-mc_rate_control_simple_inline                            [m 263d2d6e2b mc_rate_control: RateControl inline simple getters/setters
  [31mremotes/origin/pr-microdds_start                                           [m 60e7cc11ce WIP: microdds_start if installed
  [31mremotes/origin/pr-mission-checksum                                         [m 2647663cae mavlink_mission: Update mission checksum to calculate checksum based on mavlink_endianness. Send the mission checksum constantly at a frequency of 1 Hz.
  [31mremotes/origin/pr-mixed-invariant-state-prediction                         [m e9b012fad9 [AUTO COMMIT] update change indication
  [31mremotes/origin/pr-mixer_module_parameters_always                           [m 62714d6c5d mixer_module always use MIN/MAX/DIS/FAIL parameters
  [31mremotes/origin/pr-module_params_parameter_update                           [m 9586ad0006 [RFC] broadcast parameter changes and ModuleParams skip full updateParams() when possible
  [31mremotes/origin/pr-modulebase_refactor                                      [m 700961daf2 ModuleBase add common base type and cleanup
  [31mremotes/origin/pr-mro_cannode                                              [m 6d6b72d4bf WIP: mRO CANnode m10025
  [31mremotes/origin/pr-mro_ctrl-zero-h7                                         [m b56ba14910 WIP: mRo ctrl-zero-h7
  [31mremotes/origin/pr-nav-accept-vtol-bt-only-once-in-mc-main                  [m 0a97343091 Mission_block: only accept VTOL backtransition once in full MC mode
  [31mremotes/origin/pr-navigator_yaw_acceptance_bad_heading                     [m a3605b4bc8 navigator: mission_block relax yaw acceptance if heading not good for control
  [31mremotes/origin/pr-nuttx-sdio-fix-hang                                      [m ac588a2c73 [WIP] testing nuttx 10.3.0+ pr sdio fix hang
  [31mremotes/origin/pr-nuttx_arch_generic_pin_init                              [m 6b0cf1652e WIP: nuttx common early pin init per architecture
  [31mremotes/origin/pr-nuttx_jenkins_debug_ostest                               [m 64e9792a25 Jenkins: hardware px4_fmu-v5_debug run ostest
  [31mremotes/origin/pr-nuttx_littlefs                                           [m 89b9701a33 [RFC] NuttX use littlefs for F-RAM parameter storage
  [31mremotes/origin/pr-nuttx_opt                                                [m 7827555e6d NuttX: increase default optimization to ${MAX_CUSTOM_OPT_LEVEL}
  [31mremotes/origin/pr-nuttx_sd_testing                                         [m 5ac354628b Jenkins testing
  [31mremotes/origin/pr-nuttx_sem_holder                                         [m c6c956f382 Merge remote-tracking branch 'px4/master' into pr-nuttx_sem_holder
  [31mremotes/origin/pr-nuttx_sem_preallocholders-v1.13                          [m 0bddca6b9b boards: NuttX update all boards to preallocated sem holder list
  [31mremotes/origin/pr-nxp-dma-drivers                                          [m b6b3f46bdf nxp_fmurt1062-v1:All LPUARTS Use DMA
  [31mremotes/origin/pr-odometry_quality                                         [m 97922f3e7e WIP: ODOMETRY quality metric
  [31mremotes/origin/pr-offboard-switch                                          [m cb77844c1d Update src/modules/offboard_switch/offboard_switch.cpp
  [31mremotes/origin/pr-optical-flow-sensors_pmw3901_cleanup                     [m 13f77de208 WIP: pmw3901 rewrite
  [31mremotes/origin/pr-output_module_float_config                               [m 3d7f6ea09c WIP: output modules native units
  [31mremotes/origin/pr-pab-carrier                                              [m 673e5898ea Start rc.baseboard
  [31mremotes/origin/pr-param_notify_schedule                                    [m 75ad606ae4 Merge remote-tracking branch 'px4/master' into pr-param_notify_schedule
  [31mremotes/origin/pr-param_type_check                                         [m a749c787ca mavlink: param_get proper type to silence errors
  [31mremotes/origin/pr-parameter_hacks                                          [m bbc9a8c9bb WIP: parameter export hacks
  [31mremotes/origin/pr-parameter_improvements_continued                         [m 6598a82a71 WIP: ROMFS: explicitly configure airframe outputs
  [31mremotes/origin/pr-parameter_server                                         [m 5247db7647 parameters: new parameter server backend
  [31mremotes/origin/pr-parameter_server_overhaul                                [m 6fd19df64e parameter server continued
  [31mremotes/origin/pr-parameters_bool                                          [m f23740a3c5 parameters: add boolean support (PARAM_DEFINE_BOOL)
  [31mremotes/origin/pr-parameters_remove_public_reset                           [m b6157c89a0 parameters: eliminate param_reset(param_t param) from public C API
  [31mremotes/origin/pr-parameters_save_test                                     [m b499b53cc1 [DO NOT MERGE] test param save from HPWORK
  [31mremotes/origin/pr-parity-f1-4                                              [m bfe8ce4e18 Update Bootloaders with discard data with PE
  [31mremotes/origin/pr-per_sensor_noise_defaults                                [m 3ccbe8f8b6 [WIP] per sensor noise defaults
  [31mremotes/origin/pr-pixracerpro_gps_dshot                                    [m 0c19fabbd7 [DO NOT MERGE] boards: mro_pixracerpro remap GPS1 to 4xPWM/DSHOT
  [31mremotes/origin/pr-plane-hitl                                               [m ca93b7e444 Enable control allocation
  [31mremotes/origin/pr-platform_i2c_external_bus_check                          [m cd1939a14e platforms/common/i2c.cpp: px4_i2c_bus_external check bus validity
  [31mremotes/origin/pr-platforms_board_init_common                              [m a1fa3cc699 WIP: common board init
  [31mremotes/origin/pr-pos-refactor                                             [m ea6f95781f Add fw pos control states F
  [31mremotes/origin/pr-pwm14_camera_trigger_testing                             [m bca78c0379 boards: CUAV Nora disable icm20649 until SPI6 BDMA is working
  [31mremotes/origin/pr-pwm_calibration                                          [m 959471ffec delete PWM_SERVO_CLEAR_ARM_OK
  [31mremotes/origin/pr-pwm_defaults_per_channel                                 [m 03a9f5753f [WIP] explicit PWM configuration per channel
  [31mremotes/origin/pr-px4_fmu-v6xrt-1170-T1-base                               [m 2e5842dbeb px4_fmu-v6xrt:Use multi-PHY
  [31mremotes/origin/pr-px4_fmu-v6xrt-1170-T1-base-multi-phy                     [m e3ece343f0 px4_fmu-v6xrt:SPI1 is icm42686p
  [31mremotes/origin/pr-px4_git_improve                                          [m fdd8afc542 cmake: px4_add_git_submodule add git index and HEAD as dependencies
  [31mremotes/origin/pr-px4_i2c_device_external_hacks                            [m 338ca2cc2a [DO NOT MERGE] px4_i2c_device_external hacks
  [31mremotes/origin/pr-px4_protected_build_pub                                  [m b3bf1d8b0a px4_platform: fix linking for sitl
  [31mremotes/origin/pr-px4io_cleanup                                            [m b24ccc27c9 px4io: add RC update monotonic count and cleanup unused
  [31mremotes/origin/pr-px4io_dumb_output_module                                 [m a01b029578 Breakout px4io.hpp and delete unneeded #includes.
  [31mremotes/origin/pr-px4io_serial_generic                                     [m 64f4256b58 px4io remove custom serial driver
  [31mremotes/origin/pr-px4log_everywhere                                        [m b4555db0e7 WIP: px4log
  [31mremotes/origin/pr-qiotek_f427                                              [m f6da5474a7 boards: new QioTek ZealotF427 board support
  [31mremotes/origin/pr-qiotek_zealotf427                                        [m 0e9cbd4606 boards: initial QioTek ZealotF427 support
  [31mremotes/origin/pr-qmc5883l_clarify_set_clear_bits                          [m b147f8d811 drivers/magnetometer/qmc5883l: clarify set/clear bits and add overflow perf count
  [31mremotes/origin/pr-rate-p-max                                               [m 42bcc3d088 MC rate control: let the user set P gain to 1 to use K instead
  [31mremotes/origin/pr-rc_buffer_shared                                         [m 9bbe1aa419 lib/rc: don't share decode buffer between protocols
  [31mremotes/origin/pr-rc_input_blocking                                        [m edb9fce9b3 drivers/rc_input: cleanup and simplify data processing per type
  [31mremotes/origin/pr-rc_input_fixes                                           [m 9acd39521d rc_input: correctly init SBUS inversion for other RC protocols
  [31mremotes/origin/pr-rcl-manual                                               [m c0204e8f31 Enable fallback RTL for RCL in manual modes
  [31mremotes/origin/pr-remove-sys_mc_est_group                                  [m 9be379b581 delete SYS_MC_EST_GROUP
  [31mremotes/origin/pr-remove_scale_from_fifo_msgs                              [m f16ef917f7 Merge remote-tracking branch 'upstream/main' into pr-remove_scale_from_fifo_msgs
  [31mremotes/origin/pr-remove_sprintf                                           [m 5d2443c390 migrate remaining sprintf usage -> snprintf
  [31mremotes/origin/pr-replay_multi-ekf                                         [m af8d1c9c6c WIP: multi-EKF replay hacks and replay overhaul
  [31mremotes/origin/pr-replay_overhaul                                          [m 7881a76b94 WIP: replay overhaul
  [31mremotes/origin/pr-reset-acro-yawrate                                       [m 749f645bd6 Optionally reset yaw rate integral in acromode for fixedwings
  [31mremotes/origin/pr-restore_old_mission_if_a_new_missionis_not_feasible      [m ce301a17ea mission: don't validate old mission if the vehicle is armed
  [31mremotes/origin/pr-romfs_includes_kconfig_dep                               [m 0361568967 ROMFS: only add R1 airframe with differential drive control
  [31mremotes/origin/pr-ros2_initial_nodes_wip                                   [m 6a61738fcb WIP
  [31mremotes/origin/pr-ros2_msg_compat                                          [m 9ee8fa21cd PX4 ROS2 msg conformity and explicit topics
  [31mremotes/origin/pr-rover-ratesp                                             [m aa004b9c6c update angular accelerations Rebase fixes
  [31mremotes/origin/pr-rover-ratesp-junwoo                                      [m d58cad456b RoverPosControl: Using torque setpoint directly as steering input
  [31mremotes/origin/pr-rover-ratesp-lowspeed                                    [m 69d6fc2a7f Do not use a differential input
  [31mremotes/origin/pr-sdp3x_reset                                              [m 34ccd5c99f sdp3x issue I2C general call reset if alone
  [31mremotes/origin/pr-sdp3x_reset_and_speed_hacks                              [m df8c9b4efb [DO NOT MERGE] run sdp3x at lower rate (20 Hz sample rate)
  [31mremotes/origin/pr-sens_cal_conf_split                                      [m bbb19f7c7f split sensor calibration and configuration
  [31mremotes/origin/pr-sens_gps_parameters                                      [m 93232f9ee7 WIP: SENS_GNSS instance configuration and GPS blending updates
  [31mremotes/origin/pr-sensor-voltage                                           [m e367997707 px4_fmuv5:Establish reset state of GPIO_VDD_3V3_SENSORS_EN
  [31mremotes/origin/pr-sensor_accel_or_fifo_separately                          [m 050fa47aca IMU handle accel/gyro FIFO as separate type
  [31mremotes/origin/pr-sensor_calibration_opt                                   [m 5e9459fcbd lib/sensor_calibration: FindCurrentCalibrationIndex only call param_find once
  [31mremotes/origin/pr-sensor_gps_heading                                       [m 2b060a0000 gps: ublox sensor_gps_heading message
  [31mremotes/origin/pr-sensor_imu_fifo                                          [m 52e98f99f4 sensors/vehicle_imu: use accel & gyro FIFO if available
  [31mremotes/origin/pr-sensors_airspeed                                         [m 592b2e9dbe move analog differential pressure to standalone optional driver
  [31mremotes/origin/pr-sensors_auto_cal_unify                                   [m 77f3ce553d sensor: unify accel/gyro/mag estimated bias -> cal update
  [31mremotes/origin/pr-sensors_gps_status                                       [m 87f74b7ea6 WIP: move GPS status to sensors
  [31mremotes/origin/pr-sensors_gps_time_sync                                    [m 3df000f77b [WIP]: GPS clock set time updates
  [31mremotes/origin/pr-sensors_imu_consolidate                                  [m 8ac5f7cfc0 consolidate IMU processing and initial sensor_imu_fifo support
  [31mremotes/origin/pr-sensors_imu_hrt_trivial                                  [m 26d971b93a drivers/imu: minimize unnecessary hrt calls
  [31mremotes/origin/pr-sensors_optical_flow                                     [m 16d3f536a7 WIP: optical_flow sensor & vehicle
  [31mremotes/origin/pr-serial_driver_simple                                     [m 68d756d319 WIP: Serial driver
  [31mremotes/origin/pr-serial_port_label                                        [m 3ec84d28da serial port config include device (eg /dev/ttyS3) in label (eg TELEM/SERIAL 4 (/dev/ttyS3))
  [31mremotes/origin/pr-sf1xx_startup_timing                                     [m 823adac42f [WIP] lightware_laser_i2c startup timing debug
  [31mremotes/origin/pr-sik-radio-id-improvements                                [m b7dc8855df mavlink: improve SiK radio configuration
  [31mremotes/origin/pr-simulation_airframes                                     [m 331cefc1d1 init.d-posix: make all simulation airframes simulator specific
  [31mremotes/origin/pr-sitl-gps                                                 [m ab35ce0e3f Update sitl_gazebo Update sitl_gazebo
  [31mremotes/origin/pr-sitl-helicopter                                          [m 77a50d846e Update submodule
  [31mremotes/origin/pr-skip_vtol_takeoff_when_in_fw_mode                        [m 22b477f759 mission: start looking for first fw waypoint at the first mission index
  [31mremotes/origin/pr-sliding_dft                                              [m 9824c6b4ae [WIP] sliding DFT
  [31mremotes/origin/pr-smart-rtl                                                [m ee75ebac8c Adds Smart Return To Home capability by recording the flight path in a memory optimimized format. This means the UAV can return to home by using only known-good flight paths.
  [31mremotes/origin/pr-stable-littlefs_reverse_transition                       [m ae3d3689f2 Parameter reverse transition from LittleFS to block device
  [31mremotes/origin/pr-standard_vtol_ctrlalloc-fix                              [m c1ca5b50a4 control_allocator: Fixes for standard VTOL frames
  [31mremotes/origin/pr-stm32-bp                                                 [m 355db041cc NuttX with stm32 pinmap changes, USB OTG, ULPI Updates
  [31mremotes/origin/pr-support_custom_action_waypoints                          [m 346d01b496 px4-rc.mavlink: add COMMAND_CANCEL stream through the OFFBOARD udp link
  [31mremotes/origin/pr-system_power_module                                      [m 3fff512ec0 new standalone drivers/system_power (split out of adc)
  [31mremotes/origin/pr-system_power_usb_connect_count                           [m 70b507f980 drivers/adc/board_adc: count number of times USB has been connected (disconnected -> connected)
  [31mremotes/origin/pr-systemcmds_boardctl                                      [m f82eae800d boardctl hacks
  [31mremotes/origin/pr-tecs-fix-airspeed-rate-input-main                        [m 98b847fbe6 FWPositionControl: add extra check for low wind variance to use TAS rate estimate in TECS
  [31mremotes/origin/pr-tmotor_alpha_esc_telem                                   [m 42a9e540a8 WIP: TMotor Alpha ESC telemetry driver
  [31mremotes/origin/pr-topic-remap                                              [m bf8840d109 Add topic remap plugin
  [31mremotes/origin/pr-uavcan_adc                                               [m 49d241245e analog_measurement: Scale values by ADC ref voltage
  [31mremotes/origin/pr-uavcan_button                                            [m edfb437f92 revert cannode publishing
  [31mremotes/origin/pr-uavcan_default_timer                                     [m 054b8093de [DO NOT MERGE] try changing UAVCAN default timer to 6
  [31mremotes/origin/pr-uavcan_firmware_embed                                    [m 9756226dc3 [DO NOT MERGE] UAVCAN embedded peripheral firmware hacks
  [31mremotes/origin/pr-uavcan_fw_copy_verify_crc                                [m 7c55834f3c uavcan: verify CRC when copying firmware
  [31mremotes/origin/pr-uavcan_hrt_monotonic_source                              [m 72f7f0d787 [WIP] uavcan use HRT as monotonic source
  [31mremotes/origin/pr-uavcan_logmsg_sub_param                                  [m f716224c45 uavcan: add UAVCAN_SUB_LOG param to enable/disable uavcan::protocol::debug::LogMessage subscription handling
  [31mremotes/origin/pr-uavcan_node_reset_cli                                    [m 622c833768 uavcan: cleanup reset CLI and document
  [31mremotes/origin/pr-uavcan_server_init_order                                 [m 24d67d1666 uavcan: servers handle move firmware early
  [31mremotes/origin/pr-ucan-pca9685                                             [m bb6179311b update spi_led driver.
  [31mremotes/origin/pr-uorb_parameter_operations                                [m 58006c0a93 WIP parameter server
  [31mremotes/origin/pr-uorb_publication_unadvertise                             [m ce67fee6b9 uORB::DeviceNode mark advertised on write
  [31mremotes/origin/pr-uorb_type_map                                            [m 2859fc66fa uORB: generate ORBTypeMap and example usage (uORB::Publication2)
  [31mremotes/origin/pr-update-all-nuttx-defconfig                               [m ba62cf8a52 boards: update all NuttX defconfigs
  [31mremotes/origin/pr-update-arm-none-eabi-gcc                                 [m bac468b827 fixup: try docker login action
  [31mremotes/origin/pr-update-world_magnetic_model                              [m 04c882b15b [AUTO COMMIT] update change indication
  [31mremotes/origin/pr-update_platforms/nuttx/NuttX/nuttx                       [m bbf8004b3e Update submodule nuttx to latest Fri Jan 12 12:40:03 UTC 2024
  [31mremotes/origin/pr-update_src/drivers/cyphal/libcanard                      [m 7665d14e44 Update submodule libcanard to latest Fri Jan 12 12:40:05 UTC 2024
  [31mremotes/origin/pr-update_src/drivers/cyphal/public_regulated_data_types    [m 725203df3e Update submodule public_regulated_data_types to latest Fri Jan 12 12:40:07 UTC 2024
  [31mremotes/origin/pr-update_src/drivers/gps/devices                           [m 5811f451ad Update submodule devices to latest Fri Jan 12 12:40:09 UTC 2024
  [31mremotes/origin/pr-update_src/drivers/uavcan/libuavcan                      [m 1210f80af4 Update submodule libuavcan to latest Fri Jan 12 12:40:11 UTC 2024
  [31mremotes/origin/pr-update_src/lib/cdrstream/cyclonedds                      [m 90c5a32436 Update submodule cyclonedds to latest Fri Jan 12 12:40:14 UTC 2024
  [31mremotes/origin/pr-update_src/lib/cdrstream/rosidl                          [m 8a8c28a4df Update submodule rosidl to latest Fri Jan 12 12:40:16 UTC 2024
  [31mremotes/origin/pr-update_src/modules/mavlink/mavlink                       [m f50bc689db Update submodule mavlink to latest Fri Jan 12 12:40:18 UTC 2024
  [31mremotes/origin/pr-update_src/modules/microdds_client/Micro-XRCE-DDS-Client [m 23af1a3311 Update submodule Micro-XRCE-DDS-Client to latest Wed Oct 19 12:38:54 UTC 2022
  [31mremotes/origin/pr-uwb_1.14_backport                                        [m 66df5c1bd1 drivers: rework NXP UWB driver (#21124)
  [31mremotes/origin/pr-uxrce-dds-multi-subscriber                               [m 64257391c6 [uxrce_dds_client] Allow for arbitrary topic instances to be bridged
  [31mremotes/origin/pr-uxrce_dds_retry                                          [m 5240d60f61 uxrce_dds_client: optimizations and instrumentation
  [31mremotes/origin/pr-v5x_debug_hacks                                          [m a82c0091c1 [DO NOT MERGE] v5x debug hacks
  [31mremotes/origin/pr-v6x-rev3-sensors-with-ADC                                [m 3d77bc5de5 px4_fmuv-6x Sensor set 3
  [31mremotes/origin/pr-vectornav_ins                                            [m 920dd62918 vectornav fix local position and add perf counts
  [31mremotes/origin/pr-vehicle_imu_status_mean                                  [m ac30a36ba5 vehicle_imu: only reset raw accel/gyro Welford mean periodically
  [31mremotes/origin/pr-vehicle_magnetometer_in_flight_bias_debug                [m 23d6f0b2fe sensors/vehicle_magnetometer: add PX4_DEBUG for in flight mag cal gathering
  [31mremotes/origin/pr-vehicle_model_estimator                                  [m 1bb0bf95d3 Fixup vehicle_model
  [31mremotes/origin/pr-ver-rev-ids                                              [m 405e90288a px4_fmu-v6x:Use BOARD_HAS_HW_SPLIT_VERSIONING
  [31mremotes/origin/pr-vtol-acceptance-master                                   [m 5698e49bc5 Mission block: only accept transition WP as accepted once no longer in transition mode
  [31mremotes/origin/pr-vtol-backtransition-flaps-spoilers-master                [m 123447bff9 VTOL: add options to set flaps and spoilers in backtransition
  [31mremotes/origin/pr-vtol-move-quadchute-to-commander                         [m 76ea472696  Move Quadchute from the VTOL module to Commander (failure detector)
  [31mremotes/origin/pr-vtol-move-quadchute-to-commander-direct                  [m 4232e92ac1 Move VTOL Quadchute logic to Commander (failure detector)
  [31mremotes/origin/pr-vtol-pwm-min-master                                      [m b4e66b76a3 VTOL: always set min PWM, not only once on entering a new VTOL phase
  [31mremotes/origin/pr-vtol-spoilers-descent                                    [m 99e8338ab5 also use VT_LND_PITCH_MIN in NAVIGATION_STATE_DESCEND
  [31mremotes/origin/pr-windshear-plugin                                         [m f5a73e0e3c Add windshear plugin
  [31mremotes/origin/pr-yaw-setpoint-fw-master                                   [m 3c03d48876 FW L1 controller: also handle negative loiter direction case
  [31mremotes/origin/pr-zenoh_playground                                         [m 25dd6ab845 WIP: mavlink zenoh pico playground
  [31mremotes/origin/pr/upstream-nuttx-can-backport                              [m 4f2c8b4469 Update submodule nuttx
  [31mremotes/origin/pr_battery_status_v2                                        [m d08aaa7013 Add BATTERY_CELL_VOLTAGES streaming
  [31mremotes/origin/pr_dynamic_cruise_throttle                                  [m 8be8f06923 FixedWingPositionControl: throttle compensation - compensate cruise throttle for wind for vehicles without airspeed sensor - compensate cruise and maximum throttle for air density
  [31mremotes/origin/pr_generic_button_based_action                              [m d486143eba Move param handles in rc_update into unified struct
  [31mremotes/origin/pr_littlefs_tests                                           [m 3574e0d5da update
  [31mremotes/origin/pr_payload_deliverer_improvements                           [m 4b33632615 Only command Gripper grab when we are actually initializing gripper
  [31mremotes/origin/pr_tecs_climbrate                                           [m f847586b10 review comments
  [31mremotes/origin/pr_tecs_min_sinkrate                                        [m f7d7a7123c addressed review comments
  [31mremotes/origin/pr_vtol_land                                                [m bc8dfc0554 fix vehicle not disarming after land
  [31mremotes/origin/precision_land_library                                      [m a5663d4d3c Made a separate precision landing library. Now, the Precision Landing mode can use it the same as the RTL and the mission mode.
  [31mremotes/origin/quad-tailsitter-no-surfaces-testing                         [m a38ec6aed2 update quad vtol tailsitter mixer
  [31mremotes/origin/rate-gps                                                    [m 02fe211e4c mavlink mode NORMAL: increase GPS_RAW_INT rate to 5Hz
  [31mremotes/origin/rc-handling                                                 [m c600315c76 Commander: gate manual control setpoint processing on new data
  [31mremotes/origin/release/1.10                                                [m e0f016c2b3 logger: add safety (switch) at minimal rate
  [31mremotes/origin/release/1.11                                                [m 21c82a4814 FlightTasks: hotfix guard against accessing no task
  [31mremotes/origin/release/1.12                                                [m 55374fe589 boards: CUAV-x7pro: reoder brick to  fixed cuav hvpm cannot detect voltage
  [31mremotes/origin/release/1.13                                                [m 437b6b9844 remove unused debug.h
  [31mremotes/origin/release/1.14                                                [m beb834af2b fmu-v6x: add crossfire UART driver
  [31mremotes/origin/release/1.14-mod                                            [m beb834af2b fmu-v6x: add crossfire UART driver
  [31mremotes/origin/release/1.7                                                 [m 50bd148f53 Aero: Update maintainer
  [31mremotes/origin/release/1.8                                                 [m f13bbacd52 mc_att_control: copy sensor_correction topic once initially
  [31mremotes/origin/release/1.9                                                 [m b076cfd4ed uorb: do not open a node exclusively for an advertiser
  [31mremotes/origin/release/FAKE_RELEASE_JUNWOO_TEST                            [m e7896215be Echo git branch and also content of GITHUB_ENV file
  [31mremotes/origin/revert-20760-pr-mission_feasibility                         [m 53fc158a9f Revert "started with feasibility checks"
  [31mremotes/origin/romain-chiap-patch-1                                        [m b89a5d67e7 Update px4-rc.mavlink
  [31mremotes/origin/rotate-2D-vector                                            [m 5c9fac79ae Vector2: Add function to rotate a 2D vector
  [31mremotes/origin/rover_module                                                [m a471755dac Rover module: Temp
  [31mremotes/origin/rroche/setup-update                                         [m 115568eb82 setup