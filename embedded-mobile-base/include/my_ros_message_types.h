/* Generated ROS message type definitions
 * Using the workshop container and the following command (in-situ)
 * ~/Pico-ROS-software/tools/type-gen# ./generate_type_header.py --packages-dir
 * /opt/ros/jazzy/share/std_msgs /opt/ros/jazzy/share/builtin_interfaces
 * /opt/ros/jazzy/share/sensor_msgs /opt/ros/jazzy/share/lifecycle_msgs /opt/ros/jazzy/share/control
 * /opt/ros/jazzy/share/service_msgs /opt/ros/jazzy/share/geometry_msgs --output-dir out
 */
#ifndef GENERATED_TYPES_H
#define GENERATED_TYPES_H

#define MSG_LIST(BTYPE, CTYPE, TTYPE, FIELD, ARRAY, SEQUENCE)                                      \
  CTYPE(                                                                                           \
    ros_Duration, "builtin_interfaces::msg::dds_::Duration",                                       \
    "e8d009f659816f758b75334ee1a9ca5b5c0b859843261f14c7f937349599d93b",                            \
    FIELD(int32_t, sec) FIELD(uint32_t, nanosec))                                                  \
  CTYPE(                                                                                           \
    ros_Time, "builtin_interfaces::msg::dds_::Time",                                               \
    "b106235e25a4c5ed35098aa0a61a3ee9c9b18d197f398b0e4206cea9acf9c197",                            \
    FIELD(int32_t, sec) FIELD(uint32_t, nanosec))                                                  \
  CTYPE(                                                                                           \
    ros_Point, "geometry_msgs::msg::dds_::Point",                                                  \
    "6963084842a9b04494d6b2941d11444708d892da2f4b09843b9c43f42a7f6881",                            \
    FIELD(double, x) FIELD(double, y) FIELD(double, z))                                            \
  CTYPE(                                                                                           \
    ros_Point32, "geometry_msgs::msg::dds_::Point32",                                              \
    "2fc4db7cae16a4582c79a56b66173a8d48d52c7dc520ddc55a0d4bcf2a4bfdbc",                            \
    FIELD(float, x) FIELD(float, y) FIELD(float, z))                                               \
  CTYPE(                                                                                           \
    ros_Polygon, "geometry_msgs::msg::dds_::Polygon",                                              \
    "3782f9f0bf044964d692d6c017d705e37611afb1f0bf6a9dee248a7dda0f784a",                            \
    SEQUENCE(ros_Point32, points))                                                                 \
  CTYPE(                                                                                           \
    ros_PolygonInstance, "geometry_msgs::msg::dds_::PolygonInstance",                              \
    "fa1cb3dc774329865258afef74f65b0553d487510c6d0f93ba38cc32d62ac0e5",                            \
    FIELD(ros_Polygon, polygon) FIELD(int64_t, id))                                                \
  CTYPE(                                                                                           \
    ros_Pose2D, "geometry_msgs::msg::dds_::Pose2D",                                                \
    "d68efa5b46e70f7b16ca23085474fdac5a44b638783ec42f661da64da4724ccc",                            \
    FIELD(double, x) FIELD(double, y) FIELD(double, theta))                                        \
  CTYPE(                                                                                           \
    ros_Quaternion, "geometry_msgs::msg::dds_::Quaternion",                                        \
    "8a765f66778c8ff7c8ab94afcc590a2ed5325a1d9a076ffff38fbce36f458684",                            \
    FIELD(double, x) FIELD(double, y) FIELD(double, z) FIELD(double, w))                           \
  CTYPE(                                                                                           \
    ros_Pose, "geometry_msgs::msg::dds_::Pose",                                                    \
    "d501954e9476cea2996984e812054b68026ae0bfae789d9a10b23daf35cc90fa",                            \
    FIELD(ros_Point, position) FIELD(ros_Quaternion, orientation))                                 \
  CTYPE(                                                                                           \
    ros_PoseWithCovariance, "geometry_msgs::msg::dds_::PoseWithCovariance",                        \
    "9a7c0fd234b7f45c6098745ecccd773ca1085670e64107135397aee31c02e1bb",                            \
    FIELD(ros_Pose, pose) ARRAY(double, covariance, 36))                                           \
  CTYPE(                                                                                           \
    ros_Vector3, "geometry_msgs::msg::dds_::Vector3",                                              \
    "cc12fe83e4c02719f1ce8070bfd14aecd40f75a96696a67a2a1f37f7dbb0765d",                            \
    FIELD(double, x) FIELD(double, y) FIELD(double, z))                                            \
  CTYPE(                                                                                           \
    ros_Accel, "geometry_msgs::msg::dds_::Accel",                                                  \
    "dc448243ded9b1fcbcca24aba0c22f013dae06c354ba2d849571c0a2a3f57ca0",                            \
    FIELD(ros_Vector3, linear) FIELD(ros_Vector3, angular))                                        \
  CTYPE(                                                                                           \
    ros_AccelWithCovariance, "geometry_msgs::msg::dds_::AccelWithCovariance",                      \
    "230d51bd53bc36f260574e73b42941cefe44684753480b6fc330c032c5db5997",                            \
    FIELD(ros_Accel, accel) ARRAY(double, covariance, 36))                                         \
  CTYPE(                                                                                           \
    ros_Inertia, "geometry_msgs::msg::dds_::Inertia",                                              \
    "2ddd5dab5c347825ba2e56c895ddccfd0b8efe53ae931bf67f905529930b4bd7",                            \
    FIELD(double, m) FIELD(ros_Vector3, com) FIELD(double, ixx) FIELD(double, ixy)                 \
      FIELD(double, ixz) FIELD(double, iyy) FIELD(double, iyz) FIELD(double, izz))                 \
  CTYPE(                                                                                           \
    ros_Transform, "geometry_msgs::msg::dds_::Transform",                                          \
    "beb83fbe698636351461f6f35d1abb20010c43d55374d81bd041f1ba2581fddc",                            \
    FIELD(ros_Vector3, translation) FIELD(ros_Quaternion, rotation))                               \
  CTYPE(                                                                                           \
    ros_Twist, "geometry_msgs::msg::dds_::Twist",                                                  \
    "9c45bf16fe0983d80e3cfe750d6835843d265a9a6c46bd2e609fcddde6fb8d2a",                            \
    FIELD(ros_Vector3, linear) FIELD(ros_Vector3, angular))                                        \
  CTYPE(                                                                                           \
    ros_TwistWithCovariance, "geometry_msgs::msg::dds_::TwistWithCovariance",                      \
    "49f574f033f095d8b6cd1beaca5ca7925e296e84af1716d16c89d38b059c8c18",                            \
    FIELD(ros_Twist, twist) ARRAY(double, covariance, 36))                                         \
  CTYPE(                                                                                           \
    ros_Wrench, "geometry_msgs::msg::dds_::Wrench",                                                \
    "018e8519d57c16adbe97c9fe1460ef21fec7e31bc541de3d653a35895677ce52",                            \
    FIELD(ros_Vector3, force) FIELD(ros_Vector3, torque))                                          \
  CTYPE(                                                                                           \
    ros_State, "lifecycle_msgs::msg::dds_::State",                                                 \
    "dd2d02b82f3ebc858e53c431b1e6e91f3ffc71436fc81d0715214ac6ee2107a0",                            \
    FIELD(uint8_t, id) FIELD(rstring, label))                                                      \
  CTYPE(                                                                                           \
    ros_Transition, "lifecycle_msgs::msg::dds_::Transition",                                       \
    "c65d7b31ea134cba4f54fc867b817979be799f7452035cd35fac9b7421fb3424",                            \
    FIELD(uint8_t, id) FIELD(rstring, label))                                                      \
  CTYPE(                                                                                           \
    ros_TransitionDescription, "lifecycle_msgs::msg::dds_::TransitionDescription",                 \
    "c5f1cd4bb1ad2ba0e3329d4ac7015c52a674a72c1faf7974c37a33f4f6048b28",                            \
    FIELD(ros_Transition, transition) FIELD(ros_State, start_state) FIELD(ros_State, goal_state))  \
  CTYPE(                                                                                           \
    ros_TransitionEvent, "lifecycle_msgs::msg::dds_::TransitionEvent",                             \
    "d5f8873a2f0146498f812d7885c7327ce27e463d36811d8792f35ee38c0d6c38",                            \
    FIELD(uint64_t, timestamp) FIELD(ros_Transition, transition) FIELD(ros_State, start_state)     \
      FIELD(ros_State, goal_state))                                                                \
  CTYPE(                                                                                           \
    ros_ChannelFloat32, "sensor_msgs::msg::dds_::ChannelFloat32",                                  \
    "92665437ddf39346f4ba39ee32e648390605b633cc077d40f4bd4d7b58af6cd4",                            \
    FIELD(rstring, name) SEQUENCE(float, values))                                                  \
  CTYPE(                                                                                           \
    ros_JoyFeedback, "sensor_msgs::msg::dds_::JoyFeedback",                                        \
    "231dd362f71d6fc08272770d07120ad5fe5874ce2dbac70109b28986834290cd",                            \
    FIELD(uint8_t, type) FIELD(uint8_t, id) FIELD(float, intensity))                               \
  CTYPE(                                                                                           \
    ros_JoyFeedbackArray, "sensor_msgs::msg::dds_::JoyFeedbackArray",                              \
    "3287c32e1b688cae04555e465443df3cca7dae76ee4ebf85c4658d585037bcaa",                            \
    SEQUENCE(ros_JoyFeedback, array))                                                              \
  CTYPE(                                                                                           \
    ros_LaserEcho, "sensor_msgs::msg::dds_::LaserEcho",                                            \
    "0fbc05a0db7d37fe52c0f0375356db55da0046f7ef5bd27ca6b34bd0582bc952", SEQUENCE(float, echoes))   \
  CTYPE(                                                                                           \
    ros_NavSatStatus, "sensor_msgs::msg::dds_::NavSatStatus",                                      \
    "d1ed3befa628e09571bd273b888ba1c1fd187c9a5e0006b385d7e5e9095a3204",                            \
    FIELD(int8_t, status) FIELD(uint16_t, service))                                                \
  CTYPE(                                                                                           \
    ros_PointField, "sensor_msgs::msg::dds_::PointField",                                          \
    "5c6a4750728c2bcfbbf7037225b20b02d4429634732146b742dee1726637ef01",                            \
    FIELD(rstring, name) FIELD(uint32_t, offset) FIELD(uint8_t, datatype) FIELD(uint32_t, count))  \
  CTYPE(                                                                                           \
    ros_RegionOfInterest, "sensor_msgs::msg::dds_::RegionOfInterest",                              \
    "ad16bcba5f9131dcdba6fbded19f726f5440e3c513b4fb586dd3027eeed8abb1",                            \
    FIELD(uint32_t, x_offset) FIELD(uint32_t, y_offset) FIELD(uint32_t, height)                    \
      FIELD(uint32_t, width) FIELD(bool, do_rectify))                                              \
  CTYPE(                                                                                           \
    ros_ServiceEventInfo, "service_msgs::msg::dds_::ServiceEventInfo",                             \
    "41bcbbe07a75c9b52bc96bfd5c24d7f0fc0a08c0cb7921b3373c5732345a6f45",                            \
    FIELD(uint8_t, event_type) FIELD(ros_Time, stamp) ARRAY(uint8_t, client_gid, 16)               \
      FIELD(int64_t, sequence_number))                                                             \
  BTYPE(                                                                                           \
    ros_Bool, "std_msgs::msg::dds_::Bool",                                                         \
    "feb91e995ff9ebd09c0cb3d2aed18b11077585839fb5db80193b62d74528f6c9", bool)                      \
  TTYPE(                                                                                           \
    ros_Byte, "std_msgs::msg::dds_::Byte",                                                         \
    "41e1a3345f73fe93ede006da826a6ee274af23dd4653976ff249b0f44e3e798f", uint8_t)                   \
  BTYPE(                                                                                           \
    ros_Char, "std_msgs::msg::dds_::Char",                                                         \
    "3ad2d04dd29ba19d04b16659afa3ccaedd691914b02a64e82e252f2fa6a586a9", uint8_t)                   \
  CTYPE(                                                                                           \
    ros_ColorRGBA, "std_msgs::msg::dds_::ColorRGBA",                                               \
    "77a7a5b9ae477306097665106e0413ba74440245b1f3d0c6d6405fe5c7813fe8",                            \
    FIELD(float, r) FIELD(float, g) FIELD(float, b) FIELD(float, a))                               \
  BTYPE(                                                                                           \
    ros_Empty, "std_msgs::msg::dds_::Empty",                                                       \
    "20b625256f32d5dbc0d04fee44f43c41e51c70d3502f84b4a08e7a9c26a96312", uint8_t)                   \
  BTYPE(                                                                                           \
    ros_Float32, "std_msgs::msg::dds_::Float32",                                                   \
    "7170d3d8f841f7be3172ce5f4f59f3a4d7f63b0447e8b33327601ad64d83d6e2", float)                     \
  BTYPE(                                                                                           \
    ros_Float64, "std_msgs::msg::dds_::Float64",                                                   \
    "705ba9c3d1a09df43737eb67095534de36fd426c0587779bda2bc51fe790182a", double)                    \
  CTYPE(                                                                                           \
    ros_Header, "std_msgs::msg::dds_::Header",                                                     \
    "f49fb3ae2cf070f793645ff749683ac6b06203e41c891e17701b1cb597ce6a01",                            \
    FIELD(ros_Time, stamp) FIELD(rstring, frame_id))                                               \
  CTYPE(                                                                                           \
    ros_AccelStamped, "geometry_msgs::msg::dds_::AccelStamped",                                    \
    "ef1df9eabae0a708cc049a061ebcddc4e2a5f745730100ba680e086a9698b165",                            \
    FIELD(ros_Header, header) FIELD(ros_Accel, accel))                                             \
  CTYPE(                                                                                           \
    ros_AccelWithCovarianceStamped, "geometry_msgs::msg::dds_::AccelWithCovarianceStamped",        \
    "61c9ad8928e71dd95ce791b2f02809ee2a0bbcc42cd0e4047fd00a822a08e444",                            \
    FIELD(ros_Header, header) FIELD(ros_AccelWithCovariance, accel))                               \
  CTYPE(                                                                                           \
    ros_InertiaStamped, "geometry_msgs::msg::dds_::InertiaStamped",                                \
    "766be45976252babf7f9d8ac4ae7c912a7ceccf71035622529f27518b695aa09",                            \
    FIELD(ros_Header, header) FIELD(ros_Inertia, inertia))                                         \
  CTYPE(                                                                                           \
    ros_PointStamped, "geometry_msgs::msg::dds_::PointStamped",                                    \
    "4c0296af86e01e562e9e0405d138a01537247580076c58ea38d7923ac1045897",                            \
    FIELD(ros_Header, header) FIELD(ros_Point, point))                                             \
  CTYPE(                                                                                           \
    ros_PolygonInstanceStamped, "geometry_msgs::msg::dds_::PolygonInstanceStamped",                \
    "802f37ea4398d7ce547936aab1fd278923716a13c63373887cd896957434ce2f",                            \
    FIELD(ros_Header, header) FIELD(ros_PolygonInstance, polygon))                                 \
  CTYPE(                                                                                           \
    ros_PolygonStamped, "geometry_msgs::msg::dds_::PolygonStamped",                                \
    "b7cf07932f1523d4b4088075945c1a0141f7cd21da87cc940fc61652e9138b46",                            \
    FIELD(ros_Header, header) FIELD(ros_Polygon, polygon))                                         \
  CTYPE(                                                                                           \
    ros_PoseArray, "geometry_msgs::msg::dds_::PoseArray",                                          \
    "af0cc36d190e104d546d168d6b39df04fa4b4ccecf59cb4c9ed328d3d5004aa0",                            \
    FIELD(ros_Header, header) SEQUENCE(ros_Pose, poses))                                           \
  CTYPE(                                                                                           \
    ros_PoseStamped, "geometry_msgs::msg::dds_::PoseStamped",                                      \
    "10f3786d7d40fd2b54367835614bff85d4ad3b5dab62bf8bca0cc232d73b4cd8",                            \
    FIELD(ros_Header, header) FIELD(ros_Pose, pose))                                               \
  CTYPE(                                                                                           \
    ros_PoseWithCovarianceStamped, "geometry_msgs::msg::dds_::PoseWithCovarianceStamped",          \
    "26432f9803e43727d3c8f668d1fdb3c630f548af631e2f4e31382371bfea3b6e",                            \
    FIELD(ros_Header, header) FIELD(ros_PoseWithCovariance, pose))                                 \
  CTYPE(                                                                                           \
    ros_QuaternionStamped, "geometry_msgs::msg::dds_::QuaternionStamped",                          \
    "381add86c6c3160644d228ca342182c7fd6c7fab11c7a85ad817a9cc22dbac6e",                            \
    FIELD(ros_Header, header) FIELD(ros_Quaternion, quaternion))                                   \
  CTYPE(                                                                                           \
    ros_TransformStamped, "geometry_msgs::msg::dds_::TransformStamped",                            \
    "0a241f87d04668d94099cbb5ba11691d5ad32c2f29682e4eb5653424bd275206",                            \
    FIELD(ros_Header, header) FIELD(rstring, child_frame_id) FIELD(ros_Transform, transform))      \
  CTYPE(                                                                                           \
    ros_TwistStamped, "geometry_msgs::msg::dds_::TwistStamped",                                    \
    "5f0fcd4f81d5d06ad9b4c4c63e3ea51b82d6ae4d0558f1d475229b1121db6f64",                            \
    FIELD(ros_Header, header) FIELD(ros_Twist, twist))                                             \
  CTYPE(                                                                                           \
    ros_TwistWithCovarianceStamped, "geometry_msgs::msg::dds_::TwistWithCovarianceStamped",        \
    "77b67434531e6529b7a0091357b186b6ebdb17fd9ffd3e0c7ce9d3fb11a44563",                            \
    FIELD(ros_Header, header) FIELD(ros_TwistWithCovariance, twist))                               \
  CTYPE(                                                                                           \
    ros_Vector3Stamped, "geometry_msgs::msg::dds_::Vector3Stamped",                                \
    "d4829622288cbb443886e7ea94ea5671a3b1be6bab4ad04224432a65f7d7887a",                            \
    FIELD(ros_Header, header) FIELD(ros_Vector3, vector))                                          \
  CTYPE(                                                                                           \
    ros_VelocityStamped, "geometry_msgs::msg::dds_::VelocityStamped",                              \
    "55e7196186c8dbe4375278d7f1ac050dd8c9bacade1cf3eef8460fa667bd2457",                            \
    FIELD(ros_Header, header) FIELD(rstring, body_frame_id) FIELD(rstring, reference_frame_id)     \
      FIELD(ros_Twist, velocity))                                                                  \
  CTYPE(                                                                                           \
    ros_WrenchStamped, "geometry_msgs::msg::dds_::WrenchStamped",                                  \
    "8dc3deaf06b2ab281f9f9a742a8961c328ca7cec16e3fd6586d3a5c83fa78f77",                            \
    FIELD(ros_Header, header) FIELD(ros_Wrench, wrench))                                           \
  CTYPE(                                                                                           \
    ros_BatteryState, "sensor_msgs::msg::dds_::BatteryState",                                      \
    "4bee5dfce981c98faa6828b868307a0a73f992ed0789f374ee96c8f840e69741",                            \
    FIELD(ros_Header, header) FIELD(float, voltage) FIELD(float, temperature)                      \
      FIELD(float, current) FIELD(float, charge) FIELD(float, capacity)                            \
        FIELD(float, design_capacity) FIELD(float, percentage) FIELD(uint8_t, power_supply_status) \
          FIELD(uint8_t, power_supply_health) FIELD(uint8_t, power_supply_technology)              \
            FIELD(bool, present) SEQUENCE(float, cell_voltage) SEQUENCE(float, cell_temperature)   \
              FIELD(rstring, location) FIELD(rstring, serial_number))                              \
  CTYPE(                                                                                           \
    ros_CameraInfo, "sensor_msgs::msg::dds_::CameraInfo",                                          \
    "b3dfd68ff46c9d56c80fd3bd4ed22c7a4ddce8c8348f2f59c299e73118e7e275",                            \
    FIELD(ros_Header, header) FIELD(uint32_t, height) FIELD(uint32_t, width)                       \
      FIELD(rstring, distortion_model) SEQUENCE(double, d) ARRAY(double, k, 9) ARRAY(double, r, 9) \
        ARRAY(double, p, 12) FIELD(uint32_t, binning_x) FIELD(uint32_t, binning_y)                 \
          FIELD(ros_RegionOfInterest, roi))                                                        \
  CTYPE(                                                                                           \
    ros_CompressedImage, "sensor_msgs::msg::dds_::CompressedImage",                                \
    "15640771531571185e2efc8a100baf923961a4d15d5569652e6cb6691e8e371a",                            \
    FIELD(ros_Header, header) FIELD(rstring, format) SEQUENCE(uint8_t, data))                      \
  CTYPE(                                                                                           \
    ros_FluidPressure, "sensor_msgs::msg::dds_::FluidPressure",                                    \
    "22dfb2b145a0bd5a31a1ac3882a1b32148b51d9b2f3bab250290d66f3595bc32",                            \
    FIELD(ros_Header, header) FIELD(double, fluid_pressure) FIELD(double, variance))               \
  CTYPE(                                                                                           \
    ros_Illuminance, "sensor_msgs::msg::dds_::Illuminance",                                        \
    "b954b25f452fcf81a91c9c2a7e3b3fd85c4c873d452aecb3cfd8fd1da732a22d",                            \
    FIELD(ros_Header, header) FIELD(double, illuminance) FIELD(double, variance))                  \
  CTYPE(                                                                                           \
    ros_Image, "sensor_msgs::msg::dds_::Image",                                                    \
    "d31d41a9a4c4bc8eae9be757b0beed306564f7526c88ea6a4588fb9582527d47",                            \
    FIELD(ros_Header, header) FIELD(uint32_t, height) FIELD(uint32_t, width)                       \
      FIELD(rstring, encoding) FIELD(uint8_t, is_bigendian) FIELD(uint32_t, step)                  \
        SEQUENCE(uint8_t, data))                                                                   \
  CTYPE(                                                                                           \
    ros_Imu, "sensor_msgs::msg::dds_::Imu",                                                        \
    "7d9a00ff131080897a5ec7e26e315954b8eae3353c3f995c55faf71574000b5b",                            \
    FIELD(ros_Header, header) FIELD(ros_Quaternion, orientation)                                   \
      ARRAY(double, orientation_covariance, 9) FIELD(ros_Vector3, angular_velocity)                \
        ARRAY(double, angular_velocity_covariance, 9) FIELD(ros_Vector3, linear_acceleration)      \
          ARRAY(double, linear_acceleration_covariance, 9))                                        \
  CTYPE(                                                                                           \
    ros_JointState, "sensor_msgs::msg::dds_::JointState",                                          \
    "a13ee3a330e346c9d87b5aa18d24e11690752bd33a0350f11c5882bc9179260e",                            \
    FIELD(ros_Header, header) SEQUENCE(rstring, name) SEQUENCE(double, position)                   \
      SEQUENCE(double, velocity) SEQUENCE(double, effort))                                         \
  CTYPE(                                                                                           \
    ros_Joy, "sensor_msgs::msg::dds_::Joy",                                                        \
    "0d356c79cad3401e35ffeb75a96a96e08be3ef896b8b83841d73e890989372c5",                            \
    FIELD(ros_Header, header) SEQUENCE(float, axes) SEQUENCE(int32_t, buttons))                    \
  CTYPE(                                                                                           \
    ros_LaserScan, "sensor_msgs::msg::dds_::LaserScan",                                            \
    "64c191398013af96509d518dac71d5164f9382553fce5c1f8cca5be7924bd828",                            \
    FIELD(ros_Header, header) FIELD(float, angle_min) FIELD(float, angle_max)                      \
      FIELD(float, angle_increment) FIELD(float, time_increment) FIELD(float, scan_time)           \
        FIELD(float, range_min) FIELD(float, range_max) SEQUENCE(float, ranges)                    \
          SEQUENCE(float, intensities))                                                            \
  CTYPE(                                                                                           \
    ros_MagneticField, "sensor_msgs::msg::dds_::MagneticField",                                    \
    "e80f32f56a20486c9923008fc1a1db07bbb273cbbf6a5b3bfa00835ee00e4dff",                            \
    FIELD(ros_Header, header) FIELD(ros_Vector3, magnetic_field)                                   \
      ARRAY(double, magnetic_field_covariance, 9))                                                 \
  CTYPE(                                                                                           \
    ros_MultiDOFJointState, "sensor_msgs::msg::dds_::MultiDOFJointState",                          \
    "4d4ded702cfba7ff3ec783835c1a1425f75e53939a430ff355d1fee4b3bbc40b",                            \
    FIELD(ros_Header, header) SEQUENCE(rstring, joint_names) SEQUENCE(ros_Transform, transforms)   \
      SEQUENCE(ros_Twist, twist) SEQUENCE(ros_Wrench, wrench))                                     \
  CTYPE(                                                                                           \
    ros_MultiEchoLaserScan, "sensor_msgs::msg::dds_::MultiEchoLaserScan",                          \
    "ba5eac341cd5bbb2701527aa4568e8baec172b69cadb9a1945d6f149d087ee48",                            \
    FIELD(ros_Header, header) FIELD(float, angle_min) FIELD(float, angle_max)                      \
      FIELD(float, angle_increment) FIELD(float, time_increment) FIELD(float, scan_time)           \
        FIELD(float, range_min) FIELD(float, range_max) SEQUENCE(ros_LaserEcho, ranges)            \
          SEQUENCE(ros_LaserEcho, intensities))                                                    \
  CTYPE(                                                                                           \
    ros_NavSatFix, "sensor_msgs::msg::dds_::NavSatFix",                                            \
    "62223ab3fe210a15976021da7afddc9e200dc9ec75231c1b6a557fc598a65404",                            \
    FIELD(ros_Header, header) FIELD(ros_NavSatStatus, status) FIELD(double, latitude)              \
      FIELD(double, longitude) FIELD(double, altitude) ARRAY(double, position_covariance, 9)       \
        FIELD(uint8_t, position_covariance_type))                                                  \
  CTYPE(                                                                                           \
    ros_PointCloud, "sensor_msgs::msg::dds_::PointCloud",                                          \
    "614593df71d3c2b9bd4604a71b750fd218f0d65c045ea988b713719455a65b3b",                            \
    FIELD(ros_Header, header) SEQUENCE(ros_Point32, points)                                        \
      SEQUENCE(ros_ChannelFloat32, channels))                                                      \
  CTYPE(                                                                                           \
    ros_PointCloud2, "sensor_msgs::msg::dds_::PointCloud2",                                        \
    "9198cabf7da3796ae6fe19c4cb3bdd3525492988c70522628af5daa124bae2b5",                            \
    FIELD(ros_Header, header) FIELD(uint32_t, height) FIELD(uint32_t, width)                       \
      SEQUENCE(ros_PointField, fields) FIELD(bool, is_bigendian) FIELD(uint32_t, point_step)       \
        FIELD(uint32_t, row_step) SEQUENCE(uint8_t, data) FIELD(bool, is_dense))                   \
  CTYPE(                                                                                           \
    ros_Range, "sensor_msgs::msg::dds_::Range",                                                    \
    "b42b62562e93cbfe9d42b82fe5994dfa3d63d7d5c90a317981703f7388adff3a",                            \
    FIELD(ros_Header, header) FIELD(uint8_t, radiation_type) FIELD(float, field_of_view)           \
      FIELD(float, min_range) FIELD(float, max_range) FIELD(float, range) FIELD(float, variance))  \
  CTYPE(                                                                                           \
    ros_RelativeHumidity, "sensor_msgs::msg::dds_::RelativeHumidity",                              \
    "8687c99b4fb393cb2e545e407b5ea7fd0b5d8960bcd849a0f86c544740138839",                            \
    FIELD(ros_Header, header) FIELD(double, relative_humidity) FIELD(double, variance))            \
  CTYPE(                                                                                           \
    ros_Temperature, "sensor_msgs::msg::dds_::Temperature",                                        \
    "72514a14126ab9f8a9abec974c78e5610a367b59db5da355ff1fb982d5bad4b8",                            \
    FIELD(ros_Header, header) FIELD(double, temperature) FIELD(double, variance))                  \
  CTYPE(                                                                                           \
    ros_TimeReference, "sensor_msgs::msg::dds_::TimeReference",                                    \
    "dd66e84cf40bbb5d5a40472e6ecf2675a031334d4c426abdb2ad41801a8efc99",                            \
    FIELD(ros_Header, header) FIELD(ros_Time, time_ref) FIELD(rstring, source))                    \
  BTYPE(                                                                                           \
    ros_Int16, "std_msgs::msg::dds_::Int16",                                                       \
    "1dcc3464e47c288a55f943a389d337cdb06804de3f5cd7a266b0de718eee17e5", int16_t)                   \
  BTYPE(                                                                                           \
    ros_Int32, "std_msgs::msg::dds_::Int32",                                                       \
    "b6578ded3c58c626cfe8d1a6fb6e04f706f97e9f03d2727c9ff4e74b1cef0deb", int32_t)                   \
  BTYPE(                                                                                           \
    ros_Int64, "std_msgs::msg::dds_::Int64",                                                       \
    "8cd1048c2f186b6bd9a92472dc1ce51723c0833a221e2b7aecfff111774f4b49", int64_t)                   \
  BTYPE(                                                                                           \
    ros_Int8, "std_msgs::msg::dds_::Int8",                                                         \
    "26525065a403d972cb672f0777e333f0c799ad444ae5fcd79e43d1e73bd0f440", int8_t)                    \
  CTYPE(                                                                                           \
    ros_MultiArrayDimension, "std_msgs::msg::dds_::MultiArrayDimension",                           \
    "5e773a60a4c7fc8a54985f307c7837aa2994252a126c301957a24e31282c9cbe",                            \
    FIELD(rstring, label) FIELD(uint32_t, size) FIELD(uint32_t, stride))                           \
  CTYPE(                                                                                           \
    ros_MultiArrayLayout, "std_msgs::msg::dds_::MultiArrayLayout",                                 \
    "4c66e6f78e740ac103a94cf63259f968e48c617e7699e829b63c21a5cb50dac6",                            \
    SEQUENCE(ros_MultiArrayDimension, dim) FIELD(uint32_t, data_offset))                           \
  CTYPE(                                                                                           \
    ros_ByteMultiArray, "std_msgs::msg::dds_::ByteMultiArray",                                     \
    "972fec7f50ab3c1d06783c228e79e8a9a509021708c511c059926261ada901d4",                            \
    FIELD(ros_MultiArrayLayout, layout) SEQUENCE(uint8_t, data))                                   \
  CTYPE(                                                                                           \
    ros_Float32MultiArray, "std_msgs::msg::dds_::Float32MultiArray",                               \
    "0599f6f85b4bfca379873a0b4375a0aca022156bd2d7021275d116ed1fa8bfe0",                            \
    FIELD(ros_MultiArrayLayout, layout) SEQUENCE(float, data))                                     \
  CTYPE(                                                                                           \
    ros_Float64MultiArray, "std_msgs::msg::dds_::Float64MultiArray",                               \
    "1025ddc6b9552d191f89ef1a8d2f60f3d373e28b283d8891ddcc974e8c55397f",                            \
    FIELD(ros_MultiArrayLayout, layout) SEQUENCE(double, data))                                    \
  CTYPE(                                                                                           \
    ros_Int16MultiArray, "std_msgs::msg::dds_::Int16MultiArray",                                   \
    "b58810e8e5b90fb19a5062469eb8409f5ab11a446d60de7157a1457e52a076ce",                            \
    FIELD(ros_MultiArrayLayout, layout) SEQUENCE(int16_t, data))                                   \
  CTYPE(                                                                                           \
    ros_Int32MultiArray, "std_msgs::msg::dds_::Int32MultiArray",                                   \
    "84a7346323525d1b4dfca899df3820f245e54009dac5a6b69217d14fdefd1701",                            \
    FIELD(ros_MultiArrayLayout, layout) SEQUENCE(int32_t, data))                                   \
  CTYPE(                                                                                           \
    ros_Int64MultiArray, "std_msgs::msg::dds_::Int64MultiArray",                                   \
    "e60f9fe34d697f0939ad49d33158693c1277fbac0e2f04b7c2995dc21c89b422",                            \
    FIELD(ros_MultiArrayLayout, layout) SEQUENCE(int64_t, data))                                   \
  CTYPE(                                                                                           \
    ros_Int8MultiArray, "std_msgs::msg::dds_::Int8MultiArray",                                     \
    "f21998d4b492abd63330765d75d5831238d400740386f651f13a872a4d2188db",                            \
    FIELD(ros_MultiArrayLayout, layout) SEQUENCE(int8_t, data))                                    \
  BTYPE(                                                                                           \
    ros_String, "std_msgs::msg::dds_::String",                                                     \
    "df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18", rstring)                   \
  BTYPE(                                                                                           \
    ros_UInt16, "std_msgs::msg::dds_::UInt16",                                                     \
    "08a406e4b022bc22e907f985d6a9e9dd1d4fbecae573549cf49350113e7757b1", uint16_t)                  \
  CTYPE(                                                                                           \
    ros_UInt16MultiArray, "std_msgs::msg::dds_::UInt16MultiArray",                                 \
    "94fe73428ec63baecc774f8fb82406123e9291cf728f1b7c91caf5335129492b",                            \
    FIELD(ros_MultiArrayLayout, layout) SEQUENCE(uint16_t, data))                                  \
  BTYPE(                                                                                           \
    ros_UInt32, "std_msgs::msg::dds_::UInt32",                                                     \
    "a5c874829b752bc5fa190024b0ad76f578cc278271e855c7d02a818b3516fb4a", uint32_t)                  \
  CTYPE(                                                                                           \
    ros_UInt32MultiArray, "std_msgs::msg::dds_::UInt32MultiArray",                                 \
    "6c2577c7ad3cbdcc2164a41c12f1d5ad314ea320f3fb1ee47e78019fe16bb5b0",                            \
    FIELD(ros_MultiArrayLayout, layout) SEQUENCE(uint32_t, data))                                  \
  BTYPE(                                                                                           \
    ros_UInt64, "std_msgs::msg::dds_::UInt64",                                                     \
    "fbdc52018fc13755dce18024d1a671c856aa8b4aaf63adfb095b608f98e8c943", uint64_t)                  \
  CTYPE(                                                                                           \
    ros_UInt64MultiArray, "std_msgs::msg::dds_::UInt64MultiArray",                                 \
    "fc1c685c2f76bdc6983da025cb25d2db5fb5157b059e300f6d957d86f981b366",                            \
    FIELD(ros_MultiArrayLayout, layout) SEQUENCE(uint64_t, data))                                  \
  BTYPE(                                                                                           \
    ros_UInt8, "std_msgs::msg::dds_::UInt8",                                                       \
    "6138bd83d8c3569cb80a667db03cfc1629f529fee79d944c39c34e352e72f010", uint8_t)                   \
  CTYPE(                                                                                           \
    ros_UInt8MultiArray, "std_msgs::msg::dds_::UInt8MultiArray",                                   \
    "5687e861b8d307a5e48b7515467ae7a5fc2daf805bd0ce6d8e9e604bade9f385",                            \
    FIELD(ros_MultiArrayLayout, layout) SEQUENCE(uint8_t, data))

#define SRV_LIST(SRV, REQUEST, REPLY, FIELD, ARRAY, SEQUENCE)                          \
  SRV(                                                                                 \
    srv_GetState, "lifecycle_msgs::srv::dds_::GetState",                               \
    "77a9c844e7975f15bc2221e8bbebafc4d118afb78c7f7071eef91f83f940f2f6",                \
    REQUEST(FIELD(uint8_t, empty)), REPLY(FIELD(ros_State, current_state)))            \
  SRV(                                                                                 \
    srv_ChangeState, "lifecycle_msgs::srv::dds_::ChangeState",                         \
    "d74c6f524e14cade7b67e7297190360ed749264af300524fc1456f27cc2a553a",                \
    REQUEST(FIELD(ros_Transition, transition)), REPLY(FIELD(bool, success)))           \
  SRV(                                                                                 \
    srv_GetAvailableTransitions, "lifecycle_msgs::srv::dds_::GetAvailableTransitions", \
    "52a73c0b5d7489bee0df6ddb0aec45c41289d223ff225b58fe11d859ac8109a6",                \
    REQUEST(FIELD(uint8_t, empty)),                                                    \
    REPLY(SEQUENCE(ros_TransitionDescription, available_transitions)))                 \
  SRV(                                                                                 \
    srv_GetAvailableStates, "lifecycle_msgs::srv::dds_::GetAvailableStates",           \
    "c2c0c12de6023822f284929577d4ca2862e0501150010966e8a90b632e4424de",                \
    REQUEST(FIELD(uint8_t, empty)), REPLY(SEQUENCE(ros_State, available_states)))      \
  SRV(                                                                                 \
    srv_SetCameraInfo, "sensor_msgs::srv::dds_::SetCameraInfo",                        \
    "27dc5497730fdb6930e66cb879c210e31a83022acc5bbc999f24f55286029f87",                \
    REQUEST(FIELD(ros_CameraInfo, camera_info)),                                       \
    REPLY(FIELD(bool, success) FIELD(rstring, status_message)))

#endif /* GENERATED_TYPES_H */
