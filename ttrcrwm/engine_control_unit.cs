using System;
using System.Collections.Generic;

using Sandbox.Game.Entities;
using Sandbox.ModAPI;
using VRage.Game;
using VRage.Game.Components;
using VRage.Game.Entity;
using VRage.Game.ModAPI;
using VRage.Utils;
using VRageMath;

namespace ttrcrwm
{
    class engine_control_unit
    {
        #region fields

        const int   NUM_ROTATION_SAMPLES = 6, PHYSICS_ENABLE_DELAY = 6;
        const float MAX_THRUST_LEVEL     = 0.01f;

        enum thrust_dir { fore = 0, aft = 3, starboard = 1, port = 4, dorsal = 2, ventral = 5, uncontrolled = 6 };
        class thruster_info     // Technically a struct
        {
            public float      max_force, actual_max_force;
            public Vector3    torque_factor, grid_centre_pos, CoM_offset, reference_vector, static_moment;
            public thrust_dir nozzle_direction;
            public float      current_setting, prev_setting;
            public bool       is_RCS, group_no_RCS;
        };

        private static float[] __control_vector     = new float[6];
        private static float[] __actual_force       = new float[6];
        private static float[] __linear_component   = new float[6];
        private static float[] __thrust_vector      = new float[6];
        private static float[] __braking_vector     = new float[6];
        private static float[] __cur_firing_vector  = new float[6];
        private static float[] __settings           = new float[6];
        private static float[] __linear_velocity    = new float[6];
        private static  bool[] __is_override_active = new  bool[6];
        private static float[] __initial_setting    = new float[6];

        private static List<IMyThrust> __thrusters_to_move = new List<IMyThrust>();

        private MyCubeGrid _grid;
        private List<IMyBlockGroup   > _all_groups      = new List<IMyBlockGroup   >();
        private List<IMyTerminalBlock> _blocks_in_group = new List<IMyTerminalBlock>();

        private Dictionary<IMyThrust, thruster_info>[] _thrusters =
        {
            new Dictionary<IMyThrust, thruster_info>(),   // fore
            new Dictionary<IMyThrust, thruster_info>(),   // starboard
            new Dictionary<IMyThrust, thruster_info>(),   // dorsal
            new Dictionary<IMyThrust, thruster_info>(),   // aft
            new Dictionary<IMyThrust, thruster_info>(),   // port
            new Dictionary<IMyThrust, thruster_info>(),   // ventral
            new Dictionary<IMyThrust, thruster_info>()    // uncontrolled
        };
        private HashSet<thruster_info>[] _control_sets_pos =
        {
            new HashSet<thruster_info>(),   // roll clockwise
            new HashSet<thruster_info>(),   // pitch down
            new HashSet<thruster_info>(),   // yaw right
            new HashSet<thruster_info>(),   // roll counter-clockwise
            new HashSet<thruster_info>(),   // pitch up
            new HashSet<thruster_info>()    // yaw left
        };
        private HashSet<thruster_info>[] _control_sets_neg =
        {
            new HashSet<thruster_info>(),   // roll clockwise
            new HashSet<thruster_info>(),   // pitch down
            new HashSet<thruster_info>(),   // yaw right
            new HashSet<thruster_info>(),   // roll counter-clockwise
            new HashSet<thruster_info>(),   // pitch up
            new HashSet<thruster_info>()    // yaw left
        };
        private float[] _max_force   = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
        private float[] _lin_force   = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
        private  bool[] _RCS_control = new  bool[6];

        private HashSet<MyGyro> _gyroscopes = new HashSet<MyGyro>();

        private Vector3D _grid_CoM_location = Vector3D.Zero;
        private MatrixD  _inverse_world_transform;
        private float    _max_gyro_torque = 0.0f, _spherical_moment_of_inertia;

        private Vector3 _manual_thrust, _manual_rotation, _prev_rotation, _target_rotation, _gyro_override = Vector3.Zero, _local_angular_velocity;
        private bool    _is_gyro_override_active = false, _all_engines_off = false, _under_player_control = false, _rotation_active = false, _thruster_added_or_removed = false;
        private bool    _force_override_refresh = false;

        private Vector3[] _rotation_samples = new Vector3[NUM_ROTATION_SAMPLES];
        private Vector3   _sample_sum       = Vector3.Zero;
        private int       _current_index    = 0, _physics_enable_delay = PHYSICS_ENABLE_DELAY;

        private MyEntity _match_velocity_with = null;

        #endregion

        #region Properties

        public bool autopilot_on      { get; set; }
        public bool linear_dampers_on { get; set; }

        #endregion

        #region DEBUG

        private void screen_info(string message, int display_time_ms, MyFontEnum font, bool controlled_only)
        {
            bool display = !controlled_only;

            if (!display)
            {
                var controller = MyAPIGateway.Session.ControlledObject as MyShipController;
                if (controller != null)
                    display = controller.CubeGrid == _grid;
            }
            if (display)
                MyAPIGateway.Utilities.ShowNotification(message, display_time_ms, font);
        }

        private void log_ECU_action(string method_name, string message)
        {
            MyLog.Default.WriteLine(string.Format("TTDTWM\tengine_control_unit<{0} [{1}]>.{2}(): {3}", _grid.DisplayName, _grid.EntityId, method_name, message));
            int num_controlled_thrusters = 0;
            foreach (var cur_direction in _thrusters)
                num_controlled_thrusters += cur_direction.Count;
            MyLog.Default.WriteLine(string.Format("TTDTWM\ttotal thrusters: {0} ({1}/{2}/{3}/{4}/{5}/{6} controlled)", 
                num_controlled_thrusters,
                _thrusters[(int) thrust_dir.fore        ].Count,
                _thrusters[(int) thrust_dir.aft         ].Count,
                _thrusters[(int) thrust_dir.starboard   ].Count,
                _thrusters[(int) thrust_dir.port        ].Count,
                _thrusters[(int) thrust_dir.dorsal      ].Count,
                _thrusters[(int) thrust_dir.ventral     ].Count,
                _thrusters[(int) thrust_dir.uncontrolled].Count));
        }

        private void screen_text(string method_name, string message, int display_time_ms, bool controlled_only)
        {
            if (method_name == "")
                screen_info(string.Format("\"{0}\" {1}", _grid.DisplayName, message), display_time_ms, MyFontEnum.White, controlled_only);
            else
                screen_info(string.Format("engine_control_unit.{0}(): \"{1}\" {2}", method_name, _grid.DisplayName, message), display_time_ms, MyFontEnum.White, controlled_only);
        }

        private void screen_vector<type>(string method_name, string vector_name, type[] vector, int display_time_ms, bool controlled_only)
        {
            screen_text(method_name, string.Format("{0} = {1:F3}/{2:F3}/{3:F3}/{4:F3}/{5:F3}/{6:F3}", 
                vector_name,
                vector[(int) thrust_dir.fore     ],
                vector[(int) thrust_dir.aft      ],
                vector[(int) thrust_dir.starboard],
                vector[(int) thrust_dir.port     ],
                vector[(int) thrust_dir.dorsal   ],
                vector[(int) thrust_dir.ventral  ]), display_time_ms, controlled_only);
        }

        #endregion

        #region torque calculation

        private void refresh_thruster_info_for_single_direction(Dictionary<IMyThrust, thruster_info> thrusters)
        {
            thruster_info cur_thruster_info;

            foreach (var cur_thruster in thrusters)
            {
                cur_thruster_info = cur_thruster.Value;
                cur_thruster_info.CoM_offset    = cur_thruster_info.grid_centre_pos - _grid_CoM_location;
                cur_thruster_info.torque_factor = settings.TORQUE_MULTIPLIER * Vector3.Cross(cur_thruster_info.CoM_offset, -((MyThrust) cur_thruster.Key).ThrustForwardVector);
            }
        }

        private void refresh_thruster_info()
        {
            for (int dir_index = 0; dir_index < 6; ++dir_index)
                refresh_thruster_info_for_single_direction(_thrusters[dir_index]);
        }

        private void calculate_and_apply_torque()
        {
            //if (MyAPIGateway.Multiplayer != null && !MyAPIGateway.Multiplayer.IsServer)
            //    return;
            //const float MIN_TORQUE = 10.0f;
            const float MIN_ANGULAR_ACCELERATION = (float) (0.1 * Math.PI / 180.0);

            Vector3   torque = Vector3.Zero, useful_torque/*, adverse_torque*/;
            IMyThrust thruster_entry;

            for (int dir_index = 0; dir_index < 6; ++dir_index)
            {
                foreach (var cur_thruster in _thrusters[dir_index])
                {
                    thruster_entry = cur_thruster.Key;
                    if (thruster_entry.IsWorking)
                        torque += cur_thruster.Value.torque_factor * thruster_entry.CurrentThrust;
                }
            }

            if (!_rotation_active || autopilot_on)
            {
                useful_torque  = Vector3.Zero;
                //adverse_torque = torque;
            }
            else
            {
                /*
                Vector3 reference_vector = (_manual_rotation.LengthSquared() > 0.0001f) ? _manual_rotation : _local_angular_velocity;
                if (reference_vector.LengthSquared() > 0.0001f)
                {
                    useful_torque    = (Vector3.Dot(torque, reference_vector) / reference_vector.LengthSquared()) * reference_vector;
                    parasitic_torque = torque - useful_torque;
                }
                else
                {
                    useful_torque    = torque;
                    parasitic_torque = Vector3.Zero;
                }
                */

                float manual_rotation_length2 = _manual_rotation.LengthSquared(), angular_velocity_length2 = _local_angular_velocity.LengthSquared();

                if (manual_rotation_length2 <= 0.0001f)
                    useful_torque = Vector3.Zero;
                else
                {
                    float projection_dot_product = Vector3.Dot(torque, _manual_rotation);

                    useful_torque = (projection_dot_product > 0.0f) ? ((projection_dot_product / manual_rotation_length2) * _manual_rotation) : Vector3.Zero;
                }
                Vector3 leftover_torque = torque - useful_torque;
                if (angular_velocity_length2 > 0.0001f)
                {
                    float projection_dot_product = Vector3.Dot(leftover_torque, _local_angular_velocity);

                    if (projection_dot_product < 0.0f)
                        useful_torque += (projection_dot_product / angular_velocity_length2) * _local_angular_velocity;
                }
                //adverse_torque = torque - useful_torque;
            }

            //screen_text("", string.Format("UT = {0:F4} MN*m, PT = {1:F4} MN*m", useful_torque.Length() / 1.0E+6f, parasitic_torque.Length() / 1.0E+6f), 16, controlled_only: true);

            //float gyro_limit = _max_gyro_torque;
            /*
            if (_is_gyro_override_active || _manual_rotation.LengthSquared() <= 0.0001f)
            {
                Vector3 angular_velocity_diff = _local_angular_velocity;

                if (_is_gyro_override_active)
                    angular_velocity_diff -= _gyro_override;
                gyro_limit -= angular_velocity_diff.Length() * _spherical_moment_of_inertia;
            }
            */
            //if (gyro_limit < 1.0f)
            //    gyro_limit = 1.0f;
            /*
            if (adverse_torque.LengthSquared() <= gyro_limit * gyro_limit)
                adverse_torque = Vector3.Zero;
            else
                adverse_torque -= Vector3.Normalize(adverse_torque) * gyro_limit;

            torque = useful_torque + adverse_torque;
            */
            if (_physics_enable_delay > 0)
                --_physics_enable_delay;
            else if (useful_torque.LengthSquared() > MIN_ANGULAR_ACCELERATION * MIN_ANGULAR_ACCELERATION * _spherical_moment_of_inertia * _spherical_moment_of_inertia)
            {
                torque = Vector3.Transform(useful_torque, _grid.WorldMatrix.GetOrientation());
                _grid.Physics.AddForce(MyPhysicsForceType.APPLY_WORLD_IMPULSE_AND_WORLD_ANGULAR_IMPULSE, Vector3.Zero, null, torque);
            }
        }

        #endregion

        #region thrust control

        private static void decompose_vector(Vector3 source_vector, float[] decomposed_vector)
        {
            decomposed_vector[(int) thrust_dir.fore     ] = (source_vector.Z > 0.0f) ? ( source_vector.Z) : 0.0f;
            decomposed_vector[(int) thrust_dir.aft      ] = (source_vector.Z < 0.0f) ? (-source_vector.Z) : 0.0f;
            decomposed_vector[(int) thrust_dir.port     ] = (source_vector.X > 0.0f) ? ( source_vector.X) : 0.0f;
            decomposed_vector[(int) thrust_dir.starboard] = (source_vector.X < 0.0f) ? (-source_vector.X) : 0.0f;
            decomposed_vector[(int) thrust_dir.ventral  ] = (source_vector.Y > 0.0f) ? ( source_vector.Y) : 0.0f;
            decomposed_vector[(int) thrust_dir.dorsal   ] = (source_vector.Y < 0.0f) ? (-source_vector.Y) : 0.0f;
        }

        private static void recompose_vector(float[] decomposed_vector, out Vector3 result_vector)
        {
            result_vector.Z = decomposed_vector[(int) thrust_dir.fore   ] - decomposed_vector[(int) thrust_dir.aft      ];
            result_vector.X = decomposed_vector[(int) thrust_dir.port   ] - decomposed_vector[(int) thrust_dir.starboard];
            result_vector.Y = decomposed_vector[(int) thrust_dir.ventral] - decomposed_vector[(int) thrust_dir.dorsal   ];
        }

        private void fill_control_sets(thruster_info cur_thruster_info)
        {
            const float MIN_LEVERAGE = 0.01f;

            if (cur_thruster_info.reference_vector.LengthSquared() < _grid.GridSize * _grid.GridSize)
                return;

            Vector3 sample_vector, reference_norm = Vector3.Normalize(cur_thruster_info.reference_vector);
            for (int dir_index = 0; dir_index < 6; ++dir_index)
            {
                __cur_firing_vector[dir_index] = 1.0f;
                recompose_vector(__cur_firing_vector, out sample_vector);
                decompose_vector(Vector3.Cross(sample_vector, reference_norm), __linear_component);
                if (__linear_component[(int) cur_thruster_info.nozzle_direction] > MIN_LEVERAGE)
                    _control_sets_pos[dir_index].Add(cur_thruster_info);
                else if (__linear_component[((int) cur_thruster_info.nozzle_direction + 3) % 6] > MIN_LEVERAGE)
                    _control_sets_neg[dir_index].Add(cur_thruster_info);
                __cur_firing_vector[dir_index] = 0.0f;
            }
        }

        private void refresh_control_sets()
        {
            for (int dir_index = 0; dir_index < 6; ++dir_index)
            {
                _control_sets_pos[dir_index].Clear();
                _control_sets_neg[dir_index].Clear();
            }
            for (int dir_index = 0; dir_index < 6; ++dir_index)
            {
                foreach (var cur_thruster_info in _thrusters[dir_index].Values)
                    fill_control_sets(cur_thruster_info);
            }
        }

        private void apply_thrust_settings(bool reset_all_thrusters)
        {
            const float MIN_OVERRIDE = 1.001f;

            float         setting;
            bool          dry_run;
            thruster_info cur_thruster_info;
            IMyThrust     thruster_entry;

            if (reset_all_thrusters && _all_engines_off && !_force_override_refresh)
                return;

            if (MyAPIGateway.Multiplayer == null || MyAPIGateway.Multiplayer.IsServer)
                dry_run = false;
            else
            {
                bool is_rotation_small = (_manual_rotation - _prev_rotation).LengthSquared() < 0.0001f;

                dry_run = !_force_override_refresh || is_rotation_small;
                if (!is_rotation_small)
                    _prev_rotation = _manual_rotation;
            }

            for (int dir_index = 0; dir_index <= 6; ++dir_index)
            {
                foreach (var cur_thruster in _thrusters[dir_index])
                {
                    thruster_entry = cur_thruster.Key;
                    if (!thruster_entry.ShowInTerminal)
                        continue;

                    cur_thruster_info = cur_thruster.Value;
                    if (!_RCS_control[(int) cur_thruster_info.nozzle_direction])
                        continue;

                    if (_force_override_refresh)
                        cur_thruster_info.prev_setting = thruster_entry.CurrentThrust;
                    if (reset_all_thrusters || cur_thruster_info.actual_max_force < 1.0f || !thruster_entry.IsWorking)
                    {
                        if (cur_thruster_info.prev_setting != 0)
                        {
                            if (!dry_run)
                                thruster_entry.ThrustOverride = 0.0f;
                            cur_thruster_info.current_setting = cur_thruster_info.prev_setting = 0;
                        }
                        continue;
                    }

                    setting = cur_thruster_info.current_setting * cur_thruster_info.max_force;
                    if (_rotation_active && dir_index < 6 && setting < MIN_OVERRIDE)
                        setting = MIN_OVERRIDE;
                    if (setting != cur_thruster_info.prev_setting)
                    {
                        if (!dry_run)
                            thruster_entry.ThrustOverride = setting;
                        cur_thruster_info.prev_setting = setting;
                    }
                }
            }

            _all_engines_off        = reset_all_thrusters;
            _force_override_refresh = false;
        }

        // Ensures that resulting linear force is zero (to prevent undesired drift when turning)
        void normalise_thrust()
        {
            int   opposite_dir = 3;
            float new_force_ratio, current_force, opposite_force;

            for (int dir_index = 0; dir_index < 3; ++dir_index)
            {
                current_force = opposite_force = 0.0f;
                foreach (var cur_thruster_info in _thrusters[dir_index].Values)
                    current_force += cur_thruster_info.current_setting * cur_thruster_info.actual_max_force;
                foreach (var cur_thruster_info in _thrusters[opposite_dir].Values)
                    opposite_force += cur_thruster_info.current_setting * cur_thruster_info.actual_max_force;

                float force1 = __braking_vector[   dir_index] + __thrust_vector[   dir_index] * _lin_force[   dir_index];
                float force2 = __braking_vector[opposite_dir] + __thrust_vector[opposite_dir] * _lin_force[opposite_dir];
                if (current_force >= 1.0f && current_force - force1 > opposite_force)
                {
                    new_force_ratio = (opposite_force + force1) / current_force;
                    foreach (var cur_thruster_info in _thrusters[dir_index].Values)
                        cur_thruster_info.current_setting *= new_force_ratio;
                }
                if (opposite_force >= 1.0f && opposite_force - force2 > current_force)
                {
                    new_force_ratio = (current_force + force2) / opposite_force;
                    foreach (var cur_thruster_info in _thrusters[opposite_dir].Values)
                        cur_thruster_info.current_setting *= new_force_ratio;
                }

                ++opposite_dir;
            }
        }

        private void handle_thrust_control()
        {
            const float DAMPING_CONSTANT = 10.0f;

            Array.Clear(__is_override_active, 0, 6);
            for (int dir_index = 0; dir_index <= 6; ++dir_index)
            {
                thruster_info cur_thruster_info;
                IMyThrust     thruster_entry;

                foreach (var cur_thruster in _thrusters[dir_index])
                {
                    cur_thruster_info = cur_thruster.Value;
                    thruster_entry    = cur_thruster.Key;
                    if (!thruster_entry.ShowInTerminal)
                        __is_override_active[(int) cur_thruster_info.nozzle_direction] |= thruster_entry.ThrustOverride >= 1.0f;
                }
            }

            Matrix inverse_world_rotation   = _inverse_world_transform.GetOrientation();
            _local_angular_velocity         = Vector3.Transform(_grid.Physics.AngularVelocity, inverse_world_rotation);
            float   manual_rotation_length2 = _manual_rotation.LengthSquared();

            Vector3 world_linear_velocity   = _grid.Physics.LinearVelocity;
            if (_match_velocity_with?.Physics != null)
                world_linear_velocity -= _match_velocity_with.Physics.LinearVelocity;

            Vector3 desirted_angular_velocity, local_linear_velocity = Vector3.Transform(world_linear_velocity, inverse_world_rotation);
            if (manual_rotation_length2 <= 0.0001f)
                desirted_angular_velocity = -_local_angular_velocity;
            else
            {
                float   projection_dot_prduct     = Vector3.Dot(_local_angular_velocity, _manual_rotation);
                Vector3 local_velocity_projection = (projection_dot_prduct / manual_rotation_length2) * _manual_rotation,
                        local_velocity_rejection  = _local_angular_velocity - local_velocity_projection;

                desirted_angular_velocity = _manual_rotation * settings.MANUAL_INPUT_SENSITIVITY - local_velocity_rejection;
                if (projection_dot_prduct >= 0.0f)
                    desirted_angular_velocity += local_velocity_projection;
                else
                    desirted_angular_velocity -= local_velocity_projection;
            }

            _rotation_active = desirted_angular_velocity.LengthSquared() >= 0.0005f;
            if (!_rotation_active || _is_gyro_override_active || autopilot_on)
            {
                apply_thrust_settings(reset_all_thrusters: true);
                return;
            }
            decompose_vector(desirted_angular_velocity, __control_vector);
            decompose_vector(    local_linear_velocity, __linear_velocity);

            Vector3 linear_damping = !linear_dampers_on ? Vector3.Zero : Vector3.Transform((2.0f * world_linear_velocity + _grid.Physics.Gravity) * (-_grid.Physics.Mass), inverse_world_rotation);
            decompose_vector(_manual_thrust, __thrust_vector );
            decompose_vector(linear_damping, __braking_vector);

            for (int dir_index = 0; dir_index < 3; ++dir_index)
            {
                int opposite_dir = dir_index + 3;

                if (__thrust_vector[dir_index] >= MAX_THRUST_LEVEL || __thrust_vector[opposite_dir] >= MAX_THRUST_LEVEL)
                    __braking_vector[dir_index] = __braking_vector[opposite_dir] = 0.0f;
            }
            for (int dir_index = 0; dir_index < 6; ++dir_index)
            {
                int opposite_dir = (dir_index + 3) % 6;

                if (_RCS_control[dir_index] && !__is_override_active[dir_index] && !__is_override_active[opposite_dir]) 
                    __initial_setting[dir_index] = (_max_force[dir_index] >= 1.0f && _lin_force[dir_index] >= 1.0f) ? (__thrust_vector[dir_index] + __braking_vector[dir_index] / _lin_force[dir_index]) : 0.0f;
                else
                    __initial_setting[dir_index] = 0.0f;
                if (__initial_setting[dir_index] > 1.0f)
                    __initial_setting[dir_index] = 1.0f;

                foreach (var cur_thruster_info in _thrusters[dir_index].Values)
                    cur_thruster_info.current_setting = __initial_setting[dir_index];
            }
            foreach (var cur_thruster_info in _thrusters[(int) thrust_dir.uncontrolled].Values)
                cur_thruster_info.current_setting = __initial_setting[(int) cur_thruster_info.nozzle_direction];
            for (int dir_index = 0; dir_index < 6; ++dir_index)
            {
                float control = DAMPING_CONSTANT * __control_vector[dir_index] * _grid.Physics.Mass;

                for (int thruster_dir = 0; thruster_dir < 6; ++thruster_dir)
                    __settings[thruster_dir] = (_max_force[thruster_dir] >= 1.0f) ? (control / _max_force[thruster_dir]) : 0.0f;
                foreach (var cur_thruster_info in _control_sets_pos[dir_index])
                {
                    cur_thruster_info.current_setting += __settings[(int) cur_thruster_info.nozzle_direction];
                    if (cur_thruster_info.current_setting > 1.0f)
                        cur_thruster_info.current_setting = 1.0f;
                }
                foreach (var cur_thruster_info in _control_sets_neg[dir_index])
                {
                    cur_thruster_info.current_setting -= __settings[(int) cur_thruster_info.nozzle_direction];
                    if (cur_thruster_info.current_setting < 0.0f)
                        cur_thruster_info.current_setting = 0.0f;
                }
            }

            normalise_thrust();
            apply_thrust_settings(reset_all_thrusters: false);
        }

        #endregion

        #region thruster manager

        private static thrust_dir get_nozzle_orientation(IMyThrust thruster)
        {
            Vector3I dir_vector = ((MyThrust) thruster).ThrustForwardVector;
            if (dir_vector == Vector3I.Forward)
                return thrust_dir.fore;
            if (dir_vector == Vector3I.Backward)
                return thrust_dir.aft;
            if (dir_vector == Vector3I.Left)
                return thrust_dir.port;
            if (dir_vector == Vector3I.Right)
                return thrust_dir.starboard;
            if (dir_vector == Vector3I.Up)
                return thrust_dir.dorsal;
            if (dir_vector == Vector3I.Down)
                return thrust_dir.ventral;
            throw new ArgumentException("Thruster " + thruster.CustomName  + " is not grid-aligned");
        }

        private void update_reference_vectors()
        {
            Vector3 total_static_moment, CoT_location;

            for (int dir_index = 0; dir_index < 3; ++dir_index)
            {
                if (_max_force[dir_index] < 1.0f || _max_force[dir_index + 3] < 1.0f)
                {
                    foreach (var cur_thruster_info in _thrusters[dir_index].Values)
                        cur_thruster_info.reference_vector = cur_thruster_info.CoM_offset;
                    foreach (var cur_thruster_info in _thrusters[dir_index + 3].Values)
                        cur_thruster_info.reference_vector = cur_thruster_info.CoM_offset;
                }
                else
                {
                    total_static_moment = Vector3.Zero;
                    foreach (var cur_thruster_info in _thrusters[dir_index].Values)
                        total_static_moment += cur_thruster_info.static_moment;
                    foreach (var cur_thruster_info in _thrusters[dir_index + 3].Values)
                        total_static_moment += cur_thruster_info.static_moment;
                    CoT_location = total_static_moment / (_max_force[dir_index] + _max_force[dir_index + 3]);
                    foreach (var cur_thruster_info in _thrusters[dir_index].Values)
                        cur_thruster_info.reference_vector = cur_thruster_info.grid_centre_pos - CoT_location;
                    foreach (var cur_thruster_info in _thrusters[dir_index + 3].Values)
                        cur_thruster_info.reference_vector = cur_thruster_info.grid_centre_pos - CoT_location;
                }
            }
        }

        void check_thruster_control_changed()
        {
            bool changes_made = false;

            /*
            foreach (var cur_direction in _thrusters)
            {
                foreach (var cur_thruster in cur_direction)
                    cur_thruster.Value.group_no_RCS = false;
            }

            IMyGridTerminalSystem grid_terminal = MyAPIGateway.TerminalActionsHelper?.GetTerminalSystemForGrid(_grid);
            if (grid_terminal == null)
                return;
            _all_groups.Clear();
            grid_terminal.GetBlockGroups(_all_groups);

            foreach (var cur_group in _all_groups)
            {
                IMyThrust cur_thruster;

                if (cur_group.Name.ToUpper().Contains("[NO RCS]"))
                {
                    _blocks_in_group.Clear();
                    cur_group.GetBlocks(_blocks_in_group);
                    foreach (var cur_block in _blocks_in_group)
                    {
                        cur_thruster = cur_block as IMyThrust;
                        if (cur_thruster == null)
                            continue;
                        foreach (var cur_direction in _thrusters)
                        {
                            if (cur_direction.ContainsKey(cur_thruster))
                            {
                                cur_direction[cur_thruster].group_no_RCS = true;
                                break;
                            }
                        }
                    }
                }
            }
            */

            for (int dir_index = 0; dir_index <= 6; ++dir_index)
            {
                thruster_info cur_thruster_info;

                __thrusters_to_move.Clear();
                foreach (var cur_thruster in _thrusters[dir_index])
                {
                    cur_thruster_info = cur_thruster.Value;
                    if (cur_thruster_info.actual_max_force < 0.01f * cur_thruster_info.max_force || !cur_thruster.Key.IsWorking)
                    {
                        cur_thruster_info.is_RCS = false;
                        if (dir_index < 6)
                        {
                            __thrusters_to_move.Add(cur_thruster.Key);
                            changes_made = true;
                        }
                    }
                    else if (cur_thruster_info.is_RCS == cur_thruster_info.group_no_RCS)
                    {
                        cur_thruster_info.is_RCS = !cur_thruster_info.group_no_RCS;
                        changes_made             = true;
                        __thrusters_to_move.Add(cur_thruster.Key);
                    }
                }
                if (dir_index < 6)
                {
                    foreach (var cur_thruster in __thrusters_to_move)
                    {
                        if (cur_thruster.ShowInTerminal)
                            cur_thruster.ThrustOverride = 0.0f;
                        _thrusters[(int) thrust_dir.uncontrolled].Add(cur_thruster, _thrusters[dir_index][cur_thruster]);
                        _thrusters[                    dir_index].Remove(cur_thruster);
                    }
                    _RCS_control[dir_index] = _thrusters[dir_index].Count > 0;
                }
                else
                {
                    foreach (var cur_thruster in __thrusters_to_move)
                    {
                        if (cur_thruster.ShowInTerminal)
                            cur_thruster.ThrustOverride = 0.0f;
                        cur_thruster_info = _thrusters[(int) thrust_dir.uncontrolled][cur_thruster];
                        _thrusters[(int) cur_thruster_info.nozzle_direction].Add(cur_thruster, cur_thruster_info);
                        _thrusters[(int)            thrust_dir.uncontrolled].Remove(cur_thruster);
                    }
                }
                __thrusters_to_move.Clear();
            }

            //screen_text("", string.Format("THR = {0}/{1}", num_thr - num_nc_thr, num_thr), 2000, controlled_only: true);

            if (changes_made)
            {
                for (int dir_index = 0; dir_index < 6; ++dir_index)
                {
                    _max_force[dir_index] = 0.0f;
                    foreach (var cur_thruster_info in _thrusters[dir_index].Values)
                        _max_force[dir_index] += cur_thruster_info.actual_max_force;
                    _max_force[dir_index] *= settings.TORQUE_MULTIPLIER;
                }
                /*
                log_ECU_action("check_thruster_control_changed", string.Format("{0}/{1}/{2}/{3}/{4}/{5} kN",
                    _max_force[(int) thrust_dir.fore     ] / 1000.0f,
                    _max_force[(int) thrust_dir.aft      ] / 1000.0f,
                    _max_force[(int) thrust_dir.starboard] / 1000.0f,
                    _max_force[(int) thrust_dir.port     ] / 1000.0f,
                    _max_force[(int) thrust_dir.dorsal   ] / 1000.0f,
                    _max_force[(int) thrust_dir.ventral  ] / 1000.0f));
                */
            }
            if (changes_made || _thruster_added_or_removed)
            {
                refresh_thruster_info();
                update_reference_vectors();
                refresh_control_sets();
                _thruster_added_or_removed = false;
            }
        }

        private void refresh_real_max_forces_for_single_direction(int dir_index, bool atmosphere_present, float air_density)
        {
            thruster_info      cur_thruster_info;
            //float              thrust_multiplier, planetoid_influence;
            //MyThrustDefinition thruster_definition;

            if (dir_index < 6)
                _lin_force[dir_index] = _max_force[dir_index] = 0.0f;
            foreach (var cur_thruster in _thrusters[dir_index])
            {
                cur_thruster_info = cur_thruster.Value;
                /*
                thruster_definition = cur_thruster.Key.BlockDefinition;

                if (!atmosphere_present && thruster_definition.NeedsAtmosphereForInfluence)
                    planetoid_influence = 0.0f;
                else if (thruster_definition.MaxPlanetaryInfluence <= thruster_definition.MinPlanetaryInfluence)
                    planetoid_influence = 1.0f;
                else
                {
                    planetoid_influence = (air_density - thruster_definition.MinPlanetaryInfluence) / (thruster_definition.MaxPlanetaryInfluence - thruster_definition.MinPlanetaryInfluence);
                    if (planetoid_influence < 0.0f)
                        planetoid_influence = 0.0f;
                    else if (planetoid_influence > 1.0f)
                        planetoid_influence = 1.0f;
                }
                thrust_multiplier = (1.0f - planetoid_influence) * thruster_definition.EffectivenessAtMinInfluence + planetoid_influence * thruster_definition.EffectivenessAtMaxInfluence;
                */

                cur_thruster_info.actual_max_force                    = cur_thruster.Key.MaxEffectiveThrust;
                cur_thruster_info.static_moment                       = cur_thruster_info.grid_centre_pos * cur_thruster_info.actual_max_force * settings.TORQUE_MULTIPLIER;
                _lin_force[(int) cur_thruster_info.nozzle_direction] += cur_thruster_info.actual_max_force;
                if (dir_index < 6)
                    _max_force[dir_index] += cur_thruster_info.actual_max_force;
            }
            if (dir_index < 6)
                _max_force[dir_index] *= settings.TORQUE_MULTIPLIER;
        }

        private void refresh_real_max_forces()
        {
            BoundingBoxD grid_bounding_box = _grid.PositionComp.WorldAABB;
            MyPlanet     closest_planetoid = MyGamePruningStructure.GetClosestPlanet(ref grid_bounding_box);
            bool         atmosphere_present;
            float        air_density;

            if (closest_planetoid == null)
            {
                atmosphere_present = false;
                air_density        = 0.0f;
            }
            else
            {
                atmosphere_present = closest_planetoid.HasAtmosphere;
                air_density        = closest_planetoid.GetAirDensity(grid_bounding_box.Center);
            }

            for (int dir_index = 0; dir_index <= 6; ++dir_index)
                refresh_real_max_forces_for_single_direction(dir_index, atmosphere_present, air_density);
        }

        public void assign_thruster(IMyThrust thruster)
        {
            //if (MyAPIGateway.Multiplayer == null || MyAPIGateway.Multiplayer.IsServer)
            //    thruster.ThrustOverride = 0.0f;
            var new_thruster = new thruster_info();
            new_thruster.grid_centre_pos  = (thruster.Min + thruster.Max) * (_grid.GridSize / 2.0f);
            new_thruster.max_force        = thruster.MaxThrust;
            new_thruster.actual_max_force = thruster.MaxEffectiveThrust;
            new_thruster.CoM_offset       = new_thruster.reference_vector = new_thruster.grid_centre_pos - _grid_CoM_location;
            new_thruster.static_moment    = new_thruster.grid_centre_pos * new_thruster.actual_max_force;
            new_thruster.nozzle_direction = get_nozzle_orientation(thruster);
            new_thruster.is_RCS           = false;
            new_thruster.group_no_RCS     = _thruster_added_or_removed = true;

            foreach (var cur_subtype in settings.RCS_subtypes)
            {
                if (thruster.BlockDefinition.SubtypeName == cur_subtype)
                {
                    new_thruster.group_no_RCS = false;
                    break;
                }
            }

            _lin_force[(int) new_thruster.nozzle_direction] += new_thruster.actual_max_force;
            _thrusters[(int)       thrust_dir.uncontrolled].Add(thruster, new_thruster);
            //log_ECU_action("assign_thruster", string.Format("{0} ({1}) [{2}]\n\t\t\tCentre position: {3}",
            //    ((PB.IMyTerminalBlock) thruster).CustomName, new_thruster.nozzle_direction.ToString(), thruster.EntityId, 
            //    new_thruster.grid_centre_pos));
        }

        public void dispose_thruster(IMyThrust thruster)
        {
            for (int dir_index = 0; dir_index <= 6; ++dir_index)
            {
                Dictionary<IMyThrust, thruster_info> cur_direction = _thrusters[dir_index];

                if (cur_direction.ContainsKey(thruster))
                {
                    thruster_info cur_thruster_info = cur_direction[thruster];
                    if (cur_thruster_info.is_RCS)
                        _max_force[(int) cur_thruster_info.nozzle_direction] -= settings.TORQUE_MULTIPLIER * cur_thruster_info.actual_max_force;
                    _lin_force[(int) cur_thruster_info.nozzle_direction] -= cur_thruster_info.actual_max_force;
                    cur_direction.Remove(thruster);
                    _thruster_added_or_removed = true;
                    //log_ECU_action("dispose_thruster", string.Format("{0} ({1}) [{2}]", ((PB.IMyTerminalBlock) thruster).CustomName, get_nozzle_orientation(thruster).ToString(), thruster.EntityId));
                    break;
                }
            }
        }

        public engine_control_unit(IMyCubeGrid grid_ref)
        {
            _grid = (MyCubeGrid) grid_ref;
            _inverse_world_transform = _grid.PositionComp.WorldMatrixNormalizedInv;
        }

        #endregion

        #region Gyroscope handling

        private void calc_spherical_moment_of_inertia()
        {
            Vector3I grid_dim = _grid.Max - _grid.Min + Vector3I.One;
            int low_dim = grid_dim.X, med_dim = grid_dim.Y, high_dim = grid_dim.Z, temp;

            if (low_dim < 0)
                low_dim = -low_dim;
            if (med_dim < 0)
                med_dim = -med_dim;
            if (high_dim < 0)
                high_dim = -high_dim;
            do
            {
                temp = -1;
                if (low_dim > med_dim)
                {
                    temp = low_dim;
                    low_dim = med_dim;
                    med_dim = temp;
                }
                if (med_dim > high_dim)
                {
                    temp = med_dim;
                    med_dim = high_dim;
                    high_dim = temp;
                }
            }
            while (temp >= 0);
            float smallest_area = low_dim * med_dim * _grid.GridSize * _grid.GridSize;
            float reference_radius = (float)Math.Sqrt(smallest_area / Math.PI);
            _spherical_moment_of_inertia = 0.4f * ((_grid.Physics.Mass >= 1.0f) ? _grid.Physics.Mass : 1.0f) * reference_radius * reference_radius;
            //log_ECU_action("calc_spherical_moment_of_inertia", string.Format("smallest area = {0} m2, radius = {1} m, SMoI = {2} t*m2", smallest_area, reference_radius, _spherical_moment_of_inertia / 1000.0f));
        }

        private void refresh_gyro_info()
        {
            uint num_overriden_gyroscopes = 0;

            _gyro_override   = Vector3.Zero;
            _max_gyro_torque = 0.0f;
            foreach (var cur_gyroscope in _gyroscopes)
            {
                if (cur_gyroscope.IsWorking)
                {
                    _max_gyro_torque += cur_gyroscope.MaxGyroForce;
                    if (cur_gyroscope.GyroOverride)
                    {
                        _gyro_override += cur_gyroscope.GyroOverrideVelocityGrid;
                        ++num_overriden_gyroscopes;
                    }
                }
            }

            if (autopilot_on)
            {
                _gyro_override           = Vector3.Zero;
                _is_gyro_override_active = true;
            }
            else if (num_overriden_gyroscopes > 0)
            {
                _gyro_override          /= num_overriden_gyroscopes;
                _is_gyro_override_active = true;
            }
            else if (_is_gyro_override_active)
            {
                reset_user_input(reset_gyros_only: true);
                _is_gyro_override_active = false;
            }
        }

        public void assign_gyroscope(IMyGyro new_gyroscope)
        {
            _gyroscopes.Add((MyGyro) new_gyroscope);
        }

        public void dispose_gyroscope(IMyGyro gyroscope_to_remove)
        {
            _gyroscopes.Remove((MyGyro) gyroscope_to_remove);
        }

        #endregion

        #region Flight controls handling

        public bool is_under_control_of(VRage.Game.ModAPI.Interfaces.IMyControllableEntity current_controller)
        {
            var    controller = current_controller as MyShipController;
            return controller != null && controller.CubeGrid == _grid;
        }

        public void check_autopilot(IMyRemoteControl RC_block)
        {
            autopilot_on |= RC_block.IsAutoPilotEnabled;
        }

        public void reset_user_input(bool reset_gyros_only)
        {
            _manual_rotation       = _target_rotation = Vector3.Zero;
            _under_player_control &= reset_gyros_only;
        }

        public void translate_linear_input(Vector3 input_thrust, VRage.Game.ModAPI.Interfaces.IMyControllableEntity current_controller)
        {
            var controller = current_controller as MyShipController;
            if (controller?.CubeGrid != _grid)
            {
                reset_user_input(reset_gyros_only: false);
                return;
            }

            Matrix cockpit_matrix;
            controller.Orientation.GetMatrix(out cockpit_matrix);
            _manual_thrust        = Vector3.Clamp(Vector3.Transform(input_thrust, cockpit_matrix), -Vector3.One, Vector3.One);
            _under_player_control = true;

            _match_velocity_with = ((Sandbox.Game.Entities.IMyControllableEntity) current_controller).RelativeDampeningEntity;
        }

        public void translate_rotation_input(Vector3 input_rotation, VRage.Game.ModAPI.Interfaces.IMyControllableEntity current_controller)
        {
            var controller = current_controller as MyShipController;
            if (controller == null || controller.CubeGrid != _grid)
            {
                reset_user_input(reset_gyros_only: false);
                return;
            }

            Matrix cockpit_matrix;
            controller.Orientation.GetMatrix(out cockpit_matrix);
            _target_rotation.X = input_rotation.X * (-0.05f);
            _target_rotation.Y = input_rotation.Y * (-0.05f);
            _target_rotation.Z = input_rotation.Z * (-0.2f);
            _target_rotation   = Vector3.Transform(_target_rotation, cockpit_matrix);
            _under_player_control = true;
        }

        #endregion

        public void handle_60Hz()
        {
            //screen_text("", string.Format("Manager = {0}, exceptions = {1}, complete = {2}", _thrust_manager_task.valid, (_thrust_manager_task.Exceptions == null) ? 0 : _thrust_manager_task.Exceptions.GetLength(0), _thrust_manager_task.IsComplete), 16, controlled_only: false);
            if (_grid.Physics == null || _grid.Physics.IsStatic)
            {
                _physics_enable_delay = PHYSICS_ENABLE_DELAY;
                return;
            }

            // Suppress input noise caused by analog controls
            _sample_sum += _target_rotation - _rotation_samples[_current_index];
            _rotation_samples[_current_index] = _target_rotation;
            if (++_current_index >= NUM_ROTATION_SAMPLES)
                _current_index = 0;
            _manual_rotation = _sample_sum / NUM_ROTATION_SAMPLES;

            _inverse_world_transform = _grid.PositionComp.WorldMatrixNormalizedInv;
            handle_thrust_control();
            calculate_and_apply_torque();
        }

        public void handle_4Hz()
        {
            if (_grid.Physics == null || _grid.Physics.IsStatic)
                return;

            calc_spherical_moment_of_inertia();
            refresh_gyro_info();
            var  current_grid_CoM = Vector3D.Transform(_grid.Physics.CenterOfMassWorld, _inverse_world_transform);
            bool CoM_shifted      = (current_grid_CoM - _grid_CoM_location).LengthSquared() > 0.01f;
            if (CoM_shifted)
            {
                _grid_CoM_location = current_grid_CoM;
                refresh_thruster_info();
                update_reference_vectors();
                refresh_control_sets();
                //log_ECU_action("handle_4Hz", "CoM refreshed");
            }
            refresh_real_max_forces();
        }

        public void handle_2s_period()
        {
            if (_grid.Physics == null || _grid.Physics.IsStatic)
                return;
            check_thruster_control_changed();
            _force_override_refresh = true;
        }
    }
}
