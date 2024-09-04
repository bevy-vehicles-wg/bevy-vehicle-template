use bevy::color::palettes::basic::*;
use bevy::color::palettes::css::*;
use bevy::math::vec3;
use bevy::prelude::*;
use bevy_infinite_grid::{InfiniteGridBundle, InfiniteGridPlugin};
use bevy_panorbit_camera::{PanOrbitCamera, PanOrbitCameraPlugin};
use bevy_rapier3d::prelude::*;

fn main() {
    App::new()
        .insert_resource(ClearColor(Color::srgb(0.1, 0.1, 0.2)))
        .add_plugins((
            DefaultPlugins,
            PanOrbitCameraPlugin,
            RapierPhysicsPlugin::<NoUserData>::default(),
            RapierDebugRenderPlugin::default(),
        ))
        .add_plugins(InfiniteGridPlugin)
        .add_systems(Startup, (setup_graphics, setup_environment, setup_physics))
        .add_systems(Update, car_controller)
        .run();
}

fn setup_graphics(mut commands: Commands) {
    // either I'm crazy or moving the camera perturbs the rigid bodies
    // set the chassis to fixed so the car is floating in the air,
    // then swivel the view around and the wheels should start bouncing
    commands.spawn((
        Camera3dBundle {
            transform: Transform::from_xyz(0.0, 10.0, 20.0)
                .looking_at(Vec3::new(0.0, 0.0, 0.0), Vec3::Y),
            ..Default::default()
        },
        PanOrbitCamera::default(),
    ));
}

fn setup_environment(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    commands.insert_resource(AmbientLight {
        color: Color::WHITE,
        brightness: 1000.0,
    });

    commands.spawn(InfiniteGridBundle::default());

    // add other obstacles here later
    let ground_size = 500.0;
    let ground_height = 0.1;

    commands.spawn((
        PbrBundle {
            mesh: meshes.add(Cuboid::new(ground_size, ground_height, ground_size)),
            material: materials.add(Color::Srgba(GRAY)),
            transform: Transform::from_xyz(0.0, -ground_height, 0.0),
            ..default()
        },
        Collider::cuboid(ground_size, ground_height, ground_size),
        Friction::new(2.0),
    ));

    let obstacle_size = 60.0;
    let obstacle = Cuboid::new(obstacle_size, obstacle_size, obstacle_size);
    let obstacle_collider = Collider::cuboid(
        obstacle_size / 2.0,
        obstacle_size / 2.0,
        obstacle_size / 2.0,
    );
    commands.spawn((
        PbrBundle {
            mesh: meshes.add(obstacle),
            material: materials.add(Color::Srgba(YELLOW)),
            transform: Transform {
                translation: vec3(-20.0, -20.0, 40.0),
                rotation: Quat::from_rotation_y(-60f32.to_radians())
                    * Quat::from_rotation_x(60f32.to_radians()),
                ..default()
            },
            ..default()
        },
        Friction::new(2.0),
        obstacle_collider,
    ));


    let obstacle_size = 60.0;
    let obstacle = Cuboid::new(obstacle_size, obstacle_size, obstacle_size);
    let obstacle_collider = Collider::cuboid(
        obstacle_size / 2.0,
        obstacle_size / 2.0,
        obstacle_size / 2.0,
    );
    commands.spawn((
        PbrBundle {
            mesh: meshes.add(obstacle),
            material: materials.add(Color::Srgba(ORANGE)),
            transform: Transform {
                translation: vec3(10.0, -20.0, -40.0),
                rotation: Quat::from_rotation_y(-30f32.to_radians())
                    * Quat::from_rotation_x(30f32.to_radians()),
                ..default()
            },
            ..default()
        },
        Friction::new(2.0),
        obstacle_collider,
    ));
}

#[derive(Component)]
pub struct Wheel {
    is_driving: bool,
    is_left: bool,
}

#[derive(Component)]
pub enum Axle {
    Steered,
    Fixed,
}

fn setup_physics(mut commands: Commands) {
    let car_scale = 4.0;
    let start_height = 1.0 * car_scale;

    // side to side
    let chassis_width = 0.5 * car_scale;
    // front to back
    let chassis_length = 1.0 * car_scale;
    // top to bottom
    let chassis_height = 0.25 * car_scale;

    println!("car width: {}", chassis_width);
    println!("car height: {}", chassis_height);
    println!("car length: {}", chassis_length);

    // range of motion, a very stiff spring will hold the
    // suspension in the middle of this range
    let suspension_height = 0.3 * car_scale;

    // wheel dimensions
    let wheel_radius = 0.2 * car_scale;
    let wheel_width = 0.1 * car_scale;

    println!("wheel radius: {}", wheel_radius);
    println!("wheel width: {}", wheel_width);

    // collision group to prevent collision between car parts
    let car_group = Group::GROUP_1;

    // simple cuboid to represent the chassis
    let chassis_collider = (
        Collider::cuboid(
            chassis_width / 2.0,
            chassis_height / 2.0,
            chassis_length / 2.0,
        ),
        ColliderMassProperties::Mass(2000.0),
        CollisionGroups::new(car_group, !car_group),
    );

    // bevy entity with rigid body marker and chassis collider
    // set to spawn at start height
    let chassis_id = commands
        .spawn((
            TransformBundle::from(Transform::from_xyz(0.0, start_height, 0.0)),
            RigidBody::Dynamic,
            chassis_collider,
            Sleeping::disabled(),
        ))
        .id();

    // wheel positions are defined relative to the chassis size
    // this was important before using collision groups,
    // now its just for the aesthetic
    let x = 0.6 * (chassis_width + wheel_width);
    let y = -chassis_height;
    let z = 0.45 * chassis_length;
    let wheel_positions = vec![(-x, y, z),(x, y, z),  (-x, y, -z), (x, y, -z)];

    let front = true;
    let rear = !front;
    let left = true;
    let right = !left;

    let wheel_label = vec![(left, rear), (right, rear),  (left, front), (right, front)];

    for (i, (x, y, z)) in wheel_positions.into_iter().enumerate() {
        let (is_left, is_front) = wheel_label[i];
        let is_rear = !is_front;
        let is_right = !is_left;

        //let is_driving = is_rear | is_front;
        let is_driving = is_rear;
        // anchors set the joint relative to the collider, this sets the axle
        // joint to be level with the chassis and directly above the axle
        let axle_anchor1 = Vec3::new(x / 2.0, 0.0, z);
        let axle_anchor2 = Vec3::new(0.0, -y, 0.0);

        // sets the locked axes of the axle joint, both are free
        // in linear Y (suspension) and the front is free in angular Y (steer)
        let locked_axle_axes = if is_front {
            JointAxesMask::all() ^ JointAxesMask::ANG_Y ^ JointAxesMask::LIN_Y
        }else{
            JointAxesMask::all() ^ JointAxesMask::LIN_Y
        };

        // set the axle marker indicating if an axle is fixed or free to steer,
        // not really used anymore, could just be a steered axle marker
        let axle_marker = match is_front {
            true => Axle::Steered,
            false => Axle::Fixed,
        };

        // I think a spring joint would work here, but it's pretty simple to define everything
        // as generic. Also where's the into()?
        let axle_joint = GenericJointBuilder::new(locked_axle_axes)
            .local_anchor1(axle_anchor1)
            .local_anchor2(axle_anchor2)
            .limits(
                JointAxis::LinY,
                [-suspension_height / 2.0, suspension_height / 2.0],
            )
            .set_motor(JointAxis::LinY, 0.0, 0.0, 500.0, 500.0)
            .build();

        // hacky jump feature, should really have a second collider fixed
        // to the chassis to actually model the suspension expansion.
        // but this is simpler and fun
        let axle_impulse = ExternalImpulse::at_point(
            Vec3::ZERO,
            Vec3::new(x / 2.0, y + start_height, z),
            Vec3::new(x / 2.0, y + start_height, z),
        );

        // collision for the axle, because it won't move without one
        let axle_collider = (
            // making the ball any smaller makes the joints to weak...
            Collider::ball(y.abs() / 2.0),
            ColliderMassProperties::Mass(50.0),
            CollisionGroups::new(car_group, !car_group),
        );

        // bevy entity for an axle
        let axle_id = commands
            .spawn((
                TransformBundle::from(Transform::from_xyz(x / 2.0, y + start_height, z)),
                RigidBody::Dynamic,
                axle_collider,
                axle_marker,
                axle_impulse,
                Sleeping::disabled(),
                ImpulseJoint::new(chassis_id, TypedJoint::GenericJoint(axle_joint)),
            ))
            .id();

        // Would be nice if RevouteJointBuilder took two axes, instead the collisions
        // it connects need to have the same orientation.
        // Generic joints are fine for now, also more flexible
        let wheel_joint = GenericJointBuilder::new(JointAxesMask::LOCKED_REVOLUTE_AXES)
            .local_anchor1(Vec3::new(x / 2.0, 0.0, 0.0))
            .local_axis1(Vec3::X)
            .local_axis2(Vec3::Y)
            .set_motor(JointAxis::AngX, 0.0, 0.0, 0.0, 1.0)
            .build();

        // cylinder wheel collider, needs to be rotated to face the right way,
        // that means it can't use a revolute joint
        let wheel_collider = (
            Collider::cylinder(wheel_width / 2.0, wheel_radius),
            Friction::coefficient(1.0), // make sure it doesn't slide around
            Restitution::coefficient(0.5), // wheels should bounce right?
            ColliderMassProperties::Mass(20.0),
            CollisionGroups::new(car_group, !car_group),
        );

        // bevy wheel entity with 90 degree rotation
        commands.spawn((
            TransformBundle::from(Transform {
                translation: Vec3::new(x, y + start_height, z),
                rotation: Quat::from_rotation_z(-std::f32::consts::PI / 2.0),
                scale: Vec3::ONE,
            }),
            RigidBody::Dynamic,
            wheel_collider,
            Wheel {
                is_driving,
                is_left,
            },
            Sleeping::disabled(),
            ImpulseJoint::new(axle_id, TypedJoint::GenericJoint(wheel_joint)),
        ));
    }
}

fn car_controller(
    _time: Res<Time>,
    keys: Res<ButtonInput<KeyCode>>,
    mut wheel_query: Query<(&mut ImpulseJoint, &Wheel)>,
    mut axle_query: Query<
        (&mut ImpulseJoint, &mut ExternalImpulse, &Transform, &Axle),
        Without<Wheel>,
    >,
) {
    let mut speed = 0.0;
    let mut steer = 0.0;
    let mut jump = 0.0;

    if keys.pressed(KeyCode::KeyW) {
        speed = -30.0;
    }
    if keys.pressed(KeyCode::KeyS) {
        speed = 30.0;
    }
    if keys.pressed(KeyCode::KeyA) {
        steer = std::f32::consts::PI / 6.0;
    }
    if keys.pressed(KeyCode::KeyD) {
        steer = -std::f32::consts::PI / 6.0;
    }
    if keys.just_pressed(KeyCode::Space) {
        jump = 400.0;
    }


    let differential_strength = 0.5;
    let sideways_shift = steer.sin() * differential_strength;
    let speed_diff = if sideways_shift > 0.0 {
        f32::hypot(1.0, sideways_shift)
    } else {
        1.0 / f32::hypot(1.0, sideways_shift)
    };

    let ms = [1.0 / speed_diff, speed_diff];

    for (mut impulse_joint, wheel) in wheel_query.iter_mut() {
        if wheel.is_driving {
            let ms = if wheel.is_left{
                ms[0]
            }else{
                ms[1]
            };
            impulse_joint
                .data
                .as_mut()
                .set_motor_velocity(JointAxis::AngX, speed * ms, 50.0);
        }
    }

    for (mut joint, mut impulse, tf, axle) in axle_query.iter_mut() {
        match axle {
            Axle::Steered => {
                joint
                    .data
                    .as_mut()
                    .set_motor_position(JointAxis::AngY, steer, 10000.0, 1000.0);
            }
            _ => {}
        }
        // jump is just an impulse on each of the axle colliders.
        // it should be an impulse that expands the suspension to it's limit
        // then the car jumps because of the momentum of the chassis.
        impulse.impulse = tf.rotation * Vec3::Y * jump;
        impulse.torque_impulse = tf.rotation * Vec3::X * -jump / 60.0;
    }
}
