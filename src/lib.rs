use nalgebra::{Isometry2, Point2, Vector2};
use ncollide2d::shape::{Ball, Cuboid, ShapeHandle, Compound};
use nphysics2d::force_generator::DefaultForceGeneratorSet;
use nphysics2d::joint::DefaultJointConstraintSet;
use nphysics2d::material::{BasicMaterial, MaterialHandle};
use nphysics2d::math::{Inertia, Velocity};
use nphysics2d::object::{
    BodyPartHandle, BodyStatus, ColliderDesc, DefaultBodySet, DefaultColliderSet, Ground,
    RigidBodyDesc,
};
use nphysics2d::world::{DefaultGeometricalWorld, DefaultMechanicalWorld};
use rand::prelude::*;
use std::f32::consts::PI;
use wasm_bindgen::prelude::*;

// When the `wee_alloc` feature is enabled, use `wee_alloc` as the global
// allocator.
#[cfg(feature = "wee_alloc")]
#[global_allocator]
static ALLOC: wee_alloc::WeeAlloc = wee_alloc::WeeAlloc::INIT;

#[wasm_bindgen]
extern "C" {
    fn alert(s: &str);
    #[wasm_bindgen(js_namespace = console)]
    fn log(s: &str);
}

#[wasm_bindgen]
pub struct World {
    mech: DefaultMechanicalWorld<f32>,
    geo: DefaultGeometricalWorld<f32>,
    bodies: DefaultBodySet<f32>,
    colliders: DefaultColliderSet<f32>,
    joint_constraints: DefaultJointConstraintSet<f32>,
    force_generators: DefaultForceGeneratorSet<f32>,
    positions: Vec<(f32, f32)>,
}

#[wasm_bindgen]
impl World {
    pub fn new(num_particles: usize, width: f32, height: f32, radius: f32, heat: f32) -> Self {
        console_error_panic_hook::set_once();
        let mechanical_world = DefaultMechanicalWorld::new(Vector2::new(0.0, -98.1));
        let geometrical_world = DefaultGeometricalWorld::new();

        let mut bodies = DefaultBodySet::new();
        let mut colliders = DefaultColliderSet::new();
        let joint_constraints = DefaultJointConstraintSet::new();
        let force_generators = DefaultForceGeneratorSet::new();

        // Create box boundaries
        const WALL_WIDTH:f32 = 100.;
        let walls = [
            // x, y, width, height
            (0.0f32, 0., 1.2 * width, WALL_WIDTH), // bottom
            (0.0f32, 0., WALL_WIDTH, 1.2 * height), // left
            (width, 0., WALL_WIDTH, 1.2 * height), // right
            (0.0f32, height, width, WALL_WIDTH), // top
        ];
        let ground = Compound::new(walls.into_iter().copied().map(|(x, y, w, h)| {
            let delta = Isometry2::new(Vector2::new(x, y), 0.);
            let shape = ShapeHandle::new(Cuboid::new(Vector2::new(w, h)));
            (delta, shape)
        }).collect());
        let ground_shape = ShapeHandle::new(ground);
        let ground_handle = bodies.insert(Ground::new());
        colliders.insert(
            ColliderDesc::new(ground_shape)
                //.translation(Vector2::y() * height)
                .material(MaterialHandle::new(BasicMaterial::new(1.0, 0.0)))
                .build(BodyPartHandle(ground_handle, 0)),
        );


        let mut rng = rand::thread_rng();
        (0..num_particles).for_each(|_i| {
            let rigid_body = RigidBodyDesc::new()
                // The rigid body translation.
                // Default: zero vector.
                //.translation(Vector2::y() * (i * height / num_particles) as f32
                //    + Vector2::x() * (i * width / num_particles) as f32)
                // The rigid body rotation.
                // Default: no rotation.
                //.rotation(5.0)
                // The rigid body position. Will override `.translation(...)` and `.rotation(...)`.
                // Default: the identity isometry.
                .position(Isometry2::new(
                    Vector2::new(
                        width * rng.gen::<f32>(),
                        height * rng.gen::<f32>(),
                    ),
                    0.,
                ))
                // Whether or not this rigid body is affected by gravity.
                // Default: true
                .gravity_enabled(true)
                // The velocity of this body.
                // Default: zero velocity.
                .velocity(Velocity::linear(
                    heat * rng.gen::<f32>() - heat / 2.,
                    heat * rng.gen::<f32>() - heat / 2.,
                ))
                // The linear damping applied to this rigid body velocity to slow it down automatically.
                // Default: zero (no damping at all).
                //.linear_damping(10.0)
                // The angular damping applied to this rigid body velocity to slow down its rotation automatically.
                // Default: zero (no damping at all).
                //.angular_damping(5.0)
                // The maximum linear velocity this rigid body can reach.
                // Default: f32::max_value() or f64::max_value() (no limit).
                //.max_linear_velocity(10.0)
                // The maximum angular velocity this rigid body can reach.
                // Default: f32::max_value() or f64::max_value() (no limit).
                //.max_angular_velocity(1.7)
                // The angular inertia tensor of this rigid body, expressed on its local-space.
                // Default: the zero matrix.
                //.angular_inertia(3.0)
                // The rigid body mass.
                // Default: 0.0
                .mass(1.2)
                // The threshold for putting this rigid body to sleep.
                // Default: Some(ActivationStatus::default_threshold())
                .sleep_threshold(None)
                // All done, actually build the rigid-body.
                .build();
            let handle = bodies.insert(rigid_body);

            let shape = ShapeHandle::new(Ball::new(10.));
            let collider = ColliderDesc::new(shape)
                // The material of this collider.
                // Default: BasicMaterial::default()
                // with restitution: 0.0, friction: 0.5, combine mode: Average.
                .material(MaterialHandle::new(BasicMaterial::new(1.0, 0.0)))
                .margin(radius / 2.)
                // Whether this collider is a sensor, i.e., generate only proximity events.
                // Default: false
                //.sensor(true)
                // Arbitrary user-defined data associated to the rigid body to be built.
                // Default: no associated data
                //.user_data(10)
                // All done, actually build the collider into the `world`.
                .build(BodyPartHandle(handle, 0));
            colliders.insert(collider);
        });

        World {
            mech: mechanical_world,
            geo: geometrical_world,
            bodies,
            colliders,
            joint_constraints,
            force_generators,
            positions: vec![],
        }
    }
    pub fn step(&mut self) {
        // Run the simulation.
        self.mech.step(
            &mut self.geo,
            &mut self.bodies,
            &mut self.colliders,
            &mut self.joint_constraints,
            &mut self.force_generators,
        );

        self.positions = self
            .bodies
            .iter()
            .filter(|(_, body)| !body.is_ground())
            .map(|(_, body)| {
                let pos = body.part(0).unwrap().position().translation.vector;
                (pos.x, pos.y)
            })
            .collect();
    }
    pub fn position(&self) -> *const (f32, f32) {
        self.positions.as_ptr()
    }
}

#[wasm_bindgen]
pub fn hello() -> Result<(), JsValue> {
    log("Hello, wasm!");
    // Use `web_sys`'s global `window` function to get a handle on the global
    // window object.
    let window = web_sys::window().expect("no global `window` exists");
    let document = window.document().expect("should have a document on window");
    let body = document.body().expect("document should have a body");

    // Manufacture the element we're gonna append
    let val = document.create_element("p")?;
    val.set_inner_html("Hello from Rust!");

    body.append_child(&val)?;
    Ok(())
}
