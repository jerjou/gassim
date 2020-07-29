use nalgebra::{Isometry2, Vector2};
use ncollide2d::shape::{Ball, Compound, Cuboid, ShapeHandle};
use nphysics2d::force_generator::DefaultForceGeneratorSet;
use nphysics2d::joint::DefaultJointConstraintSet;
use nphysics2d::material::{BasicMaterial, MaterialHandle};
use nphysics2d::math::Velocity;
use nphysics2d::object::{
    BodyPartHandle, ColliderDesc, DefaultBodySet, DefaultColliderSet, Ground,
    RigidBodyDesc,
};
use nphysics2d::world::{DefaultGeometricalWorld, DefaultMechanicalWorld};
use rand;
use rand::prelude::*;
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

macro_rules! log {
    ( $( $x:expr ),* ) => {
        log(&format!($($x,)*));
    };
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
    width: f32,
    height: f32,
    pub num_particles: usize,
    rng: rand::rngs::ThreadRng,
}

#[wasm_bindgen]
impl World {
    pub fn new(
        num_particles: usize,
        width: f32,
        height: f32,
        radius: f32,
        mass: f32,
        heat: f32,
    ) -> Self {
        console_error_panic_hook::set_once();
        let mechanical_world = DefaultMechanicalWorld::new(Vector2::new(0.0, -98.1));
        let geometrical_world = DefaultGeometricalWorld::new();

        let mut bodies = DefaultBodySet::new();
        let mut colliders = DefaultColliderSet::new();
        let joint_constraints = DefaultJointConstraintSet::new();
        let force_generators = DefaultForceGeneratorSet::new();

        // Create box boundaries
        const WALL_WIDTH: f32 = 100.; // Thick wall width to ensure it's impenetrable
        let walls = [
            // x, y, width, height
            (0.0f32, -WALL_WIDTH, 1.2 * width, WALL_WIDTH), // bottom
            (-WALL_WIDTH, 0., WALL_WIDTH, 1.2 * height),    // left
            (width + WALL_WIDTH, 0., WALL_WIDTH, 1.2 * height), // right
            (0.0f32, height + WALL_WIDTH, width, WALL_WIDTH), // top
        ];
        let ground = Compound::new(
            walls
                .iter()
                .copied()
                .map(|(x, y, w, h)| {
                    let delta = Isometry2::new(Vector2::new(x, y), 0.);
                    let shape = ShapeHandle::new(Cuboid::new(Vector2::new(w, h)));
                    (delta, shape)
                })
                .collect(),
        );
        let ground_shape = ShapeHandle::new(ground);
        let ground_handle = bodies.insert(Ground::new());
        colliders.insert(
            ColliderDesc::new(ground_shape)
                //.translation(Vector2::y() * height)
                .material(MaterialHandle::new(BasicMaterial::new(1.0, 0.0)))
                .build(BodyPartHandle(ground_handle, 0)),
        );

        // Seed it with particles
        let rng = rand::thread_rng();
        let mut world = World {
            mech: mechanical_world,
            geo: geometrical_world,
            bodies,
            colliders,
            joint_constraints,
            force_generators,
            width,
            height,
            rng,
            num_particles: 0,
            positions: vec![],
        };
        let mut rng = rand::thread_rng();
        (0..num_particles).for_each(|_i| {
            world.add_particles(
                width * rng.gen::<f32>(),
                height * rng.gen::<f32>(),
                radius,
                mass,
                heat,
                1,
            );
        });
        world
    }

    pub fn add_particles(&mut self, x: f32, y: f32, radius: f32, mass: f32, heat: f32, n: usize) {
        log!("Adding {} particle(s) at {}, {}", n, x, y);
        (0..n).for_each(|_| {
            let rigid_body = RigidBodyDesc::new()
                .position(Isometry2::new(Vector2::new(x, y), 0.))
                .gravity_enabled(true)
                .velocity(Velocity::linear(
                        heat * self.rng.gen::<f32>() - heat / 2.,
                        heat * self.rng.gen::<f32>() - heat / 2.,
                ))
                .mass(mass)
                .sleep_threshold(None)
                .build();
            let handle = self.bodies.insert(rigid_body);

            self.colliders.insert(
                ColliderDesc::new(ShapeHandle::new(Ball::new(radius)))
                // restitution, friction: 0.5
                .material(MaterialHandle::new(BasicMaterial::new(1.0, 0.0)))
                .margin(0.5)
                .build(BodyPartHandle(handle, 0)),
            );
        });

        self.num_particles += n;
    }

    pub fn remove_particles(&mut self, x: f32, y: f32, radius: f32) {
        log!("Removing particle from {}, {}", x, y);
        let r2 = radius.powi(2);
        let bodies: Vec<_> = self.colliders
            .iter()
            .map(|(col_h, collider)| {
                (
                    col_h,
                    collider.body(),
                    self.bodies.get(collider.body()).unwrap(),
                )
            })
            .filter(|(_, _, body)| !body.is_ground())
            .filter(|(_, _, body)| {
                let pos = body.part(0).unwrap().position().translation.vector;
                let (dx, dy) = ((pos.x - x).abs(), (pos.y - y).abs());
                dx < radius && dy < radius && dx.powi(2) + dy.powi(2) < r2
            })
            .map(|(col_h, bod_h, _)| (col_h, bod_h))
            .collect();
        bodies.iter().for_each(|(col_h, bod_h)| {
            self.bodies.remove(*bod_h);
            self.colliders.remove(*col_h);
        });
    }

    pub fn step(&mut self) -> *const (f32, f32) {
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
        self.positions.as_ptr()
    }

    pub fn temperature(&mut self, x: f32, y: f32, radius: f32, factor: f32) {
        let r2 = radius.powi(2);
        self.bodies
            .iter_mut()
            .filter(|(_, body)| !body.is_ground())
            .filter(|(_, body)| {
                let pos = body.part(0).unwrap().position().translation.vector;
                let (dx, dy) = ((pos.x - x).abs(), (pos.y - y).abs());
                dx < radius && dy < radius && dx.powi(2) + dy.powi(2) < r2
            })
            .for_each(|(_, body)| {
                body.generalized_velocity_mut()
                    .row_iter_mut()
                    .for_each(|mut v| {
                        v.x = v.x * factor;
                    });
            });
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
