import init, { World } from './js/gassim.js';

const $ = q => document.querySelector(q);
const $el = (t, p) => {
  const el = document.createElement(t);
  if (p) for (const [k,v] of Object.entries(p)) el[k] = v;
  return el;
};
const pint = v => parseInt(v, 10);
const pfloat = v => parseFloat(v, 10);

// Returns coordinates relative to the canvas
function coord(e) {
  const rect = e.target.getBoundingClientRect();
  const y = rect.height - (e.clientY - rect.top);
  const left = e.clientX - rect.left;
  return [left, y];
}

let control = {
  init: (wasm, canvas) => {
    let play = $('#play');
    play.addEventListener('change', (e) => {
      if (play.checked) {
        control.renderLoop();
      }
    });
    Object.assign(control, {
      playEl: play,
      ctx: canvas.getContext('2d'),
      wasm: wasm,
    });

    control.initMouse(canvas);
    control.initKeyboard();
    control.initHeat(canvas);
    control.initParticles(canvas);
    control.initNumberScroll();
    control.initWorld(canvas);
  },

  initWorld: canvas => {
    const TIMESTEP_FACTOR = 2000;
    let form = $('form.tab.setup');
    let resetWorld = _ => {
      let width = pint(form.width.value);
      let height = pint(form.height.value);
      let numPoints = pint(form.n.value);
      let radius = pint(form.radius.value);
      let mass = pfloat(form.density.value) * radius * radius * 3.14159;
      let heat = pint(form.heat.value);
      let gravity = pfloat(form.gravity.value) * 100;
      let vdw = pint(form.vdw.value) * 1000;

      canvas.style.width = `${canvas.width = width}px`;
      canvas.style.height = `${canvas.height = height}px`;

      control.world = World.new(width, height, gravity, numPoints, radius, mass, heat, vdw);
      form.timestep.value = Math.round(control.world.timestep() * TIMESTEP_FACTOR);
    };

    form.addEventListener('submit', e => {
      e.preventDefault();
      resetWorld();
    });
    resetWorld();

    // Ability to update some properties on the fly
    form.timestep.addEventListener('change', e => {
      control.world.set_timestep(pint(e.target.value, 1) / TIMESTEP_FACTOR);
    });
    form.gravity.addEventListener('change', e => {
      control.world.set_gravity(pfloat(e.target.value, -9.8) * 100);
    });

    control.renderLoop();
  },

  // Use the scroll wheel to change the number like a dial
  initNumberScroll: () => {
    document.body.addEventListener('wheel', e => {
      let el = e.target;
      if (!el.matches('input[type=number]')) return;
      if (el.step === 'any') return;
      let numIters = ~~Math.abs(Math.round(e.deltaY));
      if (e.deltaY < 0) {
        for (let i = 0; i< numIters; i++) el.stepUp();
      } else {
        for (let i = 0; i< numIters; i++) el.stepDown();
      }
    });
  },

  initMouse: canvas => {
    window.addEventListener('mouseup', e => control.mousedown = false);
    canvas.addEventListener('mousemove', e => control.mouseCoord = coord(e));
    canvas.addEventListener('mouseenter', e => control.mouseCoord = coord(e));
  },

  // keyboard shortcuts
  kbdShortcuts: {
    f: e => $('#heatTab').checked = true,
    h: e => $('#setupTab').checked = true,
    o: e => $('#addTab').checked = true,
    p: e => {
      let p = $('#play');
      p.checked = !p.checked;
      p.dispatchEvent(new InputEvent('change'));
    },
  },
  initKeyboard: _ => {
    window.addEventListener('keydown', e => {
      (control.kbdShortcuts[e.key] || console.log)(e);
    });
  },

  initHeat: canvas => {
    let heatTab = $('#heatTab');
    canvas.addEventListener('mouseenter', e => {
      if (pfloat($('form.tab.heat').factor.value) < 1) {
        canvas.classList.add('cold');
      } else {
        canvas.classList.remove('cold');
      }
    });
    canvas.addEventListener('mousedown', e => {
      if (heatTab.checked) control.addHeat(e);
    });
  },

  // Adding/remove particles as you drag around the canvas
  initParticles: canvas => {
    canvas.addEventListener('mousedown', (e) => {
      if ($('#addTab').checked) control.addParticles(e);
    });
  },

  // `rate` is in particles/60 frames
  _continuouslyDo: (rate, f, threshold=30, iter=0) => {
    control.mousedown = true;
    let remainder = 1;
    let numParticles = _ => {
      let thistime = (rate / 60) + remainder;
      remainder = thistime % 1;
      return thistime - remainder;
    };

    function doF(e) {
      let n = numParticles(iter++);
      if (n) {
        f(...control.mouseCoord, n, iter);
      }
      if (control.mousedown) requestAnimationFrame(doF);
    }
    doF();
  },

  addParticles: e => {
    let form = $('form.tab.add');
    let radius = pint(form.radius.value);
    let [rate, mass, heat] = [
      pint(form.rate.value),
      pfloat(form.density.value) * radius * radius * 3.14159,
      pint(form.heat.value)];
    let add = form.modifyParticles.value === 'add';

    if (add) {
      control._continuouslyDo(rate, (x, y, numParticles) => {
        control.world.add_particles(x, y, radius, mass, heat, numParticles);
        // Update the world so we can see the new particle
        if (!control.playEl.checked) {
          control.renderLoop(0, control.world.update_positions());
        }
      });
    } else {
      control._continuouslyDo(rate, (x, y) => {
        control.world.remove_particles(x, y, 128);
        // Update the world so we can see the new particle
        if (!control.playEl.checked) {
          control.renderLoop(0, control.world.update_positions());
        }
      });
    }
  },

  addHeat: e => {
    let form = $('form.tab.heat');
    let [factor, radius] = [pfloat(form.factor.value), pint(form.radius.value)];

    control._continuouslyDo(30, (x, y) => control.world.temperature(x, y, radius, factor));
  },

  renderLoop: (time, positionsPtr) => {
    const tuple_len = 3;
    if (!positionsPtr) {
      positionsPtr = control.world.step();
    }
    const numPoints = control.world.num_particles;
    const coords = new Float32Array(
      control.wasm.memory.buffer, positionsPtr, numPoints * tuple_len)

    //control.world.temperature(.99);

    let ctx = control.ctx;
    let [width, height] = [ctx.canvas.width, ctx.canvas.height];
    ctx.clearRect(0, 0, width, height);
    for (let i = 0; i < tuple_len * numPoints; i += tuple_len) {
      ctx.beginPath();
      ctx.ellipse(
        // x, y
        coords[i], height - coords[i + 1],
        // radiusX, radiusY
        coords[i + 2], coords[i + 2],
        // rotation
        0,
        // startAngle, endAngle
        0, 2 * Math.PI);
      ctx.stroke();
    }
    if (control.playEl.checked) {
      requestAnimationFrame(control.renderLoop);
    }
  },
};

async function run() {
  const wasm = await init();

  // Configure canvas
  const canvas = $('canvas');

  control.init(wasm, canvas);
}

run();
