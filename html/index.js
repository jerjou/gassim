import init, { World } from '../pkg/wasm.js';

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
  init: (wasm, canvas, numPoints, width, height, radius) => {
    control.world = World.new(numPoints, width, height, radius, 1, 1000);

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
    control.initHeat(canvas);
    control.initParticles(canvas);
    control.initNumberScroll();
  },

  // Use the scroll wheel to change the number like a dial
  initNumberScroll: () => {
    document.body.addEventListener('wheel', (e) => {
      let el = e.target;
      if (!el.matches('input[type=number]')) return;
      let numIters = ~~Math.abs(Math.round(e.deltaY));
      if (e.deltaY < 0) {
        for (let i = 0; i< numIters; i++) el.stepUp();
      } else {
        for (let i = 0; i< numIters; i++) el.stepDown();
      }
    });
  },
  // `rate` is in particles/60 frames
  _continuouslyDo: (rate, f, threshold=30, iter=0) => {
    control.mousedown = true;
    let remainder = 0;
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
    e.preventDefault();
    let form = $('form.tab.add');
    let [rate, radius, mass, heat] = [
      10 / pint(form.rate.value), pint(form.radius.value), pint(form.mass.value), pint(form.heat.value)];
    let add = form.modifyParticles.value === 'add';

    if (add) {
      control._continuouslyDo(rate, (x, y, numParticles) => {
        control.world.add_particles(x, y, radius, mass, heat, numParticles);
        // Update the world so we can see the new particle
        if (!control.playEl.checked) {
          control.renderLoop();
        }
      });
    } else {
      control._continuouslyDo(rate, (x, y) => {
        control.world.remove_particles(x, y, 128);
        // Update the world so we can see the new particle
        if (!control.playEl.checked) {
          control.renderLoop();
        }
      });
    }
  },
  addHeat: e => {
    e.preventDefault();
    let form = $('form.tab.heat');
    let [factor, radius] = [pfloat(form.factor.value), pint(form.radius.value)];

    control._continuouslyDo(30, (x, y) => control.world.temperature(x, y, radius, factor));
  },
  initMouse: canvas => {
    window.addEventListener('mouseup', e => control.mousedown = false);
    canvas.addEventListener('mousemove', e => control.mouseCoord = coord(e));
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

  renderLoop: () => {
    const coordsPtr = control.world.step();
    const numPoints = control.world.num_particles;
    const coords = new Float32Array(
      control.wasm.memory.buffer, coordsPtr, numPoints * 2)

    //control.world.temperature(.99);

    let ctx = control.ctx;
    let [width, height] = [ctx.canvas.width, ctx.canvas.height];
    ctx.clearRect(0, 0, width, height);
    for (let i = 0; i < 2 * numPoints; i += 2) {
      ctx.beginPath();
      ctx.ellipse(
        // x, y
        coords[i], height - coords[i + 1],
        // radiusX, radiusY
        10, 10,
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

async function run(width, height, numPoints=30, radius=10) {
  const wasm = await init();

  // Configure canvas
  const canvas = $('canvas');
  canvas.style.width = `${canvas.width = width}px`;
  canvas.style.height = `${canvas.height = height}px`;

  control.init(wasm, canvas, numPoints, width, height, radius);
  control.renderLoop();
}

run(800, 600);
