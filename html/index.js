import init, { World } from '../pkg/wasm.js';

const $ = q => document.querySelector(q);
const $el = (t, p) => {
  const el = document.createElement(t);
  if (p) for (const [k,v] of Object.entries(p)) el[k] = v;
  return el;
};

// Returns coordinates relative to the canvas
function coord(e) {
  const canvas = e.target;
  const rect = canvas.getBoundingClientRect();
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
  },
  _continuouslyDo: f => {
    let iter = 0;
    function doF(e) {
      let r = f(iter++, ...control.mouseCoord);
      iter++;
      if (control.mousedown) {
        requestAnimationFrame(doF);
      }
      return r;
    }
    doF();
  },
  initMouse: (canvas) => {
    canvas.addEventListener('mousedown', (e) => {
      console.log(e);
      e.preventDefault();
      control.mousedown = true;
      if ($('#addTab').checked) {
        let form = $('form.tab.add');
        let rate = parseInt(form.rate.value, 10);
        let radius = parseInt(form.radius.value, 10);
        let mass = parseInt(form.mass.value, 10);
        let heat = parseInt(form.heat.value, 10);

        control._continuouslyDo((iter, x, y) => {
          if (iter % rate === 0) {
            control.world.add_particle(x, y, radius, mass, heat);
            // Update the world so we can see the new particle
            if (!control.playEl.checked) {
              control.renderLoop();
            }
          }
        });

      } else if ($('#heatTab').checked) {
        control.addHeatLoop();
      }
    });
    window.addEventListener('mouseup', (e) => {
      console.log(e);
      e.preventDefault();
      control.mousedown = false;
    });
    canvas.addEventListener('mousemove', (e) => {
      control.mouseCoord = coord(e);
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
