# Ideal gas simulation

A webpage that simulates how gas behaves using bouncing balls. The intention is
for education purposes, to provide a visual aid when explaining things like:

* What heat is
* How heat effects changes in states of matter
* How water reaches an equilibrium state where its exchange between water vapor
  is continuous but unseen
* How adding ice to water causes the ice to melt while cooling down the water
* Similarly, how a spot of heat propogates through a medium

## Building

The simulation is written in `Rust` targeting `WebAssembly` (`wasm`) using
`wasm-pack`. To build, run:

```sh
wasm-pack build -d html/js --target=web
```

Viewing it on a local machine requires a webserver to serve the javascript
module required to load wasm. The `serve.sh` script uses the built-in webserver
in `python3` for this. You can start it by running:

```sh
python3 -m http.server 8080
```

Then you can open the page at
[http://localhost:8080/html/](http://localhost:8080/html/) in your web browser.
