% scale(1000) import("body_1.stl");

// Sketch PureShapes 10
multmatrix([[0.0, 0.0, 1.0, 13.0], [1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]]) {
thickness = 10.000000;
translate([0, 0, -thickness]) {
  translate([0.000000, 0.000000, 0]) {
    cylinder(r=52.500000,h=thickness);
  }
}
}
