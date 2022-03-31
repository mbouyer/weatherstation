//$fn = 50;
$fa = 5;
$fs = 1;

fa_small = 6;
fs_small = 0.2;

module clip(b=0, h=3, l=4, e=3) {
	translate([-l/2, -e, 0]) {
	    translate([0, e+0.5, 0]) cube([l, 3, b]);
	    cube([l, e, h+b]);
	    translate([0, 0, b+h]) cube([l, e+1.5, 3]);
	}
}

module buttons(d = 9.5) {
	translate([0, 10.60660172, 0]) cylinder(d = d, h = 5);
	translate([0, -10.60660172, 0]) cylinder(d = d, h = 5);
	translate([-10.60660172, 0, 0]) cylinder(d = d, h = 5);
	translate([10.60660172, 0, 0]) cylinder(d = d, h = 5);
}

module base(h = 20) {
	linear_extrude(height = h)
	    import(file="display.dxf", layer="extrude");
}

module base_m(h = 20) {
	minkowski() {
		translate([0,0,2]) base(h - 4);
		//sphere(r = 2, $fa = fa_small, $fs = fs_small);
		cube([4,4,4], center=true);

	}
}


difference() {
	base_m();
	translate([0,0,3]) linear_extrude(height = 20)
	    import(file="display.dxf", layer="hidden");
	translate([80.05330086, 5.21254627, 0]) buttons();
}

clip(b = 4);

