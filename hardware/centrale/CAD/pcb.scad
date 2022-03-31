$fn = 50;
$fa = 5;
$fs = 1;

fa_small = 6;
fs_small = 0.2;

module base(h = 20) {
	difference() {
		linear_extrude(height = h)
		    import(file="PCB.dxf", layer="extrude");
		translate([0,0,1 + h/2]) cube([130, 82, h], center = true);
		translate([0,0,1 + h/2]) cube([96, 55*2, h], center = true);
	}
}

module base_m(h = 20) {
	minkowski() {
		translate([0,0,2]) base(h - 4);
		sphere(r = 2, $fa = fa_small, $fs = fs_small);
	}
}

module plot(h = 6, d1 = 4, d2 = 6) {
	difference() {
		cylinder(h = h, d = d2);
		translate([0,0,-1]) cylinder(h = h + 2, d = d1);
	}
}


translate([25.115805,-21.45,5]) plot(h = 6);
translate([25.115805,19.41,5]) plot(h = 6);
translate([-25.091195,-21.45,5]) plot(h = 6);
translate([-25.091195,19.41,5]) plot(h = 6);
difference() {
	base_m();
	translate([61, -48, 8]) rotate([0,-90,0]) cylinder(h = 10, d = 5);
	translate([-61, -48, 8]) rotate([0,90,0]) cylinder(h = 10, d = 5);
	translate([25.115805,-21.45,0]) cylinder(h = 5, d = 3);
	translate([25.115805,19.41,0]) cylinder(h = 5, d = 3);
	translate([-25.091195,-21.45,0]) cylinder(h = 5, d = 3);
	translate([-25.091195,19.41,0]) cylinder(h = 5, d = 3);
}

