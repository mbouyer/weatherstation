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

module cubeandcyl(l = 6, h = 11) {
    translate([0, 91/2, h / 2]) cube([103 * 2, 91, h], center = true);
    translate([0, 0, l/2]) hull() {
 	translate([0, -l/2, 0]) cube([132, l, l], center = true);
	translate([0, -20 + l/2, 0]) rotate([0, 90, 0]) cylinder(d = l, h = 132, center = true);
    }
}

module base(h = 20) {
    intersection() {
	linear_extrude(height = h)
	    import(file="display.dxf", layer="extrude");
        translate([0, -49.9, 0]) cubeandcyl(l = 6, h = h);
    }
}

module base_m(h = 15) {
	minkowski() {
		translate([0,0,2]) base(h - 4);
		sphere(r = 2, $fa = fa_small, $fs = fs_small);
		//cube([4,4,4], center=true);

	}
}


mirror([1,0,0]) {
    difference() {
    	base_m();
    	translate([0,0,3]) linear_extrude(height = 20)
    	    import(file="display.dxf", layer="hidden");
    	translate([80.05330086, 5.21254627, 0]) buttons();
    	translate([0, -71.9 + 5, 5]) rotate([0,90,0]) cylinder(d = 3.2, h = 132, center = true);
    }

    // clips for PCB
    translate([94.05330086, 26.71254627 + 0.25, 0]) rotate([0,0,180]) clip(b = 8, h=1.8);
    translate([93.80330086, -5.78745373 - 0.25, 0]) clip(b = 8, h=1.8);
    translate([66.05330086, 26.71254627 + 0.25, 0]) rotate([0,0,180]) clip(b = 8, h=1.8);
    translate([66.05330086, -11.78745373 - 0.25, 0]) clip(b = 8, h=1.8);

    // clips for LCD
    translate([41.1,  -43.9 - 0.25, 0]) clip(b = 0, h=6.7, e = 2);
    translate([-41.1,  -43.9 - 0.25, 0]) clip(b = 0, h=6.7, e = 2);

    difference() {
        union() {
    	// add plots for LCD
    	translate([-61.1, 35.9, 3 + 5]) cube([6, 6, 10], center = true);
    	translate([61.1, 35.9, 3 + 5]) cube([6, 6, 10], center = true);
    	translate([-61.1, -43.9 + 5, 3 + 3]) cube([6, 6, 6], center = true);
    	translate([61.1, -43.9 + 5, 3 + 3]) cube([6, 6, 6], center = true);
        }
        // the LCD itself
        translate([0, -4, 6.7 / 2 + 3]) cube([122.2 + 0.5, 79.8 + 0.5, 6.7], center = true);
    }
}
