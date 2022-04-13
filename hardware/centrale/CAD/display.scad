$fn = 100;
$fa = 5;
$fs = 1;

fa_small = 6;
fs_small = 0.2;


module clipslock_base(l, h2) {
    translate([l,0,0]) rotate([0,-90,0])
	    linear_extrude(height=l) polygon(points = [
		[0,0],
		[0, 2],
		[h2, 3],
		[h2, 0],
		]);
}

module clipslock(el = 4, eh= 6, l = 20, h1 = 5, h2 = 10)
{
	difference() {
	    union() {
	    	cube([l, el, h1]);
	    	translate([0,0,h1]) cube([l, eh, h2 - h1 - 1]);
	    }
	    clipslock_base(l = l, h2 = h2);
	}
}

module plot(h = 6, d1 = 4, d2 = 6) {
	difference() {
		cylinder(h = h, d = d2);
		translate([0,0,-1]) cylinder(h = h + 2, d = d1);
	}
}

module support_lcd(l = 6, h = 10) {
    translate([-1,-1,0]) cube([1, l+1, h]);
    translate([-1,-1,0]) cube([l+1, 1, h]);
    linear_extrude(height=h) polygon(points = [
	[0,0],
	[0, l],
	[l, 0],
	]);
}

module clip_top(l, e) {
	translate([l, 0, 0]) rotate([0, -90, 0])
	    linear_extrude(height=l) polygon(points = [
	        [0, 0],
	        [0, e+1.5],
	        [0.5, e+1.5],
	        [3, e],
	        [3, 0]
	    ]);
}

module clip(b=0, h=3, l=4, e=3) {
	translate([-l/2, -e, 0]) {
	    translate([0, e+0.5, 0]) cube([l, 3, b]);
	    cube([l, e, h+b]);
	    translate([0, 0, b+h]) clip_top(l=l, e=e);
	}
}

module buttons(d = 9.5) {
	translate([0, 10.60660172, 0]) cylinder(d = d, h = 5);
	translate([0, -10.60660172, 0]) cylinder(d = d, h = 5);
	translate([-10.60660172, 0, 0]) cylinder(d = d, h = 5);
	translate([10.60660172, 0, 0]) cylinder(d = d, h = 5);
}

module buttons_support(h = 8) {
    plot(h = h, d1 = 4, d2 = 7);
    translate([-16.5,18,0]) cube([6,6,h]);
    translate([10.5,18,0]) cube([6,6,h]);
    translate([10.5,-13.5,0]) cube([6,6,h]);
    translate([-12.5,-19.5,0]) cube([6,6,h]);
}

module cubeandcyl(l = 4.5, h = 11) {
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
        translate([0, -49.9, 0]) cubeandcyl(l = 4.5, h = h);
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
    	translate([0, -71.9 + 4.5, 4.5]) rotate([0,90,0]) cylinder(d = 3.2, h = 132, center = true);
    }

    translate([80.05330086, 5.21254627, 3]) buttons_support(h = 8);

    // clips for LCD
    translate([61.1 - 20 - 5, -48.9, 3]) clipslock_base(l = 20, h2 = 10);
    translate([-61.1 + 5,  -48.9, 3]) clipslock_base(l = 20, h2 = 10);
    // translate([61.1 - 20 - 5, -48.9 + 1, 3]) clipslock(el = 5, eh = 8, l = 20, h1 = 6.7, h2 = 10);
    // translate([-61.1 + 5,  -48.9 + 1, 3]) clipslock(el = 5, eh = 8, l = 20, h1 = 6.7, h2 = 10);

    difference() {
        union() {
    	// add plots for LCD
    	translate([-63, 38, 3]) rotate([0,0,-90]) support_lcd(l = 6, h = 10);
    	translate([63, 38, 3]) rotate([0,0,-180]) support_lcd(l = 6, h = 10);
    	translate([-61.1, -43.9 + 5, 3 + 3]) cube([6, 6, 6], center = true);
    	translate([61.1, -43.9 + 5, 3 + 3]) cube([6, 6, 6], center = true);
        }
        // the LCD itself
        translate([0, -4, 6.7 / 2 + 3]) cube([122.2 + 0.5, 79.8 + 0.5, 6.7], center = true);
    }
}
