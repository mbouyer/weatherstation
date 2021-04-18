// $fn = 50;
$fa = 5;
$fs = 1;

fa_small = 6;
fs_small = 0.2;

module quart_tore(rayon, conge) {
	rotate_extrude(angle = 84, convexity=10, $fa = fa_small, $fs = fs_small) translate([rayon + conge, 0, 0])
	    difference() {
		translate([-conge/2, conge/2, 0]) square([conge, conge], center=true);
		translate([0, conge, 0]) circle(r=conge, $fa = fa_small, $fs = fs_small);
	    };
}

module quart_rond(l, conge) {
	difference() {
		translate([-conge/2, -conge/2, 0]) cube([conge, conge, l], center=true);
		translate([-conge, -conge, 0]) cylinder(r=conge, h=l, center=true, $fa = fa_small, $fs = fs_small);
	}
}

module enveloppe_tube(l = 100) {
	translate([0,0,2])
		linear_extrude(height = l) import("enveloppe-tube-pluvio.dxf");
};

module cube_arrondi(long, larg, h, r) {
	c_x = long / 2 - r;
	c_y = larg / 2 - r;
	hull() {
		translate([-c_x, -c_y, 0]) cylinder(r = r, h = h, center = true, $fa = fa_small, $fs = fs_small);
		translate([-c_x, c_y, 0]) cylinder(r = r, h = h, center = true, $fa = fa_small, $fs = fs_small);
		translate([c_x, c_y, 0]) cylinder(r = r, h = h, center = true, $fa = fa_small, $fs = fs_small);
		translate([c_x, -c_y, 0]) cylinder(r = r, h = h, center = true, $fa = fa_small, $fs = fs_small);
	}
}

module plaque_pluvio() {
	translate([0,0,2]) difference() {
		linear_extrude(height = 5 - 4) import("plaque-pluvio.dxf");
		translate([101.5 + 2 - 30, (113 / 2 - (35)/2), 0])
			cube_arrondi(long = 40 + 4, larg = 35 + 4, h = 10, r = 4);
		translate([101.5 + 2 - 30, -(113 / 2 - (35)/2), 0])
			cube_arrondi(long = 40 + 4, larg = 35 + 4, h = 10, r = 4);
	};
};

module plaque_boitier() {
	translate([0,0,2])
		linear_extrude(height = 5 - 4) import("plaque-boitier.dxf");
};

module support(l) {
    enveloppe_tube(l);
    translate([0, 0, l - 1]) plaque_pluvio();
    translate([0, 0, 0]) plaque_boitier();
    translate([86.5, -2, l + 1 - 8]) cube([101.5 - 86.5, 4, 8]);
    translate([62, 2, l + 1]) rotate([90, -90, 90]) quart_rond(79, 5);
    translate([62, -2, l + 1]) rotate([-90, 180, -90]) quart_rond(79, 5);

    translate([54.5, 2, 3]) rotate([-90, 0, -90]) quart_rond(64, 5);
    translate([54.5, -2, 3]) rotate([90, 90, 90]) quart_rond(64, 5);

    translate([0,0,l + 1]) rotate([180,0,+6]) quart_tore(rayon = 23.5, conge = 4);
    translate([0,0,l + 1]) rotate([180,0,90-6-6]) quart_tore(rayon = 23.5, conge = 4);
    translate([0,0,3]) rotate([0,0,-6]) quart_tore(rayon = 23.5, conge = 4);
    translate([0,0,3]) rotate([0,0,-90+6+6]) quart_tore(rayon = 23.5, conge = 4);
};

module support_light() {
    l = 100;
    difference() {
	support(l);
        hull() {
	    r = 10;
	    translate([86.5, 0, l + 1 - 8 - r]) rotate([90, 0, 0]) cylinder(r=r, h=20, center = true);
	    translate([86.5, 0, 3 + r]) rotate([90, 0, 0]) cylinder(r=r, h=20, center = true);
        };
    };
	
};

difference() {
	minkowski() {
		support_light();
		sphere(r = 2, $fa = fa_small, $fs = fs_small);
	}
	translate([0,31.5,25]) rotate([0, 90, 0]) cylinder(d = 5.5, h = 8, $fa = fa_small, $fs = fs_small);
	translate([0,-31.5,25]) rotate([0, 90, 0]) cylinder(d = 5.5, h = 8, $fa = fa_small, $fs = fs_small);
	translate([0,31.5,75]) rotate([0, 90, 0]) cylinder(d = 5.5, h = 8, $fa = fa_small, $fs = fs_small);
	translate([0,-31.5,75]) rotate([0, 90, 0]) cylinder(d = 5.5, h = 8, $fa = fa_small, $fs = fs_small);
	translate([101.5 - 30, (136 / 2), 100 - 2]) cylinder(d = 5.5, h = 8, $fa = fa_small, $fs = fs_small);
	translate([101.5 - 30, -(136 / 2), 100 - 2 ]) cylinder(d = 5.5, h = 8, $fa = fa_small, $fs = fs_small);
	translate([86.5 + 2 - (65 / 2) + (52 / 2), (87 / 2), -1 ]) cylinder(d = 4.5, h = 8, $fa = fa_small, $fs = fs_small);
	translate([86.5 + 2 - (65 / 2) - (52 / 2), -(87 / 2), -1 ]) cylinder(d = 4.5, h = 8, $fa = fa_small, $fs = fs_small);
};
