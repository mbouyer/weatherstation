$fn = 50;

module glissiere() {

	l = 125;
	b1 = 8.75;
	b1_e = 25.8 / 2;
	b2 = 12.75;
	b2_e = 34.2 / 2;
	h = 5;
	ep= 5;
	epe= 5 - 2;
	profile_glissiere = [[-b1, -h-ep], [-b1-ep, -h-ep], [-b2-ep, 0], [b2+ep, 0], [b1+ep, -h-ep], [b1, -h-ep], [b2, -ep], [-b2, -ep]];
	profile_glissiere_i = [[-b1, -h-ep], [-b2, -ep], [b2, -ep], [b1, -h-ep]];
	profile_glissiere_e = [[-b1_e, -h-epe], [-b2_e, 0], [b2_e, 0], [b1_e, -h-epe]];

	difference() {
		difference() {
			minkowski() {
			    translate([0,-0,2]) linear_extrude(l+epe+10) polygon(profile_glissiere_e);
			    sphere(r = 2);
			};
			translate([0,0,ep]) linear_extrude(l + 10+5)
			    polygon(profile_glissiere_i);
		};
		translate([0,2,ep + l + 5]) rotate([90, 0, 0]) cylinder(h = ep + 2, d = 4, center=false);
	}
}

module enveloppe_tube(l = 100) {
	translate([0,0,2]) minkowski() {
		linear_extrude(height = l) import("enveloppe-tube-thermo.dxf");
	sphere(r=2);
	};
};

module support() {
	difference() {
		l = 100;
		enveloppe_tube(l);
		translate([28,-40/2,-1]) cube([10,40,l+5]);
	};
	translate([28-0.5,0,0]) rotate([0,0,90]) glissiere();
};

difference() {
	support();
	translate([0,31.5,25]) rotate([0, 90, 0]) cylinder(d = 5.5, h = 8);
	translate([0,-31.5,25]) rotate([0, 90, 0]) cylinder(d = 5.5, h = 8);
	translate([0,31.5,75]) rotate([0, 90, 0]) cylinder(d = 5.5, h = 8);
	translate([0,-31.5,75]) rotate([0, 90, 0]) cylinder(d = 5.5, h = 8);
};
