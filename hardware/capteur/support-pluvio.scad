$fn = 100;

module glissiere() {

	l = 125;
	b1 = 8.5;
	b2 = 12;
	h = 5;
	ep= 5;
	profile_glissiere = [[-b1, -h-ep], [-b1-ep, -h-ep], [-b2-ep, 0], [b2+ep, 0], [b1+ep, -h-ep], [b1, -h-ep], [b2, -ep], [-b2, -ep]];
	profile_glissiere_e = [[-b1-ep, -h-ep], [-b2-ep, 0], [b2+ep, 0], [b1+ep, -h-ep]];

	difference() {
		union() {
			linear_extrude(ep) polygon(profile_glissiere_e);
			translate([0,0,ep]) linear_extrude(l + 10)
			    polygon(profile_glissiere);
		};
		translate([0,0,ep + l + 5]) rotate([90, 0, 0]) cylinder(h = ep, d = 4, center=false);
	}
}

glissiere();
