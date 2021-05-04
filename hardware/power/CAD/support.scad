$fa = 5;    
$fs = 1;

fa_small = 6;   
fs_small = 0.2;
// fa_small = 15;   
// fs_small = 0.5;

d_ins = 3.8;

module base() {
	translate([0,0,2]) {
		linear_extrude(height = 7 - 4)
		    import("base.dxf", layer="0");
		translate([-20, 79, 7 - 4]) cube([6, 2, 2], center=true);
		translate([-20, -79, 7 - 4]) cube([6, 2, 2], center=true);
	};
}

module supp_pcb() {
	translate([0, (8 - 4) / 2, 0]) rotate([90, 0, 0]) {
		linear_extrude(height = 8 - 4)
			    import("pcb.dxf", layer="0");
	};
}

module support() {
	minkowski() {
		union() {
			base();
			translate([-7 + 2, -23.5, 7]) supp_pcb();
			translate([-7 + 2, 64.5, 7]) supp_pcb();
		}
		sphere(r = 2, $fa = fa_small, $fs = fs_small);
	};
}

difference() {
	support();
	translate([-7+2+1,-23.5,7 + 6]) rotate([0,-90,0]) cylinder(d = d_ins, h = 11 + 2, $fa = fa_small, $fs = fs_small);
	translate([-7+2+1,-23.5,7 + 88]) rotate([0,-90,0]) cylinder(d = d_ins, h = 11 + 2, $fa = fa_small, $fs = fs_small);
	translate([-7+2+1,64.5,7 + 6]) rotate([0,-90,0]) cylinder(d = d_ins, h = 11 + 2, $fa = fa_small, $fs = fs_small);
	translate([-7+2+1,64.5,7 + 88]) rotate([0,-90,0]) cylinder(d = d_ins, h = 11 + 2, $fa = fa_small, $fs = fs_small);
}
