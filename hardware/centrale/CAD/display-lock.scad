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
		translate([-1, 2, 0]) cube([l + 3, el - 2, h1]);
	    	translate([-1, 2, h1]) cube([l + 3, eh - 2, h2 - h1 - 1]);
	    }
	    clipslock_base(l = l + 1, h2 = h2);
	}
}

rotate([180,0,0]) clipslock(el = 5, eh = 8, l = 20, h1 = 6.7, h2 = 10);
