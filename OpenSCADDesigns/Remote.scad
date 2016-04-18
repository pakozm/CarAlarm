use <pcb_details.scad>
use <pcb_size.scad>

// pcb_details(4);
 //translate([0,0,2]) pcb_size(1);

union() {
    translate([-18,-18,0]) {
        difference() {
            translate([0,0,0]) cube([36,36,6]);
            translate([2,2,1]) cube([32,32,7]);
            
            //translate([-1,12-1,2]) cube([4,2,1]);
            //translate([-1,24-1,2]) cube([4,2,1]);
            translate([-1,24-1,1.5]) cube([4,2,10]);
        }
        difference() {
            translate([2,2,0]) cube([32,32,2]);
            translate([3,3,1]) cube([30,30,3]);
            translate([-1,24-1,1.5]) cube([5,2,10]);
        }
        translate([33-2.9,33-3.7,0]) {
            difference() {
                cylinder(2, r=2, $fn=200);
                translate([0,0,-1]) cylinder(10, r=1, $fn=200);
            }
        }
        // translate([-15+4.5,-15+5.5,0]) cylinder(40, r=3, $fn=200);
    }
}

rotate([0,180,0]) {
    union() {
        translate([-18,40,-17]) {
            difference() {
                translate([0,0,4]) cube([36,36,13]);
                translate([3,3,3]) cube([30,30,13]);
                
                translate([3,13.2,0]) cube([10,0.5,100]);
                translate([3,3.2,0]) cube([0.5,10,100]);
                translate([3,3.2,0]) cube([10,0.5,100]);
        
                translate([3.3,3.4,13.5]) cube([9.7,10,3]);
                
            }
            difference() {
                translate([2.15,2.15,0]) cube([31.7,31.7,5]);
                translate([3,3,-1]) cube([30,30,7]);
            }
            
            //translate([0,12-1,0]) cube([3,2,1]);
            //translate([0,24-1,0]) cube([3,2,1]);
        
            
            translate([3+4.5,3+5.5,12]) cylinder(4.8, r=3, $fn=200);
            
            translate([33-2.9,33-3.7,-1]) cylinder(16, r=0.7, $fn=200);
        }
    }
}