module case()
{
  union() {
    difference() {
      cube( [ 72, 33, 13 ] );
      translate([1.25,1.25,1]) {
        cube( [ 69.5, 30.5, 13 ] );
      }
      translate([0.75,0.75,6.5]) {
        cube( [ 70.5, 31.5, 13 ] );
      }
      translate([-1,14.5,10]) {
        cube([2,4,2]);
      }
      translate([71,7.25,12]) {
        cube([2,2,0.5]);
        translate([0,16.5,0]) {
          cube([2,2,0.5]);
        }
      }
    }
    translate([54.6+1, 3.4+1, 0.5]) {
      difference() {
        cylinder(6, r=2.5, $fn=200);
        cylinder(7, r=1.5, $fn=200);
      }
      translate([0, 24, 0]) {
        difference() {
          cylinder(6, r=2.5, $fn=200);
          cylinder(7, r=1.5, $fn=200);
        }
      }
    }
  }
}
module cover() {
    union() {
      translate([0.75,0.75,12]) {
        cube([70.5,31.5,1]);
      }
      translate([71,7.30,12.05]) {
        cube([0.75,1.9,0.4]);
        translate([0,16.5,0]) {
          cube([0.75,1.9,0.4]);
        }
      }
    }
}
//case();
cover();
