module thing()
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
thing();
