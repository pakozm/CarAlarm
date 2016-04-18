
// Module names are of the form poly_<inkscape-path-id>().  As a result,
// you can associate a polygon in this OpenSCAD program with the corresponding
// SVG element in the Inkscape document by looking for the XML element with
// the attribute id="inkscape-path-id".

// fudge value is used to ensure that subtracted solids are a tad taller
// in the z dimension than the polygon being subtracted from.  This helps
// keep the resulting .stl file manifold.
fudge = 0.1;

module poly_path84(h)
{
  scale([25.4/90, -25.4/90, 1]) union()
  {
    linear_extrude(height=h)
      polygon([[46.680189,-40.233086],[46.373892,-38.715945],[45.538593,-37.477032],[44.299680,-36.641733],[42.782539,-36.335437],[41.265398,-36.641733],[40.026484,-37.477032],[39.191186,-38.715945],[38.884889,-40.233086],[39.191186,-41.750228],[40.026484,-42.989141],[41.265398,-43.824440],[42.782539,-44.130736],[44.299680,-43.824440],[45.538593,-42.989141],[46.373892,-41.750228],[46.680189,-40.233086]]);
  }
}

module poly_path188(h)
{
  scale([25.4/90, -25.4/90, 1]) union()
  {
    linear_extrude(height=h)
      polygon([[-44.908082,44.130736],[-44.908082,40.587425]]);
  }
}

module poly_path80(h)
{
  scale([25.4/90, -25.4/90, 1]) union()
  {
    linear_extrude(height=h)
      polygon([[46.680188,-40.233082],[46.373892,-38.715941],[45.538595,-37.477027],[44.299683,-36.641729],[42.782543,-36.335432],[41.265401,-36.641729],[40.026487,-37.477027],[39.191187,-38.715941],[38.884890,-40.233082],[39.191187,-41.750223],[40.026487,-42.989136],[41.265401,-43.824434],[42.782543,-44.130731],[44.299683,-43.824434],[45.538595,-42.989136],[46.373892,-41.750223],[46.680188,-40.233082]]);
  }
}

module poly_path186(h)
{
  scale([25.4/90, -25.4/90, 1]) union()
  {
    linear_extrude(height=h)
      polygon([[-43.136877,42.358631],[-46.680189,42.358631]]);
  }
}

module pcb_details(h)
{

poly_path84(h);
poly_path188(h);
poly_path80(h);
poly_path186(h);
}

pcb_details(3);
