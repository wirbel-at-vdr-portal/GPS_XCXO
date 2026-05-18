$fn=100;
use <wk/RoundBox.scad>
use <wk/RoundCylinder.scad>


// PCB outer dims.
h=1.6;
t = 43.375;
b = 74.5;

// die Loecher im PCB
t1 = 40.005;
d = 3.2;
b1 = 6.2259;
b2 = 62.33;
  
  
  
module PCB() {
  t1 = 40.005;
  d = 3.2;
  b1 = 6.2259;
  b2 = 62.33;
  
  color("Green") 
  difference() {
     translate([0,0,h/2]) cube([b,t,h], true);
     translate([-b/2+b1,-t1/2,-0.5]) cylinder(5, d=d);
     translate([-b/2+b1,+t1/2,-0.5]) cylinder(5, d=d);
     translate([-b/2+b1+b2,-t1/2,-0.5]) cylinder(5, d=d);
     translate([-b/2+b1+b2,+t1/2,-0.5]) cylinder(5, d=d);
     }
}

module DC005() {
  color("Grey") 

  difference() {
    union() {
      translate([-3.3/2,0,11/2]) RoundBox(3.3,9,11, 0.5);
      translate([-14.2/2,0,6.5/2]) RoundBox(14.2,9,6.5, 0.5);     
      translate([-14.2,0,(9-1)/2+2]) rotate([0,90,0]) RoundCylinder(h=14.2, d=9-1, 0.5);
      }
    translate([-8,0,6.5]) rotate([0,90,0]) RoundCylinder(h=10, d=6.4, 0.5);
    } 
  color("Silver") translate([-9,0,6.5]) rotate([0,90,0]) RoundCylinder(h=8, d=2.5, 0.25);  


}

module SMA_female() {
  color("Gold")
  difference() {
     union() {
        translate([13.5-4-8    ,0, +0.8/2]) rotate([0,90,0]) RoundCylinder(h=8, d=5.3, 0.1);
        translate([13.5-4-8+(4.3-1.7),0, +0.8/2]) rotate([0,90,0]) RoundCylinder(h=5, d=6.35, 0.3);
        
        translate([+1.5/2,0, +0.8/2]) rotate([0,90,0]) RoundBox(6.5,6.5,1.5, 0.07);
        
        translate([-4/2,+(6-0.9)/2, +0.8/2]) rotate([0,90,0]) RoundBox(0.9,0.9,4.0, 0.07);
        translate([-4/2,-(6-0.9)/2, +0.8/2]) rotate([0,90,0]) RoundBox(0.9,0.9,4.0, 0.07);
        translate([-4/2,+(6-0.9)/2, +0.8/2-(1.7+0.9)]) rotate([0,90,0]) RoundBox(0.9,0.9,4.0, 0.07);
        translate([-4/2,-(6-0.9)/2, +0.8/2-(1.7+0.9)]) rotate([0,90,0]) RoundBox(0.9,0.9,4.0, 0.07);
        }
     translate([13.5-4-8+1,0, +0.8/2]) rotate([0,90,0]) RoundCylinder(h=8, d=4.65, 0.1);
     translate([13.5-4-8-1.6,0, +0.8/2]) rotate([0,90,0]) RoundCylinder(h=8, d=4, 0.1);
     }

  difference() {
     union() {
        color("White") translate([13.5-4-8-1.5,0, +0.8/2]) rotate([0,90,0]) RoundCylinder(h=2, d=4, 0.1);
        color("White") translate([13.5-4-8-0  ,0, +0.8/2]) rotate([0,90,0]) RoundCylinder(h=6, d=4.65, 0.1);
        }
     color("Gold") translate([13.5-4-8-1.6,0, +0.8/2]) rotate([0,90,0]) RoundCylinder(h=10, d=1, 0.1);
     }
  color("Gold") translate([-4,0, +0.8/2]) rotate([0,90,0]) RoundCylinder(h=10, d=1, 0.1);
}

module OCXO() {
  color("Silver")
  union() {
     translate([0,0,12.5/2]) RoundBox(23.65,23.65, 12.5, 1.5);
     translate([0,0,0.8/2]) RoundBox(25.2,25.2, 0.8, 0.1);  
     }
}

module OCXO_module() {
  PCB();
  translate([-b/2+17.145,-t/2-1,h]) rotate([0,0,-90]) DC005();
  translate([-b/2+46,+t/2,h]) rotate([0,0,+90])       SMA_female();
  translate([-b/2+59.6849,+t/2,h]) rotate([0,0,+90])  SMA_female();
  
  translate([-b/2+41.68+23.65/2,-t/2+3.04+23.65/2,h]) OCXO();
  
}


//OCXO_module();





module Rohr(h, di, da) {
  
  difference() {
     cylinder(h, d=da);
     translate([0,0,-1]) cylinder(h+2, d=di);  
     }

}


module case() {
  // 4x Halter
  translate([-b/2+b1,-t1/2   ,-10]) Rohr(10, 2.6, 2.6+2*2);
  translate([-b/2+b1,+t1/2   ,-10]) Rohr(10, 2.6, 2.6+2*2);
  translate([-b/2+b1+b2,-t1/2,-10]) Rohr(10, 2.6, 2.6+2*2);
  translate([-b/2+b1+b2,+t1/2,-10]) Rohr(10, 2.6, 2.6+2*2);
 
  // Boden
  spalt=1.5;
  difference() {
     translate([0,0,-10-4/2]) RoundBox(b+2*4+2*spalt, t+2*4+2*spalt, 4, 0.5);
     // 4x loch fuer halter. Vereinfacht gewinde schneiden.     
     translate([-b/2+b1,-t1/2   ,-12.5]) cylinder(10, d=2.6);
     translate([-b/2+b1,+t1/2   ,-12.5]) cylinder(10, d=2.6);
     translate([-b/2+b1+b2,-t1/2,-12.5]) cylinder(10, d=2.6);
     translate([-b/2+b1+b2,+t1/2,-12.5]) cylinder(10, d=2.6);
     }
  
  // kurze Seiten
  h_box = 4+10+1.6+12.5+6+4; // = 38.1
  difference() {
     union() {
        translate([+b/2+4/2+spalt,0,-10-4+h_box/2]) RoundBox(4, t+2*4+2*spalt, h_box, 0.5);
        translate([-b/2-4/2-spalt,0,-10-4+h_box/2]) RoundBox(4, t+2*4+2*spalt, h_box, 0.5);
        }
     translate([-50,+17.5,20]) rotate([0,+90,0]) cylinder(100, d=3.2); 
     translate([-50,-17.5,20]) rotate([0,+90,0]) cylinder(100, d=3.2);
     }

  // lange Seiten
  difference() { // SMA Seite
    translate([0,+t/2+4/2+spalt,-10-4+h_box/2]) RoundBox(b+2*4+2*spalt, 4, h_box, 0.5);
    translate([-b/2+46     ,+t/2+1.5,h]) rotate([-90,0,0]) RoundCylinder(10, 10.5, 2);    
    translate([-b/2+59.6849,+t/2+1.5,h]) rotate([-90,0,0]) RoundCylinder(10, 10.5, 2);    
    translate([-b/2+46     ,+t/2-3.2,h+6]) rotate([0,0,0]) RoundBox(11,11,22, 1);
    translate([-b/2+59.6849,+t/2-3.2,h+6]) rotate([0,0,0]) RoundBox(11,11,22, 1);    
    translate([-b/2+70,+t/2,h+5]) rotate([-90,0,0]) cylinder(10, d=5); // 1PPS
    }

  difference() { // power Seite
    translate([0,-t/2-4/2-spalt,-10-4+h_box/2]) RoundBox(b+2*4+2*spalt, 4, h_box, 0.5);
    translate([-b/2+17.145,-t/2+3,h+9])    rotate([0,0,-90]) RoundBox(11,11,22, 1);
    translate([-b/2+17.145,-t/2-1.8,h+6.5]) rotate([+90,0,0]) RoundCylinder(10, 10.5, 2);  
    }
}

module deckel() {
  // Deckel
  spalt=1.5;
  h_box = 4+10+1.6+12.5+6+4;
  translate([0,0,1.6+12.5+6+4+4/2]) RoundBox(b+2*4+2*spalt, t+2*4+2*spalt, 4, 0.5);
  difference() {
     union() {
        translate([+b/2-4/2+spalt,0,-10-8+h_box]) RoundBox(4, t-1+2*spalt, 10, 0.5);
        translate([-b/2+4/2-spalt,0,-10-8+h_box]) RoundBox(4, t-1+2*spalt, 10, 0.5);
        }
     translate([-50,+17.5,20]) rotate([0,+90,0]) cylinder(100, d=2.5); 
     translate([-50,-17.5,20]) rotate([0,+90,0]) cylinder(100, d=2.5);
     }
}

color("Aqua") case();
deckel();
