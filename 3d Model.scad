$fn = 40;

// --- 1. 1/8 INCH (3.175mm) BASE ILLUSTRATION BOARD ---
base_t = 3.175;
color("White") 
translate([-180, -140, 0]) 
cube([360, 280, base_t]);

// --- 2. COLORED CONTAINER MODULE (WITH OPTIONAL SENSOR CUTOUT) ---
module container(w=74, l=74, h=36, r=12, wall=2.2, inverted=false, c="WhiteSmoke", sensor_cutout=false) {
    color(c)
    translate([0, 0, inverted ? base_t + h : base_t])
    rotate([inverted ? 180 : 0, 0, 0])
    difference() {
        hull() {
            for (dx = [-w/2+r+3, w/2-r-3], dy = [-l/2+r+3, l/2-r-3])
                translate([dx, dy, 0]) cylinder(r=r, h=1);
            for (dx = [-w/2+r, w/2-r], dy = [-l/2+r, l/2-r])
                translate([dx, dy, h-1]) cylinder(r=r, h=1);
        }
        translate([0, 0, wall])
        hull() {
            for (dx = [-w/2+r+3, w/2-r-3], dy = [-l/2+r+3, l/2-r-3])
                translate([dx, dy, 0]) cylinder(r=r-wall, h=1);
            for (dx = [-w/2+r, w/2-r], dy = [-l/2+r, l/2-r])
                translate([dx, dy, h]) cylinder(r=r-wall, h=1);
        }
        if (sensor_cutout) {
            translate([w/2 - 12, -28, 12]) cube([20, 56, h]);
        }
    }
}

// Front Row: 3 Sorting Baskets (Black, Blue, White)
translate([-115, -85, 0]) container(c="Black");
translate([-35,  -85, 0]) container(c="RoyalBlue");
translate([45,   -85, 0]) container(c="WhiteSmoke");

// Pickup Area (Right): Upright Container (may Sensor Window) + Inverted Container Stand
translate([75,  15, 0]) container(c="WhiteSmoke", sensor_cutout=true);
translate([148, 15, 0]) container(w=66, l=82, h=35, inverted=true, c="WhiteSmoke");

// Sample Target Object sa loob ng Pickup Basket
color("RoyalBlue") translate([75, 15, base_t + 2.5]) cylinder(r=11, h=18);

// --- 3. ARDUINO UNO R3 MODULE (DIRECTLY BEHIND THE ARM PEDESTAL AT -X) ---
module arduino_uno() {
    // Blue PCB (68.6mm x 53.3mm x 1.6mm) with Mounting Holes
    color("RoyalBlue") difference() {
        cube([68.6, 53.3, 1.6]);
        for (pt = [[14, 2.5], [66, 7.6], [66, 35.5], [15.2, 50.8]])
            translate([pt[0], pt[1], -1]) cylinder(r=1.6, h=4);
    }
    // Silver USB Type-B Port
    color("Silver") translate([-6, 32, 1.6]) cube([16, 12, 11]);
    // Black DC Power Barrel Jack
    color("Black") translate([-3, 3.5, 1.6]) cube([14, 9, 11]);
    // ATmega328P 28-Pin DIP IC + Socket
    color("DimGray") translate([32, 7, 1.6]) cube([32, 9.5, 4.5]);
    color("Silver") for (px = [33.5 : 2.54 : 62.5], py = [6.5, 16.5])
        translate([px, py, 1.6]) cube([0.6, 0.8, 3]);
    // Female Pin Header Banks (Top Digital PWM & Bottom Power/Analog)
    color("Black") {
        translate([18, 49.5, 1.6]) cube([25, 2.5, 8.5]);
        translate([45, 49.5, 1.6]) cube([20, 2.5, 8.5]);
        translate([22, 1.2, 1.6])  cube([20, 2.5, 8.5]);
        translate([45, 1.2, 1.6])  cube([16, 2.5, 8.5]);
    }
    // 16MHz Crystal Oscillator & Two Electrolytic Capacitors
    color("Silver") {
        hull() {
            translate([21, 24, 1.6]) cylinder(r=2.2, h=3.5);
            translate([27, 24, 1.6]) cylinder(r=2.2, h=3.5);
        }
        translate([24, 14, 1.6]) cylinder(r=3, h=5.5);
        translate([17, 14, 1.6]) cylinder(r=3, h=5.5);
    }
    // Reset Button (Top-Left Corner)
    color("Gainsboro") translate([2, 47, 1.6]) cube([6, 4.5, 2]);
    color("Crimson") translate([5, 49.2, 3.6]) cylinder(r=1.3, h=1.2);
    // ICSP 6-Pin Headers
    color("Gold") for (gx = [64, 66.5], gy = [22, 24.5, 27])
        translate([gx, gy, 1.6]) cylinder(r=0.4, h=6);
}

// Nakapwesto diretso sa likod ng Robotic Arm Base (-X side, nakasentro sa Y = 20 ng arm)
translate([-132, -6.6, base_t]) arduino_uno();

// --- 4. ANGLED SENSOR MOUNT & SENSORS ---
module tcs230_sensor() {
    color("MidnightBlue") difference() {
        translate([-0.8, -15.5, 0]) cube([1.6, 31, 25]);
        for (dy = [-13, 13], dz = [2.5, 22.5])
            translate([-2, dy, dz]) rotate([0, 90, 0]) cylinder(r=1.2, h=4);
    }
    color("Black") translate([-2, -3, 9.5]) cube([1.5, 6, 6]);
    color("Crimson") translate([-2.2, -1.5, 11]) cube([0.5, 3, 3]);
    color("White") for (dy = [-8, 8], dz = [5.5, 19.5]) {
        translate([-0.8, dy, dz]) rotate([0, -90, 0]) {
            cylinder(r=2.5, h=2);
            translate([0, 0, 2]) sphere(r=2.5);
        }
    }
    color("Gold") for (dy = [-13.5, 13.5], dz = [8, 10.5, 13, 15.5]) {
        translate([0.8, dy, dz]) rotate([0, 90, 0]) cylinder(r=0.4, h=5);
    }
}

module hcsr04_sensor() {
    color("MediumBlue") difference() {
        translate([-0.8, -22.5, 0]) cube([1.6, 45, 20]);
        for (dy = [-20.5, 20.5], dz = [2, 18])
            translate([-2, dy, dz]) rotate([0, 90, 0]) cylinder(r=1, h=4);
    }
    for (dy = [-11.5, 11.5]) {
        color("Silver") translate([-0.8, dy, 10]) rotate([0, -90, 0]) difference() {
            cylinder(r=8, h=12);
            translate([0, 0, 1.5]) cylinder(r=6.8, h=11);
        }
        color("DimGray") translate([-11.5, dy, 10]) rotate([0, -90, 0]) cylinder(r=6.7, h=1);
    }
    color("Silver") translate([-2.2, -4, 15.5]) cube([1.5, 8, 3.5]);
    color("Black") translate([-2, -5, 0]) cube([2.5, 10, 2.5]);
    color("Gold") for (dy = [-3.8, -1.3, 1.3, 3.8]) {
        translate([-0.8, dy, -4]) cylinder(r=0.4, h=5);
    }
}

color("DimGray") {
    translate([106, -18, base_t + 35]) cube([26, 66, 2.5]);
    for (ky = [-16, 14, 45]) {
        translate([108, ky, base_t + 37.5]) rotate([0, -42, 0]) cube([2.5, 2, 24]);
    }
}

translate([108, -1, base_t + 37.5]) rotate([0, -42, 0]) tcs230_sensor();
translate([108, 31, base_t + 37.5]) rotate([0, -42, 0]) hcsr04_sensor();

// --- 5. DETAILED SPUR GEAR & PARALLEL GRIPPER MODULE ---
module spur_gear(r_pitch=9.5, teeth=14, h=3) {
    cylinder(r=r_pitch-1.2, h=h);
    for (i = [0 : teeth-1]) {
        rotate([0, 0, i * 360 / teeth])
        translate([r_pitch-1.5, -1.1, 0])
        cube([2.8, 2.2, h]);
    }
}

module link_bar(p1, p2, r=3.2, h=3, z_off=0) {
    translate([0, 0, z_off])
    difference() {
        hull() {
            translate([p1[0], p1[1], 0]) cylinder(r=r, h=h);
            translate([p2[0], p2[1], 0]) cylinder(r=r, h=h);
        }
        translate([p1[0], p1[1], -0.5]) cylinder(r=1.5, h=h+1);
        translate([p2[0], p2[1], -0.5]) cylinder(r=1.5, h=h+1);
    }
}

module realistic_gripper() {
    color("DimGray") translate([-8, -10, -28]) cube([28, 20, 28]);

    color("Silver") difference() {
        hull() {
            translate([-8, -15, 0]) cylinder(r=5, h=3);
            translate([12, -21, 0]) cylinder(r=7, h=3);
            translate([26, -15, 0]) cylinder(r=6, h=3);
            translate([26,  15, 0]) cylinder(r=6, h=3);
            translate([12,  21, 0]) cylinder(r=7, h=3);
            translate([-8,  15, 0]) cylinder(r=5, h=3);
        }
        translate([4, 0, -1]) cylinder(r=5, h=5);
        for (s = [-1, 1]) {
            translate([10, s*10, -1]) cylinder(r=1.6, h=5);
            translate([24, s*11, -1]) cylinder(r=1.6, h=5);
        }
    }

    for (s = [-1, 1]) {
        scale([1, s, 1]) {
            color("Silver") {
                translate([10, 10, 3]) rotate([0, 0, s == 1 ? 12 : 0]) spur_gear(9.5, 14, 3);
                link_bar([10, 10], [31, 19], r=3.8, h=3, z_off=3);
                link_bar([24, 11], [45, 20], r=3.2, h=3, z_off=3);
            }

            color("Silver") translate([0, 0, 6]) difference() {
                 union() {
                    hull() {
                        translate([31, 19, 0]) cylinder(r=4, h=3.5);
                        translate([45, 20, 0]) cylinder(r=4, h=3.5);
                        translate([48, 12, 0]) cylinder(r=3.5, h=3.5);
                    }
                    hull() {
                        translate([45, 16, 0]) cylinder(r=4, h=3.5);
                        translate([76, 6, 0])  cylinder(r=2.2, h=3.5);
                        translate([76, 2.5, 0]) cylinder(r=1.5, h=3.5);
                        translate([48, 8, 0])  cylinder(r=3, h=3.5);
                    }
                }
                translate([47, 1, -1]) rotate([0, 0, -10]) cube([18, 6, 6]);
                for (tx = [66, 69, 72, 75]) {
                    translate([tx, 2.2, -1]) rotate([0, 0, 45]) cube([2.2, 2.2, 6]);
                }
                translate([31, 19, -1]) cylinder(r=1.5, h=6);
                translate([45, 20, -1]) cylinder(r=1.5, h=6);
            }

            color("Gainsboro") {
                for (pt = [[10, 10], [24, 11], [31, 19], [45, 20]]) {
                    translate([pt[0], pt[1], -1]) cylinder(r=2.2, h=11.5);
                }
            }
        }
    }
}

// --- 6. ALIGNED ROBOTIC ARM ASSEMBLY ---
translate([-15, 20, base_t]) {
    color("Silver") {
        translate([-35, -22, 0]) cube([70, 44, 3]);
        for (bx = [-30, 4]) {
            translate([bx, -19, 3]) difference() {
                cube([26, 38, 54]);
                translate([2.5, 2.5, -1]) cube([21, 33, 56]);
            }
        }
        for (sy = [-18, 15]) {
            translate([-22, sy, 48]) rotate([0, -12, 0]) difference() {
                cube([22, 3, 98]);
                for (z = [15, 48, 82]) translate([11, 5, z]) rotate([90, 0, 0]) cylinder(r=3.5, h=8);
            }
        }
        translate([0, -21, 118]) difference() {
            cube([56, 42, 36]);
            translate([2.5, 2.5, 2.5]) cube([51, 37, 34]);
        }
        translate([56, -17, 112]) rotate([0, 22, 0]) difference() {
            cube([36, 34, 38]);
            translate([-2, 2.5, 2.5]) cube([36, 29, 33]);
        }
    }

    color("DimGray") {
        translate([-12, -10, 32])  cube([40, 20, 36]);
        translate([-24, -10, 122]) cube([40, 20, 36]);
        translate([12,  -10, 120]) cube([40, 20, 36]);
    }

    translate([90, -5, 112]) 
    rotate([0, 90, 0]) 
    realistic_gripper();
}