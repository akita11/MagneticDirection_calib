# === Gnuplot script to plot ellipsoid ===

reset
set parametric
set angle degrees
set hidden3d
set view equal xyz

# === Replace with your parameters ===

x0 = -2.448
y0 = -11.007
z0 = -34.202
a = 6.720*2
b = 8.866*2
c = 7.963*2

# === u: azimuth [0,360], v: elevation [-90,90] ===
splot \
      x0 + a * cos(u) * cos(v), y0 + b * sin(u) * cos(v), z0 + c * sin(v), \
      "dd"
    

set ticslevel 0
set urange [0:360]
set vrange [-90:90]
set isosamples 50, 50
