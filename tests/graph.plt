#before calling this set
#filename= "<file>.csv"
set datafile separator ","
set y2tics
set xlabel "Time (seconds)"
set ylabel "PPG"
set y2label "Pressure (PSI)"
plot filename using 1:3 with lines title "Position (mm)" axis x1y2 lc rgb "red", filename using 1:4 with lines title "660nm", filename using 1:5 with lines title "750nm", filename using 1:6 with lines title "810nm",filename using 1:7 with lines title "870nm", filename using 1:8 with lines title "960nm", filename using 1:2 with lines title "Pressure" axis x1y2 lc rgb "black"
